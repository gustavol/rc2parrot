#include "drone.h"

#include <Arduino.h>
#include <NimBLEDevice.h>

#include "config.h"
#include "ftp.h"
#include "log.h"

// ---------- ARSDK3 framing ----------
// Frame: [type, seq, project, class, cmd_lo, cmd_hi, args...]
enum : uint8_t { T_ACK = 1, T_DATA = 2, T_DATA_ACK = 4 };  // ARNetwork frame types
enum : uint8_t { PRJ_COMMON = 0, PRJ_MINIDRONE = 2 };
constexpr uint8_t CMD_PCMD = 2;  // minidrone.Piloting.PCMD

// UUID format: 9a66XXXX-0800-9191-11e4-012d1540cb8e
static const char* const NOTIFY_IDS[] = {"fb0f", "fb0e", "fb1b", "fb1c", "fd22", "fd23", "fd24", "fd52", "fd53", "fd54"};

static NimBLEClient* client = nullptr;
static NimBLERemoteCharacteristic *chNoAck, *chAckOut;  // fa0a (PCMD), fa1e (our ACKs)
static NimBLEUUID uuidAckData, uuidAckCmd, uuidAckPrio;  // fb0e, fb1b, fb1c
static uint8_t seqNoAck, seqAckOut;
static NimBLEAddress lastAddress;  // reconnects skip the scan

static volatile bool linkUp = false;
static volatile int battery = -1;
static volatile int flyingState = -1;  // minidrone.PilotingState.FlyingStateChanged; 0 = landed
static volatile bool trimEvent = false;
static volatile bool commandFailed = false;

// minidrone.PilotingState.AlertStateChanged
enum : int { ALERT_NONE = 0, ALERT_USER = 1, ALERT_CUT_OUT = 2, ALERT_CRITICAL_BATTERY = 3, ALERT_LOW_BATTERY = 4 };
static const char* const ALERT_NAMES[] = {"none", "user emergency", "motor cut-out", "critical battery",
                                          "low battery"};
static volatile int alertState = ALERT_NONE;

// The drone only reports trim success (FlatTrimChanged); there is no failure event.
static bool trimWaiting = false;
static uint32_t trimSentMs = 0;
static TrimResult trimResult = TrimResult::None;
static bool autoTrimDone = false;  // automatic trim only on the first connection after boot
static bool autoTrimPending = false;
static uint32_t linkUpMs = 0;

static Pcmd lastPcmd = {0, 0, 0, 0};
static uint32_t lastPcmdMs = 0;

// ---------- Acknowledged sends with retry ----------
// Same buffers as the Parrot SDK (libARDiscovery, BLE network config):
//   fa0b, commands:   ACK within 500 ms, 3 retries.
//   fa0c, emergency:  ACK within 100 ms, unlimited retries while connected.
// The drone acknowledges on fb1b (fa0b) or fb1c (fa0c) with [T_ACK, own seq, acknowledged seq].
// A retry repeats the same seq, so the drone drops the duplicate and acknowledges again.
struct Pending {
  uint8_t frame[32];
  uint8_t len;
  uint8_t tries;
  uint32_t sentMs;
  bool used;
};
struct AckChannel {
  const char* name;
  uint32_t timeoutMs;
  int maxRetries;  // < 0 = unlimited
  NimBLERemoteCharacteristic* chr;
  uint8_t seq;
  Pending slots[8];
};
static AckChannel ackCh = {"command", 500, 3}, prioCh = {"emergency", 100, -1};

// Events from the NimBLE task to the loop: (type << 8) | seq.
enum : uint8_t { EV_ACK_CMD = 0, EV_ACK_PRIO = 1, EV_DRONE_NEEDS_ACK = 2 };
static QueueHandle_t events = nullptr;

static void pushEvent(uint8_t type, uint8_t seq) {
  uint16_t e = (uint16_t)type << 8 | seq;
  xQueueSend(events, &e, 0);
}

static NimBLEUUID uuid(const char* id) {
  return NimBLEUUID(std::string("9a66") + id + "-0800-9191-11e4-012d1540cb8e");
}

static NimBLERemoteCharacteristic* findChr(const char* id) {
  NimBLEUUID u = uuid(id);
  for (auto* s : client->getServices(false))
    for (auto* c : s->getCharacteristics(false))
      if (c->getUUID() == u) return c;
  return nullptr;
}

static void send(NimBLERemoteCharacteristic* c, uint8_t& seq, uint8_t type, const uint8_t* body, size_t n) {
  uint8_t buf[32] = {type, ++seq};
  memcpy(buf + 2, body, n);
  c->writeValue(buf, n + 2, false);
}

static void sendWithAck(AckChannel& ch, const uint8_t* body, size_t n) {
  Pending* p = nullptr;
  for (auto& s : ch.slots)
    if (!s.used) { p = &s; break; }
  if (!p) {  // queue full: drop the oldest
    p = &ch.slots[0];
    for (auto& s : ch.slots)
      if ((int32_t)(s.sentMs - p->sentMs) < 0) p = &s;
    logf("%s queue full: dropped the oldest\n", ch.name);
  }
  p->frame[0] = T_DATA_ACK;
  p->frame[1] = ++ch.seq;
  memcpy(p->frame + 2, body, n);
  p->len = n + 2;
  p->tries = 0;
  p->sentMs = millis();
  p->used = true;
  ch.chr->writeValue(p->frame, p->len, false);
}

static void onAck(AckChannel& ch, uint8_t seq) {
  for (auto& s : ch.slots)
    if (s.used && s.frame[1] == seq) {
      s.used = false;
      if (&ch == &prioCh) logf("emergency acknowledged\n");
      return;
    }
  logf("%s ACK without a pending command (seq %u)\n", ch.name, seq);
}

static void retry(AckChannel& ch) {
  uint32_t now = millis();
  for (auto& s : ch.slots) {
    if (!s.used || now - s.sentMs < ch.timeoutMs) continue;
    if (ch.maxRetries >= 0 && s.tries >= ch.maxRetries) {
      s.used = false;
      logf("%s not acknowledged, giving up: %u.%u.%u\n", ch.name, s.frame[2], s.frame[3], s.frame[4]);
      commandFailed = true;
      continue;
    }
    s.tries++;
    s.sentMs = now;
    ch.chr->writeValue(s.frame, s.len, false);
    logf("%s: retry %u\n", ch.name, s.tries);
  }
}

static void resetChannel(AckChannel& ch, NimBLERemoteCharacteristic* chr) {
  ch.chr = chr;
  for (auto& s : ch.slots) s.used = false;
}

// ---------- Drone limits at maximum ----------
// The radio limits instead (dual rates). After AllSettings, each *Changed event carries
// current, min and max as floats from d[6]. A limit is written only if current < max.
struct Limit {
  const char* name;
  uint8_t evClass, evCmd;    // minidrone.*SettingsState event
  uint8_t setClass, setCmd;  // minidrone.*Settings command
  float current, max;
  volatile bool fresh;
};
static Limit limits[] = {
    {"max tilt (deg)", 9, 1, 8, 1},               // PilotingSettings.MaxTilt
    {"max vertical speed (m/s)", 5, 0, 1, 0},     // SpeedSettings.MaxVerticalSpeed
    {"max rotation speed (deg/s)", 5, 1, 1, 1},   // SpeedSettings.MaxRotationSpeed
};

static void serviceLimits() {
  for (auto& l : limits) {
    if (!l.fresh) continue;
    l.fresh = false;
    if (l.current >= l.max - 0.01f) {
      logf("%s: %.1f, already at max\n", l.name, l.current);
      continue;
    }
    uint8_t body[8] = {PRJ_MINIDRONE, l.setClass, l.setCmd, 0};
    memcpy(body + 4, &l.max, 4);
    sendWithAck(ackCh, body, sizeof body);
    logf("%s: %.1f -> %.1f\n", l.name, l.current, l.max);
  }
}

static void sendPcmd(const Pcmd& p) {
  uint8_t flag = (p.roll || p.pitch) ? 1 : 0;  // 0 = the drone ignores roll/pitch and holds position
  const uint8_t body[] = {PRJ_MINIDRONE, 0, CMD_PCMD, 0, flag, (uint8_t)p.roll, (uint8_t)p.pitch,
                          (uint8_t)p.yaw, (uint8_t)p.gaz, 0, 0, 0, 0};  // timestamp u32 = 0
  send(chNoAck, seqNoAck, T_DATA, body, sizeof body);
  lastPcmdMs = millis();
}

// ---------- Drone events (NimBLE task) ----------
static void onNotify(NimBLERemoteCharacteristic* c, uint8_t* d, size_t n, bool) {
  if (ftpOwns(c)) { ftpOnNotify(d, n); return; }
  const NimBLEUUID& u = c->getUUID();
  if (n >= 3 && d[0] == T_ACK) {
    if (u == uuidAckCmd) pushEvent(EV_ACK_CMD, d[2]);
    else if (u == uuidAckPrio) pushEvent(EV_ACK_PRIO, d[2]);
    return;
  }
  if (n >= 2 && d[0] == T_DATA_ACK && u == uuidAckData) pushEvent(EV_DRONE_NEEDS_ACK, d[1]);
  if (n < 5) return;
  for (auto& l : limits)
    if (n >= 18 && d[2] == PRJ_MINIDRONE && d[3] == l.evClass && d[4] == l.evCmd) {
      memcpy(&l.current, d + 6, 4);
      memcpy(&l.max, d + 14, 4);
      l.fresh = true;
    }
  // minidrone.PilotingState: FlatTrimChanged (0), FlyingStateChanged (1), AlertStateChanged (2).
  // Enums are 4 bytes, from d[6].
  if (d[2] == PRJ_MINIDRONE && d[3] == 3 && d[4] == 0) trimEvent = true;
  if (n >= 7 && d[2] == PRJ_MINIDRONE && d[3] == 3 && d[4] == 1) flyingState = d[6];
  if (n >= 7 && d[2] == PRJ_MINIDRONE && d[3] == 3 && d[4] == 2 && d[6] != alertState) {
    alertState = d[6];
    logf("drone alert: %s\n", d[6] <= ALERT_LOW_BATTERY ? ALERT_NAMES[d[6]] : "unknown");
  }
  // minidrone.MediaRecordEvent.PictureEventChanged. Error enum from d[10]:
  // 1 unknown, 2 busy, 3 not available, 4 memory full, 5 low battery.
  if (n >= 11 && d[2] == PRJ_MINIDRONE && d[3] == 2 && d[4] == 0)
    logf("picture: %s error=%u\n", d[6] ? "failed" : "taken", d[10]);
  // common.CommonState.BatteryStateChanged
  if (n >= 7 && d[2] == PRJ_COMMON && d[3] == 5 && d[4] == 1) {
    battery = d[6] > 100 ? 100 : d[6];
    logf("battery %u%%\n", d[6]);
  }
}

// NimBLE disconnect reason = HCI error + 0x200 (BLE_HS_ERR_HCI_BASE).
static const char* disconnectText(int reason) {
  switch (reason) {
    case 0x208: return "the drone stopped responding (supervision timeout); with a weak battery it likely rebooted";
    case 0x213: return "the drone closed the connection";
    case 0x216: return "the ESP32 closed the connection";
    case 0x23e: return "the connection failed to establish";
    default: return "unknown reason";
  }
}

class ClientCallbacks : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient*, int reason) override {
    linkUp = false;
    battery = -1;
    flyingState = -1;
    alertState = ALERT_NONE;
    logf("disconnected: %s (0x%x)\n", disconnectText(reason), reason);
  }
  bool onConnParamsUpdateRequest(NimBLEClient*, const ble_gap_upd_params* p) override {
    logf("drone requested interval %.2f to %.2f ms\n", p->itvl_min * 1.25f, p->itvl_max * 1.25f);
    return true;
  }
};
static ClientCallbacks callbacks;

// ---------- Connection ----------
// Manufacturer data: BLE company 0x0043 (Parrot) + USB vendor 0x19CF (Parrot) + product ID.
// The name is not reliable: the Airborne Cargo advertises just "Travis", and users can rename it.
static bool isDrone(const NimBLEAdvertisedDevice* d) {
  std::string m = d->getManufacturerData();
  return m.size() >= 4 && m[0] == 0x43 && m[1] == 0x00 && (uint8_t)m[2] == 0xcf && m[3] == 0x19;
}

class ScanCallbacks : public NimBLEScanCallbacks {
 public:
  NimBLEAddress found;
  void onResult(const NimBLEAdvertisedDevice* d) override {
    if (!isDrone(d)) return;
    found = d->getAddress();
    logf("found %s (%s)\n", d->getName().c_str(), found.toString().c_str());
    NimBLEDevice::getScan()->stop();
  }
};
static ScanCallbacks scanCallbacks;

static bool scanForDrone() {
  logf("scanning for drone...\n");
  scanCallbacks.found = NimBLEAddress();
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->start(BLE_SCAN_MS, false, true);
  while (scan->isScanning()) delay(10);
  if (scanCallbacks.found.isNull()) return false;
  lastAddress = scanCallbacks.found;
  return true;
}

static bool setupLink() {
  flyingState = -1;
  if (!client->discoverAttributes()) return false;
  for (auto* id : NOTIFY_IDS) {
    auto* c = findChr(id);
    if (c && c->canNotify()) c->subscribe(true, onNotify);
  }
  chNoAck = findChr("fa0a");
  chAckOut = findChr("fa1e");
  resetChannel(ackCh, findChr("fa0b"));
  resetChannel(prioCh, findChr("fa0c"));
  xQueueReset(events);
  for (auto& l : limits) l.fresh = false;
  ftpAttach(findChr("fd23"), findChr("fd24"));
  if (!chNoAck || !chAckOut || !ackCh.chr || !prioCh.chr) {
    logf("characteristics not found\n");
    return false;
  }

  // common.Common.CurrentDate, as node-rolling-spider sends after connecting
  const uint8_t date[] = {PRJ_COMMON, 4, 1, 0, '2', '0', '1', '4', '-', '1', '0', '-', '2', '8', 0};
  sendWithAck(ackCh, date, sizeof date);
  // common.Common.AllStates: without it the drone does not report the battery
  const uint8_t allStates[] = {PRJ_COMMON, 4, 0, 0};
  sendWithAck(ackCh, allStates, sizeof allStates);
  // common.Settings.AllSettings: the drone replies with current, min and max limits
  const uint8_t allSettings[] = {PRJ_COMMON, 2, 0, 0};
  sendWithAck(ackCh, allSettings, sizeof allSettings);

  lastPcmd = {0, 0, 0, 0};
  trimWaiting = false;
  autoTrimPending = !autoTrimDone;
  linkUpMs = millis();
  return true;
}

void droneBegin() {
  uuidAckData = uuid("fb0e");
  uuidAckCmd = uuid("fb1b");
  uuidAckPrio = uuid("fb1c");
  events = xQueueCreate(32, sizeof(uint16_t));
  NimBLEDevice::init("");
  bool ok = NimBLEDevice::setPower(BLE_TX_DBM);
  logf("BLE TX power %d dBm: %s\n", NimBLEDevice::getPower(), ok ? "ok" : "failed");

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);  // the name comes in the scan response
  scan->setScanCallbacks(&scanCallbacks, false);

  client = NimBLEDevice::createClient();
  client->setClientCallbacks(&callbacks, false);
  client->setConnectionParams(BLE_INTERVAL_MIN, BLE_INTERVAL_MAX, 0, BLE_SUPERVISION);
  client->setConnectTimeout(BLE_CONNECT_TIMEOUT_MS);
}

bool droneConnect() {
  bool connected = !lastAddress.isNull() && client->connect(lastAddress);
  if (!connected) connected = scanForDrone() && client->connect(lastAddress);
  if (!connected) return false;
  if (!setupLink()) {
    client->disconnect();
    return false;
  }
  linkUp = true;
  logf("connected\n");
  dronePrintInfo();
  return true;
}

bool droneLinkUp() { return linkUp; }
int droneBattery() { return battery; }

void droneCommand(DroneCmd cmd) {
  if (!linkUp) return;
  // With a critical battery, the motors pull the voltage down and the drone reboots mid take-off.
  if (cmd == DroneCmd::TakeOff && alertState == ALERT_CRITICAL_BATTERY) {
    logf("take-off blocked: the drone reports a critical battery\n");
    commandFailed = true;
    return;
  }
  const uint8_t body[] = {PRJ_MINIDRONE, 0, (uint8_t)cmd, 0};  // class Piloting = 0
  if (cmd == DroneCmd::FlatTrim) {
    trimEvent = false;
    trimWaiting = true;
    trimSentMs = millis();
  }
  sendWithAck(cmd == DroneCmd::Emergency ? prioCh : ackCh, body, sizeof body);
  logf("cmd %u\n", (unsigned)cmd);
}

void dronePcmd(const Pcmd& p) {
  lastPcmd = p;  // if it is too early, the keepalive or the next frame sends this value
  if (linkUp && millis() - lastPcmdMs >= PCMD_MIN_INTERVAL_MS) sendPcmd(p);
}

void droneTakePicture() {
  if (!linkUp) return;
  const uint8_t body[] = {PRJ_MINIDRONE, 6, 1, 0};  // minidrone.MediaRecord.PictureV2
  sendWithAck(ackCh, body, sizeof body);
  logf("picture requested\n");
}

bool droneListFiles(const char* path) { return linkUp && ftpList(path); }

void droneService() {
  if (!linkUp) return;
  uint16_t e;
  while (xQueueReceive(events, &e, 0) == pdTRUE) {
    uint8_t seq = e & 0xff;
    switch (e >> 8) {
      case EV_ACK_CMD: onAck(ackCh, seq); break;
      case EV_ACK_PRIO: onAck(prioCh, seq); break;
      case EV_DRONE_NEEDS_ACK: send(chAckOut, seqAckOut, T_ACK, &seq, 1); break;
    }
  }
  retry(prioCh);
  retry(ackCh);
  if (millis() - lastPcmdMs >= PCMD_PERIOD_MS) sendPcmd(lastPcmd);
  serviceLimits();

  // Automatic trim only when landed: trimming in flight or on uneven ground ruins the calibration.
  if (autoTrimPending) {
    if (flyingState == 0) {
      autoTrimPending = false;
      autoTrimDone = true;
      logf("automatic trim\n");
      droneCommand(DroneCmd::FlatTrim);
    } else if (millis() - linkUpMs > AUTO_TRIM_WAIT_MS) {
      autoTrimPending = false;
      logf("automatic trim cancelled: the drone did not report that it is landed (state %d)\n", flyingState);
      trimResult = TrimResult::Failed;
    }
  }

  if (trimWaiting && trimEvent) {
    trimWaiting = false;
    trimResult = TrimResult::Ok;
    logf("trim ok\n");
  } else if (trimWaiting && millis() - trimSentMs > TRIM_TIMEOUT_MS) {
    trimWaiting = false;
    trimResult = TrimResult::Failed;
    logf("trim not confirmed: failed\n");
  }
}

bool droneTakeCommandFailed() {
  bool f = commandFailed;
  commandFailed = false;
  return f;
}

TrimResult droneTakeTrimResult() {
  TrimResult r = trimResult;
  trimResult = TrimResult::None;
  return r;
}

void dronePrintInfo() {
  if (!linkUp) {
    logf("not connected\n");
    return;
  }
  NimBLEConnInfo info = client->getConnInfo();
  logf("interval %.2f ms, latency %u, MTU %u, RSSI %d dBm, TX power %d dBm\n", info.getConnInterval() * 1.25f,
       info.getConnLatency(), client->getMTU(), client->getRssi(), NimBLEDevice::getPower());
}
