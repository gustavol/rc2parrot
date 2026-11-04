#include "drone.h"

#include <Arduino.h>
#include <NimBLEDevice.h>

#include "config.h"
#include "ftp.h"
#include "log.h"
#include "settings.h"

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
static volatile int rssi = 0;           // dBm, read in droneService; 0 = unknown
static volatile int linkQuality = 100;  // 0..100, from the ACKs of the acknowledged commands
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
// One frame in flight per buffer, as libARNetwork does (ARNETWORK_Sender: the next frame waits
// for the ACK). The drone drops a frame up to 10 behind its last seq, so with several frames in
// flight a retry that arrives after a newer frame is acknowledged and silently dropped.
struct Pending {
  uint8_t frame[32];
  uint8_t len;
  uint8_t tries;
  uint32_t sentMs;
};
struct AckChannel {
  const char* name;
  uint32_t timeoutMs;
  int maxRetries;  // < 0 = unlimited
  NimBLERemoteCharacteristic* chr;
  uint8_t seq;
  Pending q[8];  // FIFO; q[head] is the frame in flight
  uint8_t head, count;
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

static void transmitHead(AckChannel& ch) {
  Pending& p = ch.q[ch.head];
  p.sentMs = millis();
  ch.chr->writeValue(p.frame, p.len, false);
}

static void popHead(AckChannel& ch) {
  ch.head = (ch.head + 1) % 8;
  if (--ch.count) transmitHead(ch);
}

static void sendWithAck(AckChannel& ch, const uint8_t* body, size_t n) {
  if (ch.count == 8) {
    logf("%s queue full: command dropped\n", ch.name);
    commandFailed = true;
    return;
  }
  Pending& p = ch.q[(ch.head + ch.count) % 8];
  p.frame[0] = T_DATA_ACK;
  p.frame[1] = ++ch.seq;
  memcpy(p.frame + 2, body, n);
  p.len = n + 2;
  p.tries = 0;
  if (ch.count++ == 0) transmitHead(ch);
}

static void onAck(AckChannel& ch, uint8_t seq) {
  if (ch.count && ch.q[ch.head].frame[1] == seq) {
    linkQuality = linkQuality + (100 - linkQuality + 3) / 4;
    if (&ch == &prioCh) logf("emergency acknowledged\n");
    popHead(ch);
    return;
  }
  logf("%s ACK without a pending command (seq %u)\n", ch.name, seq);
}

static void retry(AckChannel& ch) {
  if (!ch.count) return;
  Pending& p = ch.q[ch.head];
  if (millis() - p.sentMs < ch.timeoutMs) return;
  linkQuality = linkQuality - (linkQuality + 3) / 4;  // each timeout counts as a lost frame
  if (ch.maxRetries >= 0 && p.tries >= ch.maxRetries) {
    logf("%s not acknowledged, giving up: %u.%u.%u\n", ch.name, p.frame[2], p.frame[3], p.frame[4]);
    commandFailed = true;
    popHead(ch);
    return;
  }
  p.tries++;
  transmitHead(ch);  // same seq: the drone drops the duplicate and acknowledges again
  logf("%s: retry %u\n", ch.name, p.tries);
}

static void resetChannel(AckChannel& ch, NimBLERemoteCharacteristic* chr) {
  ch.chr = chr;
  ch.head = ch.count = 0;
}

// ---------- Drone limits ----------
// After AllSettings, each *Changed event carries current, min and max as floats from d[6], or one
// u8 at d[6] for on-off settings. The firmware writes the target when the current value differs.
struct Limit {
  const char* name;
  uint8_t evClass, evCmd;    // minidrone.*SettingsState event
  uint8_t setClass, setCmd;  // minidrone.*Settings command
  const uint8_t* setting;    // field of settings; nullptr = the drone maximum
  float scale;               // setting unit -> drone unit
  bool onOff;                // u8 setting instead of float
  float current, max;
  volatile bool fresh;
};
static Limit limits[] = {
    {"max tilt (deg)", 9, 1, 8, 1, &settings.maxTilt, 1},                       // PilotingSettings.MaxTilt
    {"max altitude (m)", 9, 0, 8, 0, &settings.maxAltitude, 0.1f},              // PilotingSettings.MaxAltitude
    {"max vertical speed (m/s)", 5, 0, 1, 0, &settings.maxVertSpeed, 0.01f},    // SpeedSettings.MaxVerticalSpeed
    {"max rotation speed (deg/s)", 5, 1, 1, 1, &settings.maxRotSpeed, 1},       // SpeedSettings.MaxRotationSpeed
    {"banked turn", 9, 2, 8, 2, &settings.bankedTurn, 1, true},                 // PilotingSettings.BankedTurn
    {"cut-out on impact", 11, 2, 10, 0, &settings.cutOut, 1, true},             // Settings.CutOutMode
};

static void serviceLimits() {
  for (auto& l : limits) {
    if (!l.fresh) continue;
    l.fresh = false;
    float want = l.setting ? *l.setting * l.scale : 0;
    float target = l.onOff ? want : (want > 0 && want < l.max) ? want : l.max;
    if (fabsf(l.current - target) < 0.05f) {
      logf("%s: %.1f (drone max %.1f)\n", l.name, l.current, l.max);
      continue;
    }
    uint8_t body[8] = {PRJ_MINIDRONE, l.setClass, l.setCmd, 0, (uint8_t)target};
    if (!l.onOff) memcpy(body + 4, &target, 4);
    sendWithAck(ackCh, body, l.onOff ? 5 : sizeof body);
    logf("%s: %.1f -> %.1f (drone max %.1f)\n", l.name, l.current, target, l.max);
  }
}

static uint8_t sentAccessory = 0xFF;

static void sendAccessory() {
  const uint8_t body[] = {PRJ_COMMON, 26, 0, 0, settings.accessory, 0, 0, 0};  // common.Accessory.Config
  sendWithAck(ackCh, body, sizeof body);
  sentAccessory = settings.accessory;
}

static void sendPcmd(const Pcmd& p) {
  uint8_t flag = (p.roll || p.pitch) ? 1 : 0;  // 0 = the drone ignores roll/pitch and holds position
  uint32_t t = millis() - linkUpMs;  // timestamp: ms since the connection (minidrone.xml)
  const uint8_t body[] = {PRJ_MINIDRONE, 0, CMD_PCMD, 0, flag, (uint8_t)p.roll, (uint8_t)p.pitch,
                          (uint8_t)p.yaw, (uint8_t)p.gaz, (uint8_t)t, (uint8_t)(t >> 8),
                          (uint8_t)(t >> 16), (uint8_t)(t >> 24)};
  send(chNoAck, seqNoAck, T_DATA, body, sizeof body);
  lastPcmdMs = millis();
}

const char* droneStateName() {
  static const char* const names[] = {"landed",    "taking off", "hovering", "flying",
                                      "landing",   "emergency",  "rolling",  "init"};
  return flyingState >= 0 && flyingState < 8 ? names[flyingState] : "unknown";
}

// ---------- Drone events (NimBLE task) ----------
// Each event id is logged once per connection with its raw arguments. This shows what this drone
// firmware really sends (for example NavigationDataState, which older firmware does not have).
static uint32_t seenEvents[64];
static uint8_t seenCount = 0;

static void logFirstSeen(const uint8_t* d, size_t n) {
  uint32_t id = d[2] << 16 | d[3] << 8 | d[4];
  for (uint8_t i = 0; i < seenCount; i++)
    if (seenEvents[i] == id) return;
  if (seenCount < 64) seenEvents[seenCount++] = id;
  char hex[3 * 16 + 1] = "";
  for (size_t i = 6; i < n && i < 22; i++) sprintf(hex + 3 * (i - 6), " %02x", d[i]);
  logf("event %u.%u.%u:%s\n", d[2], d[3], d[4], hex);
}

static void onNotify(NimBLERemoteCharacteristic* c, uint8_t* d, size_t n, bool) {
  if (ftpOwns(c)) { ftpOnNotify(d, n); return; }
  const NimBLEUUID& u = c->getUUID();
  if (n >= 3 && d[0] == T_ACK) {
    if (u == uuidAckCmd) pushEvent(EV_ACK_CMD, d[2]);
    else if (u == uuidAckPrio) pushEvent(EV_ACK_PRIO, d[2]);
    return;
  }
  if (n >= 2 && d[0] == T_DATA_ACK && u == uuidAckData) pushEvent(EV_DRONE_NEEDS_ACK, d[1]);
  if (n < 6) return;
  logFirstSeen(d, n);
  for (auto& l : limits)
    if (d[2] == PRJ_MINIDRONE && d[3] == l.evClass && d[4] == l.evCmd && n >= (l.onOff ? 7u : 18u)) {
      if (l.onOff) {
        l.current = d[6];
        l.max = 1;
      } else {
        memcpy(&l.current, d + 6, 4);
        memcpy(&l.max, d + 14, 4);
      }
      l.fresh = true;
    }
  // minidrone.PilotingState: FlatTrimChanged (0), FlyingStateChanged (1), AlertStateChanged (2).
  // Enums are 4 bytes, from d[6].
  if (d[2] == PRJ_MINIDRONE && d[3] == 3 && d[4] == 0) trimEvent = true;
  if (n >= 7 && d[2] == PRJ_MINIDRONE && d[3] == 3 && d[4] == 1 && d[6] != flyingState) {
    flyingState = d[6];
    logf("drone state: %s\n", droneStateName());
  }
  if (n >= 7 && d[2] == PRJ_MINIDRONE && d[3] == 3 && d[4] == 2 && d[6] != alertState) {
    alertState = d[6];
    logf("drone alert: %s\n", d[6] <= ALERT_LOW_BATTERY ? ALERT_NAMES[d[6]] : "unknown");
  }
  // minidrone.FloodControlState.FloodControlChanged: the delay the drone wants between two PCMDs.
  // Deprecated in the XML, so newer firmware may not send it.
  if (n >= 8 && d[2] == PRJ_MINIDRONE && d[3] == 12 && d[4] == 0)
    logf("drone PCMD delay: %u ms (we send every %u ms)\n", d[6] | (d[7] << 8), (unsigned)PCMD_PERIOD_MS);
  // common.AccessoryState.AccessoryConfigChanged: accessory enum at d[6], error enum at d[10].
  if (n >= 14 && d[2] == PRJ_COMMON && d[3] == 27 && d[4] == 1) {
    static const char* const names[] = {"unconfigured", "none",      "wheels",     "truck wheels",
                                        "hull",         "hydrofoil", "in progress"};
    static const char* const errors[] = {"ok", "unknown", "flying"};
    logf("accessory: %s (%s)\n", d[6] <= 6 ? names[d[6]] : "?", d[10] <= 2 ? errors[d[10]] : "?");
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
  void onConnectFail(NimBLEClient*, int reason) override {
    logf("connection not established: %s (0x%x)\n", disconnectText(reason), reason);
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
  if (!client->discoverAttributes()) {
    logf("service discovery failed: error %d, connected %d, services %u\n", client->getLastError(),
         client->isConnected(), (unsigned)client->getServices(false).size());
    return false;
  }
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

  // The drone keeps the last sequence number of each buffer until it powers off, and drops a packet
  // 0..10 behind it while still sending the ACK (libARNetwork ARNETWORK_IOBuffer_ShouldAcceptData).
  // After an ESP32 restart our counters start over and can fall in that window. Fix: one packet,
  // then a jump of 11; whether the drone took the first or dropped it, the next one is new.
  // Command buffer: common.Common.CurrentDate is the harmless first packet.
  // The emergency buffer gets no probe: anything sent there can trigger a command (a CurrentDate
  // probe made the drone take off). droneCommand sends the emergency twice instead.
  seenCount = 0;
  const uint8_t date[] = {PRJ_COMMON, 4, 1, 0, '2', '0', '1', '4', '-', '1', '0', '-', '2', '8', 0};
  sendWithAck(ackCh, date, sizeof date);
  ackCh.seq += 10;
  // common.Common.AllStates: without it the drone does not report the battery
  const uint8_t allStates[] = {PRJ_COMMON, 4, 0, 0};
  sendWithAck(ackCh, allStates, sizeof allStates);
  // common.Accessory.Config: the drone reported UNCONFIGURED even after the app set the hull,
  // so the firmware declares the accessory on each connection.
  sendAccessory();
  // common.Settings.AllSettings: the drone replies with current, min and max limits
  const uint8_t allSettings[] = {PRJ_COMMON, 2, 0, 0};
  sendWithAck(ackCh, allSettings, sizeof allSettings);

  // Same probe and jump for the PCMD buffer: a neutral PCMD, then the next one is always new.
  linkUpMs = millis();
  lastPcmd = {0, 0, 0, 0};
  sendPcmd(lastPcmd);
  seqNoAck += 10;

  trimWaiting = false;
  autoTrimPending = !autoTrimDone;
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
  // NimBLE retries a 0x3e failure in the background, even after connect() returned true. The
  // client then sits in "connecting" while setupLink runs, and the next connect() is rejected.
  // Without the retry, the failure is reported and droneConnect tries again from a clean state.
  client->setConnectRetries(0);
}

// A link that dies mid-setup (drone battery pulled) can leave the NimBLE client in a
// "not disconnected" state. From then on every connect() fails at once with BLE_HS_EREJECT,
// and the library never recovers. A restart is the only reliable reset; it runs only while
// the drone is not linked.
static bool connectTo(const NimBLEAddress& a) {
  if (client->connect(a)) return true;
  int rc = client->getLastError();
  logf("connect failed (%d)\n", rc);
  if (rc == BLE_HS_EREJECT) {
    logf("BLE client stuck: restarting the ESP32\n");
    logFlush();
    delay(100);
    esp_restart();
  }
  return false;
}

bool droneConnect() {
  bool connected = !lastAddress.isNull() && connectTo(lastAddress);
  if (!connected) connected = scanForDrone() && connectTo(lastAddress);
  if (!connected) return false;
  if (!setupLink()) {
    logf("link setup failed\n");
    client->disconnect();
    return false;
  }
  linkQuality = 100;
  rssi = 0;
  linkUp = true;
  logf("connected\n");
  dronePrintInfo();
  return true;
}

bool droneLinkUp() { return linkUp; }

uint32_t dronePcmdDueMs() {
  uint32_t elapsed = millis() - lastPcmdMs;
  return elapsed >= PCMD_PERIOD_MS ? 0 : PCMD_PERIOD_MS - elapsed;
}
int droneBattery() { return battery; }

void droneCommand(DroneCmd cmd) {
  if (!linkUp) return;
  // With a critical battery, the motors pull the voltage down and the drone reboots mid take-off.
  if (cmd == DroneCmd::TakeOff && alertState == ALERT_CRITICAL_BATTERY) {
    logf("take-off blocked: the drone reports a critical battery\n");
    commandFailed = true;
    return;
  }
  // The throttle stick has no spring: up = full climb right after take-off.
  if (cmd == DroneCmd::TakeOff && lastPcmd.gaz != 0) {
    logf("take-off blocked: center the throttle stick\n");
    commandFailed = true;
    return;
  }
  const uint8_t body[] = {PRJ_MINIDRONE, 0, (uint8_t)cmd, 0};  // class Piloting = 0
  if (cmd == DroneCmd::FlatTrim) {
    trimEvent = false;
    trimWaiting = true;
    trimSentMs = millis();
  }
  if (cmd == DroneCmd::Emergency) {
    // Two copies 11 apart: at least one is outside the drone's duplicate window (see setupLink).
    sendWithAck(prioCh, body, sizeof body);
    prioCh.seq += 10;
    sendWithAck(prioCh, body, sizeof body);
  } else {
    sendWithAck(ackCh, body, sizeof body);
  }
  logf("cmd %u\n", (unsigned)cmd);
}

void dronePcmd(const Pcmd& p) {
  lastPcmd = p;  // if it is too early, the keepalive or the next frame sends this value
  if (linkUp && millis() - lastPcmdMs >= PCMD_PERIOD_MS) sendPcmd(p);
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
  static uint32_t lastRssiMs = 0;
  if (millis() - lastRssiMs >= 1000) {  // for the CRSF telemetry; a local controller read
    lastRssiMs = millis();
    rssi = client->getRssi();
  }
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
  logf("drone state: %s, battery %d%%, alert %s\n", droneStateName(), battery,
       alertState <= ALERT_LOW_BATTERY ? ALERT_NAMES[alertState] : "unknown");
}

int droneRssi() { return linkUp ? rssi : 0; }

int droneLinkQuality() {
  if (!linkUp) return 0;
  return linkQuality > 1 ? linkQuality : 1;  // 0 would tell the radio that the link is lost
}

void droneApplySettings() {
  if (!linkUp) return;  // the next connection sends them
  for (auto& l : limits)
    if (l.max > 0) l.fresh = true;  // serviceLimits writes the ones that differ
  if (settings.accessory != sentAccessory) sendAccessory();  // the drone refuses it in flight
}

bool droneLanded() { return flyingState == 0; }

void droneDisconnect() {
  if (client && client->isConnected()) client->disconnect();
}
