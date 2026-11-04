// CRSF from the radio's external module port, full duplex: channels in, telemetry out.
#include "config.h"
#if INPUT_CRSF
#include <Arduino.h>

#include "crsf_decoder.h"
#include "crsf_menu.h"
#include "drone.h"
#include "log.h"
#include "ota.h"
#include "rc.h"

static HardwareSerial uart(1);
static CrsfDecoder decoder;
static uint16_t shared[RC_CHANNELS];
static uint32_t seq = 0, lastFrameMs = 0;
static volatile uint32_t rawBytes = 0;
static uint8_t rawRing[16];  // last raw bytes, for the bench monitor
static SemaphoreHandle_t frameReady = nullptr;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Sensors on the radio: 1RSS (BLE RSSI), RQly (link quality), Bat% and FM (drone state).
// 0xFF marks a field as "no data": the radio creates no sensor for it.
// RQly 0 tells the radio that the telemetry is lost, so it warns while the drone is not connected.
static void sendTelemetry() {
  uint8_t f[64];
  bool up = droneLinkUp();

  uint8_t link[10];  // LINK_STATISTICS 0x14
  memset(link, 0xFF, sizeof link);
  int rssi = droneRssi();
  if (rssi < 0) link[0] = (uint8_t)(int8_t)rssi;  // uplink RSSI 1, dBm
  link[2] = droneLinkQuality();                    // uplink link quality, %
  uart.write(f, crsfFrame(f, 0x14, link, sizeof link));

  int battery = droneBattery();
  if (up && battery >= 0) {
    uint8_t bat[8];  // BATTERY 0x08: voltage, current, capacity unknown; remaining %
    memset(bat, 0xFF, sizeof bat);
    bat[7] = battery;
    uart.write(f, crsfFrame(f, 0x08, bat, sizeof bat));
  }

  // FLIGHT_MODE 0x21, max 13 characters
  const char* state = otaActive() ? "wifi update" : up ? droneStateName() : "no drone";
  uart.write(f, crsfFrame(f, 0x21, (const uint8_t*)state, strlen(state) + 1));
}

// Answer to the radio's device ping. OpenI6X 2.3.0 sends pings instead of channels until a module at
// 0xEE answers (pulses/crossfire.cpp, queryCompleted). The ExpressLRS tool uses the same answer.
// Payload: destination, origin, name, serial, hardware version, software version (0, major, minor,
// revision), parameter count, parameter protocol version.
static void sendDeviceInfo() {
  static const char name[] = "rc2parrot";
  uint8_t p[2 + sizeof name + 14] = {0xEA, 0xEE};  // to the radio, from the module address
  memcpy(p + 2, name, sizeof name);
  uint8_t* q = p + 2 + sizeof name;
  memcpy(q, "RC2P", 4);  // serial; "ELRS" would turn on ELRS-only behaviour in the radio
  q[4] = q[5] = q[6] = q[7] = 0;                 // hardware version
  q[8] = 0, q[9] = 1, q[10] = 0, q[11] = 0;      // software version 1.0.0
  q[12] = kMenuCount;                            // parameter count
  q[13] = 0;                                     // parameter protocol version
  uint8_t f[64];
  uart.write(f, crsfFrame(f, 0x29, p, sizeof p));
}

// Parameter menu: answers PARAMETER_READ (0x2C) and applies PARAMETER_WRITE (0x2D).
// Radio frame: [0xEE, length, type, destination, origin, parameter id, chunk or value, CRC].
static void onMenuFrame(const uint8_t* fr) {
  if (fr[3] != 0xEE) return;  // for another device
  if (fr[2] == 0x2C) {
    uint8_t p[64] = {0xEA, 0xEE};  // to the radio, from the module address
    int n = menuEntry(fr[5], p + 2);
    if (!n) return;
    uint8_t f[72];
    uart.write(f, crsfFrame(f, 0x2B, p, 2 + n));
  } else if (fr[2] == 0x2D && menuWrite(fr[5], fr[6])) {
    logf("menu: %s = %u\n", menuParams[fr[5] - 1].name, fr[6]);
  }
}

// Runs in the UART event task when the line goes idle after a frame (RX timeout).
// Telemetry goes out from here too: the loop blocks for seconds during the drone scan.
static void onData() {
  bool got = false;
  while (uart.available()) {
    uint8_t b = uart.read();
    rawRing[rawBytes++ % sizeof rawRing] = b;
    uint8_t type = decoder.byte(b);
    if (type == 0x28) sendDeviceInfo();
    if (type == 0x2C || type == 0x2D) onMenuFrame(decoder.frame());
    if (type != 0x16) continue;
    portENTER_CRITICAL(&mux);
    for (int i = 0; i < RC_CHANNELS; i++) shared[i] = decoder.channels()[i];
    seq++;
    lastFrameMs = millis();
    portEXIT_CRITICAL(&mux);
    got = true;
  }
  if (!got) return;
  xSemaphoreGive(frameReady);  // wake the loop now so the PCMD uses this frame

  static uint32_t lastTelemetryMs = 0;
  if (CRSF_TELEMETRY_MS && millis() - lastTelemetryMs >= CRSF_TELEMETRY_MS) {
    lastTelemetryMs = millis();
    sendTelemetry();
  }
}

void rcBegin() {
  frameReady = xSemaphoreCreateBinary();
  uart.begin(CRSF_BAUD, SERIAL_8N1, CRSF_PIN, CRSF_TX_PIN, CRSF_INVERTED);
  uart.onReceive(onData, false);
}

void rcWait(uint32_t timeoutMs) { xSemaphoreTake(frameReady, pdMS_TO_TICKS(timeoutMs)); }

RcFrame rcLatest() {
  RcFrame f;
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < RC_CHANNELS; i++) f.ch[i] = shared[i];
  f.seq = seq;
  uint32_t t = lastFrameMs;
  portEXIT_CRITICAL(&mux);
  f.ageMs = f.seq ? millis() - t : UINT32_MAX;
  return f;
}
uint32_t rcRawCount() { return rawBytes; }

int rcRawDump(uint8_t* out) {
  uint32_t n = rawBytes;
  int count = n < sizeof rawRing ? n : sizeof rawRing;
  for (int i = 0; i < count; i++) out[i] = rawRing[(n - count + i) % sizeof rawRing];
  return count;
}
#endif
