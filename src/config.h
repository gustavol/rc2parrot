// Pins, channel map and tuning values.
#pragma once
#include <stdint.h>

// 1 = keep the serial console on while the radio is active. For bench setup only: the serial port adds
// some latency. The esp32c3_console environment in platformio.ini sets this.
#ifndef FORCE_CONSOLE
#define FORCE_CONSOLE 0
#endif

// 1 = radio link test: no drone scan, radio monitor on from boot. The esp32c3_crsf_test environment
// sets this.
#ifndef NO_DRONE
#define NO_DRONE 0
#endif

// ---------- Radio input ----------
// 0 = PPM from the trainer jack. 1 = CRSF full duplex on the external module port: channels in,
// telemetry out. OpenI6X: TX2 pad -> CRSF_PIN, PA15 pad <- CRSF_TX_PIN; needs a build with
// CRSF_FULLDUPLEX=YES (2.3.0) or the "CRSF Duplex" option (later versions).
// The esp32c3_crsf environments in platformio.ini set this.
#ifndef INPUT_CRSF
#define INPUT_CRSF 0
#endif
constexpr uint32_t CRSF_BAUD = 400000;  // OpenI6X: Radio Settings > Hardware > Baudrate (400k = 4 ms frames)
constexpr bool CRSF_INVERTED = false;   // full duplex is not inverted; the one-wire half-duplex mode is
constexpr uint32_t CRSF_TELEMETRY_MS = 200;  // battery, RSSI, link quality and drone state to the radio; 0 = off

// ---------- Pins (ESP32-C3) ----------
// Avoid GPIO 2, 8 and 9: strapping pins.
constexpr int PPM_PIN = 4;   // PPM from the radio trainer port
constexpr int CRSF_PIN = 4;     // CRSF from the radio TX2 pad (UART1 RX)
constexpr int CRSF_TX_PIN = 3;  // CRSF telemetry to the radio PA15 pad (UART1 TX)
constexpr int LED_PIN = 5;  // RGBW NeoPixel (SK6812)
// 0 = build without the status LED: no LED task, GPIO5 stays free. The *_noled environments set it.
#ifndef USE_LED
#define USE_LED 1
#endif

// ---------- Radio channels (index 0 = CH1) ----------
constexpr int CH_ROLL = 0, CH_PITCH = 1, CH_GAZ = 2, CH_YAW = 3;  // AETR
constexpr int CH_ARM = 4;   // switch: high = take off, low = land
constexpr int CH_KILL = 5;  // switch: high = cut motors
constexpr int CH_TRIM = 6;  // switch or button: rising edge = flat trim (landed only)

// ---------- Sticks and switches ----------
constexpr int SWITCH_US = 1250;   // switch is "high" above this; the radio switches give 1000 (off) and 1500 (on)
constexpr int DEADBAND = 5;       // stick center deadband, in %
constexpr int DEADBAND_GAZ = 12;  // the throttle stick has no spring, so it gets a wider center
// ---------- Drone settings (defaults) ----------
// Written to the drone on every connection. CRSF builds can change them from the radio menu and
// save them in flash (settings.h); these values are used until then.
// Roll and pitch command a tilt angle, which is acceleration: the speed keeps growing while the
// stick is off center, so the drone itself must cap the angle. Throttle and yaw command a speed:
// those limits stay at the drone maximum and the radio limits them (rates, expo).
constexpr uint8_t DEFAULT_MAX_TILT_DEG = 10;         // 5..25
constexpr uint8_t DEFAULT_MAX_ALTITUDE_DM = 25;      // 20..250 dm (2..25 m); the drone stops climbing here
constexpr uint8_t DEFAULT_MAX_VERT_SPEED_CMS = 200;  // 50..200 (0.5..2 m/s)
constexpr uint8_t DEFAULT_MAX_ROT_SPEED_DPS = 180;   // 50..180
constexpr uint8_t DEFAULT_BANKED_TURN = 1;           // 1 = yaw also banks the drone when it moves (app default)
constexpr uint8_t DEFAULT_CUT_OUT = 1;               // 1 = motors stop on a hard impact or when the drone turns over
constexpr uint8_t DEFAULT_ACCESSORY = 3;             // 0 none, 1 wheels, 2 truck wheels, 3 hull (propeller guard), 4 hydrofoil

// ---------- Timing ----------
constexpr uint32_t RC_TIMEOUT_MS = 300;  // no radio frame for this long = failsafe landing
// No radio frame for this long (radio off, or on another model): stop the drone scan and disconnect
// a landed drone, so the BLE radio stays quiet. Bench builds (FORCE_CONSOLE) keep scanning.
constexpr uint32_t RADIO_IDLE_MS = 10000;
// One PCMD per period, with the newest radio frame. The Parrot app sends at 50 ms. Sending on every
// PPM frame (~44 Hz) made the drone lag behind the sticks.
constexpr uint32_t PCMD_PERIOD_MS = 50;
constexpr uint32_t TRIM_TIMEOUT_MS = 2000;    // no FlatTrimChanged within this time = trim failed
constexpr uint32_t AUTO_TRIM_WAIT_MS = 3000;  // time for the drone to report that it is landed

// ---------- Console ----------
constexpr uint32_t CONSOLE_DISCARD_MS = 500;  // input discarded after the serial port turns back on

// ---------- WiFi update (radio menu "WiFi update") ----------
#define WIFI_AP_SSID "rc2parrot"
#define WIFI_AP_PASSWORD "rc2parrot"  // WiFi and OTA password; at least 8 characters

// ---------- BLE ----------
constexpr int8_t BLE_TX_DBM = 21;          // ESP32-C3 maximum
constexpr uint16_t BLE_INTERVAL_MIN = 6;   // x 1.25 ms = 7.5 ms
// x 1.25 ms. The drone asks for 7.5..12.5 ms; with a range the ESP32 took 12.5. Equal to the
// minimum = fixed 7.5 ms. Set 12 to go back to 7.5..15 ms.
constexpr uint16_t BLE_INTERVAL_MAX = 6;
constexpr uint16_t BLE_SUPERVISION = 200;  // x 10 ms = 2 s without packets = disconnected
constexpr uint32_t BLE_SCAN_MS = 5000;
constexpr uint32_t BLE_CONNECT_TIMEOUT_MS = 3000;

// ---------- LED ----------
constexpr int LED_BRIGHTNESS = 40;         // 0..255
constexpr int BATTERY_LOW_PCT = 20;        // at or below: solid red
constexpr int LED_ALERT_BRIGHTNESS = 255;  // low battery red
constexpr uint32_t LED_FLASH_MS = 150;     // trim/failure flash on and off time
