// Pins, channel map and tuning values.
#pragma once
#include <stdint.h>

// 1 = keep the serial console on while PPM is active. For bench setup only: the serial port adds
// some latency. The esp32c3_console environment in platformio.ini sets this.
#ifndef FORCE_CONSOLE
#define FORCE_CONSOLE 0
#endif

// ---------- Pins (ESP32-C3) ----------
// Avoid GPIO 2, 8 and 9: strapping pins.
constexpr int PPM_PIN = 4;  // PPM from the radio trainer port
constexpr int LED_PIN = 5;  // RGBW NeoPixel (SK6812)

// ---------- Radio channels (index 0 = CH1) ----------
constexpr int CH_ROLL = 0, CH_PITCH = 1, CH_GAZ = 2, CH_YAW = 3;  // AETR
constexpr int CH_ARM = 4;   // switch: high = take off, low = land
constexpr int CH_KILL = 5;  // switch: high = cut motors
constexpr int CH_TRIM = 6;  // switch or button: rising edge = flat trim (landed only)

// ---------- Sticks and switches ----------
constexpr int SWITCH_US = 1700;   // switch is "high" above this pulse width
constexpr int DEADBAND = 5;       // stick center deadband, in %
constexpr int DEADBAND_GAZ = 12;  // the throttle stick has no spring, so it gets a wider center
// No rate limits here: the drone runs at its maximum and the radio limits (dual rates, expo).

// ---------- Timing ----------
constexpr uint32_t PPM_TIMEOUT_MS = 300;  // no PPM for this long = failsafe landing
// With PPM, a PCMD goes out on every frame (~22.5 ms). Without a new frame, it repeats at this period.
// Parrot SDK (libARDiscovery, BLE network config): 50 ms loop, at least 20 ms between sends.
constexpr uint32_t PCMD_PERIOD_MS = 50;
constexpr uint32_t PCMD_MIN_INTERVAL_MS = 20;
constexpr uint32_t TRIM_TIMEOUT_MS = 2000;    // no FlatTrimChanged within this time = trim failed
constexpr uint32_t AUTO_TRIM_WAIT_MS = 3000;  // time for the drone to report that it is landed

// ---------- Console ----------
constexpr uint32_t CONSOLE_DISCARD_MS = 500;  // input discarded after the serial port turns back on
constexpr uint32_t PPM_MONITOR_MS = 200;      // PPM monitor (key p) print period

// ---------- BLE ----------
constexpr int8_t BLE_TX_DBM = 21;          // ESP32-C3 maximum
constexpr uint16_t BLE_INTERVAL_MIN = 6;   // x 1.25 ms = 7.5 ms
constexpr uint16_t BLE_INTERVAL_MAX = 12;  // x 1.25 ms = 15 ms; the drone picks within the range
constexpr uint16_t BLE_SUPERVISION = 200;  // x 10 ms = 2 s without packets = disconnected
constexpr uint32_t BLE_SCAN_MS = 5000;
constexpr uint32_t BLE_CONNECT_TIMEOUT_MS = 3000;

// ---------- LED ----------
constexpr int LED_BRIGHTNESS = 40;         // 0..255
constexpr int BATTERY_LOW_PCT = 20;        // at or below: solid red
constexpr int LED_ALERT_BRIGHTNESS = 255;  // low battery red
constexpr uint32_t LED_FLASH_MS = 150;     // trim/failure flash on and off time
