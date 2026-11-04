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
constexpr int SWITCH_US = 1250;   // switch is "high" above this; the radio switches give 1000 (off) and 1500 (on)
constexpr int DEADBAND = 5;       // stick center deadband, in %
constexpr int DEADBAND_GAZ = 12;  // the throttle stick has no spring, so it gets a wider center
// Throttle and yaw command a speed: those drone limits stay at maximum and the radio limits them.
// Roll and pitch command a tilt angle, which is acceleration: the speed keeps growing while the
// stick is off center, so the drone itself must cap the angle. 0 = drone maximum.
constexpr float MAX_TILT_DEG = 10;
// Drone settings written on each connection. They stay in the drone until it changes them again.
constexpr float MAX_ALTITUDE_M = 2.5;  // 2..25 m; the drone stops climbing here
constexpr float BANKED_TURN = 1;       // 1 = yaw also banks the drone when it moves, like a plane (app default)
constexpr uint8_t ACCESSORY = 3;  // 0 none, 1 wheels, 2 truck wheels, 3 hull (propeller guard), 4 hydrofoil
constexpr float CUT_OUT_MODE = 1;      // 1 = motors stop on a hard impact or when the drone turns over

// ---------- Timing ----------
constexpr uint32_t PPM_TIMEOUT_MS = 300;  // no PPM for this long = failsafe landing
// One PCMD per period, with the newest radio value. The Parrot app sends at 50 ms. Sending on every
// PPM frame (~44 Hz) made the drone lag behind the sticks.
constexpr uint32_t PCMD_PERIOD_MS = 50;
constexpr uint32_t TRIM_TIMEOUT_MS = 2000;    // no FlatTrimChanged within this time = trim failed
constexpr uint32_t AUTO_TRIM_WAIT_MS = 3000;  // time for the drone to report that it is landed

// ---------- Console ----------
constexpr uint32_t CONSOLE_DISCARD_MS = 500;  // input discarded after the serial port turns back on

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
