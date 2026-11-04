// Drone settings the pilot can change from the radio menu (CRSF builds). Saved in the ESP32 flash
// (NVS, namespace "rc2parrot"), so they survive a reboot and a firmware upload; `pio run -t erase`
// clears them. The firmware writes them to the drone on every connection.
#pragma once
#include <stdint.h>

#include "config.h"

struct Settings {
  uint8_t version;
  uint8_t maxTilt;          // deg, 5..25
  uint8_t maxAltitude;      // dm, 20..250 (the radio menu shows integers only)
  uint8_t maxVertSpeed;     // cm/s, 50..200
  uint8_t maxRotSpeed;      // deg/s, 50..180
  uint8_t bankedTurn;       // 0 off, 1 on
  uint8_t cutOut;           // 0 off, 1 on
  uint8_t accessory;        // common.Accessory.Config: 0 none, 1 wheels, 2 truck wheels, 3 hull, 4 hydrofoil
};
constexpr uint8_t SETTINGS_VERSION = 2;  // change it when the struct changes: old data is then ignored

inline Settings settings = {SETTINGS_VERSION,  DEFAULT_MAX_TILT_DEG, DEFAULT_MAX_ALTITUDE_DM,
                            DEFAULT_MAX_VERT_SPEED_CMS, DEFAULT_MAX_ROT_SPEED_DPS,
                            DEFAULT_BANKED_TURN, DEFAULT_CUT_OUT, DEFAULT_ACCESSORY};

// Radio menu "WiFi update" (0 off, 1 on). Not saved: off after every reboot. See ota.h.
inline uint8_t wifiUpdateRequest = 0;

// Set by the radio menu (UART task). The loop saves the settings and sends them to the drone.
inline volatile bool settingsChanged = false;

void settingsBegin();  // loads the saved settings; keeps the defaults if there are none
void settingsSave();
