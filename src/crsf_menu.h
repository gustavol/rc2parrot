// Parameter menu shown by the radio in Tools > ExpressLRS (CRSF parameter protocol).
// Host-testable: test/test_crsf.cpp checks the entry layout.
//
// The radio reads parameter N with PARAMETER_READ (0x2C) and shows the PARAMETER_SETTINGS_ENTRY
// (0x2B) answer. A change comes as PARAMETER_WRITE (0x2D); the radio then reads all parameters
// again, so a write needs no answer. Layout checked against OpenI6X 2.3.0 tools/elrs.cpp.
#pragma once
#include <stdint.h>
#include <string.h>

#include "settings.h"

enum : uint8_t { PARAM_UINT8 = 0, PARAM_SELECT = 9, PARAM_INFO = 12 };

struct MenuParam {
  const char* name;
  uint8_t type;
  const char* text;  // UINT8: unit; SELECT: options "A;B;C"; INFO: the text shown
  uint8_t min, max;
  uint8_t* value;    // settings field: UINT8 value or SELECT option index; nullptr for INFO
};

inline MenuParam menuParams[] = {
    {"Max tilt", PARAM_UINT8, "deg", 5, 25, &settings.maxTilt},
    {"Max alt", PARAM_UINT8, "dm", 20, 250, &settings.maxAltitude},
    {"Vert speed", PARAM_UINT8, "cm/s", 50, 200, &settings.maxVertSpeed},
    {"Yaw speed", PARAM_UINT8, "deg/s", 50, 180, &settings.maxRotSpeed},
    {"Banked turn", PARAM_SELECT, "Off;On", 0, 1, &settings.bankedTurn},
    {"Cut-out", PARAM_SELECT, "Off;On", 0, 1, &settings.cutOut},
    {"Accessory", PARAM_SELECT, "None;Wheels;Truck;Hull;Hydrofoil", 0, 4, &settings.accessory},
    {"WiFi update", PARAM_SELECT, "Off;On", 0, 1, &wifiUpdateRequest},
    {"Firmware", PARAM_INFO, "rc2parrot", 0, 0, nullptr},
};
constexpr uint8_t kMenuCount = sizeof(menuParams) / sizeof(menuParams[0]);

inline int menuPutString(uint8_t* p, const char* s) {
  int n = strlen(s) + 1;
  memcpy(p, s, n);
  return n;
}

// Entry payload after destination and origin: [id, chunks remaining, parent folder, type, name\0,
// type data]. UINT8: value, min, max, default, unit\0. SELECT: options\0, value, min, max, default,
// unit\0. INFO: text\0. Returns the size; 0 for an unknown id (ids start at 1).
inline int menuEntry(uint8_t id, uint8_t* p) {
  if (id < 1 || id > kMenuCount) return 0;
  const MenuParam& m = menuParams[id - 1];
  int n = 0;
  p[n++] = id;
  p[n++] = 0;  // one chunk
  p[n++] = 0;  // root folder
  p[n++] = m.type;
  n += menuPutString(p + n, m.name);
  if (m.type == PARAM_INFO) return n + menuPutString(p + n, m.text);
  if (m.type == PARAM_SELECT) n += menuPutString(p + n, m.text);
  p[n++] = *m.value;
  p[n++] = m.min;
  p[n++] = m.max;
  p[n++] = *m.value;  // default
  return n + menuPutString(p + n, m.type == PARAM_UINT8 ? m.text : "");
}

// Applies a PARAMETER_WRITE and flags the settings for saving. Returns false for an unknown or
// read-only id or a value out of range.
inline bool menuWrite(uint8_t id, uint8_t value) {
  if (id < 1 || id > kMenuCount) return false;
  MenuParam& m = menuParams[id - 1];
  if (m.type == PARAM_INFO || value < m.min || value > m.max) return false;
  *m.value = value;
  settingsChanged = true;
  return true;
}
