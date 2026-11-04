#include "settings.h"

#include <Preferences.h>

#include "log.h"

static Preferences prefs;

void settingsBegin() {
  prefs.begin("rc2parrot", false);
  Settings s;
  if (prefs.getBytes("settings", &s, sizeof s) == sizeof s && s.version == SETTINGS_VERSION) settings = s;
  logf("settings: tilt %u deg, altitude %u dm, vertical %u cm/s, rotation %u deg/s, banked %u, cut-out %u, "
       "accessory %u\n",
       settings.maxTilt, settings.maxAltitude, settings.maxVertSpeed, settings.maxRotSpeed, settings.bankedTurn,
       settings.cutOut, settings.accessory);
}

void settingsSave() { prefs.putBytes("settings", &settings, sizeof settings); }
