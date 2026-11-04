#include "ota.h"

#include <ArduinoOTA.h>
#include <WiFi.h>

#include "config.h"
#include "drone.h"
#include "log.h"
#include "settings.h"

static volatile bool active = false;

bool otaActive() { return active; }

void otaService() {
  bool want = wifiUpdateRequest;
  if (want && !active) {
    if (droneLinkUp() && !droneLanded()) {
      logf("wifi update refused: the drone is not landed\n");
      wifiUpdateRequest = 0;
      return;
    }
    droneDisconnect();
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
    ArduinoOTA.setPassword(WIFI_AP_PASSWORD);
    ArduinoOTA.begin();
    active = true;
    logf("wifi update on: join \"%s\" (password \"%s\"), then upload to %s\n", WIFI_AP_SSID, WIFI_AP_PASSWORD,
         WiFi.softAPIP().toString().c_str());
  } else if (!want && active) {
    ArduinoOTA.end();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    active = false;
    logf("wifi update off\n");
  }
  if (active) ArduinoOTA.handle();  // blocks during an upload, then reboots into the new firmware
}
