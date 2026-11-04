// Firmware update over WiFi, switched on from the radio menu ("WiFi update"). Off at boot.
// While on: the ESP32 runs a WiFi access point and an ArduinoOTA server, and does not look for the
// drone. Upload with the *_ota environment in platformio.ini after joining the access point.
#pragma once

void otaService();  // loop task: follows the menu switch and serves the update
bool otaActive();   // safe from any task
