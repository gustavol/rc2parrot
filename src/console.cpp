#include "console.h"

#include <Arduino.h>

#include "config.h"
#include "drone.h"
#include "log.h"
#include "ppm.h"

static String line;  // "d" command: collects the path until Enter without blocking the loop
static uint32_t discardUntil = 0;
static bool monitor = false;
static uint32_t monitorMs = 0;

static void printHelp() {
  logf("t take off | l land | k cut motors | r flat trim | f picture\n"
       "p toggle PPM monitor | i BLE link info | d<path><Enter> list files | h help\n");
}

static void printPpm() {
  PpmFrame f = ppmLatest();
  if (!f.seq) {
    logf("PPM: no frame received\n");
    return;
  }
  logf("PPM %lu ms ago:", (unsigned long)f.ageMs);
  for (int i = 0; i < PPM_CHANNELS; i++) logf(" CH%d=%u", i + 1, f.ch[i]);
  logf("\n");
}

// A key typed during flight can sit in the USB buffer; it must not run after PPM drops.
void consoleReset() {
  line = "";
  discardUntil = millis() + CONSOLE_DISCARD_MS;
}

void consolePoll() {
  if (!logSerialOn()) return;
  if ((int32_t)(millis() - discardUntil) < 0) {
    while (Serial.available()) Serial.read();
    return;
  }
  while (Serial.available()) {
    char c = Serial.read();
    if (line.length()) {
      if (c != '\r' && c != '\n') {
        line += c;
        continue;
      }
      String path = line.substring(1);
      path.trim();
      line = "";
      if (!droneListFiles(path.length() ? path.c_str() : "/")) logf("listing not available\n");
      continue;
    }
    switch (c) {
      case 'd': line = "d"; break;
      case 't': droneCommand(DroneCmd::TakeOff); break;
      case 'l': droneCommand(DroneCmd::Landing); break;
      case 'k': droneCommand(DroneCmd::Emergency); break;
      case 'r': droneCommand(DroneCmd::FlatTrim); break;
      case 'f': droneTakePicture(); break;
      case 'p':
        monitor = !monitor;
        logf("PPM monitor %s\n", monitor ? "on" : "off");
        break;
      case 'i': dronePrintInfo(); break;
      case 'h': printHelp(); break;
    }
  }
  if (monitor && millis() - monitorMs >= PPM_MONITOR_MS) {
    monitorMs = millis();
    printPpm();
  }
}
