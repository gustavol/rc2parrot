#include "console.h"

#include <Arduino.h>

#include "config.h"
#include "drone.h"
#include "log.h"
#include "ppm.h"

static String line;  // "d" command: collects the path until Enter without blocking the loop
static uint32_t discardUntil = 0;
static bool monitor = false;
static uint32_t monitorSeq = 0;

static void printHelp() {
  logf("t take off | l land | k cut motors | r flat trim | f picture\n"
       "p toggle PPM monitor | i BLE link info | d<path><Enter> list files | h help\n");
}

// One line per frame, rewritten in place with \r.
static void printPpm(const PpmFrame& f) {
  const uint16_t* c = f.ch;
  logf("\rCH1=%4u CH2=%4u CH3=%4u CH4=%4u CH5=%4u CH6=%4u CH7=%4u CH8=%4u", c[0], c[1], c[2], c[3],
       c[4], c[5], c[6], c[7]);
}

// A key typed during flight can sit in the USB buffer; it must not run after PPM drops.
void consoleReset() {
  line = "";
  monitor = false;
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
        logf(monitor ? "PPM monitor on, drone scan paused (p = off)\n" : "\nPPM monitor off\n");
        if (monitor && !ppmLatest().seq) logf("no PPM frame yet\n");
        break;
      case 'i': dronePrintInfo(); break;
      case 'h': printHelp(); break;
    }
  }
  if (monitor) {
    PpmFrame f = ppmLatest();
    if (f.seq != monitorSeq) {
      monitorSeq = f.seq;
      printPpm(f);
    }
  }
}

bool consoleMonitorOn() { return monitor; }
