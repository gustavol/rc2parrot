#include "console.h"

#include <Arduino.h>

#include "config.h"
#include "drone.h"
#include "log.h"
#include "rc.h"

static String line;  // "d" command: collects the path until Enter without blocking the loop
static uint32_t discardUntil = 0;
static bool monitor = NO_DRONE;
static uint32_t monitorSeq = 0;

static void printHelp() {
  logf("t take off | l land | k cut motors | r flat trim | f picture\n"
       "p toggle radio monitor | i BLE link info | d<path><Enter> list files | h help\n");
}

// One line per frame, rewritten in place with \r.
static void printRc(const RcFrame& f) {
  const uint16_t* c = f.ch;
  logf("\rCH1=%4u CH2=%4u CH3=%4u CH4=%4u CH5=%4u CH6=%4u CH7=%4u CH8=%4u", c[0], c[1], c[2], c[3],
       c[4], c[5], c[6], c[7]);
}

// A key typed during flight can sit in the USB buffer; it must not run after the radio drops.
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
        logf(monitor ? "radio monitor on, drone scan paused (p = off)\n" : "\nradio monitor off\n");
        if (monitor && !rcLatest().seq) logf("no radio frame yet\n");
        break;
      case 'i': dronePrintInfo(); break;
      case 'h': printHelp(); break;
    }
  }
  if (monitor) {
    RcFrame f = rcLatest();
    static uint32_t lastFrameMs = 0, lastRawMs = 0, lastRaw = 0;
    if (f.seq != monitorSeq) {
      monitorSeq = f.seq;
      lastFrameMs = millis();
      printRc(f);
    } else if (millis() - lastFrameMs >= 1000 && millis() - lastRawMs >= 1000) {
      // No valid frame: show whether anything arrives at all (wiring vs. inversion or baud rate).
      uint32_t raw = rcRawCount();
      logf("no valid radio frame; raw input %lu/s (%s)\n", (unsigned long)(raw - lastRaw),
           INPUT_CRSF ? "CRSF bytes" : "PPM edges");
      uint8_t d[16];
      int n = rcRawDump(d);
      if (n) {
        logf("  last bytes:");
        for (int i = 0; i < n; i++) logf(" %02x", d[i]);
        logf("\n");
      }
      lastRaw = raw;
      lastRawMs = millis();
    }
  }
}

bool consoleMonitorOn() { return monitor; }
