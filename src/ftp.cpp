#include "ftp.h"

#include <Arduino.h>
#include <NimBLEDevice.h>

#include "log.h"

static NimBLERemoteCharacteristic *gettingChr = nullptr, *handlingChr = nullptr;
static bool eofSeen = false;

void ftpAttach(NimBLERemoteCharacteristic* getting, NimBLERemoteCharacteristic* handling) {
  gettingChr = getting;
  handlingChr = handling;
}

bool ftpOwns(const NimBLERemoteCharacteristic* c) { return c && c == gettingChr; }

// Reply: listing text, then "End of Transfer\0", then a 32-char hex MD5 (not verified).
void ftpOnNotify(const uint8_t* d, size_t n) {
  if (eofSeen) {
    eofSeen = false;
    logf("--- end of listing\n");
  } else if (n >= 15 && !memcmp(d, "End of Transfer", 15)) {
    eofSeen = true;
  } else {
    logWrite(d, n);
  }
}

bool ftpList(const char* path) {
  if (!gettingChr || !handlingChr) return false;
  std::string cmd = std::string("LIS") + (path[0] == '/' ? "" : "/") + path;
  eofSeen = false;
  logf("--- listing %s\n", cmd.c_str() + 3);
  return handlingChr->writeValue((const uint8_t*)cmd.c_str(), cmd.size() + 1, false);  // includes the \0
}
