// Parrot FTP over BLE: file listing only.
// Service fd21: fd23 = getting (notify), fd24 = handling (commands).
// This drone speaks the variant from libARUtils ARUTILS_BLEFtp.m (iOS): the command is written
// raw ("LIS/path\0") and each notification is one packet. The Android variant
// (ARUtilsBLEFtp.java, framed blocks) gets "error unknown command".
#pragma once
#include <stddef.h>
#include <stdint.h>

class NimBLERemoteCharacteristic;

void ftpAttach(NimBLERemoteCharacteristic* getting, NimBLERemoteCharacteristic* handling);
bool ftpOwns(const NimBLERemoteCharacteristic* c);  // is this notification for the FTP?
void ftpOnNotify(const uint8_t* d, size_t n);
bool ftpList(const char* path);  // the listing goes to the serial log
