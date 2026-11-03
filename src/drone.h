// BLE link to a Parrot minidrone (ARSDK3 protocol, "minidrone" project).
#pragma once
#include <stdint.h>

// minidrone.Piloting
enum class DroneCmd : uint8_t { FlatTrim = 0, TakeOff = 1, Landing = 3, Emergency = 4 };

// -100..100. gaz = vertical speed (0 = hold altitude).
struct Pcmd {
  int8_t roll, pitch, yaw, gaz;
};

void droneBegin();

// Connects to the last known address first, then scans.
// Blocks for up to BLE_CONNECT_TIMEOUT_MS + BLE_SCAN_MS.
bool droneConnect();

bool droneLinkUp();  // safe from any task
int droneBattery();  // percent; -1 = unknown

void droneCommand(DroneCmd cmd);
void dronePcmd(const Pcmd& p);  // sends now and keeps the value for the keepalive
void droneTakePicture();
bool droneListFiles(const char* path);  // the listing goes to the serial log

// Call from the loop: handles ACKs and retries, and repeats the PCMD every PCMD_PERIOD_MS.
void droneService();

// Result of the last flat trim; reading clears it. Failed = no confirmation within TRIM_TIMEOUT_MS,
// or the automatic trim did not run because the drone did not report that it is landed.
enum class TrimResult : uint8_t { None, Ok, Failed };
TrimResult droneTakeTrimResult();

// True if a command failed since the last call: no ACK after the retries, or take-off blocked
// by a critical battery alert. Reading clears it.
bool droneTakeCommandFailed();

void dronePrintInfo();  // connection interval, MTU, RSSI
