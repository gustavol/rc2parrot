#include "pilot.h"

#include <Arduino.h>

#include "config.h"
#include "drone.h"
#include "log.h"

static bool switchesKnown = false;
static bool prevArm, prevKill, prevTrim;

void pilotOnFrame(const uint16_t* ch) {
  dronePcmd({stickToCmd(ch[CH_ROLL], DEADBAND), stickToCmd(ch[CH_PITCH], DEADBAND),
             stickToCmd(ch[CH_YAW], DEADBAND), stickToCmd(ch[CH_GAZ], DEADBAND_GAZ)});

  bool arm = ch[CH_ARM] > SWITCH_US, kill = ch[CH_KILL] > SWITCH_US, trim = ch[CH_TRIM] > SWITCH_US;
  if (!switchesKnown) {
    prevArm = arm;
    prevKill = false;  // a kill switch already high on (re)connect still cuts the motors
    prevTrim = trim;
    switchesKnown = true;
  }
  if (kill && !prevKill) droneCommand(DroneCmd::Emergency);
  else if (arm != prevArm) droneCommand(arm ? DroneCmd::TakeOff : DroneCmd::Landing);
  else if (trim && !prevTrim && !arm) droneCommand(DroneCmd::FlatTrim);  // arm low = landed
  prevArm = arm;
  prevKill = kill;
  prevTrim = trim;
}

void pilotSignalLost() {
  if (!switchesKnown) return;  // already handled, or no signal yet
  logf("PPM lost: landing\n");
  droneCommand(DroneCmd::Landing);
  dronePcmd({0, 0, 0, 0});
  switchesKnown = false;  // when the signal returns, the arm switch position does not take off
}

void pilotReset() { switchesKnown = false; }
