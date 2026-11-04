// Radio to drone: sticks to PCMD, switches to commands, failsafe.
#pragma once
#include <stdint.h>

// 1000..2000 us -> -100..100, with a center deadband.
constexpr int8_t stickToCmd(int us, int dead) {
  int v = (us - 1500) / 5;
  if (v > 100) v = 100;
  if (v < -100) v = -100;
  if (v > -dead && v < dead) return 0;
  return v;
}
static_assert(stickToCmd(1500, 5) == 0, "center");
static_assert(stickToCmd(1520, 5) == 0, "deadband");
static_assert(stickToCmd(2000, 5) == 100, "max");
static_assert(stickToCmd(2200, 5) == 100, "clamp");
static_assert(stickToCmd(1000, 5) == -100, "min");

void pilotOnFrame(const uint16_t* ch);  // new radio frame: PCMD and switch commands go out now
void pilotSignalLost();                 // failsafe: land once and zero the PCMD

// New connection: the current arm and trim switch positions do not trigger a command.
// The kill switch is the exception: if it is high, the motors are cut.
void pilotReset();
