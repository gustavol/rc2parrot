// Radio input: PPM from the trainer jack, or CRSF from the external module port (INPUT_CRSF).
// ppm.cpp and crsf.cpp implement this interface; the build compiles one of them.
#pragma once
#include <stdint.h>

constexpr int RC_CHANNELS = 8;

struct RcFrame {
  uint16_t ch[RC_CHANNELS];  // pulse width per channel, us (CRSF values are converted)
  uint32_t seq;              // valid frame counter; 0 = no frame yet
  uint32_t ageMs;            // age of the last valid frame
};

void rcBegin();

// Blocks until a new valid frame or the timeout. Uses its own semaphore: a task notification
// would also wake NimBLE, which waits on task notifications inside connect() and discovery.
void rcWait(uint32_t timeoutMs);

RcFrame rcLatest();

// Raw input activity for the bench monitor: PPM edges or CRSF bytes received since boot.
uint32_t rcRawCount();
int rcRawDump(uint8_t* out);  // last raw bytes (up to 16, oldest first); CRSF only
