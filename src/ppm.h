// PPM input from the radio trainer port.
#pragma once
#include <stdint.h>

constexpr int PPM_CHANNELS = 8;

struct PpmFrame {
  uint16_t ch[PPM_CHANNELS];  // pulse width per channel, us
  uint32_t seq;               // valid frame counter; 0 = no frame yet
  uint32_t ageMs;             // age of the last valid frame
};

void ppmBegin(int pin);  // attaches the pin interrupt

// Blocks until a new valid frame or the timeout. Uses its own semaphore: a task notification
// would also wake NimBLE, which waits on task notifications inside connect() and discovery.
void ppmWait(uint32_t timeoutMs);

PpmFrame ppmLatest();
