// Text output over the USB serial port.
// Only the loop task touches Serial. Other tasks (NimBLE callbacks) go through a queue that the
// loop drains in logFlush(), so Serial can be shut down safely. With the port off, logf() and
// logWrite() return immediately without formatting.
#pragma once
#include <stddef.h>
#include <stdint.h>

void logBegin();  // call from setup(): the calling task becomes the Serial owner
void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void logWrite(const uint8_t* d, size_t n);

// Loop task only.
void logSetSerial(bool on);  // false = Serial.end(): no output, no USB interrupts
bool logSerialOn();
void logFlush();
