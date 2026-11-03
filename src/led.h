// Status LED (RGBW NeoPixel), driven by its own task.
// Blinking blue = searching for the drone. Solid blue = connected, battery not reported yet.
// Connected: green (100 %) -> yellow -> red (BATTERY_LOW_PCT); at or below it, bright solid red.
#pragma once
#include <stdint.h>

void ledBegin();
void ledFlash(uint8_t r, uint8_t g, uint8_t b, int times);  // flashes over the normal color
