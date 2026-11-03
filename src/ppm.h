// PPM input from the radio trainer port.
#pragma once
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

constexpr int PPM_CHANNELS = 8;

struct PpmFrame {
  uint16_t ch[PPM_CHANNELS];  // pulse width per channel, us
  uint32_t seq;               // valid frame counter; 0 = no frame yet
  uint32_t ageMs;             // age of the last valid frame
};

// Attaches the pin interrupt. Every valid frame notifies the `wake` task.
void ppmBegin(int pin, TaskHandle_t wake);

PpmFrame ppmLatest();
