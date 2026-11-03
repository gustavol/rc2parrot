#include "ppm.h"

#include <Arduino.h>

#include "ppm_decoder.h"

static PpmDecoder decoder;
static uint16_t shared[PPM_CHANNELS];
static uint32_t seq = 0, lastFrameMs = 0;
static SemaphoreHandle_t frameReady = nullptr;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR onEdge() {
  if (!decoder.edge(micros())) return;
  portENTER_CRITICAL_ISR(&mux);
  for (int i = 0; i < PPM_CHANNELS; i++) shared[i] = decoder.channels()[i];
  seq++;
  lastFrameMs = millis();
  portEXIT_CRITICAL_ISR(&mux);

  // Wake the loop now so the PCMD goes out right after the frame.
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(frameReady, &woken);
  portYIELD_FROM_ISR(woken);
}

void ppmBegin(int pin) {
  frameReady = xSemaphoreCreateBinary();
  pinMode(pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin), onEdge, RISING);
}

void ppmWait(uint32_t timeoutMs) { xSemaphoreTake(frameReady, pdMS_TO_TICKS(timeoutMs)); }

PpmFrame ppmLatest() {
  PpmFrame f;
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < PPM_CHANNELS; i++) f.ch[i] = shared[i];
  f.seq = seq;
  uint32_t t = lastFrameMs;
  portEXIT_CRITICAL(&mux);
  f.ageMs = f.seq ? millis() - t : UINT32_MAX;
  return f;
}
