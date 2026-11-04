// PPM input from the radio trainer jack.
#include "config.h"
#if !INPUT_CRSF
#include <Arduino.h>

#include "ppm_decoder.h"
#include "rc.h"

static PpmDecoder decoder;
static uint16_t shared[RC_CHANNELS];
static uint32_t seq = 0, lastFrameMs = 0;
static volatile uint32_t edges = 0;
static SemaphoreHandle_t frameReady = nullptr;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR onEdge() {
  edges++;
  if (!decoder.edge(micros())) return;
  portENTER_CRITICAL_ISR(&mux);
  for (int i = 0; i < RC_CHANNELS; i++) shared[i] = decoder.channels()[i];
  seq++;
  lastFrameMs = millis();
  portEXIT_CRITICAL_ISR(&mux);

  // Wake the loop now so the PCMD goes out right after the frame.
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(frameReady, &woken);
  portYIELD_FROM_ISR(woken);
}

void rcBegin() {
  frameReady = xSemaphoreCreateBinary();
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), onEdge, RISING);
}

void rcWait(uint32_t timeoutMs) { xSemaphoreTake(frameReady, pdMS_TO_TICKS(timeoutMs)); }

RcFrame rcLatest() {
  RcFrame f;
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < RC_CHANNELS; i++) f.ch[i] = shared[i];
  f.seq = seq;
  uint32_t t = lastFrameMs;
  portEXIT_CRITICAL(&mux);
  f.ageMs = f.seq ? millis() - t : UINT32_MAX;
  return f;
}
uint32_t rcRawCount() { return edges; }
int rcRawDump(uint8_t*) { return 0; }
#endif
