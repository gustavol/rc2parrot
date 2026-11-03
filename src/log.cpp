#include "log.h"

#include <Arduino.h>
#include <esp_log.h>
#include <freertos/message_buffer.h>
#include <stdarg.h>

static TaskHandle_t owner = nullptr;
static MessageBufferHandle_t queue = nullptr;
static SemaphoreHandle_t queueLock = nullptr;  // a message buffer allows one writer at a time
static volatile bool serialOn = false;

void logBegin() {
  owner = xTaskGetCurrentTaskHandle();
  queue = xMessageBufferCreate(2048);
  queueLock = xSemaphoreCreateMutex();
  logSetSerial(true);
}

void logWrite(const uint8_t* d, size_t n) {
  if (!serialOn || n == 0) return;
  if (xTaskGetCurrentTaskHandle() == owner) {
    Serial.write(d, n);
    return;
  }
  // Other tasks never wait: a full or busy queue drops the message.
  if (xSemaphoreTake(queueLock, 0) != pdTRUE) return;
  xMessageBufferSend(queue, d, n, 0);
  xSemaphoreGive(queueLock);
}

void logf(const char* fmt, ...) {
  if (!serialOn) return;
  char buf[192];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, args);
  va_end(args);
  if (n > 0) logWrite((const uint8_t*)buf, n < (int)sizeof buf ? n : sizeof buf - 1);
}

void logFlush() {
  uint8_t buf[256];
  size_t n;
  while ((n = xMessageBufferReceive(queue, buf, sizeof buf, 0)) > 0)
    if (serialOn) Serial.write(buf, n);
}

void logSetSerial(bool on) {
  if (on == serialOn) return;
  if (on) {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // never block when no host is attached
    esp_log_level_set("*", ESP_LOG_ERROR);
    serialOn = true;
  } else {
    serialOn = false;  // close the gate for other tasks first
    logFlush();        // discards what is left
    Serial.end();
    // ESP-IDF writes errors straight to UART0 and USB, bypassing Serial.
    esp_log_level_set("*", ESP_LOG_NONE);
  }
}

bool logSerialOn() { return serialOn; }
