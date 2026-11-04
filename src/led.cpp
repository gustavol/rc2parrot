#include "led.h"

#include <Arduino.h>

#include "config.h"
#include "drone.h"

#if USE_LED

// The LED is RGBW: 32 bits per frame in G R B W order. The core's rgbLedWrite() sends only
// 24 bits, which misaligns the bytes. Same RMT timing as the core (0.1 us per tick), with W = 0.
static void ledWrite(uint8_t r, uint8_t g, uint8_t b) {
  static bool ok = rmtInit(LED_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);
  if (!ok) return;
  uint32_t grbw = (uint32_t)g << 24 | (uint32_t)r << 16 | (uint32_t)b << 8;
  rmt_data_t bits[32];
  for (int i = 0; i < 32; i++) {
    bool one = grbw & (1u << (31 - i));
    bits[i].level0 = 1;
    bits[i].duration0 = one ? 8 : 4;
    bits[i].level1 = 0;
    bits[i].duration1 = one ? 4 : 8;
  }
  rmtWrite(LED_PIN, bits, 32, RMT_WAIT_FOR_EVER);
}

static volatile uint32_t flashRgb = 0;
static volatile int flashTimes = 0;

void ledFlash(uint8_t r, uint8_t g, uint8_t b, int times) {
  flashRgb = (uint32_t)r << 16 | (uint32_t)g << 8 | b;
  flashTimes = times;
}

// Two linear segments so yellow keeps full brightness; a direct green-red blend is dark at 50 %.
static void batteryColor(int pct, uint8_t& r, uint8_t& g) {
  if (pct <= BATTERY_LOW_PCT) {
    r = LED_ALERT_BRIGHTNESS;
    g = 0;
    return;
  }
  pct = (pct - BATTERY_LOW_PCT) * 100 / (100 - BATTERY_LOW_PCT);
  if (pct >= 50) {
    r = (100 - pct) * 2 * LED_BRIGHTNESS / 100;
    g = LED_BRIGHTNESS;
  } else {
    r = LED_BRIGHTNESS;
    g = pct * 2 * LED_BRIGHTNESS / 100;
  }
}

// Own task because the BLE scan blocks the loop for seconds.
static volatile bool idle = false;  // see ledSetIdle

static void ledTask(void*) {
  uint32_t shown = UINT32_MAX;  // forces the first write
  for (;;) {
    if (flashTimes > 0) {
      int n = flashTimes;
      flashTimes = 0;
      uint32_t c = flashRgb;
      for (int i = 0; i < n; i++) {
        ledWrite(c >> 16, c >> 8, c);
        vTaskDelay(pdMS_TO_TICKS(LED_FLASH_MS));
        ledWrite(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(LED_FLASH_MS));
      }
      shown = UINT32_MAX;
    }
    int b = droneBattery();
    uint8_t r = 0, g = 0, bl = 0;
    if (!droneLinkUp() && idle) bl = millis() % 3000 < 100 ? LED_BRIGHTNESS : 0;
    else if (!droneLinkUp()) bl = (millis() / 250) % 2 ? LED_BRIGHTNESS : 0;
    else if (b < 0) bl = LED_BRIGHTNESS;
    else batteryColor(b, r, g);
    uint32_t c = r << 16 | g << 8 | bl;
    if (c != shown) {
      ledWrite(r, g, bl);
      shown = c;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void ledSetIdle(bool on) { idle = on; }

// Priority 1, below the loop (2).
void ledBegin() { xTaskCreate(ledTask, "led", 2048, nullptr, 1, nullptr); }
#else
void ledBegin() {}
void ledSetIdle(bool) {}
void ledFlash(uint8_t, uint8_t, uint8_t, int) {}
#endif
