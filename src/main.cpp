// RC radio (PPM from the trainer port, or CRSF) -> ESP32-C3 -> BLE -> Parrot minidrone.
//
//   config.h        pins, channel map, tuning, input selection (INPUT_CRSF)
//   rc.h            radio input interface
//   ppm.cpp         PPM input (interrupt); ppm_decoder.h is the host-testable decoder
//   crsf.cpp        CRSF input (UART); crsf_decoder.h is the host-testable decoder
//   pilot.*         sticks and switches -> drone commands, failsafe
//   drone.*         BLE link and ARSDK3 protocol
//   ftp.*           drone file listing (FTP over BLE)
//   led.*           status LED
//   log.*           USB serial output, fully off while the radio is active
//   console.*       serial test commands (without radio, or with FORCE_CONSOLE)
#include <Arduino.h>

#include "config.h"
#include "console.h"
#include "drone.h"
#include "led.h"
#include "log.h"
#include "pilot.h"
#include "rc.h"
#include "settings.h"

void setup() {
  // Priorities: BLE controller (23) and NimBLE host (21) > loop (2) > LED (1).
  vTaskPrioritySet(nullptr, 2);
  logBegin();
  logf("build %s %s\n", __DATE__, __TIME__);
  logf("radio input: %s\n", INPUT_CRSF ? "CRSF" : "PPM");
  if (FORCE_CONSOLE) logf("FORCED CONSOLE: serial stays on with the radio. Not for normal flight.\n");
  settingsBegin();
  rcBegin();
  ledBegin();
  droneBegin();
}

// While the radio is active the serial port is off: no output, no input, no USB interrupts.
static void updateSerial(bool rc) {
  if (rc == !logSerialOn()) return;
  if (rc) {
    logf("radio active: serial off\n");
    logSetSerial(false);
  } else {
    logSetSerial(true);
    consoleReset();
    logf("no radio: console on (h = help)\n");
  }
}

void loop() {
  // Sleep until a new radio frame or the next PCMD slot.
  rcWait(droneLinkUp() ? dronePcmdDueMs() : PCMD_PERIOD_MS);

  if (settingsChanged) {  // changed in the radio menu
    settingsChanged = false;
    settingsSave();
    droneApplySettings();
  }

  RcFrame f = rcLatest();
  bool rc = f.ageMs < RC_TIMEOUT_MS;
  bool console = FORCE_CONSOLE || !rc;
  if (!FORCE_CONSOLE) updateSerial(rc);

  // Before the link check, so a failure right before a disconnect still flashes the LED.
  switch (droneTakeTrimResult()) {
    case TrimResult::Ok: ledFlash(0, LED_BRIGHTNESS, 0, 3); break;
    case TrimResult::Failed: ledFlash(LED_BRIGHTNESS, 0, 0, 3); break;
    case TrimResult::None: break;
  }
  if (droneTakeCommandFailed()) ledFlash(LED_BRIGHTNESS, 0, 0, 3);

  if (!droneLinkUp()) {
    pilotReset();
    if (console) {  // lets you watch the radio without the drone
      consolePoll();
      logFlush();
    }
    // Bench: the scan blocks the loop for seconds, so it waits while the monitor runs.
    if (NO_DRONE || (console && consoleMonitorOn())) return;
    if (!droneConnect()) delay(500);
    logFlush();
    return;
  }

  static uint32_t lastSeq = 0;
  if (rc && f.seq != lastSeq) {
    lastSeq = f.seq;
    pilotOnFrame(f.ch);
  } else if (!rc) {
    pilotSignalLost();
  }
  droneService();

  if (console) {
    consolePoll();
    logFlush();
  }
}
