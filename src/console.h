// Serial test console. Runs without valid PPM, or always with FORCE_CONSOLE.
#pragma once

void consolePoll();   // reads and runs keys
void consoleReset();  // serial port back on: discard input for CONSOLE_DISCARD_MS
bool consoleMonitorOn();  // PPM monitor (key p) on: the drone scan pauses
