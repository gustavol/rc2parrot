# rc2parrot

Fly a Parrot minidrone with a real RC radio. An ESP32-C3 reads the radio's trainer port (PPM),
or its external module port (CRSF), and drives the drone over Bluetooth Low Energy, replacing the
discontinued FreeFlight phone app.

```
RC radio --(PPM trainer port, or CRSF)--> ESP32-C3 --(BLE, ARSDK3)--> Parrot minidrone
```

## Status

| Part | State |
|------|-------|
| BLE connection, battery, take-off, landing, emergency, flat trim, picture | Tested on an Airborne Cargo |
| File listing over BLE FTP | Tested |
| PPM input from a FlySky i6X (OpenI6X) | Tested in flight |
| ACK retry, drone settings on connect, alerts | Tested |
| CRSF input and telemetry | **Not tested yet** (decoder tested on the host) |

Tested drone: Parrot Airborne Cargo ("Travis"). The Rolling Spider, Airborne Night, Mambo and
Swing use the same protocol family but are untested.

## Features

- PCMD every 50 ms, like the Parrot app, always with the newest radio frame.
- BLE connection interval fixed at 7.5 ms, the lowest value the drone asks for.
- Serial port fully off while the radio is active, so logging cannot add latency.
- Acknowledged commands with the same retry rules as the Parrot SDK. The emergency command
  repeats every 100 ms until the drone acknowledges it.
- Failsafe: no radio frame for 300 ms lands the drone.
- Kill switch is level-triggered on reconnect: if it is high, the motors are cut.
- Take-off is blocked while the drone reports a critical battery.
- Automatic flat trim on the first connection, only if the drone reports that it is landed.
- Drone settings written on every connection: max tilt, max altitude, max vertical and rotation
  speed, banked turn, cut-out on impact and the accessory (hull). Defaults in `src/config.h`. CRSF
  builds show them in the radio's Tools > ExpressLRS menu; a change is saved in the ESP32 flash
  (NVS) and sent to the drone at once if it is connected. Firmware uploads keep the saved values;
  `pio run -t erase` clears them.
- Status LED: connection, battery gradient, trim and failure flashes.
- Serial test console: commands, radio monitor, link info, file listing.

## Hardware

- ESP32-C3 board (for example an ESP32-C3 SuperMini).
- RC radio with PPM output on the trainer port, or a CRSF external module output. Developed for a
  FlySky i6X with OpenI6X.
- Optional: one RGBW NeoPixel (SK6812).
- Power for the ESP32 (USB power bank or a 1S cell on the 5 V pin). The trainer port does not
  power it.

| ESP32-C3 | Connects to |
|----------|-------------|
| GPIO4 | PPM signal from the trainer port, or CRSF from the radio (OpenI6X: `TX2` pad) |
| GPIO3 | CRSF telemetry to the radio (OpenI6X: `PA15` pad), CRSF builds only |
| GPIO5 | LED DIN |
| 3V3 | LED VCC |
| GND | Trainer port GND, LED GND |

> **Warning:** measure the PPM signal voltage before you connect it. ESP32-C3 pins accept 3.3 V
> maximum. Use a resistor divider if the signal is higher.

Avoid GPIO 2, 8 and 9 (strapping pins). Pins are set in `src/config.h`.

## Radio setup (OpenI6X / OpenTX)

1. Model Setup: set Internal RF to OFF.
2. Choose the input:
   - **PPM:** set the trainer mode to Slave (Slave/Jack), so the radio outputs PPM on the trainer
     port. OpenI6X needs version 2.3.0 or later: older versions only offer Master modes. Keep
     External RF OFF. OpenI6X 2.3.0 has a bug: after power-on there is no PPM until you switch the
     trainer mode to Master/Jack and back to Slave
     ([fix](https://github.com/gustavol/opentx/tree/fix/trainer-slave-boot)).
   - **CRSF (full duplex, with telemetry):** solder the radio `TX2` pad to GPIO4 and the `PA15`
     pad to GPIO3, plus GND. OpenI6X 2.3.0 needs a firmware built with `CRSF_FULLDUPLEX=YES`;
     later versions have a "CRSF Duplex" option. Set External RF to CRSF, 400k baud (Radio
     Settings > Hardware), and build with `-e esp32c3_crsf`. The radio shows the sensors 1RSS (BLE
     RSSI), RQly (link quality from the command ACKs), Bat% and FM (drone state). RQly drops to 0
     while the drone is not connected, so the radio warns that telemetry is lost.
3. Map the channels:

| Channel | Function |
|---------|----------|
| CH1 | Roll |
| CH2 | Pitch |
| CH3 | Throttle (vertical speed; center = hold altitude) |
| CH4 | Yaw |
| CH5 | 2-position switch: take off / land |
| CH6 | 2-position switch: cut motors |
| CH7 | Switch or button: flat trim (landed only) |

4. Set dual rates on a 3-position switch (slow, normal, sport). The drone runs at its maximum
   limits, so start with low rates.

## Build and flash

Requires [PlatformIO](https://platformio.org/).

```sh
pio run -t upload                           # flight build, PPM input
pio run -e esp32c3_console -t upload        # bench build: console stays on with the radio
pio run -e esp32c3_crsf -t upload           # flight build, CRSF input
pio run -e esp32c3_crsf_console -t upload   # bench build, CRSF input
pio device monitor
```

Host tests for the decoders:

```sh
g++ -std=c++17 -Wall -Isrc test/test_ppm.cpp -o /tmp/test_ppm && /tmp/test_ppm
g++ -std=c++17 -Wall -Isrc test/test_crsf.cpp -o /tmp/test_crsf && /tmp/test_crsf
```

## Serial console

Available without a radio signal, or always in the bench build.

| Key | Action |
|-----|--------|
| `t` | Take off |
| `l` | Land |
| `k` | Cut motors |
| `r` | Flat trim |
| `f` | Take a picture |
| `p` | Toggle the radio monitor (channels in us) |
| `i` | BLE link info (interval, MTU, RSSI, TX power) |
| `d<path>` + Enter | List files on the drone (`d` alone lists `/`) |
| `h` | Help |

Pictures are stored on the drone in `/internal_000/<product>/media/`.

## Status LED

| LED | Meaning |
|-----|---------|
| Blinking blue | Searching for the drone |
| Solid blue | Connected, battery not reported yet |
| Green -> yellow -> red | Battery from 100 % down to 20 % |
| Bright solid red | Battery at 20 % or less |
| 3 green flashes | Flat trim confirmed |
| 3 red flashes | Trim or command failed |

## Project layout

```
src/
  config.h         pins, channel map, tuning, input selection (INPUT_CRSF)
  rc.h             radio input interface
  ppm_decoder.h    PPM decoder (host-testable)
  ppm.cpp          PPM input: pin interrupt and latest frame
  crsf_decoder.h   CRSF decoder (host-testable)
  crsf.cpp         CRSF input: UART and latest frame
  pilot.*          sticks and switches -> drone commands, failsafe
  drone.*          BLE link and ARSDK3 protocol
  ftp.*            file listing over BLE FTP
  led.*            status LED
  log.*            serial output
  console.*        serial test console
test/test_ppm.cpp   PPM decoder test
test/test_crsf.cpp  CRSF decoder test
wiki/              project notes and protocol findings (Portuguese)
```

## References

- Parrot ARSDK3: [arsdk-xml](https://github.com/Parrot-Developers/arsdk-xml),
  [libARDiscovery](https://github.com/Parrot-Developers/libARDiscovery) (BLE buffer timing),
  [libARUtils](https://github.com/Parrot-Developers/libARUtils) (BLE FTP)
- [pyparrot](https://github.com/amymcgovern/pyparrot) (minidrone command XML, BLE handshake)
- [node-rolling-spider](https://github.com/voodootikigod/node-rolling-spider)
- [DIY-Multiprotocol-TX-Module](https://github.com/pascallanger/DIY-Multiprotocol-TX-Module)
  (PPM frame validation)
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)

## Credits

Built with the help of [Claude](https://claude.ai) (Anthropic).

Not affiliated with or endorsed by Parrot. Parrot, Airborne Cargo, Rolling Spider and Mambo are
trademarks of Parrot SA.
