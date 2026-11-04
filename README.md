# rc2parrot

Fly a Parrot minidrone with a real RC radio. An ESP32-C3 reads the radio, either PPM from the
trainer port or CRSF from the external module port, and drives the drone over Bluetooth Low
Energy. It replaces the discontinued FreeFlight phone app.

```
RC radio --(PPM trainer port, or CRSF)--> ESP32-C3 --(BLE, ARSDK3)--> Parrot minidrone
         <--(CRSF telemetry, settings menu)--
```

With CRSF the radio also shows the drone telemetry (battery, BLE signal, flight state) and a
settings menu in Tools > ExpressLRS, and the firmware can be updated over WiFi. The ESP32 can
live inside the radio.

## Status

| Part | State |
|------|-------|
| BLE connection, battery, take-off, landing, emergency, flat trim, picture | Tested on an Airborne Cargo |
| File listing over BLE FTP | Tested |
| PPM input from a FlySky i6X (OpenI6X) | Tested in flight |
| ACK retry, drone settings on connect, alerts | Tested |
| CRSF input, telemetry, settings menu | Tested on the bench with the drone |
| WiFi update | Tested |
| Idle mode without the radio | Builds; test pending |

Tested drone: Parrot Airborne Cargo ("Travis", firmware 2.6.8). The Rolling Spider, Airborne
Night, Mambo and Swing use the same protocol family but are untested.

## Features

- PCMD every 50 ms, like the Parrot app, always with the newest radio frame.
- BLE connection interval fixed at 7.5 ms, the lowest value the drone asks for.
- Serial port fully off while the radio is active, so logging cannot add latency.
- Acknowledged commands with the same retry rules as the Parrot SDK. The emergency command
  repeats every 100 ms until the drone acknowledges it.
- Failsafe: no radio frame for 300 ms lands the drone.
- Kill switch is level-triggered on reconnect: if it is high, the motors are cut.
- Take-off is blocked while the drone reports a critical battery, or while the throttle stick is
  off center.
- Automatic flat trim on the first connection, only if the drone reports that it is landed.
- Drone settings written on every connection (see [Settings](#settings)).
- Idle mode: radio off, or on another model, for 10 s: the drone scan stops and a landed drone is
  disconnected, so the ESP32 draws little power. It reconnects when the radio frames come back.
- CRSF only: telemetry to the radio, settings menu, WiFi update.
- Status LED (optional): connection, battery gradient, trim and failure flashes.
- Serial test console: commands, radio monitor with input diagnostics, link info, file listing.

## Hardware

- ESP32-C3 board with 4 MB flash (for example an ESP32-C3 SuperMini).
- RC radio with PPM on the trainer port, or a CRSF external module output. Developed for a
  FlySky i6X with OpenI6X.
- Optional: one RGBW NeoPixel (SK6812).
- Power for the ESP32 (USB power bank, a 1S cell on the 5 V pin, or a 5 V supply inside the
  radio). The trainer port does not power it.

| ESP32-C3 | PPM build | CRSF build |
|----------|-----------|------------|
| GPIO4 | PPM signal from the trainer port | CRSF from the radio (OpenI6X: `TX2` pad) |
| GPIO3 | – | CRSF telemetry to the radio (OpenI6X: `PA15` pad) |
| GPIO5 | LED DIN (optional) | LED DIN (optional) |
| 3V3 | LED VCC | LED VCC |
| GND | Trainer port GND, LED GND | Radio GND, LED GND |

> **Warning:** measure the signal voltage before you connect it. ESP32-C3 pins accept 3.3 V
> maximum. Use a resistor divider if the signal is higher. Never connect the radio battery to the
> ESP32 pins.

Avoid GPIO 2, 8 and 9 (strapping pins) and 18/19 (USB). Pins are set in `src/config.h`.

> **Note:** if the ESP32 takes its power from the radio, switching the radio off also switches
> the ESP32 off, so the firmware failsafe cannot run: the drone only loses the BLE link. Test
> what your drone does then, in a low hover, before you rely on it.

## Radio setup (OpenI6X / OpenTX)

1. Model Setup: set Internal RF to OFF.
2. Choose the input.

### PPM (trainer port)

- Set the trainer mode to Slave (Slave/Jack), so the radio outputs PPM on the trainer port.
  OpenI6X needs version 2.3.0 or later: older versions only offer Master modes. Keep External
  RF OFF.
- OpenI6X 2.3.0 bug: after power-on there is no PPM until you switch the trainer mode to
  Master/Jack and back to Slave. Fix:
  [OpenI6X pull request](https://github.com/gustavol/opentx/tree/fix/trainer-slave-boot-master).
- Build: `esp32c3` (or `esp32c3_noled`).

### CRSF (full duplex, with telemetry)

- Wiring: radio `TX2` pad to GPIO4, radio `PA15` pad to GPIO3, plus GND. The pads are inside the
  radio (see the OpenI6X wiki, Modifications).
- The radio must send CRSF in full duplex, not inverted:
  - OpenI6X after 2.3.0 (nightly): enable "CRSF Duplex" in Radio Settings > Hardware.
  - OpenI6X 2.3.0: only a firmware built with `CRSF_FULLDUPLEX=YES` has it.
  - The one-wire half-duplex mode of the official 2.3.0 firmware is not supported yet.
- Radio Settings > Hardware > Baudrate: 400k (`CRSF_BAUD` in `src/config.h`).
- Model Setup: set External RF to CRSF.
- Build: `esp32c3_crsf` (or `esp32c3_crsf_noled`).
- The ESP32 answers the radio's device ping as the module "rc2parrot". OpenI6X sends channels
  only after a module answers.

Telemetry: in the model's Telemetry page, select Discover new sensors with the drone connected,
then Stop discovery. The sensors are listed in the **Sensors** section; press ENTER on that line
to expand it.

| Sensor | Value |
|--------|-------|
| 1RSS | BLE RSSI, dBm |
| RQly | Link quality from the command ACKs, %. 0 while the drone is not connected, so the radio warns that telemetry is lost. |
| Bat% | Drone battery |
| FM | Drone state: landed, hovering, emergency... "no drone" or "wifi update" |

### Channels (both inputs)

| Channel | Function |
|---------|----------|
| CH1 | Roll |
| CH2 | Pitch |
| CH3 | Throttle (vertical speed; center = hold altitude) |
| CH4 | Yaw |
| CH5 | 2-position switch: take off / land |
| CH6 | 2-position switch: cut motors |
| CH7 | Switch or button: flat trim (landed only) |

Switches are "high" above 1250 us, so both -100..+100 and -100..0 switches work.
Set dual rates on a 3-position switch (slow, normal, sport). Vertical and rotation speed stay at
the drone maximum by default, so start with low rates.

## Settings

The firmware writes these to the drone on every connection. Defaults are in `src/config.h`. CRSF
builds show them in the radio's Tools > ExpressLRS menu: a change is saved in the ESP32 flash
(NVS) and sent to the drone at once if it is connected. Firmware uploads keep the saved values;
`pio run -t erase` clears them.

| Menu item | Range | Default | Notes |
|-----------|-------|---------|-------|
| Max tilt | 5..25 deg | 10 | Roll and pitch are a tilt angle, which is acceleration: raise it slowly |
| Max alt | 20..250 dm | 25 (2.5 m) | dm = 0.1 m; the menu shows whole numbers only |
| Vert speed | 50..200 cm/s | 200 | Drone maximum |
| Yaw speed | 50..180 deg/s | 180 | Drone maximum |
| Banked turn | Off, On | On | On: yaw also banks the drone when it moves, like a plane |
| Cut-out | Off, On | On | Motors stop on a hard impact or when the drone turns over |
| Accessory | None, Wheels, Truck, Hull, Hydrofoil | Hull | The drone refuses a change in flight |
| WiFi update | Off, On | Off | Not saved; see [WiFi update](#wifi-update-crsf-builds) |
| Firmware | – | rc2parrot | Read-only |

## Build and flash

Requires [PlatformIO](https://platformio.org/).

| Environment | Input | Use |
|-------------|-------|-----|
| `esp32c3` (default) | PPM | Flight |
| `esp32c3_noled` | PPM | Flight, no LED |
| `esp32c3_console` | PPM | Bench: console stays on with the radio |
| `esp32c3_crsf` | CRSF | Flight |
| `esp32c3_crsf_noled` | CRSF | Flight, no LED |
| `esp32c3_crsf_noled_ota` | CRSF | Same as `esp32c3_crsf_noled`, uploaded over WiFi |
| `esp32c3_crsf_console` | CRSF | Bench: console stays on with the radio |
| `esp32c3_crsf_test` | CRSF | Radio link test: no drone scan, radio monitor on from boot |

```sh
pio run -e esp32c3_crsf -t upload
pio device monitor
```

The flight builds switch the USB port off while the radio is active. To upload over USB, switch
the radio off or select another model first. If the port still does not appear, hold BOOT while
you plug in the USB cable.

The ESP32 uses the `min_spiffs.csv` partition table (two 1.9 MB app slots for the WiFi update).
The partition table changes only with a USB upload.

Build options in `src/config.h`, also settable with `-D` in `platformio.ini`:

| Option | Default | Meaning |
|--------|---------|---------|
| `INPUT_CRSF` | 0 | 1 = CRSF input instead of PPM |
| `FORCE_CONSOLE` | 0 | 1 = serial console stays on with the radio (bench) |
| `NO_DRONE` | 0 | 1 = no drone scan, radio monitor on from boot (radio link test) |
| `USE_LED` | 1 | 0 = no status LED, GPIO5 free |
| `CRSF_BAUD` | 400000 | Must match the radio |
| `CRSF_INVERTED` | false | Line polarity; full duplex is not inverted |
| `CRSF_TELEMETRY_MS` | 200 | Telemetry period; 0 = off |
| `RADIO_IDLE_MS` | 10000 | No radio for this long: idle mode |
| `WIFI_AP_SSID`, `WIFI_AP_PASSWORD` | rc2parrot | WiFi update network and OTA password |

Host tests:

```sh
g++ -std=c++17 -Wall -Isrc test/test_ppm.cpp -o /tmp/test_ppm && /tmp/test_ppm
g++ -std=c++17 -Wall -Isrc test/test_crsf.cpp -o /tmp/test_crsf && /tmp/test_crsf
```

## WiFi update (CRSF builds)

For an ESP32 built into the radio. The WiFi stays off until you switch it on, and it is off again
after every restart.

1. Land the drone. In the radio's Tools > ExpressLRS menu, set "WiFi update" to On. The FM sensor
   shows "wifi update" and the ESP32 disconnects the drone. With the drone in the air, the ESP32
   refuses.
2. Join the WiFi network `rc2parrot` (password `rc2parrot`).
3. Run `pio run -e esp32c3_crsf_noled_ota -t upload`. The ESP32 restarts with the new firmware.
   To leave without an update, set "WiFi update" to Off or restart the radio.

On Linux, with WiFi as the only connection, this one line joins the ESP32 network, uploads and
goes back (replace `HomeWifi` with your connection name):

```sh
nmcli dev wifi connect rc2parrot password rc2parrot && pio run -e esp32c3_crsf_noled_ota -t upload; nmcli con up HomeWifi --ask
```

The first upload with this feature must go over USB. The upload needs a TCP connection from the
ESP32 back to the PC: allow it in the PC firewall.

## Serial console

Available without a radio signal, or always in the bench builds.

| Key | Action |
|-----|--------|
| `t` | Take off |
| `l` | Land |
| `k` | Cut motors |
| `r` | Flat trim |
| `f` | Take a picture |
| `p` | Toggle the radio monitor (channels in us; pauses the drone scan) |
| `i` | BLE link info (interval, MTU, RSSI, TX power) and drone state |
| `d<path>` + Enter | List files on the drone (`d` alone lists `/`) |
| `h` | Help |

When no valid radio frame arrives, the monitor shows the raw input rate every second (PPM edges or
CRSF bytes) and, for CRSF, the last 16 bytes in hex:

| Raw input | Valid frames | Likely cause |
|-----------|--------------|--------------|
| 0 | 0 | Wiring, wrong pad, or the radio is not sending |
| Many | 0 | Baud rate or polarity (CRSF), or the radio sends only pings |
| Many | Many | Working |

The firmware logs each drone event id once per connection, with its raw bytes (bench builds show
it during flight). This shows what your drone firmware sends.

Pictures are stored on the drone in `/internal_000/<product>/media/`.

## Status LED

| LED | Meaning |
|-----|---------|
| Blinking blue | Searching for the drone |
| One blue blink every 3 s | Idle: no radio for 10 s, drone scan stopped |
| Solid blue | Connected, battery not reported yet |
| Green -> yellow -> red | Battery from 100 % down to 20 % |
| Bright solid red | Battery at 20 % or less |
| 3 green flashes | Flat trim confirmed |
| 3 red flashes | Trim or command failed |

## Project layout

```
src/
  config.h         pins, channel map, tuning, build options, setting defaults
  main.cpp         loop: radio -> pilot -> drone, idle mode, settings, WiFi update
  rc.h             radio input interface (ppm.cpp or crsf.cpp is compiled)
  ppm_decoder.h    PPM decoder (host-testable)
  ppm.cpp          PPM input: pin interrupt and latest frame
  crsf_decoder.h   CRSF decoder and frame builder (host-testable)
  crsf_menu.h      ExpressLRS settings menu (host-testable)
  crsf.cpp         CRSF input: UART, device ping, menu, telemetry
  settings.*       drone settings saved in NVS
  ota.*            WiFi update (access point + ArduinoOTA)
  pilot.*          sticks and switches -> drone commands, failsafe
  drone.*          BLE link and ARSDK3 protocol
  ftp.*            file listing over BLE FTP
  led.*            status LED
  log.*            serial output
  console.*        serial test console
test/test_ppm.cpp   PPM decoder test
test/test_crsf.cpp  CRSF decoder and menu test
wiki/               project notes and protocol findings (Portuguese, not in git)
```

## References

- Parrot ARSDK3: [arsdk-xml](https://github.com/Parrot-Developers/arsdk-xml),
  [libARDiscovery](https://github.com/Parrot-Developers/libARDiscovery) (BLE buffer timing),
  [libARUtils](https://github.com/Parrot-Developers/libARUtils) (BLE FTP)
- [pyparrot](https://github.com/amymcgovern/pyparrot) (minidrone command XML, BLE handshake)
- [node-rolling-spider](https://github.com/voodootikigod/node-rolling-spider)
- [DIY-Multiprotocol-TX-Module](https://github.com/pascallanger/DIY-Multiprotocol-TX-Module)
  (PPM frame validation)
- [OpenI6X](https://github.com/OpenI6X/opentx) (CRSF module handling, ExpressLRS menu)
- [ExpressLRS](https://github.com/ExpressLRS/ExpressLRS) (CRSF handset side)
- [TBS CRSF spec](https://github.com/tbs-fpv/tbs-crsf-spec)
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)

## Credits

Built with the help of [Claude](https://claude.ai) (Anthropic).

Not affiliated with or endorsed by Parrot. Parrot, Airborne Cargo, Rolling Spider and Mambo are
trademarks of Parrot SA.
