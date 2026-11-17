# ESP32 robot firmware (`esp32_ultrasonic.ino`, v2.2.0)

The microcontroller side of the robot. It measures the ultrasonic range, outputs three 50 Hz
servo-style pulse channels, listens to the handheld remote (`../esp32_remote`), and brings the drive channels
to neutral whenever nobody is allowed to drive. It makes no decisions about *where* to go.

Files: `esp32_ultrasonic.ino` (pins, serial, radio), `drive_logic.h` (every decision, pure C++, tested by
`../tests/run_tests.sh`), `honebot_link.h` (radio packets; identical copy in `../esp32_remote`).

Arduino sketch for the **esp32 core 3.x** (`ledcAttach`/`ledcWrite` by pin; 2.x has a different LEDC API).
Build and flash from the Mac (commands in the top-level README).

## Pins

| GPIO | Use |
|---|---|
| 5 | HC-SR04 Trig |
| 18 | HC-SR04 Echo, through a 1 kΩ / 2 kΩ divider (5 V → 3.3 V) |
| 2 | onboard LED: on = someone may drive; slow blink = fail-safe, Pi link up; fast blink = remote STOP / lost; off = no Pi, no remote |
| 25 | channel A (left ESC in diff mode / throttle in ackermann) |
| 26 | channel B (right ESC / steering) |
| 27 | camera pan servo |

## Structure of the sketch

```
setup()   attach LEDC 50 Hz 16-bit on pins 25/26/27, write neutral immediately, print V,<version>
loop()    readSerial()         accumulate bytes, dispatch each complete line to handleLine()
          linkOk = heartbeat within WATCHDOG_MS; LED mirrors it
          updateOutputs(now)   fail-safe decision, caps and ramps, write all three pulses
          every 60 ms          readDistanceMeters() → print R,<m>,<link>
          every 100 ms         print S,<failsafe>,<a>,<b>
```

`handleLine()` recognises `H` (heartbeat), `C,a,b` (drive) and `P,pan` (camera); anything else is ignored.
Parsing uses `strtof` with full validation (both fields present, nothing trailing, finite) so a corrupted
line is dropped rather than half-applied. It is the one place to extend for new commands.

## Who may drive (`drive::decide` in `drive_logic.h`)

First match wins; every "nobody" is fail-safe:

| # | Condition | Who drives |
|---|---|---|
| 1 | inside the 2 s boot window (`ARM_MS`, ESC arming) | nobody |
| 2 | remote says STOP, or it was heard and has been silent for 500 ms | nobody |
| 3 | `REMOTE_REQUIRED` and the remote was never heard | nobody |
| 4 | remote in RUN and the stick is deflected (after being seen centred in RUN) | the stick (works with the Pi off) |
| 5 | Pi heartbeat within 500 ms and a `C` within 200 ms | the Pi |

**Fail-safe is a ramp, not a cut.** Both wheel channels are scaled down together (so the robot stays on its
arc) at `FAILSAFE_DECEL_G` = 0.5 g; from 15 mph that takes ~1.4 s. An instant cut with the ESC's brake is a
near-maximum stop, close to the 0.88 g at which the robot pitches over. After `FAILSAFE_MAX_MS` (2.5 s) the
outputs are forced to neutral whatever the ramp says, and at boot they are neutral from the first instruction.

Normal driving is slew-limited too: `ACCEL_G` 0.8 speeding up, `DECEL_G` 0.7 slowing down, both written as
robot accelerations and converted with `FULL_SCALE_MPS` (9.6 m/s at command 1.0). Commands are clamped to
`±THROTTLE_LIMIT` (0.5 for bench work). Manual driving is capped at `MANUAL_LIMIT` (0.25, about 2.4 m/s).
Manual driving goes straight to the motors: **the ultrasonic obstacle stop on the Pi does not apply to it.**

**`REMOTE_REQUIRED`** (default `true`): the robot does not move at all without the remote on, in range and in
RUN. Set it `false` only for bench work with the wheels off the ground; the bridge then logs a warning at
every start. A remote that is switched on is obeyed in both modes.

**Motor direction** is set here (`THROTTLE_REVERSED` = left wheel, `STEER_REVERSED` = right wheel), not with
the bridge's `left_reversed` / `right_reversed`, so the Pi and the stick agree.

The pan channel is independent of fail-safe: it slews toward the last `P` and is centred only at boot.

## Build, flash, check (Mac)

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32_ultrasonic
```
```bash
arduino-cli upload -p /dev/cu.usbserial-0001 --fqbn esp32:esp32:esp32 firmware/esp32_ultrasonic
```
On the Pi the bridge log should then show `ESP32 firmware version 2.2.0` and `Remote (kill switch): never`.

## Why it is built this way

- **Hardware PWM (LEDC) for the pulses.** The pulse timing is done by the peripheral, so the ~25 ms the
  `pulseIn` echo measurement can block the main loop never jitters the ESC signal. 16-bit resolution at
  50 Hz gives ~0.3 µs steps on a 1000–2000 µs pulse.
- **60 ms ping period, 25 ms echo timeout.** The HC-SR04 datasheet asks for ≥ 60 ms between pings;
  50 ms produced ~33 % dropouts on the bench. 25 ms of echo ≈ 4.3 m, the sensor's useful range.
- **Normalised −1..1 in, pulse widths out.** The firmware does not know the chassis; the Pi's bridge
  decides what the two channels mean (`drive_mode`). A chassis change is not a reflash.
- **Status line reports what is *output*, not what was commanded.** `S,<failsafe>,<a>,<b>` lets the Pi
  see when caps, ramps or fail-safe are overriding its commands (`/esp32/failsafe`, `/esp32/actuators`).
- **Version banner.** `V,2.2.0` at boot so the bridge can log which firmware is actually flashed; the bridge
  ignores lines it does not know, so firmware and bridge can be updated independently.
- **Bench caps in firmware, not only on the Pi.** `THROTTLE_LIMIT` protects against a wrong Pi-side
  parameter during the first drivetrain tests. Raise it once the robot behaves.
- **Blocking `pulseIn` is accepted for now.** Serial bytes buffer in the UART during the ≤ 25 ms blocking
  window and are read on the next loop pass, well inside the 200 ms command timeout. An interrupt-driven
  echo would be the upgrade if a faster or second sensor is added.

## Known limitation

The CP2102 USB chip on this DevKit makes the Pi's `close()` block ~5 s (see the udev rule in
`src/honebot_hardware/udev/`). Harmless for safety: the ESP32 fails safe on its own 500 ms watchdog.
