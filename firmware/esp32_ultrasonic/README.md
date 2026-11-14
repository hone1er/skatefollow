# ESP32 firmware (`esp32_ultrasonic.ino`, v2.1.0)

The microcontroller side of the robot. It does three things and nothing else: measure the ultrasonic
range, output three 50 Hz servo-style pulse channels, and force the drive channels to neutral whenever the
Pi cannot be trusted. It makes no decisions about *where* to go.

Arduino sketch for the **esp32 core 3.x** (`ledcAttach`/`ledcWrite` by pin; 2.x has a different LEDC API).
Build and flash from the Mac (commands in the top-level README).

## Pins

| GPIO | Use |
|---|---|
| 5 | HC-SR04 Trig |
| 18 | HC-SR04 Echo, through a 1 kΩ / 2 kΩ divider (5 V → 3.3 V) |
| 2 | onboard LED = Pi link healthy |
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

## Fail-safe logic (`updateOutputs`)

```
failsafe = !(linkOk && cmdFresh && armed)
   linkOk   : an H arrived within WATCHDOG_MS (500)
   cmdFresh : a C arrived within CMD_TIMEOUT_MS (200)
   armed    : millis() >= ARM_MS (2000)  — ESCs need ~2 s of neutral at power-up
```
In fail-safe both drive channels go to 1500 µs **immediately** (no ramp). Out of fail-safe the command is
clamped to `±THROTTLE_LIMIT` (0.5 for bench work) and ramped at `THROTTLE_RATE_PER_S` (2.0/s). With
`DIFF_DRIVE = true` both channels are wheel throttles and both get the cap and ramp; with it false,
channel B is steering and passes straight through. The pan channel is independent of fail-safe: it
slews toward the last `P` at `PAN_RATE_PER_S` and is centred only at boot.

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
- **Version banner.** `V,2.1.0` at boot so the bridge can log which firmware is actually flashed; the bridge
  ignores lines it does not know, so firmware and bridge can be updated independently.
- **Bench caps in firmware, not only on the Pi.** `THROTTLE_LIMIT` protects against a wrong Pi-side
  parameter during the first drivetrain tests. Raise it once the robot behaves.
- **Blocking `pulseIn` is accepted for now.** Serial bytes buffer in the UART during the ≤ 25 ms blocking
  window and are read on the next loop pass, well inside the 200 ms command timeout. An interrupt-driven
  echo would be the upgrade if a faster or second sensor is added.

## Known limitation

The CP2102 USB chip on this DevKit makes the Pi's `close()` block ~5 s (see the udev rule in
`src/honebot_hardware/udev/`). Harmless for safety: the ESP32 fails safe on its own 500 ms watchdog.
