# Handheld remote (`esp32_remote.ino`, v1.0.0)

Kill switch and manual driving on a second ESP32 with the 5-pin joystick module. It talks straight to the
robot's ESP32 by radio (ESP-NOW): no router, no Pi, no pairing step.

## Wiring

| Joystick pin | ESP32 pin | Note |
|---|---|---|
| GND | GND | |
| +5V | **3V3** | not 5 V / VIN: the stick's outputs would exceed what the pins accept |
| VRx | GPIO 34 | |
| VRy | GPIO 35 | |
| SW | GPIO 32 | the stick's click button |

Power the board from a USB power bank.

## Using it

| You do | Result |
|---|---|
| Switch on (stick at rest) | STOP. LED blinks fast when the robot answers, slowly when it does not |
| Let go of the stick, press and **hold** the button 1.5 s | RUN. LED solid. The Pi may drive the robot |
| Click the button | STOP at once (the robot ramps down at ~0.5 g) |
| Move the stick while in RUN | You drive, slowly (about 2.4 m/s at full stick). The Pi is ignored, and this works with the Pi off |
| Let go of the stick | The robot stops; about 1 s later the Pi may drive again |
| Switch off / walk out of range | The robot stops within 0.5 s |

LED showing 3 quick flashes and a pause: the stick read wrong at power-up (loose wire, or the stick was held).
Fix it and press EN.

## Build, flash, check (Mac)

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32_remote
```
```bash
arduino-cli upload -p /dev/cu.usbserial-0001 --fqbn esp32:esp32:esp32 firmware/esp32_remote
```
```bash
arduino-cli monitor -p /dev/cu.usbserial-0001 -c baudrate=115200
```
You should see five lines a second like `state=STOP x=+0.00 y=+0.00 btn=0 robot=0 pi=0 failsafe=0 manual=0`.

1. Push the stick away from you: `x` should go to `+1.00`. Push it left: `y` should go to `+1.00`. If not, set
   `SWAP_XY`, `X_REVERSED`, `Y_REVERSED` at the top of the sketch and flash again.
2. Click the button: `btn=1` while pressed. Hold it 1.5 s with the stick at rest: `state=RUN`. Click: `state=STOP`.
3. Power the robot's ESP32 (firmware 2.2.0): `robot=1` within a second.

Only one board can be on the Mac's USB port name at a time; if both are plugged in, the second shows up under
a different `/dev/cu.usbserial-…` name (`arduino-cli board list`).

## Notes

- Both sketches share `HONEBOT_PAIR_ID` and the Wi-Fi channel in `honebot_link.h` (two identical copies; the
  test script checks they match). Change the ID if someone nearby runs the same firmware.
- The link is not encrypted or authenticated: anyone with this code and your ID could send STOP or RUN.
  Fine for an empty lot; not a security feature.
- Range has not been measured. The robot's ESP32 sits inside a carbon-filled tub, which weakens radio: test
  it with the shell on before trusting it (walk away until the remote's LED starts blinking slowly).
