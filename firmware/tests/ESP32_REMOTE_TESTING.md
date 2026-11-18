# Testing the handheld remote with turtlesim

The handheld remote (`firmware/esp32_remote`) is the robot's kill switch and manual drive. Until the robot is
built, a turtle in turtlesim stands in for it: the turtle moves the way the robot's wheels would.

## How the pieces fit

On the **real robot** the Pi is not in the remote's path at all:

```
remote --ESP-NOW--> robot ESP32 --> wheel ESCs          (works with the Pi off)
Pi follower --> /cmd_vel_raw --> safety_gate --> /cmd_vel --> esp32_bridge --USB--> robot ESP32
```

The robot ESP32 decides who drives: nobody in STOP or when the remote is lost; the stick in RUN when it is
pushed; the Pi (the follow-bot code) in RUN with the stick centred. So the remote is the input to the follow
bot already, through the firmware. Nothing extra runs on the Pi for it.

For **turtlesim**, the `honebot_remote_bridge` package turns the remote into turtle motion, from one of two
sources:

| `source:=` | Plugged into the Pi | What the turtle shows |
|---|---|---|
| `serial` (default) | the **remote** board | what the robot *would* do: same STOP/RUN rule, deadband, mixing and 2.4 m/s limit as the robot firmware, no ramps |
| `esp32` | the **robot** ESP32 (the remote talks to it by radio) | what the robot ESP32 *really* outputs, from its `S` line: kill switch, ramps and limits included. This is the real chain |

`follow:=true` also starts the follow-bot code (`honebot_follow`, turtle2) chasing turtle1. The remote drives
turtle1, which stands in for the skater.

## 0. Get the code onto the Pi and build it (Pi)

The Pi copy is the source of truth, so copy these changes there (commit and `git pull`, or edit there), then:

```bash
cd ~/honebot_ws
```
```bash
colcon build --symlink-install --packages-select honebot_remote_bridge honebot_hardware
```
```bash
source install/setup.bash
```

You should see `Finished <<< honebot_remote_bridge` and no errors. turtlesim needs a screen: run the launch
commands below on the Pi's HDMI console with `QT_QPA_PLATFORM=linuxfb` in front, as for the turtle-follow
demo (or from SSH with the `sg video` wrapper in CLAUDE.md).

## A. No hardware: fake remote (Pi, or any machine with ROS 2)

Checks the bridge and turtlesim before any board is involved. One command, from the repo root:
```bash
tests/test_esp32_remote.sh fake
```
It checks the setup, starts the launch, runs the automatic check below and stops everything again.

Or step by step. Terminal 1 (the bridge keeps retrying until the fake port exists):
```bash
QT_QPA_PLATFORM=linuxfb ros2 launch honebot_remote_bridge bridge_launch.py port:=/tmp/honebot_fake_remote
```
Terminal 2, automatic check, from the repo root:
```bash
python3 tests/test_remote_turtlesim.py
```
You should see six `PASS` lines and `6/6 checks passed`.

Or drive it by hand: run `python3 firmware/tests/sim_remote.py` in terminal 2 and type `run`, then `w`
(forward), `a` / `d` (turn), `c` (centre), `stop`. `--demo` drives a loop by itself.

## B. Remote board plugged into the Pi (source:=serial)

1. Flash the remote from the Mac (`firmware/tests/test_remote_turtle.sh <port>`), then let a script walk you
   through the stick and button checks (no ROS needed; it says PASS/FAIL per step):
   ```bash
   python3 tests/test_remote_basic.py /dev/cu.usbserial-0001
   ```
   (`--watch` instead just prints every line with what the robot would do.)
2. Plug the remote into the Pi's USB. With only that board plugged in it is `/dev/ttyUSB0` (check with
   `ls /dev/ttyUSB*`).
3. On the Pi (`tests/test_esp32_remote.sh serial /dev/ttyUSB0` does the same after checking the setup):
   ```bash
   QT_QPA_PLATFORM=linuxfb ros2 launch honebot_remote_bridge bridge_launch.py port:=/dev/ttyUSB0
   ```
4. Check, stick at rest at power-up:

| You do | Turtle | Log line |
|---|---|---|
| nothing (remote boots in STOP) | still | `Remote: stop` |
| push the stick in STOP | still | |
| let go, hold the button 1.5 s | still | `Remote: run` |
| stick away from you | forward | |
| stick left | turns left (counter-clockwise) | |
| click the button | stops at once | `Remote: stop` |
| unplug the remote | stops within 0.5 s | `Remote: lost` |

## C. The real chain: remote -> robot ESP32 -> Pi (source:=esp32)

This tests exactly what the robot will run, including the kill switch.

1. Flash both boards: robot firmware 2.2.1, remote 1.0.0 (`firmware/tests/test_remote_turtle.sh <remote_port>
   <robot_port>`). Keep `REMOTE_REQUIRED = true`.
2. Plug the **robot** ESP32 into the Pi (`/dev/ttyUSB0`). Power the remote from a power bank. Its LED should
   blink fast (robot answering).
3. On the Pi (stop the boot service first if it is installed, it holds the port: `sudo systemctl stop honebot`;
   `tests/test_esp32_remote.sh esp32 /dev/ttyUSB0` checks that and then runs the same launch):
   ```bash
   QT_QPA_PLATFORM=linuxfb ros2 launch honebot_remote_bridge bridge_launch.py source:=esp32 port:=/dev/ttyUSB0
   ```
4. Same checks as the table in B. Differences: the esp32_bridge log shows `Remote (kill switch): run/stop`
   and `Manual drive from the remote`; starts and stops are ramped like the real robot; switching the
   remote off stops the turtle within 0.5 s.

Watch the raw numbers in another terminal:
```bash
ros2 topic echo /esp32/outputs
```
`x` = left wheel, `y` = right wheel (-1..1), `z` = 1.0 while the ESP32 forces neutral.

## D. Remote drives the "skater", follow-bot code chases it

Add `follow:=true` to any of the launches above, for example:
```bash
QT_QPA_PLATFORM=linuxfb ros2 launch honebot_remote_bridge bridge_launch.py follow:=true
```
turtle2 (the follower, through the same simulated-UWB `/target` pipeline the robot uses) keeps
`follow_distance` (1.5 m) behind turtle1. Drive turtle1 around with the stick and watch it follow; switch the
remote to STOP and turtle1 stops, then turtle2 stops at its follow distance.

## Output format of the remote (USB serial, 115200, 5 lines a second)

```
state=RUN x=+0.50 y=-0.20 btn=0 robot=1 pi=1 failsafe=0 manual=1
```

| Field | Meaning |
|---|---|
| `state` | `STOP`, `RUN`, or `FAULT` (stick read wrong at power-up; RUN refused) |
| `x` | forward/back, -1..1; **+1 = stick pushed away from you** |
| `y` | left/right, -1..1; **+1 = stick pushed left** |
| `btn` | 1 while the button is pressed |
| `robot` | 1 = the remote hears the robot ESP32 (within 0.5 s) |
| `pi` | 1 = the robot ESP32 hears the Pi's heartbeat |
| `failsafe` | 1 = the robot ESP32 is forcing its outputs to neutral |
| `manual` | 1 = the robot is being driven by the stick |

The firmware prints `x`/`y` even in STOP, but sends no stick data over the radio then; the bridge ignores the
stick unless `state=RUN`, like the robot.

## Speeds

Full stick forward = wheel command 0.25 = about **2.4 m/s** on the robot. The launch's `scale:=0.5` halves that
for the 11 m turtlesim window (the turtle's path is unchanged). Turning on the spot with the stick fully left
is fast (about 7 rad/s on the robot, 3.7 on the turtle): the effective track width is still a guess until the
spin test.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Cannot open /dev/ttyUSB0` | wrong port (`ls /dev/ttyUSB*`), or the `honebot` service or another launch holds it |
| `/dev/esp32` points at the wrong board | the udev rule matches every CP2102 board; with two plugged in use `/dev/ttyUSB0`/`1` or `/dev/serial/by-path/...` |
| turtle never moves, log stays `Remote: stop` | hold the button a full 1.5 s with the stick **centred** (`state=FAULT`: stick held or loose at power-up, press EN) |
| forward drives backward / turns are mirrored | set `SWAP_XY`, `X_REVERSED`, `Y_REVERSED` in `esp32_remote.ino` so stick away = `x=+1.00` and stick left = `y=+1.00` |
| source:=esp32, turtle never moves | remote `robot=0`: `HONEBOT_PAIR_ID` / channel differ, or robot firmware older than 2.2.0; esp32_bridge log should show `ESP32 firmware version 2.2.1` |
| turtlesim window does not open | no display: use the HDMI console with `QT_QPA_PLATFORM=linuxfb` |

## Files

| File | What it is |
|---|---|
| `src/honebot_remote_bridge/` | the ROS 2 package: `remote_bridge` node, `bridge_launch.py`, `config/remote.yaml`, unit tests |
| `src/honebot_hardware/.../esp32_bridge.py` | publishes `/esp32/outputs` from the robot ESP32's `S` line |
| `firmware/tests/sim_remote.py` | fake remote on a pseudo-terminal (no ESP32 needed) |
| `tests/test_remote_turtlesim.py` | end-to-end check: fake remote -> bridge -> turtle, PASS/FAIL |
| `tests/test_esp32_remote.sh` | Pi: checks the setup, then launches `fake` (+ the check), `serial` or `esp32` mode |
| `tests/test_remote_basic.py` | no ROS: guided stick/button check of a remote on USB (`--watch`, `--sim`) |
| `firmware/tests/test_remote_turtle.sh` | flash the remote (and robot) from the Mac, then print the checks |
| `firmware/tests/quick_remote_test.md` | one-page checklist of this guide |
