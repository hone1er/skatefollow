# tests/: checks for the handheld remote, run by hand

These are not part of CI (CI runs the unit tests in each `src/honebot_*/test/` folder and
`firmware/tests/run_tests.sh`, which need no ROS and no hardware). Full guide:
`docs/remote_turtlesim.md`.

| Script | Where | Needs |
|---|---|---|
| `test_remote_basic.py` | Mac or Pi | Python 3 + pyserial; the remote board on USB (or `--sim`: nothing) |
| `test_esp32_remote.sh` | Pi | ROS 2 + `honebot_remote_bridge` built; the HDMI console for turtlesim |
| `test_remote_turtlesim.py` | Pi | the bridge launch running (the shell script starts it for you in `fake` mode) |

## test_remote_basic.py: is the remote itself right? (no ROS)

```bash
python3 tests/test_remote_basic.py /dev/cu.usbserial-0001
```
Asks you to centre, push away, pull back, push left and right, then hold the button for RUN and click for
STOP, checking each step from the serial lines. Expected: `9/9 steps passed`. `--watch` just prints each line
with the wheel commands and speed the robot would use. `--sim` runs the same steps against the fake remote
(no hardware) to check the script itself.

## test_esp32_remote.sh: turtlesim with the remote (Pi)

```bash
tests/test_esp32_remote.sh fake
```
```bash
tests/test_esp32_remote.sh serial /dev/ttyUSB0
```
```bash
tests/test_esp32_remote.sh esp32 /dev/ttyUSB0
```
Checks ROS, the build, the port and the `honebot` service first, then launches. `fake` also runs
`test_remote_turtlesim.py` and stops everything afterwards (exit code 0 = all passed). Add `follow` as the
last word to test the follow bot (turtle2) with the remote overriding it; turtle1 is the skater, driven from a
second terminal with `ros2 run turtlesim turtle_teleop_key` (in `fake follow` the script drives it):
```bash
tests/test_esp32_remote.sh fake follow
```
```bash
tests/test_esp32_remote.sh esp32 /dev/ttyUSB0 follow
```

## test_remote_turtlesim.py: end-to-end PASS/FAIL (Pi)

A fake remote (`tests/sim_remote.py`) writes the remote board's lines into a pseudo-terminal,
`honebot_remote_bridge` reads them, and the script watches `/turtle1/pose`. By hand:

Terminal 1:
```bash
QT_QPA_PLATFORM=linuxfb ros2 launch honebot_remote_bridge bridge_launch.py port:=/tmp/honebot_fake_remote
```
Terminal 2, from the repo root:
```bash
python3 tests/test_remote_turtlesim.py
```

It checks, in order: STOP with the stick pushed (no motion), RUN centred (no motion), RUN forward (drives
forward), RUN left (turns counter-clockwise), click (stops), remote silent (stops after the 0.5 s timeout).
Expected: six `PASS` lines and `6/6 checks passed`. Restart the launch between runs so the turtle starts in the
middle of the window.

`--follow` (launch with `follow:=true`) checks the follow bot with the remote override instead, 7 checks: STOP
holds the robot although the follower wants to drive; RUN centred lets the follower close in; it keeps up when
the skater drives away; pulling the stick back overrides it; releasing the stick hands back to the follower
after the hold; a click stops it; a silent remote stops it. Expected: `7/7 checks passed`.
