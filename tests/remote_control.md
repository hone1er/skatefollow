# tests/: checks that need ROS 2 running (run by hand on the Pi)

These are not part of CI (CI runs the unit tests in each `src/honebot_*/test/` folder and
`firmware/tests/run_tests.sh`, which need no ROS and no hardware).

## test_remote_turtlesim.py

End-to-end check of the handheld-remote path with no ESP32: a fake remote (`firmware/tests/sim_remote.py`)
writes the remote board's serial lines into a pseudo-terminal, `honebot_remote_bridge` reads them, and the
script watches `/turtle1/pose`.

Terminal 1 (Pi HDMI console, after building `honebot_remote_bridge` and sourcing `install/setup.bash`):
```bash
QT_QPA_PLATFORM=linuxfb ros2 launch honebot_remote_bridge bridge_launch.py port:=/tmp/honebot_fake_remote
```
Terminal 2, from the repo root:
```bash
python3 tests/test_remote_turtlesim.py
```

It checks, in order: STOP with the stick pushed (no motion), RUN centred (no motion), RUN forward (drives
forward), RUN left (turns counter-clockwise), click (stops), remote silent (stops after the 0.5 s timeout).
Expected: six `PASS` lines and `6/6 checks passed`, exit code 0. Restart the launch between runs so the turtle
starts in the middle of the window.

The full guide, including the real boards: `firmware/tests/ESP32_REMOTE_TESTING.md`.
