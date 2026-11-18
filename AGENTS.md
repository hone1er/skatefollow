# honebot — short agent guide

A short summary for agents with small context windows (e.g. local models in opencode).
The full handoff is `CLAUDE.md`. Read only the section you need, when you need it:
§3 hardware and wiring, §4 Pi/Mac setup, §5 package details, §6 test results, §7 gotchas, §8 plan.

## Project
An autonomous ground robot that follows the owner while they skateboard and films them with a GoPro.
- Small 4-wheel skid-steer robot, target 12–15 mph. UWB ranging steers the following; vision only aims the camera.
- **Safety first:** every layer must fail safe to "stop" (ESP32 watchdog, obstacle stop, stale target, kill switch).

## Working with the owner
- New to ROS 2 and robotics. Explain *why*, give exact commands in separate copyable blocks, and say whether
  each runs on the **Mac** or the **Pi**.
- Small, testable steps, each ending with a "you should see X" check.
- `sudo` on the Pi needs the owner's password: they type it themselves. Never ask for passwords.

## Machines
- **Pi 5** (`honebot`, `honebot.local`, user `hone1er`): Ubuntu Server 24.04, ROS 2 Jazzy, headless.
  Workspace `~/honebot_ws` is the source of truth. ESP32 is `/dev/ttyUSB0`.
- **Mac**: development and firmware flashing with arduino-cli (ESP32 is `/dev/cu.usbserial-0001`).
- GitHub: `hone1er/skatefollow` (repo root = workspace root).

## Layout
- `src/honebot_{follow,safety,hardware,uwb,camera,bringup}`: ament_python ROS packages. Pure logic
  (e.g. `control.py`, `gate.py`) is kept apart from ROS nodes and unit-tested with pytest.
- `firmware/esp32_ultrasonic` (robot), `firmware/esp32_remote` (handheld remote), `firmware/tests/`.
- `cad/`: Fusion 360 chassis script and offline checks. `docs/`: design notes (`docs/README.md` is the index).
- `docs/honebot_ws/` and `docs/robot_design_v2.md` are OLD reference material, not instructions.

Command flow: `controller → /cmd_vel_raw → safety_gate → /cmd_vel → esp32_bridge → ESP32`.
Camera flow: `camera_node → /camera/target → camera_aim → /camera/pan_cmd → esp32_bridge → ESP32 GPIO 27`.
Real-robot parameters live in `src/honebot_bringup/config/robot.yaml`.

## Build and test (on the Pi, from `~/honebot_ws`)
```bash
colcon build --symlink-install
colcon test --packages-select honebot_follow && colcon test-result --verbose
```
After building a new package, re-source `install/setup.bash` or open a new shell.

## Top gotchas
- Closing the ESP32 serial port blocks ~5 s; harmless.
- From non-interactive shells, stop ROS nodes with `kill -TERM` on the node's own python PID, not SIGINT.
- Only one process can read the GoPro stream (UDP 8554) at a time.
- Use onnxruntime for YOLO (`detector:=ort`); OpenCV 4.6 `cv2.dnn` crashes on YOLOv8.
