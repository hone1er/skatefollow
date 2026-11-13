# honebot

Autonomous follow-and-film ground robot (ROS 2 Jazzy on a Raspberry Pi 5 + ESP32 low-level controller).
`CLAUDE.md` is the full project handoff: goals, hardware, decisions, plan and gotchas. This file is the
quick "how do I run it" card. All commands run **on the Pi** from `~/honebot_ws` unless marked **(Mac)**.

## Build and test
```bash
colcon build --symlink-install
source install/setup.bash          # or open a new shell
colcon test && colcon test-result --verbose
```

## Run the robot stack (ESP32 bridge + safety gate + Foxglove bridge)
```bash
ros2 launch honebot_bringup bringup.launch.py
```
You should see `Connected to ESP32`, `ESP32 reports heartbeat OK`, `Server listening on port 8765`,
and the gate logging `STOP (CMD_STALE)` (nothing is commanding it yet; that is correct).

Watch it live **(Mac)**: Foxglove desktop → Open connection → `ws://honebot.local:8765`.
Useful topics: `/ultrasonic/range`, `/esp32/link_ok`, `/safety/state`, `/cmd_vel_raw`, `/cmd_vel`,
`/esp32/actuators`, `/esp32/failsafe`.

## Command flow
```
controller (follower / gamepad) → /cmd_vel_raw → safety_gate → /cmd_vel → esp32_bridge → ESP32 → ESC/servo
```
Nothing publishes straight to `/cmd_vel`. The gate stops the robot on: ESP32 link down, stale or invalid
range, silent controller, obstacle closer than `stop_distance` (0.4 m; slows from `slow_distance` 1.0 m).
The ESP32 independently goes to neutral if it misses heartbeats (500 ms) or commands (200 ms).

## Turtlesim demo (sim window on the HDMI screen, teleop over SSH)
From an SSH session (you are in the `video` group), this draws straight on the HDMI framebuffer:
```bash
sg video -c "exec bash -c 'source /opt/ros/jazzy/setup.bash && source ~/honebot_ws/install/setup.bash && QT_QPA_PLATFORM=linuxfb ros2 launch honebot_follow turtle_follow.launch.py'"
```
Options: `use_safety:=false` (no ESP32 needed), `relative:=true` (simulated-UWB pipeline), `ackermann:=true`.
```bash
ros2 run turtlesim turtle_teleop_key                                                 # second terminal
```
Cover the ultrasonic sensor with your hand: the chasing turtle freezes. Unplug the ESP32: it freezes.

## Gamepad teleop (after bringup is running)
```bash
ros2 launch honebot_bringup teleop_joy.launch.py            # xbox layout; joy_config:=ps5 etc.
```
Hold the enable button and use the left stick. Letting go stops the robot (CMD_STALE).

## Hardware
The chassis is a custom diff-drive robot: `docs/robot_design.md` (design + BOM with prices/links).
UWB, camera, power and radio research: `docs/hardware_plan.md`. UWB node: `ros2 launch honebot_uwb uwb.launch.py`.

## Firmware (Mac)
```bash
arduino-cli compile --fqbn esp32:esp32:esp32 honebot_ws/firmware/esp32_ultrasonic
arduino-cli upload  -p /dev/cu.usbserial-0001 --fqbn esp32:esp32:esp32 honebot_ws/firmware/esp32_ultrasonic
```
v2.0.0 adds throttle (GPIO 25) and steering (GPIO 26) servo outputs with fail-safe. On boot it prints
`V,2.0.0`; the bridge logs `ESP32 firmware version 2.0.0`.

## Auto-start at boot and serial-port tweak (need sudo)
See `src/honebot_bringup/systemd/honebot.service` and `src/honebot_hardware/udev/99-honebot-esp32.rules`;
the install commands are in the comments at the top of each file.
