# honebot_bringup

Top-level launch files and the boot-time service. No code of its own (the `console_scripts` list is empty).

| File | Starts | Notes |
|---|---|---|
| `launch/bringup.launch.py` | `esp32_bridge` + `safety_gate` + `foxglove_bridge` (+ `camera.launch.py` with `camera:=true`) | **Never starts a controller.** Args: `port`, `stop_distance`, `slow_distance`, `foxglove`, `camera`, `camera_source`, `record`, `model_path`. |
| `launch/teleop_joy.launch.py` | `joy` + `teleop_twist_joy` from the stock ROS packages | Output remapped to `/cmd_vel_raw`. Args: `joy_config` (xbox, ps5, …), `joy_dev`. |
| `launch/record.launch.py` | `ros2 bag record -a` | Into `~/bags/honebot_<timestamp>` or `out:=`. |
| `systemd/honebot.service` | `bringup.launch.py` at boot as the owner's user | Install commands in the file header (sudo). `KillSignal=SIGINT` so nodes exit cleanly. |

## Why

- **Safety layer first, controllers second.** Bringup brings up everything that can *stop* the robot and
  nothing that can *move* it. Controllers (follower, gamepad) are separate launches run afterwards, so the
  gate and the bridge are always already watching before a `Twist` can appear on `/cmd_vel_raw`.
- **Gamepad is a dead-man switch by construction.** `teleop_twist_joy` only publishes while the enable
  button is held; releasing it makes the gate report `CMD_STALE` and stop. No extra code was needed.
- **Stock packages where they exist.** `joy`, `teleop_twist_joy`, `foxglove_bridge` and `rosbag2` were
  already installed with ROS; this package just wires them. No GUI runs on the Pi: Foxglove on the Mac over
  WebSocket (`ws://honebot.local:8765`) is the dashboard.
- **Launch includes, not copies.** `bringup.launch.py` includes each package's own launch file and
  forwards arguments, so a package's defaults live in one place.

## Running

```bash
ros2 launch honebot_bringup bringup.launch.py [camera:=true record:=true]
ros2 launch honebot_bringup teleop_joy.launch.py            # second terminal
ros2 launch honebot_bringup record.launch.py                # third terminal, Ctrl-C to finalise the bag
journalctl -u honebot -f                                    # once the service is installed
```
`colcon test` reports this package as failed only because it has no tests.

## Real robot: `robot.launch.py` and `film.launch.py`

`config/robot.yaml` holds every real-robot parameter. `robot.launch.py` (bridge + gate + Foxglove) is the safety
layer that `honebot.service` starts at boot. `film.launch.py` (UWB + follower + camera + a bag) is started by hand
and is the only thing that makes the robot move by itself. `bringup.launch.py` remains the bench version with
default parameters. Test plan: `docs/first_drive.md`.
