# honebot_remote_bridge

Handheld ESP32 remote -> ROS 2 `Twist`, so the remote can drive a turtle in turtlesim the way it will drive
the robot. Full guide: `firmware/tests/ESP32_REMOTE_TESTING.md`.

On the real robot this package is not in the driving path: the remote talks to the robot ESP32 by radio and
the firmware chooses between the stick and the Pi (the follow-bot code). This package only mirrors that.

## Node `remote_bridge`

| Parameter | Default | Meaning |
|---|---|---|
| `source` | `serial` | `serial`: read the remote board's USB printout on `port` and apply the robot firmware's rules. `esp32`: use `/esp32/outputs` from `esp32_bridge` (the robot ESP32's real wheel outputs) |
| `port` / `baud` | `/dev/ttyUSB0` / 115200 | remote board's serial port (`source: serial`) |
| `remote_timeout` | 0.5 | s without a line / an output message -> stop |
| `rate_hz` | 20 | Twist output rate |
| `track_width`, `full_scale_mps` | 0.39, 9.6 | same as `esp32_bridge` and the firmware |
| `scale` | 1.0 (launch: 0.5) | slows the turtle down; path unchanged |

Publishes `cmd_vel_out` (`geometry_msgs/Twist`, zeros when stopped or stale; the launch remaps it to
`/turtle1/cmd_vel`) and `remote/state` (`std_msgs/String`: run | stop | fault | lost, serial source).
Subscribes `esp32/outputs` (`geometry_msgs/Vector3`, x = left, y = right, z = failsafe) with `source: esp32`.

Pure logic (line parser, firmware stick rules, wheels -> Twist) is in `remote_logic.py`, tested in
`test/test_remote_logic.py` (`python3 -m pytest -q test` from this folder; no ROS needed).

## Launch

```bash
ros2 launch honebot_remote_bridge bridge_launch.py                      # remote board on /dev/ttyUSB0
```
```bash
ros2 launch honebot_remote_bridge bridge_launch.py source:=esp32        # robot ESP32 on /dev/ttyUSB0
```
```bash
ros2 launch honebot_remote_bridge bridge_launch.py follow:=true         # + follower turtle2 chasing turtle1
```
Arguments: `source`, `port`, `scale` (0.5), `follow` (false), `follow_distance` (1.5).
