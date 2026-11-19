# honebot_remote_bridge

Handheld ESP32 remote -> ROS 2 `Twist`, so the remote can drive a turtle in turtlesim the way it will drive
the robot. Full guide: `docs/remote_turtlesim.md`.

On the real robot this package is not in the driving path: the remote talks to the robot ESP32 by radio and
the firmware chooses between the stick and the Pi (the follow-bot code). This package only mirrors that.

## Node `remote_bridge`

| Parameter | Default | Meaning |
|---|---|---|
| `source` | `serial` | `serial`: read the remote board's USB printout on `port`, take the follower's commands on `cmd_vel_in`, and decide who drives with the robot firmware's rules (`Arbiter`). `esp32`: use `/esp32/outputs` from `esp32_bridge` (the robot ESP32's real wheel outputs; its firmware decided) |
| `manual_hold` / `cmd_timeout` | 0.8 / 0.5 | s before the follower drives again after the stick is released / follower command freshness (serial) |
| `port` / `baud` | `/dev/ttyUSB0` / 115200 | remote board's serial port (`source: serial`) |
| `remote_timeout` | 0.5 | s without a line / an output message -> stop |
| `rate_hz` | 20 | Twist output rate |
| `track_width`, `full_scale_mps` | 0.39, 9.6 | same as `esp32_bridge` and the firmware |
| `scale` | 1.0 (launch: 0.5) | slows the turtle down; path unchanged |

Publishes `cmd_vel_out` (`geometry_msgs/Twist`, zeros when nobody drives; the launch remaps it to
`/turtle1/cmd_vel`, or `/turtle2/cmd_vel` with `follow:=true`), `remote/state` (never | lost | stop | run) and
`remote/source` (none | pi | manual), both `std_msgs/String`. Subscribes `cmd_vel_in` (Twist, the follower;
serial source) or `esp32/outputs`, `esp32/manual`, `esp32/remote` (esp32 source).

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
ros2 launch honebot_remote_bridge bridge_launch.py follow:=true source:=esp32   # follow bot on turtle2, remote overrides
```
With `follow:=true`, turtle1 is the skater (drive it with `ros2 run turtlesim turtle_teleop_key`) and turtle2 the
robot: follower -> `/cmd_vel_raw` -> (serial: remote_bridge's Arbiter | esp32: safety_gate -> esp32_bridge ->
robot ESP32 firmware -> `/esp32/outputs`) -> `/turtle2/cmd_vel`.
Arguments: `source`, `port`, `follow` (false), `scale` (auto: 0.5, 1.0 with follow), `follow_distance` (1.5),
`max_linear` (2.0).
