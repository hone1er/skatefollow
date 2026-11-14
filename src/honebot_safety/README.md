# honebot_safety

The gate between whatever produces velocity commands and the motors. It is the one piece of software that
every motion passes through, so it is kept small, pure and heavily tested.

```
/cmd_vel_raw [Twist]  ──┐
/ultrasonic/range     ──┼──► safety_gate ──► /cmd_vel [Twist, 50 Hz]
/esp32/link_ok        ──┘         └─────────► /safety/state [String]
```

## Files

| File | Role |
|---|---|
| `honebot_safety/gate.py` | Pure logic: `GateParams`, `slow_factor()`, `gate_command()`. No ROS. |
| `honebot_safety/safety_gate.py` | Node `safety_gate`: stores last message + stamp per input, 50 Hz timer calls `gate_command`, logs transitions. |
| `launch/safety_gate.launch.py` | Starts the node with `stop_distance` / `slow_distance` arguments. |
| `test/test_gate.py` | 12 tests, one or more per rule. |

## The rules (first match wins)

| # | Condition | State | Output |
|---|---|---|---|
| 1 | link flag never seen, older than `range_timeout`, or false | `LINK_DOWN` | 0, 0 |
| 2 | range never seen or older than `range_timeout` | `RANGE_STALE` | 0, 0 |
| 3 | range is NaN or ≤ 0 | `RANGE_INVALID` | 0, 0 |
| 4 | command never seen or older than `cmd_timeout` | `CMD_STALE` | 0, 0 |
| 5 | command is reverse or zero | `OK` | pass through (forward sensor is irrelevant) |
| 6 | range ≤ `stop_distance` | `OBSTACLE` | linear 0, angular kept |
| 7 | range < `slow_distance` | `SLOW` | linear × linear taper, angular kept |
| 8 | otherwise | `OK` | pass through |

`+inf` range means "nothing in range" (REP 117) and is clear. The taper in rule 7 is
`(range − stop) / (slow − stop)`, so speed reaches zero exactly at `stop_distance` with no step.

Defaults: `stop_distance` 0.4 m, `slow_distance` 1.0 m, `range_timeout` 0.3 s, `cmd_timeout` 0.5 s,
`rate_hz` 50. All live-settable with `ros2 param set /safety_gate …`.

## Why it is built this way

- **Fixed-rate output instead of pass-through.** Publishing at 50 Hz regardless of input means the next
  layer down (`esp32_bridge`) can treat "no `/cmd_vel` for 0.3 s" as "the gate is dead" and go neutral.
  A pass-through gate would be silent when its input is silent, which is indistinguishable from death.
- **It only subtracts.** `gate_command` returns either the input, a scaled-down input, or zeros. It never
  adds motion, so any controller (follower, gamepad, a future planner) can sit in front of it safely.
- **Liveness before geometry.** Rules 1–4 are about whether the inputs can be trusted at all; only then are
  the numbers used. A stale-but-plausible range must not be allowed to say "clear".
- **Reverse and steering survive an obstacle.** There is one forward-facing ultrasonic. Blocking reverse
  would trap the robot against whatever it stopped for.
- **One timeout for range and link.** Both arrive on the same `R` line from the ESP32, so one `range_timeout`
  covers both; a separate link timeout would be redundant.
- **State string on a topic.** `/safety/state` is the single thing to watch in Foxglove to know *why* the
  robot is not moving. The node logs only transitions so the console stays readable.

## Running and testing

```bash
ros2 launch honebot_safety safety_gate.launch.py          # needs /ultrasonic/range and /esp32/link_ok to pass anything
colcon test --packages-select honebot_safety && colcon test-result --verbose
```
Bench procedure (with `bringup.launch.py` up): `ros2 topic pub /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 1.0}}"`,
watch `ros2 topic echo /safety/state` and `/cmd_vel`; cover the sensor, raise `stop_distance`, kill the publisher,
kill the bridge. Each should produce the expected state. Known-good results are in `CLAUDE.md` §6.
