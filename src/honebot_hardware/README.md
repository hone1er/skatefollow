# honebot_hardware

The only node that talks to the ESP32. Everything the robot physically does, and everything it senses
through the ESP32, passes through `esp32_bridge`.

```
/cmd_vel [Twist] ───────► esp32_bridge ──C,a,b (50 Hz)──► ESP32
/camera/pan_cmd [Float32] ─►    │      ──P,pan (20 Hz)──►
                                │      ──H (10 Hz)───────►
                                │      ◄─R,dist,link─────  → /ultrasonic/range [Range], /esp32/link_ok [Bool]
                                │      ◄─S,fs,a,b────────  → /esp32/failsafe [Bool]
                                │      ◄─V,version───────  → log line
                                └──────────────────────────→ /esp32/actuators [Vector3: x=a, y=b, z=pan]
```

## Files

| File | Role |
|---|---|
| `honebot_hardware/esp32_bridge.py` | Node `esp32_bridge`: serial I/O, line parsing, timers, reconnect. |
| `honebot_hardware/diffdrive.py` | Pure: `twist_to_diffdrive(v, ω, DiffDriveParams) -> (left, right)` in −1..1. |
| `honebot_hardware/ackermann.py` | Pure: `twist_to_ackermann(v, ω, AckermannParams) -> (throttle, steer)` in −1..1. |
| `launch/esp32.launch.py` | Starts the bridge with a `port` argument. |
| `udev/99-honebot-esp32.rules` | Optional: `/dev/esp32` symlink and the `closing_wait` fix (needs sudo + `setserial`). |
| `test/test_diffdrive.py`, `test/test_ackermann.py` | 7 + 10 tests. |

## How the node is structured

Five timers, no threads:

| Timer | Rate | Does |
|---|---|---|
| `_poll` | 100 Hz | non-blocking read of whatever bytes are waiting; splits on `\n`; dispatches complete lines |
| `_heartbeat` | 10 Hz | writes `H` |
| `_send_cmd` | 50 Hz | converts the last `Twist` (if younger than `cmd_timeout` 0.3 s) or neutral, writes `C,a,b`, publishes `/esp32/actuators` |
| `_send_pan` | 20 Hz | writes `P,pan` once any pan command has ever arrived |
| `_ensure_open` | 1 Hz | reopens the port if it was lost |

Any write or read error drops the port and the 1 Hz timer reopens it, so an unplug/replug recovers by
itself. Lines that are not `R`, `S` or `V` (boot messages, a newer firmware's extra lines) are counted and
ignored at debug level. On exit the node sends one neutral `C,0,0` and closes the port.

Parameters: `port`, `baud`, `frame_id`, `heartbeat_hz`, `cmd_rate_hz`, `cmd_timeout`, `pan_rate_hz`,
`send_commands` (false = sensor-only), `drive_mode` (`diff` | `ackermann`), and the kinematic numbers
`track_width`, `max_wheel_speed`, `left_reversed`, `right_reversed` (diff) or `wheelbase`,
`max_steer_angle`, `max_speed`, `min_speed_for_steer`, `steer_reversed`, `throttle_reversed` (ackermann).
**All kinematic numbers are placeholders until the drivetrain exists.**

## The two converters

**Differential drive** (the chosen chassis):
```
left  = (v − ω·track/2) / max_wheel_speed
right = (v + ω·track/2) / max_wheel_speed
if max(|left|, |right|) > 1: divide both by it
```
Scaling both wheels together when one saturates preserves the commanded turn radius; clipping only the
saturated wheel would make the robot veer toward the other side at high speed.

**Ackermann** (kept for an RC-car variant): `δ = atan(wheelbase · ω / v)`, with `|v|` floored at
`min_speed_for_steer` so near-zero speed does not request infinite steering, and the sign of `v` kept so
reversing steers the correct way. Normalised by `max_steer_angle`.

## Why it is built this way

- **The firmware knows nothing about the chassis.** Both converters produce two −1..1 numbers and the
  ESP32 maps them to pulse widths. Switching chassis is a `drive_mode` parameter, not a reflash.
- **Conversion inside the bridge, not a separate node.** A kinematics node would need a custom message
  (or an abuse of `Twist`) between it and the bridge. Keeping `Twist` as the last ROS-level contract keeps
  every upstream node and tool chassis-agnostic.
- **Commands are sent every cycle, including neutral.** The ESP32's 200 ms command watchdog then also
  detects a dead bridge, and `cmd_timeout` here (0.3 s) is shorter than the gate's own (0.5 s) so a dead
  gate shows up as neutral from the bridge before anything upstream notices.
- **`+inf` for "no echo".** The firmware sends `-1`; the bridge publishes `math.inf` per REP 117 so the gate
  can tell "clear" from "broken" (NaN / ≤ 0).
- **Non-blocking serial on a timer** rather than a reader thread keeps the node single-threaded: no locks,
  and `rclpy` callbacks and serial handling cannot race.
- **`/esp32/actuators` and `/esp32/failsafe` exist for tuning and trust.** The first shows what the Pi
  *sent*; the `S` line (via the second) shows what the ESP32 is *actually outputting*. A mismatch means the
  firmware's caps or fail-safe are active.

## Running and testing

```bash
ros2 launch honebot_hardware esp32.launch.py [port:=/dev/ttyACM0]
ros2 topic hz /ultrasonic/range            # ~16.7 Hz
ros2 topic echo /esp32/link_ok             # true within ~0.5 s
colcon test --packages-select honebot_hardware && colcon test-result --verbose
```
Shutdown takes ~5 s because this CP2102 makes `close()` wait for a USB timeout; the udev rule fixes it.
