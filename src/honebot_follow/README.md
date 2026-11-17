# honebot_follow

The follow-me controller, plus the pieces that let it run against turtlesim before any field hardware exists.

```
turtlesim:   /turtle1/pose, /turtle2/pose ──► follower (pose mode) ──► /turtle2/cmd_vel
             /turtle1/pose, /turtle2/pose ──► pose_to_target ──► /target ──► follower (relative mode) ──► /cmd_vel_raw
robot:       uwb_node ─────────────────────────────────────────► /target ──► follower (relative mode) ──► /cmd_vel_raw
```

## Files

| File | Role |
|---|---|
| `honebot_follow/control.py` | Pure control law: `FollowGains`, `follow_command_relative(distance, bearing, gains)`, `relative_target()` for the pose form, `follow_command()` wrapper. |
| `honebot_follow/follower.py` | Node `follower`. Two input modes, one control law, one stale-target stop. |
| `honebot_follow/pose_to_target.py` | Node `pose_to_target`: simulated UWB. World poses → leader position in the follower's frame as `PointStamped`. |
| `launch/turtle_follow.launch.py` | turtlesim + follower, with `use_safety`, `relative`, `ackermann` switches. |
| `test/test_control.py` | 14 tests for the control law. |

## The control law

Inputs are the target's `distance` and `bearing` in the robot frame (0 = dead ahead, + = left).

```
angular = clamp(kp_angular · bearing)
linear  = clamp(kp_linear · (distance − follow_distance) · max(0, cos(bearing)), 0, max_linear)
```

- `cos(bearing)` scaling: slow down while turning; produce *no* forward motion toward a target that is
  beside or behind. The robot turns first, then drives.
- `linear ≥ 0` always: a robot chasing a skater should never reverse into whoever is behind it. Inside
  `follow_distance` it stops and waits; very close (< 0.2 m) it also stops turning, because the bearing of
  something on top of you is meaningless.
- `ackermann=True`: a car cannot rotate in place, so when the target is far but off to the side the
  robot creeps at `creep_speed` with full steer rather than sitting still. Off by default now that the
  chassis is differential drive; kept for an RC-car variant.

Gains (`follow_distance`, `kp_linear`, `kp_angular`, `max_linear`, `max_angular`, `ackermann`,
`creep_speed`) are ROS parameters and live-settable: `ros2 param set /follower follow_distance 3.0`.

## The node

`follower` picks its mode from `target_topic`:

- **empty (turtlesim):** subscribes `/<leader>/pose` and `/<follower>/pose`, converts to
  `(distance, bearing)` in the leader callback, publishes `/<follower>/cmd_vel`. Can spawn the follower
  turtle via `/spawn` (`spawn_follower`, default false so the real robot never waits for a sim service).
- **set (relative, the real one):** subscribes a `PointStamped` in the robot frame, publishes
  `cmd_vel_topic` (default `cmd_vel_raw`, i.e. into the safety gate).

Either way a 20 Hz timer runs the law and **publishes zero when the target is older than
`target_timeout`** (0.5 s). Losing the skater is therefore a stop, not a continuation of the last command.

## Why it is built this way

- **Relative-frame control law** so the same function works whether the target comes from two world poses
  (turtlesim), two UWB ranges, or an angle-of-arrival module. The pose form is a thin wrapper.
- **`pose_to_target` exists only to make simulation honest.** It publishes exactly the message `uwb_node`
  will publish, so `relative:=true` in turtlesim exercises the real pipeline end to end: target → follower
  → gate → motors (turtle).
- **Proportional control, no integral term.** The skater moves unpredictably; an integrator would wind up
  during every stop and overshoot on restart. Tuning is expected in the field via live parameters.

## Running and testing

```bash
# on the HDMI console, or via sg video from SSH (see the top-level README)
ros2 launch honebot_follow turtle_follow.launch.py                       # pose mode through the gate
ros2 launch honebot_follow turtle_follow.launch.py use_safety:=false     # no ESP32 needed
ros2 launch honebot_follow turtle_follow.launch.py relative:=true        # the UWB-style pipeline
ros2 run turtlesim turtle_teleop_key                                      # second terminal: drive the leader
colcon test --packages-select honebot_follow && colcon test-result --verbose
```

## Real-robot options

All off by default; the robot's values are in `honebot_bringup/config/robot.yaml` and the follower is started by
`honebot_bringup/launch/film.launch.py`: speed-dependent gap (`brake_decel`, `time_headway`), arcs instead of
pivots (`min_turn_radius`, `arc_speed`, `pivot_bearing`, `pivot_rate`), cornering cap (`max_lat_accel`), and
`follow_bearing` to hold the skater off to one side. See the comments in `control.py` and the simulations in
`test/test_control.py`.
