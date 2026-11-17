# honebot_uwb

Turns ultra-wideband (UWB) ranging from a serial board into the skater's position in the robot frame.
No hardware exists yet; the node is written against the two line formats the board's firmware must print.

```
UWB board ──serial "U,r_left,r_right" or "A,range,bearing_deg"──► uwb_node ──► /target [PointStamped, base_link]
                                                                           └──► /uwb/ranges [Float32MultiArray] (debug)
```

## Files

| File | Role |
|---|---|
| `honebot_uwb/localize.py` | Pure: `UwbGeometry`, `two_anchor_position()`, `aoa_position()`, `RangeFilter`, `to_distance_bearing()`. |
| `honebot_uwb/uwb_node.py` | Node `uwb_node`: serial reader (same pattern as `esp32_bridge`), per-anchor filters, publishes. |
| `launch/uwb.launch.py` | `port`, `baseline` arguments. |
| `test/test_localize.py` | 11 tests. |

## Geometry

Robot frame: x forward, y left, origin `base_link`. Two anchors sit `baseline` apart, symmetric about the
centre line, `anchor_x` ahead of `base_link`:

```
left anchor  (anchor_x, +baseline/2)        r_r² − r_l² = 2·b·y   →  y
right anchor (anchor_x, −baseline/2)        x = √(r_l² − (y − b/2)²)   (taken positive = "in front")
```

Two ranges fix the tag up to a front/back mirror. The code assumes in front, which holds because the
robot's job is to stay behind the skater. A third anchor would remove the ambiguity. Ranges whose
difference exceeds `baseline + tolerance` are geometrically impossible and are rejected.

An angle-of-arrival anchor (the recommended Makerfabs MaUWB_AOA) reports range and bearing from one
module; `aoa_position` just converts polar to cartesian. `bearing_sign` flips the convention if the board
reports positive = right.

## Filtering

`RangeFilter` per anchor: reject out-of-range or non-finite samples (keep the previous estimate), median of
the last 3 to kill single-sample spikes (UWB multipath produces these), then an EMA with `filter_alpha`.
Median before EMA matters: an EMA alone would let a 10 m spike pull the estimate for several samples.

## Why it is built this way

- **Output is a point, not range + bearing.** `PointStamped` in `base_link` is what `pose_to_target`
  publishes in simulation and what the follower consumes, so the follower is identical in sim and on the
  robot. The follower does the `hypot`/`atan2` itself.
- **No fix → publish nothing.** There is no "lost" message. The follower's `target_timeout` already stops
  the robot on silence, so silence *is* the lost signal and there is one fewer code path to get wrong.
- **Two line formats, one node.** Which UWB board ends up on the robot is still open; either prints one
  of two trivial lines and the rest of the stack does not change.
- **Serial handling copied from the bridge**, not shared: the two are small, and keeping them independent
  avoids a shared module that both safety-relevant nodes would depend on.

## Running and testing

```bash
ros2 launch honebot_uwb uwb.launch.py port:=/dev/ttyUSB1 baseline:=0.30
ros2 topic echo /target
colcon test --packages-select honebot_uwb && colcon test-result --verbose
```
The node logs fixes/s and rejected samples every 5 s. Without hardware you can test the pipeline by
`echo "A,3.0,15" > /dev/ttyUSB1`-style injection through a loopback or a USB-serial adapter.

## Dropouts (fixed 2026-11-17)

A sample that is no fix, out of range or outside the anchor's usable angle (`max_bearing_deg`) publishes
**nothing**: the follower's `target_timeout` then stops the robot. (`RangeFilter` used to return its last value,
which kept a lost tag looking alive.) AoA bearings are filtered too (`bearing_alpha`).
