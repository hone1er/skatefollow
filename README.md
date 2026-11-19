# honebot

Autonomous follow-and-film ground robot: it follows the owner while they skateboard and keeps a GoPro
pointed at them. ROS 2 Jazzy on a Raspberry Pi 5 makes the decisions; an ESP32 does the real-time I/O and
is the last line of defence if the Pi misbehaves.

Where to read what:

| You want to… | Read |
|---|---|
| Understand how the code is put together and why | **this file, sections 1–9** |
| Run something right now | **section 10 (run card)** |
| Know the hardware, history, gotchas and the phase plan | `CLAUDE.md` (project handoff) |
| Details of one package | the `README.md` inside each `src/honebot_*/` and `firmware/esp32_ultrasonic/` |
| Chassis, electronics and camera decisions with prices | `docs/robot_design.md`, `docs/hardware_plan.md`, `docs/camera_plan.md` |

All commands run **on the Pi** from `~/honebot_ws` unless marked **(Mac)**.

---

## 1. The system in one picture

```
            skater                                  robot
   ┌──────────────────┐        ┌──────────────────────────────────────────────────────────┐
   │ UWB tag (planned) │ radio  │  UWB anchor ──UART──┐                                     │
   └──────────────────┘ ─────► │                     ▼                                     │
                                │   Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy)              │
   GoPro ──USB (network)──────► │   ┌────────────────────────────────────────────────┐    │
                                │   │ perception → controller → safety gate → bridge │    │
                                │   └────────────────────────┬───────────────────────┘    │
                                │                            │ USB serial, ASCII lines     │
                                │   ┌────────────────────────▼───────────────────────┐    │
                                │   │ ESP32: ultrasonic ping, heartbeat watchdog,     │    │
   gamepad / Foxglove ◄─Wi-Fi─► │   │ fail-safe, 3 × 50 Hz servo-pulse outputs        │    │
   (Mac)                        │   └───┬────────────────┬────────────────┬───────────┘    │
                                │       ▼                ▼                ▼                │
                                │   left ESC         right ESC       camera pan servo       │
                                └──────────────────────────────────────────────────────────┘
```

Two computers, with a deliberate split of responsibilities:

- **The Pi** runs everything that needs maths, state or a network: localisation, the follow controller,
  the safety gate, the camera pipeline, logging, remote monitoring. It is a normal Linux box and is
  allowed to be slow or to crash.
- **The ESP32** runs everything that needs microsecond timing or must keep working when the Pi does not:
  the HC-SR04 echo measurement, the 50 Hz ESC/servo pulses, and a watchdog that forces the motors to
  neutral the moment the Pi stops talking. It holds no state beyond "the last command" and does no
  decision-making.

The two talk over a **plain ASCII line protocol** on USB serial (section 6). That was chosen over
micro-ROS because it can be debugged with `cat /dev/ttyUSB0`, unit-tested with string comparisons, and
needs nothing installed on the microcontroller.

---

## 2. ROS 2 in two minutes (enough for this repo)

If you know ROS 2, skip this. If not, these are the only concepts the code uses:

- A **node** is a process. Ours are Python classes extending `rclpy.node.Node`, one per file, started by
  a `main()` registered as a console script in the package's `setup.py`.
- Nodes talk through named **topics** carrying typed **messages** (`geometry_msgs/Twist` is a velocity
  command, `sensor_msgs/Range` a distance reading, and so on). Publishing is fire-and-forget; any number
  of nodes may subscribe. There are no return values. Nodes never call each other directly.
- A **parameter** is a named, typed config value a node declares with a default. It can be set at launch
  or changed live with `ros2 param set /node name value`.
- A **launch file** (`launch/*.launch.py`) starts several nodes with parameters and **remappings**
  (renaming a topic for one node, so the same node can be wired differently in sim and on the robot).
- A **package** (`src/honebot_*`) is a build unit: `package.xml` declares dependencies, `setup.py`
  declares the executables and which data files (launch, udev, netplan) get installed.
- `colcon build --symlink-install` builds the workspace into `install/`; `source install/setup.bash`
  makes the packages visible to the `ros2` CLI. With symlink install, Python edits take effect on the
  next run without rebuilding.

Useful CLI while reading code: `ros2 topic list`, `ros2 topic echo /safety/state`, `ros2 topic hz /cmd_vel`,
`ros2 param list /follower`, `ros2 node info /esp32_bridge`.

---

## 3. Node graph

Every box is a process; every arrow is a topic (message type in brackets). Default topic names are shown;
launch files remap some of them for turtlesim.

```
  INPUTS (one of)                                      COMMAND PIPELINE (always the same)
  ──────────────                                       ───────────────────────────────────
  UWB board ─serial─► uwb_node ───────────┐
                                           │ /target [PointStamped, base_link]
  turtlesim poses ──► pose_to_target ──────┤   (sim stand-in for uwb_node)
                                           ▼
                                        follower ────────────────┐
                                                                 │ /cmd_vel_raw [Twist]
  gamepad ──► joy ──► teleop_twist_joy ──────────────────────────┤   "what the controller wants"
                                                                 ▼
          /ultrasonic/range [Range] ─────────────────────► safety_gate ──► /safety/state [String]
          /esp32/link_ok [Bool]     ─────────────────────►     │
                                                               │ /cmd_vel [Twist, fixed 50 Hz]
                                                               ▼   "what the motors may do"
                                                         esp32_bridge ◄──serial──► ESP32
                                                           │     ▲       H / C / P  lines out
                                                           │     │       R / S / V  lines in
     /esp32/failsafe [Bool], /esp32/actuators [Vector3] ◄──┘     │ /camera/pan_cmd [Float32]
                                                                 │
  CAMERA (optional, independent of driving)                      │
  ──────────────────────────────────────────                     │
  GoPro / webcam / file ──► camera_node ──/camera/target [Vector3Stamped]──► camera_aim
                              │   ▲            (bearing of the skater in the camera frame)   │
                              │   └── /target (UWB hint), /camera/pan_angle [Float32] ◄──────┤
                              └── /camera/image/compressed [CompressedImage] → Foxglove      │
                                                       /camera/bearing [Float32] (robot frame)◄┘
  MONITORING
  ──────────
  foxglove_bridge ◄── every topic ──► Foxglove app on the Mac over WebSocket :8765
```

Points to notice:

- **There is exactly one way to move the motors**: publish a `Twist` on `/cmd_vel_raw`. The gate, the
  bridge and the firmware never change when a new controller is added. Nothing publishes to `/cmd_vel`
  except the gate.
- **The follower does not know what sensor it is following.** It consumes a point in the robot frame
  (`/target`). In simulation that point comes from turtlesim poses; on the robot it will come from UWB.
- **The camera subsystem is a side branch.** It cannot move the robot. It produces one servo command and
  a bearing that a future controller *may* use when UWB is missing.

---

## 4. Code layering: pure logic, ROS wrapper, launch

Every package follows the same three-layer pattern. Learn it once and every package reads the same way.

| Layer | What it contains | Example | Tested by |
|---|---|---|---|
| **Pure module** | A `@dataclass` of tunables plus plain functions/classes. No `rclpy`, no I/O, no clocks, no globals. Takes numbers, returns numbers. | `honebot_safety/gate.py` → `gate_command(...)` | `pytest` in `test/` (96 tests across the workspace) |
| **ROS node** | A `Node` subclass that declares parameters into the dataclass, subscribes, stores "last message + timestamp", runs a fixed-rate timer that calls the pure function, publishes. Plumbing only. | `honebot_safety/safety_gate.py` | Bench tests with `ros2 topic pub/echo/hz` (recorded in `CLAUDE.md` §6) |
| **Launch** | Wiring: which nodes, which parameters, which remappings, which optional branches. | `launch/safety_gate.launch.py`, `honebot_bringup/launch/bringup.launch.py` | `ros2 launch … --print` parse check, then running it |

Why this shape:

- The maths is where bugs hurt and where hardware is not needed, so it is kept trivially unit-testable.
- Inputs change over the project's life (turtlesim poses → UWB; HOG → YOLO; Ackermann → differential).
  Keeping the control law in a pure function meant the follower was refactored from world poses to a
  relative target without touching the tests that mattered.
- ROS node code is deliberately boring: anyone who has read one node has read them all.

Pure-module map:

| Package | Pure module | Core function/class |
|---|---|---|
| `honebot_follow` | `control.py` | `follow_command_relative(distance, bearing, FollowGains)` |
| `honebot_safety` | `gate.py` | `gate_command(linear, angular, range, ages…, GateParams)` |
| `honebot_hardware` | `diffdrive.py`, `ackermann.py` | `twist_to_diffdrive`, `twist_to_ackermann` |
| `honebot_uwb` | `localize.py` | `two_anchor_position`, `aoa_position`, `RangeFilter` |
| `honebot_camera` | `aim.py`, `detector.py`, `track.py`, top half of `gopro.py` | `PanController`, `decode_yolov8`, `TargetTracker`, `plausible`, `find_gopro_in` |

---

## 5. Safety design: independent watchdogs, every layer fails to "stop"

A 2.5 kg robot at 6 m/s can hurt someone, so the architecture is built around one rule: **any single
failure, anywhere, ends in the motors going to neutral**, and no layer trusts the layer above it to be
alive. Each layer watches its *own* inputs with its *own* timeout.

| Layer (top → bottom) | Watches | Timeout | Action when it trips |
|---|---|---|---|
| `uwb_node` | each UWB sample (no fix, out of range, outside the anchor's usable angle) | per sample | publishes nothing, so the follower's timeout sees the dropout |
| `follower` | age of the last `/target` | `target_timeout` 0.5 s | publishes a zero `Twist` |
| `safety_gate` | ESP32 link flag and its age; range reading validity and age; age of the controller's command; obstacle distance; **sight limit** (robot config: never faster than it can stop within what the sensor sees, 2.7–3.8 m/s with the ultrasonic sensor) | `range_timeout` 0.3 s, `cmd_timeout` 0.5 s, `stop_distance` 0.4 m, `slow_distance` 1.0 m | publishes zero (or forward-blocked / scaled / capped) on `/cmd_vel` and the reason on `/safety/state` |
| `esp32_bridge` | age of the last `/cmd_vel` | `cmd_timeout` 0.3 s | sends `C,0,0` (neutral); also one neutral on exit |
| ESP32 firmware (2.2.1) | heartbeat `H` from the Pi; command `C` from the Pi; boot arming window; handheld remote (STOP, lost, or never heard when `REMOTE_REQUIRED`) | 500 ms, 200 ms, 2 s, 500 ms | ramps both drive channels to 1500 µs at 0.5 g (hard neutral after 2.5 s and at boot); reports via `S,1,…` and `K,…` |
| Handheld remote (`firmware/esp32_remote`) | its button (STOP/RUN) and stick | — | STOP or radio loss stops the robot; in RUN the stick drives manually and overrides the Pi (the ultrasonic gate does not apply to manual driving) |
| ESC (hardware) | signal present | ESC-specific | stops the motor |

Properties that follow from this table and that the code relies on:

- **Process death at any level is safe.** Kill the follower: the gate reports `CMD_STALE`. Kill the gate:
  the bridge sends neutral. Kill the bridge or unplug USB: the ESP32 stops hearing `H` and fails safe.
  Hang the Pi entirely: same as the last case.
- **The gate only subtracts motion.** `gate_command` can zero or scale the forward speed and can zero
  everything; it never invents motion. That makes it safe to put between *any* controller and the motors.
- **The gate publishes at a fixed 50 Hz no matter what**, rather than forwarding each incoming message.
  That is what lets the next layer (the bridge) use "message age" as a liveness signal.
- **Obstacle handling blocks only forward motion.** With an obstacle inside `stop_distance` the gate still
  passes angular and reverse commands, so a robot nosed up against something can back out or turn away.
  Only a forward-facing sensor exists, so reverse is simply not gated.
- **Timeouts are nested on purpose.** Firmware 200 ms < bridge 0.3 s < gate 0.5 s. Each lower layer is
  stricter, so a stall shows up first as `S,1` from the ESP32 and the Pi-side layers never mask it.
- **No-echo is not a fault.** The firmware reports `-1` when nothing reflects; the bridge publishes
  `+inf` (ROS REP 117 for "nothing in range"), which the gate treats as *clear*. `NaN`/`≤ 0` is
  *invalid* and stops the robot. Confusing those two would make an empty car park look like a wall.
- **The camera pan servo is outside the fail-safe.** It holds its last position when the link drops. A
  camera cannot hurt anyone, and snapping it to centre mid-run would ruin the footage.
- **Bench caps live in the firmware** (`THROTTLE_LIMIT` 0.5, ramps of 0.8 g up / 0.7 g down) so a wrong Pi-side
  parameter during early testing cannot command full power. They are meant to be raised once the robot behaves.

Not yet built: longer-range obstacle sensing. Until then the gate's sight limit holds the robot to about
6–8 mph (`docs/first_drive.md` has the full table of today's speed limits).

---

## 6. The Pi ↔ ESP32 contract

USB serial, 115200 8N1, one ASCII line per message, `\n` terminated. Unknown lines are ignored on both
sides (the ESP32 prints boot chatter; the firmware may be newer or older than the bridge).

| Direction | Line | Rate | Meaning |
|---|---|---|---|
| ESP32 → Pi | `V,<version>` | once at boot | firmware version; bridge logs it |
| ESP32 → Pi | `R,<distance_m>,<link_ok>` | 16.7 Hz | ultrasonic distance (`-1` = no echo) and whether a heartbeat arrived within 500 ms |
| ESP32 → Pi | `S,<failsafe>,<a>,<b>` | 10 Hz | 1 while outputs are forced neutral; `a`, `b` = what is actually on the two drive channels (−1..1) |
| Pi → ESP32 | `H` | 10 Hz | heartbeat |
| Pi → ESP32 | `C,<a>,<b>` | 50 Hz | drive command, each −1..1. **The ESP32 does not know what the channels mean**: in `diff` mode a/b are left/right wheels, in `ackermann` mode throttle/steer |
| Pi → ESP32 | `P,<pan>` | 20 Hz | camera pan, −1..1, +1 = full left |

Design points:

- **All actuator values are normalised −1..1.** Physical meaning (m/s, wheel speed, steering angle) is
  applied on the Pi in `diffdrive.py`/`ackermann.py`. The firmware only maps −1..1 → 1000–2000 µs. That
  keeps the firmware chassis-agnostic and means chassis changes are a Pi-side parameter edit.
- **Both values are sent every cycle, even when unchanged**, so the ESP32's 200 ms command watchdog
  doubles as a "the bridge is alive" check.
- **Link status rides on the sensor line** (`R,…,<link_ok>`) instead of a separate message, so the gate
  can use a single `range_timeout` for both "range is stale" and "link flag is stale".

---

## 7. Subsystem walkthroughs

### 7.1 Following (`honebot_follow`)

The controller works entirely in the robot frame: given the target at `(distance, bearing)`, turn toward it
proportionally to the bearing and drive forward proportionally to how far beyond `follow_distance` it is.
Two refinements matter for a skater-chasing robot:

- Forward speed is multiplied by `max(0, cos(bearing))`, so the robot slows while turning and never
  drives *away* from a target that is behind it.
- It never reverses. If the skater stops inside `follow_distance`, the robot stops and waits.

The `ackermann` flag changes one thing: a car cannot turn in place, so instead of stopping with full steer
it creeps forward at `creep_speed`. The chassis decision is now differential drive, so this is off by
default, but it is kept so an RC-car variant stays possible.

Input abstraction: the `follower` node has a turtlesim mode (two world poses → `relative_target()`) and a
relative mode (`/target` point → `(hypot, atan2)`). `pose_to_target` is a tiny node that turns turtlesim
poses into exactly the message `uwb_node` will publish, so the *real* pipeline can be exercised on screen
with no hardware. See `src/honebot_follow/README.md`.

### 7.2 Localising the skater (`honebot_uwb`)

Ultra-wideband ranging gives a distance from each anchor to the skater's tag. Two anchors `baseline` apart
give a position up to a front/back mirror; the code assumes "in front", which holds because the robot's job
is to stay behind. An angle-of-arrival module (the recommended Makerfabs board) reports range and bearing
in one line instead. Both paths funnel into one `PointStamped` in `base_link`. A median-3 filter kills
single-sample spikes and an EMA smooths. No fix → nothing published → the follower's timeout stops the
robot, with no special "lost" message needed. See `src/honebot_uwb/README.md`.

Why UWB and not vision for following: a Pi-CPU detector runs at 5–10 Hz with 0.3–0.5 s of pipeline
latency and loses people at odd angles. At 6 m/s that is a few metres of blind travel per miss. UWB gives
~10+ Hz range at centimetre accuracy regardless of lighting or pose. Vision is kept for what it is good at:
framing the shot.

### 7.3 Driving (`honebot_hardware` + firmware)

`esp32_bridge` is the only node that touches the serial port. It owns three outgoing timers (heartbeat,
drive command, pan command), one 100 Hz non-blocking read timer, and a 1 Hz reconnect timer. The
`drive_mode` parameter selects which pure converter turns a `Twist` into the two channel values:

- `diff`: `v ± ω·track/2`, normalised by `max_wheel_speed`. If either wheel would saturate, *both* are
  scaled by the same factor so the turn radius is preserved and the robot just goes slower, instead of
  clipping one wheel and veering.
- `ackermann`: `δ = atan(L·ω / v)` with a floor on `v` so near-zero speed does not demand infinite steer.

Putting the conversion in the bridge (rather than a separate "kinematics node" with a custom message)
keeps `Twist` as the universal contract and avoids defining any custom ROS message types in this repo.
See `src/honebot_hardware/README.md` and `firmware/esp32_ultrasonic/README.md`.

### 7.4 Camera: video in, person detection, pan aiming (`honebot_camera`)

Three threads in `camera_node`, by design:

1. **Grabber** reads frames as fast as the source gives them and keeps **only the newest**. Without this,
   a 30 fps source feeding a 7 fps detector would build an ever-growing backlog and the camera would be
   aiming at where the skater was seconds ago.
2. **Detector** pulls the newest frame, shrinks it to `proc_width`, runs the backend, picks the skater,
   publishes a bearing, and occasionally a JPEG preview.
3. **rclpy** spins in the main thread so parameters, subscriptions and timers keep working while the
   other two block on I/O or compute.

Detector backends share one interface (`detect(frame) -> [Detection]`): OpenCV's built-in HOG (no files
to download, mediocre, fires on jackets and poles), and YOLOv8n/YOLO11n ONNX via `cv2.dnn` or
`onnxruntime` (much better). The default is YOLOv8n from `~/models/yolov8n.onnx`, HOG if that file is
missing. On this Pi, OpenCV 4.6 cannot run the YOLO file, so `auto` falls back to onnxruntime after a
failed warm-up frame; it runs at 10–11 detections/s. All of the geometry (letterboxing, decoding the raw
YOLO tensor, NMS, target selection) is numpy-only and unit-tested without a model.

Detections never drive the servo directly. `track.py` first drops implausible boxes (aspect ratio, size,
floating in the top of the frame, wrong size for the UWB distance), then runs a single-target tracker in
the **robot frame**: a two-state Kalman filter on bearing and bearing rate predicts where the skater will
be, the nearest plausible detection inside a covariance-sized gate is associated, a new target must be
seen on several consecutive cycles before it is trusted, and a trusted target is coasted through missed
detections for up to a second. This is what stops a one-frame false positive from grabbing the camera and
keeps the lock through a pose the detector misses. A new track is seeded on the detection nearest the UWB
bearing, else the biggest box.

`camera_aim` drives an **open-loop** servo (no position feedback), so the controller is incremental:
`pan += kp · (bearing of the target from the camera axis)`. When the target is centred the error is zero
and the pan stops, whatever the gains or the exact lens field of view; those only change convergence
speed. Lost target → steer to the UWB bearing; no UWB → drift back to centre after 3 s. The published
`/camera/bearing` is `pan + image bearing`, i.e. the skater's bearing in the robot frame.

The GoPro is a USB *network* device, not a webcam: HTTP calls put it in webcam mode and it pushes an
MPEG-TS stream over UDP. Recording is a byte-level tee of that stream to a `.ts` file (zero CPU, no
re-encode). See `src/honebot_camera/README.md` and `docs/camera_plan.md`.

### 7.5 Bringup and operations (`honebot_bringup`)

`bringup.launch.py` starts the hardware bridge, the gate and Foxglove (and optionally the camera stack) but
**never a controller**. Controllers (follower, gamepad teleop) are launched separately so the safety layer
is always up before anything that can produce motion. A systemd unit runs bringup at boot; a `record`
launch wraps `ros2 bag record -a`. Foxglove on the Mac is the dashboard; no GUI runs on the Pi.

### 7.6 Handheld remote in turtlesim (`honebot_remote_bridge`)

On the robot the handheld remote talks only to the robot ESP32 (ESP-NOW), and the firmware decides who drives:
nobody in STOP or when the remote is lost, the stick when it is pushed in RUN, the Pi (follow-bot code) otherwise.
`honebot_remote_bridge` brings that into turtlesim before the robot exists. `source:=serial` reads the remote board
over USB and applies the same rules in Python (`remote_logic.Arbiter`, a port of `drive::decide`);
`source:=esp32` mirrors the robot ESP32's real wheel outputs (`/esp32/outputs` from `esp32_bridge`). With
`follow:=true` turtle1 is the skater and turtle2 the robot: the follower drives it and the remote overrides it.
Guide: `docs/remote_turtlesim.md`.

---

## 8. Conventions used everywhere

- **Frames and units** follow ROS REP 103: `base_link` has x forward, y left, z up; angles are radians,
  positive = counter-clockwise (left). Images use OpenCV's convention (x right, y down); `aim.py` is the
  only place that converts between the two.
- **Relative topic names in nodes** (`cmd_vel`, not `/cmd_vel`) so launch files can remap. Example:
  `turtle_follow.launch.py` remaps the gate's `cmd_vel` to `/turtle2/cmd_vel`.
- **Parameter pattern:** declare with a default in `__init__`, copy into the pure dataclass, register
  `add_on_set_parameters_callback` so `ros2 param set` changes behaviour live. Tuning never needs a restart.
- **Staleness pattern:** callbacks only store `(msg, now())`; a fixed-rate timer computes ages and decides.
  `None` age means "never received" and is treated as stale.
- **Clean shutdown everywhere:** `main()` catches `KeyboardInterrupt` and `ExternalShutdownException`,
  publishes/sends a stop, and tolerates `rclpy` already being shut down. This is what lets `ros2 launch`
  (SIGINT) and systemd (SIGINT, then SIGTERM) stop the stack without leaving a motor running.
- **Serial readers** (bridge, UWB) are non-blocking: 100 Hz poll, line buffer, junk dropped past 256 bytes
  with no newline, unknown lines ignored, 1 Hz reconnect loop so unplugging and replugging USB just works.
- **Logging:** state *transitions* are logged (gate state, link up/down, fail-safe on/off, tracking
  lost/found), not every message, and repeated warnings use `throttle_duration_sec`.
- **No custom message types.** Everything is `geometry_msgs`, `sensor_msgs`, `std_msgs`. Costs a little
  clarity (`Vector3Stamped` used for bearing/elevation/size) but keeps every tool and Foxglove working
  with zero setup.

---

## 9. Testing strategy and how to extend

**Three tiers of testing**

1. `pytest` on the pure modules (run by `colcon test`). Fast, no hardware, no ROS daemon.
2. Bench tests of nodes against the real ESP32 with CLI tools: `ros2 topic pub /cmd_vel_raw …`,
   `ros2 topic echo /safety/state`, `ros2 topic hz /cmd_vel`. The results that matter are logged in
   `CLAUDE.md` §6 so a regression can be compared against a known-good run.
3. The turtlesim demo as an end-to-end test of the command pipeline with a visible result.

There are no launch-level integration tests yet; adding `launch_testing` would be the natural next step.

**Recipes**

- *New controller* (e.g. a path follower): publish `Twist` on `/cmd_vel_raw`. Do not touch the gate or
  bridge. Stop publishing when you have nothing to say; the gate treats silence as "stop".
- *New sensor on the ESP32:* print a new line type in the firmware `loop()`, parse it in
  `esp32_bridge._handle`, publish a standard message. If it is safety-relevant, add a rule to
  `gate_command` *with a test* and a new state string.
- *New command to the ESP32:* add a branch in the firmware's `handleLine()`, a timer in the bridge that
  sends it every cycle, and decide explicitly whether the ESP32 should neutralise it on fail-safe.
- *New detector backend:* a class with `.name` and `.detect(frame_bgr) -> [Detection]`, registered in
  `detector.make_detector`.
- *New chassis kinematics:* a pure module next to `diffdrive.py`, a `drive_mode` value in the bridge,
  tests mirroring `test_diffdrive.py`.
- *New tunable:* add it to the dataclass, declare the parameter in the node, and it is live-settable for
  free via the existing callback.

**What is real and what is a placeholder (2026-11-14)**

| Component | State |
|---|---|
| ESP32 + ultrasonic + heartbeat + fail-safe | Built, wired, bench-verified |
| Safety gate, bridge, bringup, Foxglove | Verified against the live ESP32 |
| Follower (both modes), pose_to_target | Verified in turtlesim |
| GoPro video in + HOG detection | Verified live over USB |
| Pan servo, camera_aim | Verified with a fake target through the real bridge; servo not yet wired |
| Drivetrain, ESCs, `track_width`, `max_wheel_speed` and all kinematic numbers | Not built; **numbers are placeholders** |
| UWB board, `uwb_node` | Not bought; node written against the planned line format |
| Kill switch, systemd/udev install | Planned / awaiting sudo |

---

## 10. Run card

### Build and test
```bash
colcon build --symlink-install
source install/setup.bash          # or open a new shell
colcon test && colcon test-result --verbose
```

### Run the robot (real robot settings from `src/honebot_bringup/config/robot.yaml`)
The safety layer (bridge + gate + Foxglove) is what `honebot.service` starts at boot; by hand it is:
```bash
ros2 launch honebot_bringup robot.launch.py
```
Follow and film (UWB + follower + camera + a bag of the run), started by hand once the safety layer is up:
```bash
ros2 launch honebot_bringup film.launch.py max_linear:=2.0
```
The staged test plan is `docs/first_drive.md`.

### Bench stack with default parameters (ESP32 bridge + safety gate + Foxglove bridge)
```bash
ros2 launch honebot_bringup bringup.launch.py
```
You should see `Connected to ESP32`, `ESP32 reports heartbeat OK`, `Server listening on port 8765`,
and the gate logging `STOP (CMD_STALE)` (nothing is commanding it yet; that is correct).

Watch it live **(Mac)**: Foxglove desktop → Open connection → `ws://honebot.local:8765`.
Useful topics: `/ultrasonic/range`, `/esp32/link_ok`, `/safety/state`, `/cmd_vel_raw`, `/cmd_vel`,
`/esp32/actuators`, `/esp32/failsafe`.

### Physics simulator (Mac, Linux or Pi; no ROS needed)
`sim/` is a MuJoCo simulation of the real robot. It uses the CAD mass and geometry, the compiled firmware logic,
the follower, gate and bridge code and `robot.yaml`, with simulated HC-SR04, UWB, camera and a scripted skater.
It has a 3-D viewer on the Mac and pass/fail runs of 11 scenarios. Setup and findings: [`sim/README.md`](sim/README.md).

### Turtlesim demo (sim window on the HDMI screen, teleop over SSH) (older, simpler: an ideal point chasing another)
From an SSH session (you are in the `video` group), this draws straight on the HDMI framebuffer:
```bash
sg video -c "exec bash -c 'source /opt/ros/jazzy/setup.bash && source ~/honebot_ws/install/setup.bash && QT_QPA_PLATFORM=linuxfb ros2 launch honebot_follow turtle_follow.launch.py'"
```
Options: `use_safety:=false` (no ESP32 needed), `relative:=true` (simulated-UWB pipeline), `ackermann:=true`.
```bash
ros2 run turtlesim turtle_teleop_key                                                 # second terminal
```
Cover the ultrasonic sensor with your hand: the chasing turtle freezes. Unplug the ESP32: it freezes.

### Handheld remote + follow bot in turtlesim (HDMI console, from `~/honebot_ws`)
Checks the setup, then launches. `fake` needs no ESP32 and runs an automatic PASS/FAIL check; `serial` = remote
board on USB, `esp32` = robot ESP32 on USB with the remote by radio. `follow` adds the follow bot on turtle2,
overridden by the remote. Guide: `docs/remote_turtlesim.md`.
```bash
tests/test_esp32_remote.sh fake follow
```
```bash
tests/test_esp32_remote.sh esp32 /dev/ttyUSB0 follow
```
Flash the remote from the Mac first: `scripts/flash_remote.sh <remote_port> [robot_port]`.

### Gamepad teleop (after bringup is running)
```bash
ros2 launch honebot_bringup teleop_joy.launch.py            # xbox layout; joy_config:=ps5 etc.
```
Hold the enable button and use the left stick. Letting go stops the robot (CMD_STALE).

### Camera: GoPro video-in, person detection, pan servo
One-time (sudo): install `src/honebot_camera/netplan/99-gopro-usb.yaml` (commands in the file). GoPro: USB
Connection = GoPro Connect, plugged into a Pi USB 3 port.
```bash
ros2 run honebot_camera gopro_ctl find                                   # finds the camera's USB network link
ros2 launch honebot_bringup bringup.launch.py camera:=true [record:=true]  # robot stack + camera + aiming
ros2 launch honebot_camera camera.launch.py source:=/dev/video0 aim:=false  # just a webcam, no servo/bridge
ros2 launch honebot_camera camera.launch.py model_path:=''                 # force the old HOG detector
```
Detector: YOLOv8n from `~/models/yolov8n.onnx` by default (10–11 det/s via onnxruntime), HOG if the file is
missing. Export/re-export: `docs/camera_plan.md` §5.
Foxglove: Image panel on `/camera/image/compressed` (green box = tracked skater), `/camera/target`,
`/camera/pan_angle`, `/camera/status`. Pan servo: signal GPIO 27, own 5–6 V supply, common ground
(firmware ≥ 2.1.0). Tune live: `ros2 param set /camera_aim kp 0.4`. Details and wiring: `docs/camera_plan.md`.

### UWB (when the board arrives)
```bash
ros2 launch honebot_uwb uwb.launch.py port:=/dev/ttyUSB1
```

### Firmware (Mac)
```bash
cd <your clone of this repo>
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32_ultrasonic
arduino-cli upload  -p /dev/cu.usbserial-0001 --fqbn esp32:esp32:esp32 firmware/esp32_ultrasonic
```
On boot it prints `V,2.2.1`; the bridge logs `ESP32 firmware version 2.2.1`. The handheld remote is a second sketch,
`firmware/esp32_remote` (see its README); from 2.2.0 the robot does not move unless the remote is in RUN.

### Auto-start at boot and serial-port tweak (need sudo)
Run `./scripts/pi_setup.sh` on the Pi (it asks for sudo). It installs `setserial`, the udev rule and
`honebot.service`. Once the service runs, `sudo systemctl stop honebot` before launching the safety layer by hand.

---

## 11. Repository map

```
honebot_ws/
├── README.md                      this file: architecture + run card
├── CLAUDE.md                      project handoff: hardware, history, gotchas, phase plan
├── docs/                          start at docs/README.md (index) and docs/first_drive.md (test plan)
├── cad/                           chassis: Fusion script + offline checks (docs/chassis_cad.md)
├── sim/                           MuJoCo physics simulator of the whole robot (+ README)
├── scripts/pi_setup.sh            Pi housekeeping (udev rule, auto-start service)
├── scripts/flash_remote.sh        Mac: flash the handheld remote (and robot ESP32), then print the checks
├── firmware/esp32_ultrasonic/     robot ESP32 sketch (+ README)
├── firmware/esp32_remote/         handheld remote sketch: kill switch + manual drive (+ README)
├── firmware/tests/                PC (host g++) tests of the firmware logic, run by CI
├── tests/                         hand-run ROS checks: remote + follow bot in turtlesim, fake remote (+ README)
└── src/
    ├── honebot_follow/            follow-me controller; turtlesim demo; simulated UWB     (+ README)
    ├── honebot_safety/            safety gate between controller and motors              (+ README)
    ├── honebot_hardware/          ESP32 serial bridge; Twist → wheel/steer conversion    (+ README)
    ├── honebot_uwb/               UWB serial → relative target                           (+ README)
    ├── honebot_camera/            GoPro/webcam in, person detection, pan-servo aiming    (+ README)
    ├── honebot_remote_bridge/     handheld remote (+ follow bot) in turtlesim            (+ README)
    └── honebot_bringup/           launch files, config/robot.yaml, systemd unit          (+ README)
```
Each package: `honebot_<name>/` (Python module), `launch/`, `test/`, `setup.py`, `package.xml`.
