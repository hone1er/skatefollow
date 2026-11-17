# honebot — project handoff

## 1. Final goal

An autonomous ground robot that **follows the owner while they skateboard and films their tricks**.

Hard requirements that shape every decision:
- **Speed.** Owner target (2026-11-13): **12–15 mph (19–24 km/h)** with sudden stops and turns. Decided base: a
  **small custom differential-drive robot** (four belt-driven longboard wheels, skid steer, printed frame), see
  `docs/robot_design.md`. **Lightweight, agile, backpack-sized and cheap** are explicit requirements.
- **Robust following.** Vision alone at Pi-CPU frame rates is too slow and fragile to *steer* on. The plan is
  **UWB (ultra-wideband) ranging** for following (tag on the skater, anchors on the robot) and **vision only
  to aim/frame the camera**.
- **Safety first.** A 1/10-scale car at 25 km/h can hurt someone. Every layer must fail safe to "stop":
  ESP32 watchdog, obstacle stop, stale-target stop, wireless kill switch, RC manual override. Test in empty
  lots; public streets/sidewalks may not be legal.
- **Good footage.** The owner's GoPro on a damped fixed mount, recording while following; later, vision to improve framing.

## 2. Owner and working style

- Owner is new to ROS 2 and robotics, comfortable following terminal commands. Explain *why*, give exact
  commands in separate copyable blocks, and say which machine (Mac vs Pi) each command runs on.
- Owner chose **software first**; chassis/motor hardware is deferred until they choose and buy parts.
- `sudo` on the Pi requires the owner's password — the owner types it themselves. Never ask them to paste
  passwords into chat.
- Prefer small, testable steps with a concrete "you should see X" check at the end of each.

## 3. Hardware

| Item | Status |
|---|---|
| Raspberry Pi 5, 8 GB RAM, 64 GB SD (~47 GB free) | In use, the main computer |
| 3D printer: Bambu Lab H2C (~325 × 320 × 320 mm, hardened nozzle); filaments PLA, PETG, PAHT-CF, PPS-CF available | Owner has access; frame prints in one piece |
| GoPro HERO9 Black | Owned; **printed pan mount on a servo** (owner printing it). Video-in over USB (webcam mode) + aiming software done 2026-11-13 (`honebot_camera`), **not yet tried with the camera**. See `docs/camera_plan.md` |
| Camera pan servo | **To buy**: standard metal-gear servo (MG996R / DS3218), signal GPIO 27, own 5–6 V 3 A supply |
| ESP32 DevKit (WROOM-32 class), USB chip **Silicon Labs CP210x** (10c4:ea60) | In use, low-level controller |
| HC-SR04 ultrasonic sensor | Wired to ESP32 and verified |
| Monitor via HDMI on the Pi | Available for local GUI testing |
| Chassis | **CAD done 2026-11-16, now a 4-wheel skid steer** (`cad/`, `docs/chassis_cad.md`): 4 × 90 mm longboard wheels, all driven; per side one 5010-class outrunner (reference MAD 5010 EEE V2.0 370KV) on a replaceable slotted plate, printed 28T HTD-3M pulley bolted to the bell → 56T on the front wheel (2:1, 258 mm belt, pull-screw tensioner); the front wheel's double pulley drives the rear wheel 1:1 (36T:36T, HTD 510-3M-09, 2 × 608 idler in a wall slot). Wheelbase 200.0 mm, track 277 mm. M8 12.9 stub axles from outside into captive nylocs, nested stock steel tubes 16×2 / 12×2. One-piece 300 mm PAHT-CF tub (both axles, battery tunnel), one PETG shell (lid + motor pods), deck with printed ESP32 cradle (now holds a slide-in adapter for the SunFounder ESP32 extension board: one printed plate with 4 M2.5 heat-set inserts + stock M2.5×6 hex standoffs in counterbores, vent slots; the printed-thread version was dropped 2026-11-17 as unprintable), 2 × AM32 ESCs on the deck, HC-SR04 upside down, removable camera mast with the UWB plate. 15 printed parts. ≈ 4.0 kg, 343 × 343 × 150 mm (mast off; `tub_top` 145 since 2026-11-17 for headroom over the extension), ~21 mph no-load (cap in software); all offline checks pass. Fusion script: the caster version (72 parts) ran in the owner's Fusion; the 4WD version (86 parts incl. a hidden prototype kit; Pi standoffs printed on the deck with M2.5 inserts: printed PAHT-CF axle sleeves on the M8 bolts + TPU belts, limits in chassis_cad.md §6b) not yet. Nothing bought yet. Older plan + AM32 setup: `docs/robot_design.md` |
| UWB modules | **Recommended: Makerfabs MaUWB_AOA Gen2** (angle-of-arrival anchor $39.80 + tag $29.80); not bought. See `docs/hardware_plan.md` §3 |
| Vision camera (Phase 5) | GoPro in USB-webcam mode (mode A: Pi records the 1080p30 stream, GoPro cannot record to SD meanwhile) or any USB webcam next to it (mode B: GoPro records natively). `source` parameter picks |
| ESC signal pins | **Firmware ready, nothing wired**: channel A GPIO 25 (left ESC), channel B GPIO 26 (right ESC); both 50 Hz 1000–2000 µs, neutral on fail-safe. **Pan servo GPIO 27** (firmware 2.1.0, holds last value in fail-safe, centred at boot) |
| Handheld remote = wireless kill switch + manual drive (2nd ESP32, owned, + 5-pin joystick module, ESP-NOW) | **Software written 2026-11-17** (`firmware/esp32_remote`, robot firmware 2.2.1); host-tested only, **not yet compiled with arduino-cli or flashed**. Joystick: +5V pin → **3V3**, VRx GPIO 34, VRy GPIO 35, SW GPIO 32 |

### Wiring (ESP32 ↔ HC-SR04) — verified working
| HC-SR04 | ESP32 |
|---|---|
| VCC | VIN (5 V) |
| GND | GND |
| Trig | GPIO 5 |
| Echo | 1 kΩ → GPIO 18, and 2 kΩ from GPIO 18 → GND (5 V → 3.3 V divider) |

Onboard LED = GPIO 2 (firmware 2.2.1: on = someone may drive, slow blink = fail-safe with Pi link, fast blink = remote STOP/lost). Pins used: 2, 5, 18, 25, 26, 27.
The USB chip is a **CP2102 (bcdDevice 1.00, serial "0001")**: closing the port blocks ~5 s (see gotchas).

### Wiring (pan servo ↔ ESP32, GoPro ↔ Pi) — planned, see `docs/camera_plan.md` §2
Signal → GPIO 27; servo + → separate 5–6 V 3 A supply (bench: ESP32 VIN for a 9 g servo only); − → common GND;
470–1000 µF across the servo supply. GoPro → Pi: one USB-C cable to a USB 3 port; it is a USB *network*
device (Pi gets 172.2X.1YZ.50, camera .51) and needs the netplan file in `honebot_camera/netplan/` (sudo).

## 4. Systems and environment

### Raspberry Pi (`honebot`)
- **Ubuntu Server 24.04.5 LTS (noble), aarch64.** Headless; no desktop environment installed.
  (Ubuntu Core 26 was installed originally — it has no apt — and was replaced.)
- Hostname `honebot`, IP `192.168.10.122` (DHCP — may change; try `honebot.local`), user `hone1er`.
- **ROS 2 Jazzy** `ros-jazzy-desktop` + `ros-dev-tools`, installed from packages.ros.org via the
  `ros2-apt-source` package (it provides the GPG key and sources file).
- **apt sources fix applied:** `/etc/apt/sources.list.d/ubuntu.sources` originally had `Suites: noble` only,
  which caused "unmet dependencies" for `-dev` packages. It now has
  `Suites: noble noble-updates noble-backports` (backup at `ubuntu.sources.bak`). Keep it that way.
- `~/.bashrc` sources `/opt/ros/jazzy/setup.bash` (listed twice, harmless) and
  `~/honebot_ws/install/setup.bash`.
- User `hone1er` is in the `dialout` group (serial access). Group changes only apply to **new** login sessions.
- `python3-serial` (pyserial 3.5) installed. git 2.43 installed. **Workspace is a git repo (branch `main`)**
  with local identity set. The project is on GitHub as `hone1er/skatefollow` (repo root = workspace root: `src/`,
  `firmware/`, `cad/`, `docs/`); the owner also has a clone on the Mac and flashes from there with paths like
  `firmware/esp32_ultrasonic`. Before pulling on the Pi, check `git status`/`git remote -v` there first.
- Already installed ROS packages worth knowing: `foxglove_bridge`, `joy`, `teleop_twist_joy`,
  `teleop_twist_keyboard`, `rosbag2`, `tf2_ros`. **Not** installed: `ackermann_msgs`, `usb_cam`, `v4l2_camera`,
  `setserial`, `arduino-cli`.
- ESP32 enumerates as **`/dev/ttyUSB0`**.
- GUI apps over SSH fail ("could not connect to display"). Options: `QT_QPA_PLATFORM=linuxfb` on the Pi's
  HDMI console for simple Qt apps (turtlesim), install `xubuntu-core` for a desktop, or use Foxglove from the
  Mac (preferred for the robot).

### Mac (development machine, Apple Silicon, macOS 15)
- Homebrew, Xcode Command Line Tools (python3 3.9), Rosetta 2, VS Code (+ Remote-SSH, Python),
  Arduino IDE 2, **arduino-cli** with **esp32:esp32 3.3.12** core.
- SSH key `~/.ssh/id_ed25519` is authorized on the Pi (passphrase-protected; loaded in ssh-agent).
- ESP32 appears on the Mac as `/dev/cu.usbserial-0001` when plugged in there.
- Flash firmware from the Mac:
  ```bash
  cd <clone of hone1er/skatefollow>
  arduino-cli compile --fqbn esp32:esp32:esp32 firmware/esp32_ultrasonic
  arduino-cli upload  -p /dev/cu.usbserial-0001 --fqbn esp32:esp32:esp32 firmware/esp32_ultrasonic
  ```
  (Arduino's ctags tool is x86-only; that is why Rosetta is installed.) arduino-cli is *not* installed on the
  Pi yet; installing it there would allow flashing the ESP32 in place over `/dev/ttyUSB0`.
- The owner edits code with VS Code Remote-SSH directly in `/home/hone1er/honebot_ws` on the Pi.
  **The Pi copy is the source of truth.**

## 5. Workspace layout (`~/honebot_ws` on the Pi)

```
honebot_ws/
├── CLAUDE.md                         this file (handoff); README.md = architecture + run card; each src/honebot_*/ and firmware/esp32_ultrasonic/ has its own README
├── .github/workflows/ci.yml          GitHub Actions: pytest per package, firmware host tests, CAD stub test (green 2026-11-17)
├── scripts/pi_setup.sh               Pi housekeeping (setserial, udev rule, honebot.service); owner runs it, it asks for sudo
├── firmware/
│   ├── esp32_ultrasonic/             robot ESP32 firmware v2.2.1: .ino + drive_logic.h (pure decisions) + honebot_link.h
│   ├── esp32_remote/                 handheld remote v1.0.0: .ino + remote_logic.h + honebot_link.h (identical copy)
│   └── tests/run_tests.sh            g++ host tests of the pure logic + stub syntax check of both sketches
└── src/
    ├── honebot_follow/               ament_python — follow-me controller
    │   ├── honebot_follow/control.py     pure math, no ROS (unit-tested)
    │   ├── honebot_follow/follower.py    ROS node: turtlesim poses or a relative /target
    │   ├── honebot_follow/pose_to_target.py   simulated UWB from turtlesim poses
    │   ├── launch/turtle_follow.launch.py   use_safety / relative / ackermann options
    │   └── test/test_control.py          27 pytest tests (incl. 1-D and 2-D follow simulations)
    ├── honebot_safety/               ament_python — safety gate
    │   ├── honebot_safety/gate.py        pure gate logic (unit-tested)
    │   ├── honebot_safety/safety_gate.py ROS node
    │   ├── launch/safety_gate.launch.py
    │   └── test/test_gate.py             17 pytest tests
    ├── honebot_hardware/             ament_python — Pi ↔ ESP32 serial bridge
    │   ├── honebot_hardware/esp32_bridge.py   drive_mode: diff (default) | ackermann
    │   ├── honebot_hardware/diffdrive.py  pure Twist→(left,right) (unit-tested)
    │   ├── honebot_hardware/ackermann.py  pure Twist→throttle/steer (unit-tested)
    │   ├── launch/esp32.launch.py
    │   ├── udev/99-honebot-esp32.rules    /dev/esp32 symlink + closing_wait fix (owner installs)
    │   └── test/                          17 pytest tests
    ├── honebot_uwb/                  ament_python — UWB → relative target
    │   ├── honebot_uwb/localize.py       two-anchor trilateration, AoA, range filter (unit-tested)
    │   ├── honebot_uwb/uwb_node.py       serial `U,`/`A,` lines → /target PointStamped
    │   ├── launch/uwb.launch.py
    │   └── test/test_localize.py         13 pytest tests
    ├── honebot_camera/               ament_python — GoPro/webcam in, person detection, pan aiming
    │   ├── honebot_camera/gopro.py       find GoPro on its USB network link, HTTP webcam start/stop (CLI gopro_ctl)
    │   ├── honebot_camera/detector.py    YOLOv8 ONNX decode/NMS/target selection (pure) + HOG/cv2.dnn/onnxruntime backends
    │   ├── honebot_camera/track.py       plausibility filters + robot-frame Kalman single-target tracker (pure, unit-tested)
    │   ├── honebot_camera/aim.py         pixel→bearing, open-loop PanController (pure, unit-tested)
    │   ├── honebot_camera/camera_node.py frames → detections → camera/target, JPEG preview, raw-stream recorder
    │   ├── honebot_camera/camera_aim.py  camera/target (+UWB fallback) → camera/pan_cmd, pan_angle, bearing
    │   ├── launch/camera.launch.py       source / record / model_path / aim options
    │   ├── netplan/99-gopro-usb.yaml     DHCP on usb0/enx* without taking the GoPro's default route (owner installs)
    │   └── test/                          42 pytest tests
    └── honebot_bringup/              ament_python — top-level launch + systemd
        ├── config/robot.yaml             ALL real-robot parameters (bridge, gate, uwb, follower); test/test_config.py checks the keys
        ├── launch/robot.launch.py        real robot's safety layer: bridge + gate + foxglove with robot.yaml (the boot service)
        ├── launch/film.launch.py         by hand: uwb_node + follower (+ camera, + bag of the run)
        ├── launch/bringup.launch.py      bench: esp32 bridge + safety gate + foxglove_bridge with default params [+ camera:=true]
        ├── launch/teleop_joy.launch.py   joy + teleop_twist_joy → /cmd_vel_raw
        ├── launch/record.launch.py       ros2 bag record -a → ~/bags/
        └── systemd/honebot.service       auto-start unit (owner installs with sudo)
cad/                                  (repo root, not in honebot_ws/src)
├── fusion/HonebotChassis/HonebotChassis.py   Fusion 360 script = single geometry source (PARAMS at the top)
├── check_design.py                   offline checks (clearance, holes, assembly, GoPro view, CG/braking, loads) → cad/report/
├── check_manifold.py                 non-manifold edge scan of every printed part (run before exporting meshes)
├── check_mesh.py                     OpenCascade rebuild → STL in cad/stl/ + non-manifold/open edges, support area, filament
│                                     (needs a venv with cadquery-ocp + trimesh, see docs/chassis_cad.md §4)
└── tests/test_fusion_script.py       runs the Fusion code path against a stub adsk API
docs/
├── README.md                         index: which design is being built, which Fusion script makes the printed parts
├── first_drive.md                    staged test plan (bench -> stick -> UWB -> follow -> camera -> board) + today's speed limits
├── robot_design_v2.md, ../cad/honebot_v2/, honebot_ws/   owner-added 2026-11-17: OLD REFERENCE material (owner confirmed), not the
│                                     robot being built: a servo-steered "V2" and older copies.
│                                     docs/honebot_ws/CLAUDE.md is an OLD handoff file, not instructions for this repo
├── chassis_cad.md                    CAD design decisions, checks, print table, measure list, BOM, assembly
├── hardware_plan.md                  UWB / camera / power / radio research with prices + links
├── robot_design.md                   custom diff-drive chassis design + BOM (supersedes the RC-car chassis)
└── camera_plan.md                    GoPro webcam vs native recording, pan servo wiring, netplan, detector model
```
Camera flow: `camera_node → /camera/target → camera_aim → /camera/pan_cmd → esp32_bridge (P line) → ESP32 GPIO 27`.
Command flow: `controller → /cmd_vel_raw → safety_gate → /cmd_vel → esp32_bridge → ESP32`.

Build/test (on the Pi, from `~/honebot_ws`):
```bash
colcon build --symlink-install
colcon test --packages-select honebot_follow && colcon test-result --verbose
```
With `--symlink-install`, Python edits take effect on next run; rebuild only after adding files or editing
`setup.py`/`package.xml`. **After building a new package, re-source `install/setup.bash` or open a new shell**
— otherwise `ros2` reports "Package not found".

### honebot_follow
- `control.follow_command(fx, fy, ftheta, lx, ly, gains) -> (linear, angular, distance)`:
  proportional heading control; forward speed ∝ (distance − follow_distance) × max(0, cos(heading_error));
  never reverses; clamps to max speeds; stops inside follow distance.
- `follower.py` node `follower`: spawns `turtle2` via `/spawn`, subscribes `/<leader>/pose` and
  `/<follower>/pose` (turtlesim/Pose), publishes `/<follower>/cmd_vel` (Twist) at 20 Hz. **Stops if the
  leader pose is older than `leader_timeout` (0.5 s).** Gains are ROS parameters and can be changed live
  (`ros2 param set /follower follow_distance 3.0`).
- Params: `leader`, `follower`, `spawn_follower`, `rate_hz`, `leader_timeout`, `follow_distance`,
  `kp_linear`, `kp_angular`, `max_linear`, `max_angular`.
- Run: `QT_QPA_PLATFORM=linuxfb ros2 launch honebot_follow turtle_follow.launch.py` on the HDMI console,
  then `ros2 run turtlesim turtle_teleop_key` from SSH. Owner confirmed it runs.

- **Real-robot options (2026-11-17, all off by default so turtlesim is unchanged; robot values in
  `honebot_bringup/config/robot.yaml`, started by `film.launch.py`):** `follow_bearing` (rad, + = left) holds the
  skater at a bearing instead of dead ahead: 0.45 puts the robot behind and to the right of the skater's line
  (1.2 / 2.1 / 3.2 m to the side at 4 / 8 / 12 mph). It still steers toward a stopped skater but adds margin when
  braking is worse than assumed (simulated, 12 mph, braking 2.0 instead of 4.4 m/s²: behind ends 0.1 m away, offset
  1.3 m); keep
  |follow_bearing| + `stopped_turn_deadband` ≤ 0.785 (camera pan). `brake_decel` > 0 switches to a speed-dependent gap,
  gap(v) = `follow_distance` + `time_headway`·v + v²/(2·`brake_decel`) (1.5 m / 0.5 s / 4.4 m/s² → 4.7 m at 8 mph,
  7.5 m at 12, 10.0 m at 15), commanded as the closed-form inverse `speed_for_gap(distance)` so there is no
  self-speed feedback loop; `min_turn_radius` (0.4) + `arc_speed` (0.6) + `pivot_bearing` (2.0 rad) roll on arcs
  instead of pivoting, with a gentle `pivot_rate` (0.8 rad/s) always allowed so the command is continuous through
  `follow_distance`; `max_lat_accel` (4.0) caps v·ω; `stopped_turn_deadband` (0.3 rad). `max_linear` starts at 3.0
  and must exceed the skater's speed. 27 tests, incl. a 20 Hz loop simulation with a dead stop at 8/12/15 mph and
  2-D offset-follow simulations (straight, both turns, dead stop).

### honebot_safety — `safety_gate` node
- Subscribes `cmd_vel_raw` (Twist), `ultrasonic/range`, `esp32/link_ok`; publishes `cmd_vel` at `rate_hz` (50)
  and `safety/state` (String). Logic in `gate.py: gate_command()`; priority: LINK_DOWN → RANGE_STALE →
  RANGE_INVALID → CMD_STALE → OBSTACLE (forward blocked, reverse/steer allowed) → SLOW (linear taper) → OK.
- Params (live-settable): `stop_distance` 0.4, `slow_distance` 1.0, `range_timeout` 0.3, `cmd_timeout` 0.5.
- Bench-verified 2026-11-13 against the live ESP32 (all six states, 50 Hz output, clean exit).
- **Sight limit (2026-11-17, off unless `brake_decel` > 0; on in `robot.yaml`):** forward speed is capped at what
  the robot can stop within what the sensor sees (`sight_speed`): 2.7 m/s if the HC-SR04 reads the ground at
  ~1.95 m, 3.8 m/s if it reads nothing (`sight_distance` 3.0). The cap drops at once and rises at
  `cap_rise_rate`. New state `SIGHT_LIMIT`. **Consequence: with only the HC-SR04 the robot is limited to ~6–8 mph;
  12–15 mph needs a longer-range forward sensor** (options in `docs/robot_design_v2.md`). Not run on the Pi yet.

### honebot_hardware — `esp32_bridge` node
- Params: `port` (`/dev/ttyUSB0`), `baud` (115200), `frame_id` (`ultrasonic_link`), `heartbeat_hz` (10),
  `cmd_rate_hz` (50), `cmd_timeout` (0.3), `send_commands` (true), **`drive_mode` (`diff` default |
  `ackermann`)**; diff params `track_width` (0.39 = 0.277 × 1.4 scrub guess, set from the spin test), `max_wheel_speed` (9.6 m/s = firmware `FULL_SCALE_MPS`, calibrate), `left_reversed`,
  `right_reversed`; Ackermann params `wheelbase` (0.33), `max_steer_angle` (0.45 rad), `max_speed` (5.0),
  `min_speed_for_steer` (0.3), `steer_reversed`, `throttle_reversed`. **All numbers are placeholders until the
  drivetrain exists.** The `C` line is `C,<left>,<right>` in diff mode, `C,<throttle>,<steer>` in ackermann.
- Publishes `/ultrasonic/range` (`sensor_msgs/Range`, ULTRASOUND, FOV 0.26 rad, 0.02–4.0 m,
  **`+inf` = no echo per REP 117**), `/esp32/link_ok` (Bool), `/esp32/failsafe` (Bool, from `S` lines),
  `/esp32/actuators` (Vector3: x = throttle, y = steer, what was sent).
- Subscribes `cmd_vel` (Twist) → `ackermann.twist_to_ackermann()` → sends `C,<thr>,<steer>` at 50 Hz;
  sends neutral when `cmd_vel` is older than `cmd_timeout`; sends one neutral on exit.
- Subscribes `camera/pan_cmd` (Float32, −1..1) → sends `P,<pan>` at `pan_rate_hz` (20) once a value has
  arrived; no timeout/neutral (not safety-relevant). `/esp32/actuators.z` = last pan sent.
- Sends heartbeat `H\n` at 10 Hz. Non-blocking reads on a 100 Hz timer; auto-reconnects if USB is unplugged;
  ignores non-protocol lines (ESP32 boot messages). Catches `ExternalShutdownException` for clean SIGTERM exit.
- Run: `ros2 launch honebot_hardware esp32.launch.py [port:=/dev/ttyACM0]`.

### honebot_uwb — `uwb_node`
- Reads `U,<r_left>,<r_right>` (two-anchor) or `A,<range>,<bearing_deg>` (angle-of-arrival) lines from a UWB
  board on `port` (`/dev/ttyUSB1`); filters (median-3 + EMA); publishes `target` (PointStamped, base_link)
  and `uwb/ranges`. Params: `baseline` (0.30), `anchor_x`, `max_range` (30), `filter_alpha`, `bearing_sign`.
  No fix → nothing published → follower's `target_timeout` stops the robot. **No UWB hardware yet; the
  firmware on the UWB board must print one of the two line formats.**
- **Bug fixed 2026-11-17:** `RangeFilter.update` used to return its last value for a "no fix" sample, so a lost tag
  kept producing fresh-stamped targets and the follower never timed out. It now returns `None` (nothing is
  published) and clears its history after 5 misses. AoA bearings are filtered too (`bearing_alpha`) and dropped
  outside `max_bearing_deg` (80).

### honebot_camera — `camera_node` + `camera_aim` (written 2026-11-13; tested on a video file and with a fake detection, not yet with the GoPro)
- `camera_node` params: `source` (`gopro` | `/dev/videoN` | URL | file, files loop), `hfov_deg` (87 = GoPro Linear,
  118 = Wide), `proc_width` (640), `detector` (`auto`|`hog`|`onnx`|`ort`), `model_path`, `input_size` (320),
  `conf_threshold`, `preview_hz` (5), `use_uwb_hint`, GoPro: `gopro_ip` (auto), `gopro_res` (1080), `gopro_fov`
  (4 linear), `record` (false; tees the raw H.264 UDP stream to `record_dir` `~/videos/*.ts`, no re-encode).
  Publishes `camera/target` (Vector3Stamped: x bearing rad +left, y elevation, z box-height fraction; only when
  someone is seen), `camera/image/compressed` (JPEG with boxes, for Foxglove), `camera/status` (String, 1 Hz).
  Subscribes `target` (UWB) + `camera/pan_angle`. **Detections pass through `track.py`** (2026-11-14):
  plausibility filters (`min_aspect` 0.6, `max_aspect` 6, `min_height_frac` 0.05, `min_bottom_y_norm` −0.3,
  size vs UWB distance `size_tol_low/high` 0.4/2.5), then a robot-frame Kalman tracker (`track_confirm_hits` 3,
  `track_coast_time` 1.0, gate `track_gate_min/max_deg` 6/25 at `track_n_sigma` 3, `track_meas_std_deg` 1,
  `track_accel_std_deg_s2` 90, `track_hint_gate_deg` 15), all live-settable. `camera/target` only when the
  confirmed track matched this cycle; status shows `seen=k/n track=...`.
  Threads: grabber keeps only the newest frame; detector thread; rclpy spins in main. Low-latency FFmpeg
  options via `OPENCV_FFMPEG_CAPTURE_OPTIONS` set before cv2 import.
- `camera_aim` params (live-settable): `kp` 0.6, `deadband_deg` 1, `max_rate_deg_s` 150, `half_range_deg` 45,
  `lost_timeout` 0.5, `recenter_timeout` 3, `recenter_rate_deg_s` 30, `pan_sign` ±1, `use_uwb_fallback`.
  Open-loop incremental: pan += kp·error; lost → steer to UWB bearing; no UWB → recentre after 3 s.
  Publishes `camera/pan_cmd` (Float32 −1..1, 20 Hz), `camera/pan_angle` (rad), `camera/bearing` (pan + image
  bearing = skater bearing in robot frame, only while seen).
- `gopro_ctl find|start [res fov]|stop|status` CLI for bench checks. GoPro HTTP: Open GoPro (`:8080/gopro/webcam/…`)
  first, fallback HERO8/9 legacy `/gp/gpWebcam/START?res=1080`. Stream: `udp://0.0.0.0:8554` MPEG-TS.
- No ffmpeg CLI on the Pi; OpenCV 4.6 (with FFmpeg) + numpy are. `python3-opencv` is the only apt dep.
  **onnxruntime 1.32 is installed in the user site** (`pip install --user --break-system-packages`, 2026-11-14)
  because OpenCV 4.6's `cv2.dnn` cannot run YOLOv8 ONNX files. **Default detector is now YOLOv8n** from
  `~/models/yolov8n.onnx` (exported on the Pi in the `~/yolo` venv; `python3-venv` + `python3-pip` installed
  2026-11-14), 10–11 det/s; launch files fall back to HOG if the file is missing.

### honebot_bringup
- `bringup.launch.py` args: `port`, `stop_distance`, `slow_distance`, `foxglove` (true → port 8765),
  `camera` (false; adds `camera.launch.py`), `camera_source`, `record`, `model_path`.
- `teleop_joy.launch.py` args: `joy_config` (xbox), `joy_dev`; output remapped to `/cmd_vel_raw`.
- `robot.launch.py` (bridge + gate + Foxglove with `config/robot.yaml`; port defaults to `/dev/esp32` if it exists)
  is what `honebot.service` starts. `film.launch.py` (args `max_linear`, `follow_bearing`, `uwb_port`, `camera`,
  `camera_source`, `record`, `bag`) starts uwb_node + follower + camera + `ros2 bag record -a`; launch args override
  the YAML only when given. Neither has been launched on the Pi yet (py_compile + config test only).
- `systemd/honebot.service`: installed by `scripts/pi_setup.sh` (owner runs it; status on the Pi unknown).

### Serial protocol (USB, 115200 8N1, ASCII lines ending `\n`)
| Direction | Message | Meaning |
|---|---|---|
| ESP32 → Pi | `V,<version>` | once at boot (firmware ≥ 2.0.0) |
| ESP32 → Pi | `R,<distance_m>,<link_ok>` | e.g. `R,1.335,1`. distance `-1` = no echo. link_ok = heartbeat seen within 500 ms |
| ESP32 → Pi | `S,<failsafe>,<throttle>,<steer>` | 10 Hz (firmware ≥ 2.0.0). failsafe 1 = outputs forced neutral; actual outputs −1..1 |
| Pi → ESP32 | `H` | heartbeat, 10 Hz |
| Pi → ESP32 | `C,<a>,<b>` | normalized −1..1, 50 Hz. diff mode: a = left, b = right wheel ESC; ackermann: a = throttle, b = steer. Ignored by firmware 1.x |
| ESP32 → Pi | `K,<required>,<remote>,<source>` | 10 Hz (firmware ≥ 2.2.0). Handheld remote / kill-switch state, see firmware section |
| Pi → ESP32 | `P,<pan>` | camera pan servo −1..1 (+1 = full left), 20 Hz (firmware ≥ 2.1.0; ignored by older). Slew-limited on the ESP32, held in fail-safe, centred at boot |

### ESP32 firmware — robot `esp32_ultrasonic` source v2.2.1, remote `esp32_remote` v1.0.0 (2026-11-17). **Last build confirmed on the board: a 2.x ≤ 2.1.0**; 2.2.x and the remote have passed host tests + a stub syntax check here, and the owner ran arduino-cli compile on the Mac 2026-11-17 (result not reported; no boot banner confirmed)
- Pings every **60 ms** (~16.7 Hz; 50 ms caused ~33% dropouts — datasheet asks ≥ 60 ms), echo timeout 25 ms.
- All decisions are in `drive_logic.h` (`drive::decide`, pure C++). Who may drive, first match wins: boot window
  (2 s) → nobody; remote STOP or lost (500 ms) → nobody; `REMOTE_REQUIRED` (default **true**) and never heard →
  nobody; remote RUN + stick deflected → manual (works without the Pi, capped `MANUAL_LIMIT` 0.25, **bypasses the
  Pi's ultrasonic gate**); Pi heartbeat (500 ms) + `C` (200 ms) → Pi.
- **Fail-safe ramps** both wheel channels together to neutral at `FAILSAFE_DECEL_G` 0.5 (hard neutral after
  2.5 s and at boot). Normal slew: `ACCEL_G` 0.8, `DECEL_G` 0.7 via `FULL_SCALE_MPS` 9.6. `THROTTLE_LIMIT` 0.5.
- **Motor direction lives in the firmware** (`THROTTLE_REVERSED` = left, `STEER_REVERSED` = right), not the
  bridge's `left_reversed`/`right_reversed`, so Pi and stick agree.
- New serial line `K,<required>,<remote 0 never/1 lost/2 stop/3 run>,<source 0 none/1 pi/2 manual>` at 10 Hz →
  bridge topics `/esp32/remote` (String), `/esp32/manual` (Bool); the bridge warns if `required` is 0.
- Remote: boots STOP; RUN = hold the stick button 1.5 s with the stick centred; any click in RUN = STOP;
  broadcast ESP-NOW packets at 50 Hz with a shared `HONEBOT_PAIR_ID` (no MAC pairing, no encryption);
  robot answers at 10 Hz for the remote's LED. ADC1 pins only (radio blocks ADC2).
- LED GPIO 2 (robot): on = someone may drive; slow blink = fail-safe with Pi link; fast blink = remote STOP/lost.
- `ESC_DEADBAND` (2.2.1, default 0 = off): lifts every non-zero wheel command above the ESC's dead zone; a slow
  pivot is only ~2 % per side and would otherwise do nothing. Owner measures it in `docs/first_drive.md` Stage 1.
- `P,<pan>` → GPIO 27 as before (`PAN_RATE_PER_S` 3.0), independent of fail-safe.
- `handleLine()` is the place to add new serial commands.

## 6. Verified results

### 2026-11-13 (on the Pi, live ESP32 with v1 firmware)
- 28/28 unit tests pass (6 control, 12 gate, 10 ackermann); 4 packages build.
- Safety gate bench test with the real bridge and `ros2 topic pub /cmd_vel_raw`: OK passes 1.0 m/s through;
  `stop_distance` raised above the 1.36 m reading → OBSTACLE (linear 0, angular kept); slow zone scaled
  1.0 → 0.60 m/s as predicted; killing the publisher → CMD_STALE; killing the bridge → LINK_DOWN.
  Output rate 50.0 Hz. Transitions logged.
- Full `bringup.launch.py`: three nodes up, Foxglove listening on 0.0.0.0:8765, Twist (2.5 m/s, 1.0 rad/s)
  reached the bridge as throttle 0.5 / steer 0.292 (= atan(0.33·1/2.5)/0.45) at 50 Hz, neutral after the
  controller stopped, gate and foxglove exit cleanly on SIGINT. Bridge exits ~5 s later (CP2102 close delay).

### 2026-11-12
- Firmware compiles (20% flash, 6% RAM) and is flashed.
- On the Mac: heartbeat → `link_ok` 1, stop heartbeat → 0 within 0.5 s.
- Accuracy: sensor on the floor facing a ~2.4 m ceiling read 2.392 m, stdev 1 mm, 0% dropouts over 5 s.
- On the Pi: `/ultrasonic/range` ≈ 1.33 m steady at **16.7 Hz**, `/esp32/link_ok` true; bridge exits cleanly.
- Both packages build on the Pi; 6/6 follower tests pass.

### 2026-11-13 (camera)
- 73/73 unit tests (14 control, 12 gate, 17 hardware, 11 uwb, 19 camera); 6 packages build.
  (`colcon test` lists `honebot_bringup` as "failed" only because it has no tests — pre-existing.)
- `camera_node` on a synthetic looping 1280×720 clip: HOG detector **8–10 det/s** at 640 px wide, preview at the
  set rate, status line OK, clean exit on SIGTERM after the thread-join fix.
- `camera_aim` + real `esp32_bridge` (ESP32 on v1 firmware): a constant fake `camera/target` x = 0.3 rad drove
  `pan_angle` to the 45° limit, `pan_cmd` 1.0, `/esp32/actuators.z` = 1.0 (P lines leaving the Pi),
  `camera/bearing` = 1.085; after the publisher stopped, pan recentred to 0 within the expected 3 s + 1.5 s.

### 2026-11-13 (GoPro live)
- GoPro HERO9 over USB (netplan installed, camera 172.22.120.51): webcam start works once the `port=` parameter is
  dropped; `camera_node` then publishes `/camera/image/compressed` at the preview rate (4.4 Hz measured) with the HOG
  detector. ESP32 firmware 2.x confirmed flashed (`S` lines → `/esp32/failsafe`). Pan servo not yet tested.

### 2026-11-14 (YOLO detector)
- `yolov8n.onnx` (320 px, opset 12) exported on the Pi; onnxruntime backend on the live GoPro stream:
  **10–11 det/s**, ~60 ms/inference standalone. On 40 live frames of an empty room HOG produced 6 false
  positives, YOLOv8n 0. `make_detector('auto')` now falls back cv2.dnn → onnxruntime (4 new tests, 23 camera
  tests pass). Launch defaults changed to `model_path:=~/models/yolov8n.onnx`. Odd-pose detection with a
  person in frame NOT yet checked (nobody was in view).
- Tracker (`track.py`, 19 tests; 42 camera tests, 96 total) integrated into `camera_node` and run live on the
  GoPro at 7–12 det/s: empty room stayed `track=none` for minutes (no phantom on the jacket/purifier); with the
  owner briefly in view the track confirmed and coasted with the prediction marker on them during a missed
  frame. Live `ros2 param set` of tracker/plausibility params works. **Recall measured live** with two different
  people walking in and out of view (standing mid-room, very close, face filling the frame): `seen=` 7/7, 6/6,
  12/12, 11/11 while in view (`track=locked`), 0/8 and 0/11 with nobody in view (`track=none`). Odd poses
  (crouch, jump, on the board) still to be tried outdoors.
- **Fixed: `camera_node` exited with code 1 on Ctrl-C/SIGINT** and left the GoPro in webcam mode. Cause:
  `FrameGrabber`/`UdpTee` (Thread subclasses) stored their stop flag as `self._stop`, which shadows
  `threading.Thread._stop()` that `join()` calls → `TypeError: 'Event' object is not callable`. Renamed to
  `_stop_evt`; SIGINT now exits 0 and logs `webcam stopped`.

### 2026-11-13 (later)
- 54/54 unit tests (14 control, 12 gate, 17 hardware, 11 uwb); 5 packages build.
- `turtle_follow.launch.py relative:=true ackermann:=true` on the HDMI screen: turtle2 chased turtle1 through
  the gate via pose_to_target → /target → follower → /cmd_vel_raw; target sat at 1.49 m straight ahead.

## 7. Gotchas already hit (don't repeat)
- Ubuntu Core has no apt → reinstalled Ubuntu Server 24.04.
- Reinstall changed the SSH host key → `ssh-keygen -R 192.168.10.122` on the Mac.
- Missing `noble-updates` → `-dev` package version conflicts during ROS install.
- `~/.bashrc` source line was skipped when an earlier chained install command failed.
- Old SSH sessions don't see new `dialout` membership or newly built packages → open a new session.
- HC-SR04 Echo is 5 V → voltage divider required for the ESP32.
- ROS CLI tools (`ros2 topic echo/hz`) can take several seconds to discover topics right after start on the Pi.
- **Closing the ESP32 serial port blocks ~5 s** (both the old and new bridge). The CP2102 does not answer the
  cp210x driver's tx-empty query, so close() waits for the 5 s USB control timeout (`closing_wait` is 30 s,
  not the cause). Harmless (the ESP32 fails safe on its own) but makes `ros2 launch` shutdown slow. Fix needs
  sudo: `sudo apt install setserial` + the udev rule in `honebot_hardware/udev/` (or one-off
  `sudo setserial /dev/ttyUSB0 closing_wait none`).
- `ros2 launch ... --print` lists conditional nodes regardless of `IfCondition`; it is only a parse check.
- `pkill -f <pattern>` from a script kills the script itself if the pattern appears in the command line; anchor
  on the interpreter (`pkill -f '^/usr/bin/python3 .*esp32_bridge'`).
- An ad-hoc pyserial script sending `H` at 20 Hz never got `link_ok=1`, while the bridge node did immediately.
  Unexplained; trust the bridge, and test link behaviour through it.
- Background jobs started from a **non-interactive** bash (scripts, Claude's shell) ignore SIGINT: `kill -INT`
  on a `ros2 run … &` does nothing. Use `kill -TERM` on the node PID (all nodes handle SIGTERM cleanly).
- A YOLO ONNX output can arrive as (1, 84, N) or (1, N, 84); `decode_yolov8` tells them apart by N > 256.
- HERO9 (firmware HD9.01.01.72) returns HTTP 500 to `/gopro/webcam/start` if a `port=` parameter is present;
  `opengopro_start_path` now omits it for the default 8554. `/gopro/webcam/status` 1 = idle, 2 = streaming.
- **OpenCV 4.6 `cv2.dnn` loads a YOLOv8 ONNX file but asserts inside `forward()`** (`shape_utils.hpp:170`),
  opset 11 and 12 alike. Use onnxruntime (`detector:=ort`, or `auto` which falls back by itself).
- `python3 -I` hides user-site packages (onnxruntime); don't use `-I` for scripts that need them.
- **Never name an attribute `_stop` on a `threading.Thread` subclass**: `Thread.join()` calls `self._stop()`.
- `pgrep -f camera_node` matches the `ros2 run … camera_node` wrapper too (lower pid, first in the list);
  signal the node itself, i.e. the process whose cmdline starts with `/usr/bin/python3 …/install/…/camera_node`.
  Sending SIGINT/SIGTERM to the wrapper or to `ros2 launch` started from a non-interactive shell can orphan
  the node.
- Only one process can own the GoPro UDP stream (port 8554); stop any running `camera_node` before starting
  another, or the second one reads nothing.

## 8. Plan for the rest of the build

Each phase ends with a concrete test. Keep `control.py`-style pure logic separate from ROS plumbing so it can
be unit-tested and reused when inputs change from turtlesim → UWB.

**Plan review (2026-11-13):** the original plan was sound; these changes were made for a better chance of
success: (a) the safety gate also stops when the *controller* goes silent and publishes at a fixed rate;
(b) the Twist→throttle/steer conversion lives inside the ESP32 bridge as a pure tested module instead of a
separate node, avoiding a custom message type; (c) `foxglove_bridge`, `joy` and `teleop_twist_joy` were found
already installed, so Phase 3 shrinks to a bringup launch + systemd unit; (d) an Ackermann car cannot turn in
place, which the Phase 6 refactor must handle.

### Phase 0 — housekeeping — DONE 2026-11-13
- `git init` (branch `main`) with `.gitignore` for `build/ install/ log/`; initial commit made.
- Still to do by the owner: push to a **private** GitHub repo as a backup.

### Phase 1 — obstacle safety stop — DONE 2026-11-13 (bench-verified; turtlesim demo not yet run by the owner)
- New package `honebot_safety`, node `safety_gate`. Subscribes `/cmd_vel_raw` (controller intent),
  `/ultrasonic/range`, `/esp32/link_ok`; publishes `/cmd_vel` at a fixed rate (`rate_hz`, 50).
- Pure function `gate.py: gate_command(...)` decides, in priority order:
  1. ESP32 link down → **stop** (zero linear and angular).
  2. Range stale (> `range_timeout`, 0.3 s) or never received → **stop**.
  3. Controller command stale (> `cmd_timeout`, 0.5 s) or never received → **stop**.
  4. Range < `stop_distance` (0.4 m) → forward speed forced to 0 (reverse and steering still allowed).
  5. Range < `slow_distance` (1.0 m) → forward speed scaled linearly down toward 0 at `stop_distance`.
  6. Otherwise pass the command through. `+inf` range = clear (REP 117); NaN = invalid → stop.
- Logs every state transition with the reason. Unit tests for every rule.
- `turtle_follow.launch.py` gains `use_safety:=true` which remaps the follower's output to `/cmd_vel_raw`
  and the gate's output to `/turtle2/cmd_vel`. Bench test: hand over the sensor → chasing turtle freezes;
  unplug USB → freezes; kill the follower → freezes.

### Phase 2 — drivetrain control — SOFTWARE DONE 2026-11-13 (diff + ackermann), firmware unflashed, no hardware yet
**Chassis decision 2026-11-13 (budget revision): small custom diff-drive robot (`docs/robot_design.md`).**
Concretely for Phase 2: 2 × 5010 outrunners with belts to longboard wheels, 2 × RC-car brushless ESCs on the
ESP32's channels A/B (servo PWM, fail-safe neutral on both; firmware `DIFF_DRIVE = true`), 3S LiPo; bridge
`drive_mode: diff`, follower `ackermann: false`. Build one drive module first, then the bare frame, then the body.
Owner constraints: backpack-sized, light, 12–15 mph, far cheaper than an RC car. The Ackermann text below stays
valid only for the tricycle variant or an RC car.
- Firmware: parse `C,<throttle>,<steer>` (−1..1); output 50 Hz servo pulses (1000–2000 µs, 1500 neutral)
  with the ESP32 LEDC peripheral (**core 3.x API: `ledcAttach(pin, 50, 16)` / `ledcWrite(pin, duty)`**)
  on `THROTTLE_PIN` and `STEER_PIN`. **Neutral throttle whenever `linkOk` is false or no `C` for
  > 200 ms.** Clamp and rate-limit throttle on the ESP32 too. ESC arming: hold neutral ~2 s at boot.
  Emit `S,<failsafe>` lines so the Pi can see when the ESP32 has forced neutral, and a `V,<version>`
  banner at boot so the running firmware can be identified. Keep the `R` line format unchanged.
- Pi side: `honebot_hardware/ackermann.py` (pure, tested): Twist (v, ω) → normalized throttle/steer using
  `wheelbase`, `max_steer_angle`, `max_speed`; `δ = atan(L·ω/max(|v|, v_min_steer))`. The bridge
  subscribes `/cmd_vel`, converts, and sends `C` lines at ≤ 50 Hz; sends neutral if `/cmd_vel` is older
  than `cmd_timeout`. Publishes `/esp32/failsafe` (Bool) from the `S` lines.
- Gamepad teleop via `joy` + `teleop_twist_joy` (already installed) → `/cmd_vel_raw` → gate → bridge is the
  manual-driving path (no RC radio).
- Test on a stand with wheels off the ground first. Firmware is compiled/flashed from the Mac unless
  arduino-cli is installed on the Pi (open decision 4). Consider micro-ROS later only if the line protocol
  becomes limiting.

### Phase 3 — remote monitoring and auto-start — LAUNCH DONE 2026-11-13; systemd/udev await owner sudo
- New package `honebot_bringup` with `bringup.launch.py`: ESP32 bridge + safety gate + `foxglove_bridge`
  (`foxglove:=true`, port 8765). Foxglove desktop app on the Mac connects to `ws://honebot.local:8765`.
- `systemd/honebot.service` kept in the repo; owner installs it with `sudo` (commands in the README).
  Clean SIGTERM shutdown is already supported by all nodes.
- Static IP / DHCP reservation for the Pi (router setting, owner).

### Phase 4 — camera and recording — SOFTWARE DONE 2026-11-13 (`honebot_camera`), untested with the GoPro
- Two ways to use the HERO9 (`docs/camera_plan.md` §1): **A** webcam mode over USB → Pi sees 1080p30 and can
  record the same stream (`record:=true`, `.ts`, zero CPU) but the GoPro cannot record to SD; **B** GoPro records
  natively and a small USB webcam on the same pan platform feeds the detector. Same software, `source` param.
- `camera_node` reads `gopro` / `/dev/videoN` / URL / file with OpenCV (no `usb_cam`, no ffmpeg CLI needed).
- Owner to do: install the netplan file (sudo), plug the GoPro in, `gopro_ctl find/start`, then
  `camera.launch.py` and look at `/camera/image/compressed` in Foxglove.

### Phase 5 — person detection to aim the camera — SOFTWARE DONE 2026-11-13, needs the servo + a model
- Detector backends: HOG (default, no download, ~8–10 Hz), YOLOv8n/YOLO11n ONNX via cv2.dnn or onnxruntime
  (export on the Mac, `docs/camera_plan.md` §5). Target choice uses the UWB bearing when available.
- `camera_aim` → `P,<pan>` → ESP32 GPIO 27 pan servo (firmware 2.1.0). Open-loop incremental P control, slew
  limits, UWB fallback, recentre. Tilt not planned. Hailo AI HAT+ only if CPU FPS proves too low.
- Owner to do: buy the servo, flash firmware 2.1.0, bench-test with `ros2 topic pub /camera/pan_cmd`, set
  `half_range_deg`/`pan_sign` from the measured travel, then run the full camera stack on the HDMI/Foxglove.

### Phase 6 — UWB following — CONTROLLER REFACTOR DONE 2026-11-13 (unit-tested; on-screen run of relative:=true NOT yet done)
- `control.follow_command_relative(distance, bearing, gains)` is now the core; `follow_command` wraps it.
  `FollowGains.ackermann=True` creeps forward at `creep_speed` instead of turning in place.
- `follower` node: `target_topic` (PointStamped, robot frame) switches it to relative mode and publishes to
  `cmd_vel_topic` (default `cmd_vel_raw`). `spawn_follower` now defaults to **False**; the launch sets it.
- `pose_to_target` node = simulated UWB for turtlesim. `turtle_follow.launch.py relative:=true [ackermann:=true]`.
- `honebot_bringup/launch/record.launch.py` records a bag to `~/bags/`.
- Owner also verified the turtlesim demo with the gate on the HDMI screen (2026-11-13). Running GUI from SSH:
  owner is now in group `video`; use `sg video -c "exec bash -c 'source ... && QT_QPA_PLATFORM=linuxfb ros2 launch ...'"`
  (sg strips LD_LIBRARY_PATH and uses /bin/sh, hence the bash wrapper and re-sourcing).
- UWB tag on the skater, 2+ anchors on the robot → range and bearing (or relative x, y) to the skater.
- Node `uwb_node` publishing the target relative to the robot (e.g. `geometry_msgs/PointStamped` in
  `base_link`).
- Refactor the follower to work from a **relative** target (distance, bearing) instead of two world poses:
  add `follow_command_relative(distance, bearing, gains)` to `control.py` with tests; keep the turtlesim
  path working by converting poses to relative form. Choose a filming offset (e.g. 3 m behind, slightly
  to the side). Stale UWB → stop (same pattern as `leader_timeout`).
- **Ackermann constraint:** the car cannot turn in place. Today's "leader behind → spin at zero speed" must
  become "drive a slow forward arc at full steer" or "stop and wait" — decide and test in simulation first.
- Optional: Gazebo (Harmonic) simulation of an Ackermann car on a PC/VM for tuning before real runs.

### Phase 7 — integration and field testing
- Wireless kill switch (second ESP32, ESP-NOW) handled entirely on the ESP32 side.
- Test order: walking speed in an empty lot → jogging → rolling on the board → full speed. Tune gains,
  add speed-dependent follow distance, and log everything with `ros2 bag`.
- A single HC-SR04 (~4 m, 16 Hz, narrow) is only a low-speed safety layer; consider additional/longer-range
  sensors for higher speed.

### CAD (2026-11-16)
- Edit `PARAMS` in `cad/fusion/HonebotChassis/HonebotChassis.py`, then on any machine with numpy + matplotlib:
  `python3 cad/check_design.py` (≈ 4 min; exit code 0 = all checks pass) and `python3 cad/tests/test_fusion_script.py`.
- Software implications: robot frame origin = **front**-axle midpoint; track 0.277 m; wheelbase 0.200 m (4-wheel skid
  steer). Skid steer scrubs when turning: the real yaw rate is below (vR − vL)/track → set the bridge's `track_width`
  1.2–1.6 × 0.277 after a spin test; pivoting costs ~10 A per motor. Forward pitch-over at ≈ 0.88 g; sideways tip at
  1.77 g. Firmware 2.2.x ramps to zero at 0.5 g on fail-safe. Stop distance
  ≈ 4.5–5.5 m from 15 mph → speed-dependent follow distance.
- HC-SR04 now tilted 12° up at 102 mm height (ground return ≈ 1.95 m).

### Immediate next steps (in order)
0. **Owner:** read `docs/chassis_cad.md` (current design + BOM; `docs/robot_design.md` §1/§3 for the AM32 setup) and
   `docs/hardware_plan.md`; order per the chassis_cad.md BOM: 2 × MAD 5010 EEE 370KV (or 5010 with M3 prop holes on
   D12), 2 × AM32 ESCs + USB linker, 2 × HTD-3M 258 mm + 2 × 510 mm 9 mm belts, a 4-pack of 90 mm longboard wheels,
   4 × 608-2RS idlers, 4 × M8 × 140 12.9 bolts, 16×2 and 12×2 steel tube, heat-set inserts, 3S 5000 mAh + charger, Pololu 5 V/6 V regulators,
   ESP32 3-pack, UWB AoA anchor + tag. Measure the backpack.
   **Then follow `docs/first_drive.md` stage by stage; it supersedes the list below where they overlap.**
1. **Owner (Mac):** 2026-11-17 the **robot board is flashed with a 2.2.x build**: its serial output on the Mac
   showed `R,-1.000,0`, `S,1,0.000,0.000` and `K,1,0,0` (the `K` line exists only from 2.2.0; `K,1,0,0` = remote
   required, never heard, nobody driving, as expected). The `V` banner was not captured, so 2.2.0 vs 2.2.1 is
   unknown: reflash with 2.2.1 (`ESC_DEADBAND`) and press EN with the monitor open. **The remote board is not
   flashed yet.** Flash both. Remote bench check per
   `firmware/esp32_remote/README.md` (stick directions, STOP/RUN, `robot=1`). On the Pi the bridge log should show
   `ESP32 firmware version 2.2.1`; `/esp32/remote` follows the remote; `/esp32/failsafe` goes false only with the
   remote in RUN and `/cmd_vel` fed.
2. **Owner (Pi):** `git pull`, `colcon build --symlink-install`, then `./scripts/pi_setup.sh` (asks for sudo:
   setserial, udev rule, `honebot.service`). Once the service runs, `sudo systemctl stop honebot` before launching
   bringup by hand. The udev rule matches any CP2102: revisit it when a second CP2102 board (UWB) is plugged in.
2b. **Owner (camera):** GoPro on, USB Connection = GoPro Connect, plug into the Pi, `ros2 run honebot_camera gopro_ctl find`
   then `ros2 launch honebot_camera camera.launch.py aim:=false` → `/camera/image/compressed` in Foxglove shows
   video and (standing in front of it) a box. Then wire a servo to GPIO 27 and run with `aim:=true`.
3. **Owner:** run the turtlesim demo with `use_safety:=true` on the HDMI console and do the hand-over-sensor test.
4. **Owner:** push the repo to a private GitHub remote.
5. Configure the ESCs (bidirectional), build one drive module, bench-test `C` commands with `THROTTLE_LIMIT` low.
6. Then Phase 4 (USB webcam) or Phase 6 refactor (`follow_command_relative`), whichever hardware arrives first.

### Decisions made (2026-11-13) and still open
- Chassis: custom printed diff-drive robot (`docs/robot_design.md`). ESCs: AM32 in bidirectional mode.
- UWB: Makerfabs MaUWB_AOA Gen2, one anchor + one tag, to buy.
- Filming: owner's GoPro on a fixed damped mount; no gimbal.
- Camera (2026-11-13): HERO9 on a **pan servo** (GPIO 27), no tilt; GoPro webcam mode for development with
  Pi-side recording; native GoPro recording + a small USB webcam for vision as the filming-quality option.
- Still open: install arduino-cli on the Pi for in-place flashing (recommended; no `sudo` if under `~/bin`);
  backpack inner dimensions; which longboard wheels (pulley-mountable core); servo model; whether the GoPro's
  1080p30 webcam footage is good enough or mode B (second camera) is wanted.
