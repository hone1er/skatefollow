# honebot — project handoff

## 1. Final goal

An autonomous ground robot that **follows the owner while they skateboard and films their tricks**.

Hard requirements that shape every decision:
- **Speed.** Owner target (2026-11-13): **12–15 mph (19–24 km/h)** with sudden stops and turns. Decided base: a
  **small custom differential-drive robot** (two belt-driven longboard wheels + caster, printed frame), see
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
| Chassis | **Decided: small custom printed diff-drive robot** — 2 × 5010 360KV outrunners, 2.4:1 GT2 belts to 90 mm longboard wheels, 2 × AM32 ESCs (bidirectional mode) on the ESP32 servo channels, 3S 5000 mAh, PAHT-CF frame + PETG body. ≈ 2.5 kg, backpack-sized, 12–15 mph, ≈ $150–260 + battery/charger. Nothing bought yet. Design + BOM: `docs/robot_design.md` (§2a = drive-module drawings) |
| UWB modules | **Recommended: Makerfabs MaUWB_AOA Gen2** (angle-of-arrival anchor $39.80 + tag $29.80); not bought. See `docs/hardware_plan.md` §3 |
| Vision camera (Phase 5) | GoPro in USB-webcam mode (mode A: Pi records the 1080p30 stream, GoPro cannot record to SD meanwhile) or any USB webcam next to it (mode B: GoPro records natively). `source` parameter picks |
| ESC signal pins | **Firmware ready, nothing wired**: channel A GPIO 25 (left ESC), channel B GPIO 26 (right ESC); both 50 Hz 1000–2000 µs, neutral on fail-safe. **Pan servo GPIO 27** (firmware 2.1.0, holds last value in fail-safe, centred at boot) |
| Wireless kill switch (2nd ESP32 over ESP-NOW) | Planned (Phase 7); no RC radio planned, gamepad for manual driving |

### Wiring (ESP32 ↔ HC-SR04) — verified working
| HC-SR04 | ESP32 |
|---|---|
| VCC | VIN (5 V) |
| GND | GND |
| Trig | GPIO 5 |
| Echo | 1 kΩ → GPIO 18, and 2 kΩ from GPIO 18 → GND (5 V → 3.3 V divider) |

Onboard LED = GPIO 2 (used as the heartbeat/link indicator). Pins used: 2, 5, 18, 25, 26, 27.
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
  with local identity set; not pushed anywhere yet.
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
  arduino-cli compile --fqbn esp32:esp32:esp32 honebot_ws/firmware/esp32_ultrasonic
  arduino-cli upload  -p /dev/cu.usbserial-0001 --fqbn esp32:esp32:esp32 honebot_ws/firmware/esp32_ultrasonic
  ```
  (Arduino's ctags tool is x86-only; that is why Rosetta is installed.) arduino-cli is *not* installed on the
  Pi yet; installing it there would allow flashing the ESP32 in place over `/dev/ttyUSB0`.
- The owner edits code with VS Code Remote-SSH directly in `/home/hone1er/honebot_ws` on the Pi.
  **The Pi copy is the source of truth.**

## 5. Workspace layout (`~/honebot_ws` on the Pi)

```
honebot_ws/
├── CLAUDE.md                         this file (handoff); README.md = architecture + run card; each src/honebot_*/ and firmware/esp32_ultrasonic/ has its own README
├── firmware/
│   └── esp32_ultrasonic/esp32_ultrasonic.ino   ESP32 firmware v2.1.0 (Arduino, core 3.x)
└── src/
    ├── honebot_follow/               ament_python — follow-me controller
    │   ├── honebot_follow/control.py     pure math, no ROS (unit-tested)
    │   ├── honebot_follow/follower.py    ROS node: turtlesim poses or a relative /target
    │   ├── honebot_follow/pose_to_target.py   simulated UWB from turtlesim poses
    │   ├── launch/turtle_follow.launch.py   use_safety / relative / ackermann options
    │   └── test/test_control.py          14 pytest tests
    ├── honebot_safety/               ament_python — safety gate
    │   ├── honebot_safety/gate.py        pure gate logic (unit-tested)
    │   ├── honebot_safety/safety_gate.py ROS node
    │   ├── launch/safety_gate.launch.py
    │   └── test/test_gate.py             12 pytest tests
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
    │   └── test/test_localize.py         11 pytest tests
    ├── honebot_camera/               ament_python — GoPro/webcam in, person detection, pan aiming
    │   ├── honebot_camera/gopro.py       find GoPro on its USB network link, HTTP webcam start/stop (CLI gopro_ctl)
    │   ├── honebot_camera/detector.py    YOLOv8 ONNX decode/NMS/target selection (pure) + HOG/cv2.dnn/onnxruntime backends
    │   ├── honebot_camera/aim.py         pixel→bearing, open-loop PanController (pure, unit-tested)
    │   ├── honebot_camera/camera_node.py frames → detections → camera/target, JPEG preview, raw-stream recorder
    │   ├── honebot_camera/camera_aim.py  camera/target (+UWB fallback) → camera/pan_cmd, pan_angle, bearing
    │   ├── launch/camera.launch.py       source / record / model_path / aim options
    │   ├── netplan/99-gopro-usb.yaml     DHCP on usb0/enx* without taking the GoPro's default route (owner installs)
    │   └── test/                          19 pytest tests
    └── honebot_bringup/              ament_python — top-level launch + systemd
        ├── launch/bringup.launch.py      esp32 bridge + safety gate + foxglove_bridge [+ camera:=true]
        ├── launch/teleop_joy.launch.py   joy + teleop_twist_joy → /cmd_vel_raw
        ├── launch/record.launch.py       ros2 bag record -a → ~/bags/
        └── systemd/honebot.service       auto-start unit (owner installs with sudo)
docs/
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

### honebot_safety — `safety_gate` node
- Subscribes `cmd_vel_raw` (Twist), `ultrasonic/range`, `esp32/link_ok`; publishes `cmd_vel` at `rate_hz` (50)
  and `safety/state` (String). Logic in `gate.py: gate_command()`; priority: LINK_DOWN → RANGE_STALE →
  RANGE_INVALID → CMD_STALE → OBSTACLE (forward blocked, reverse/steer allowed) → SLOW (linear taper) → OK.
- Params (live-settable): `stop_distance` 0.4, `slow_distance` 1.0, `range_timeout` 0.3, `cmd_timeout` 0.5.
- Bench-verified 2026-11-13 against the live ESP32 (all six states, 50 Hz output, clean exit).

### honebot_hardware — `esp32_bridge` node
- Params: `port` (`/dev/ttyUSB0`), `baud` (115200), `frame_id` (`ultrasonic_link`), `heartbeat_hz` (10),
  `cmd_rate_hz` (50), `cmd_timeout` (0.3), `send_commands` (true), **`drive_mode` (`diff` default |
  `ackermann`)**; diff params `track_width` (0.40), `max_wheel_speed` (7.0 m/s), `left_reversed`,
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

### honebot_camera — `camera_node` + `camera_aim` (written 2026-11-13; tested on a video file and with a fake detection, not yet with the GoPro)
- `camera_node` params: `source` (`gopro` | `/dev/videoN` | URL | file, files loop), `hfov_deg` (87 = GoPro Linear,
  118 = Wide), `proc_width` (640), `detector` (`auto`|`hog`|`onnx`|`ort`), `model_path`, `input_size` (320),
  `conf_threshold`, `preview_hz` (5), `use_uwb_hint`, GoPro: `gopro_ip` (auto), `gopro_res` (1080), `gopro_fov`
  (4 linear), `record` (false; tees the raw H.264 UDP stream to `record_dir` `~/videos/*.ts`, no re-encode).
  Publishes `camera/target` (Vector3Stamped: x bearing rad +left, y elevation, z box-height fraction; only when
  someone is seen), `camera/image/compressed` (JPEG with boxes, for Foxglove), `camera/status` (String, 1 Hz).
  Subscribes `target` (UWB) + `camera/pan_angle` to pick the skater among several people.
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
- `systemd/honebot.service`: install commands in the file header. Not installed yet (needs sudo).

### Serial protocol (USB, 115200 8N1, ASCII lines ending `\n`)
| Direction | Message | Meaning |
|---|---|---|
| ESP32 → Pi | `V,<version>` | once at boot (firmware ≥ 2.0.0) |
| ESP32 → Pi | `R,<distance_m>,<link_ok>` | e.g. `R,1.335,1`. distance `-1` = no echo. link_ok = heartbeat seen within 500 ms |
| ESP32 → Pi | `S,<failsafe>,<throttle>,<steer>` | 10 Hz (firmware ≥ 2.0.0). failsafe 1 = outputs forced neutral; actual outputs −1..1 |
| Pi → ESP32 | `H` | heartbeat, 10 Hz |
| Pi → ESP32 | `C,<a>,<b>` | normalized −1..1, 50 Hz. diff mode: a = left, b = right wheel ESC; ackermann: a = throttle, b = steer. Ignored by firmware 1.x |
| Pi → ESP32 | `P,<pan>` | camera pan servo −1..1 (+1 = full left), 20 Hz (firmware ≥ 2.1.0; ignored by older). Slew-limited on the ESP32, held in fail-safe, centred at boot |

### ESP32 firmware (`esp32_ultrasonic.ino`) — source is v2.1.0; **a 2.x build is flashed** (verified 2026-11-13: `S` lines arrive, `/esp32/failsafe` publishes; exact version not yet read from the `V` banner)
- Pings every **60 ms** (~16.7 Hz; 50 ms caused ~33% dropouts — datasheet asks ≥ 60 ms), echo timeout 25 ms.
- Watchdog: `linkOk = heartbeat within WATCHDOG_MS (500)`; LED on GPIO 2 mirrors it.
- v2.0.0 (written 2026-11-13; compiled and flashed by the owner, `S` lines verified): LEDC 50 Hz servo pulses on GPIO 25 (throttle) / 26 (steer), 1000–2000 µs;
  `C` parsing with validation; fail-safe neutral when link lost, no `C` for 200 ms, or within the 2 s arming
  window; `THROTTLE_LIMIT` 0.5 and 2.0/s rate limit as bench caps; `S` status at 10 Hz; `V` banner.
- v2.1.0 (2026-11-13, host g++ syntax check only): `P,<pan>` → `PAN_PIN` GPIO 27, `PAN_RATE_PER_S` 3.0 slew,
  `PAN_TRIM_US`, `PAN_REVERSED`; independent of the drive fail-safe. `S` line unchanged.
- `handleLine()` is the place to add new commands.

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

### Immediate next steps (in order)
0. **Owner:** read `docs/robot_design.md` and `docs/hardware_plan.md`; order: 2 × 5010 360KV motors, 2 × AM32
   ESCs + USB linker, GT2 pulleys/belts, longboard wheels, 608 bearings, 3S 5000 mAh + charger, 5 V 5 A buck,
   ESP32 3-pack, UWB AoA anchor + tag. Measure the backpack.
1. **Owner (Mac):** `arduino-cli compile` firmware v2.1.0; fix any core-3.x compile errors; flash; on the Pi run
   `ros2 launch honebot_hardware esp32.launch.py` and check the log shows `ESP32 firmware version 2.1.0`,
   `/esp32/failsafe` goes false once `/cmd_vel` is fed (e.g. via bringup + `ros2 topic pub /cmd_vel_raw ...`).
2. **Owner (Pi, sudo):** install `setserial` + udev rule; install `honebot.service`; verify `journalctl -u honebot -f`;
   install `honebot_camera/netplan/99-gopro-usb.yaml` (`docs/camera_plan.md` §3).
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
