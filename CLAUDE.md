# honebot — project handoff

## 1. Final goal

An autonomous ground robot that **follows the owner while they skateboard and films their tricks**.

Hard requirements that shape every decision:
- **Speed.** A skater moves 15–25 km/h with sudden stops and turns. The base must be a fast, car-like
  (Ackermann) chassis, not a slow differential-drive kit.
- **Robust following.** Vision alone at Pi-CPU frame rates is too slow and fragile to *steer* on. The plan is
  **UWB (ultra-wideband) ranging** for following (tag on the skater, anchors on the robot) and **vision only
  to aim/frame the camera**.
- **Safety first.** A 1/10-scale car at 25 km/h can hurt someone. Every layer must fail safe to "stop":
  ESP32 watchdog, obstacle stop, stale-target stop, wireless kill switch, RC manual override. Test in empty
  lots; public streets/sidewalks may not be legal.
- **Good footage.** Camera (action cam/phone or Pi camera) on a 3-axis gimbal, recording while following.

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
| ESP32 DevKit (WROOM-32 class), USB chip **Silicon Labs CP210x** (10c4:ea60) | In use, low-level controller |
| HC-SR04 ultrasonic sensor | Wired to ESP32 and verified |
| Monitor via HDMI on the Pi | Available for local GUI testing |
| Chassis (1/10 brushless RC car + ESC + steering servo) | **Not chosen yet** |
| UWB modules (e.g. DWM3000 / ESP32+DW3000 boards) | **Not chosen yet** |
| Camera, gimbal | **Not chosen yet** (USB webcam suggested to start vision work) |
| Wireless kill switch (2nd ESP32 over ESP-NOW) | Planned |

### Wiring (ESP32 ↔ HC-SR04) — verified working
| HC-SR04 | ESP32 |
|---|---|
| VCC | VIN (5 V) |
| GND | GND |
| Trig | GPIO 5 |
| Echo | 1 kΩ → GPIO 18, and 2 kΩ from GPIO 18 → GND (5 V → 3.3 V divider) |

Onboard LED = GPIO 2 (used as the heartbeat/link indicator).

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
- `python3-serial` (pyserial 3.5) installed. git 2.43 installed. **The workspace is not under git yet.**
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
├── CLAUDE.md                         this file
├── firmware/
│   └── esp32_ultrasonic/esp32_ultrasonic.ino   ESP32 firmware (Arduino)
└── src/
    ├── honebot_follow/               ament_python — follow-me controller
    │   ├── honebot_follow/control.py     pure math, no ROS (unit-tested)
    │   ├── honebot_follow/follower.py    ROS node, currently drives turtlesim
    │   ├── launch/turtle_follow.launch.py
    │   └── test/test_control.py          6 pytest tests
    └── honebot_hardware/             ament_python — Pi ↔ ESP32 serial bridge
        ├── honebot_hardware/esp32_bridge.py
        └── launch/esp32.launch.py
```

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

### honebot_hardware — `esp32_bridge` node
- Params: `port` (`/dev/ttyUSB0`), `baud` (115200), `frame_id` (`ultrasonic_link`), `heartbeat_hz` (10).
- Publishes `/ultrasonic/range` (`sensor_msgs/Range`, ULTRASOUND, FOV 0.26 rad, 0.02–4.0 m,
  **`+inf` = no echo per REP 117**) and `/esp32/link_ok` (`std_msgs/Bool`).
- Sends heartbeat `H\n` at 10 Hz. Non-blocking reads on a 100 Hz timer; auto-reconnects if USB is unplugged;
  ignores non-protocol lines (ESP32 boot messages). Catches `ExternalShutdownException` for clean SIGTERM exit.
- Run: `ros2 launch honebot_hardware esp32.launch.py [port:=/dev/ttyACM0]`.

### Serial protocol (USB, 115200 8N1, ASCII lines ending `\n`)
| Direction | Message | Meaning |
|---|---|---|
| ESP32 → Pi | `R,<distance_m>,<link_ok>` | e.g. `R,1.335,1`. distance `-1` = no echo. link_ok = heartbeat seen within 500 ms |
| Pi → ESP32 | `H` | heartbeat |
| (planned) Pi → ESP32 | `C,<throttle>,<steer>` | normalized −1..1 commands; see Phase 2 |

### ESP32 firmware (`esp32_ultrasonic.ino`)
- Pings every **60 ms** (~16.7 Hz; 50 ms caused ~33% dropouts — datasheet asks ≥ 60 ms), echo timeout 25 ms.
- Watchdog: `linkOk = heartbeat within WATCHDOG_MS (500)`; LED on GPIO 2 mirrors it.
  This is the hook where motors will be forced to neutral.
- `handleLine()` is the place to add new commands.

## 6. Verified results (2026-11-12)
- Firmware compiles (20% flash, 6% RAM) and is flashed.
- On the Mac: heartbeat → `link_ok` 1, stop heartbeat → 0 within 0.5 s.
- Accuracy: sensor on the floor facing a ~2.4 m ceiling read 2.392 m, stdev 1 mm, 0% dropouts over 5 s.
- On the Pi: `/ultrasonic/range` ≈ 1.33 m steady at **16.7 Hz**, `/esp32/link_ok` true; bridge exits cleanly.
- Both packages build on the Pi; 6/6 follower tests pass.

## 7. Gotchas already hit (don't repeat)
- Ubuntu Core has no apt → reinstalled Ubuntu Server 24.04.
- Reinstall changed the SSH host key → `ssh-keygen -R 192.168.10.122` on the Mac.
- Missing `noble-updates` → `-dev` package version conflicts during ROS install.
- `~/.bashrc` source line was skipped when an earlier chained install command failed.
- Old SSH sessions don't see new `dialout` membership or newly built packages → open a new session.
- HC-SR04 Echo is 5 V → voltage divider required for the ESP32.
- ROS CLI tools (`ros2 topic echo/hz`) can take several seconds to discover topics right after start on the Pi.

## 8. Plan for the rest of the build

Each phase ends with a concrete test. Keep `control.py`-style pure logic separate from ROS plumbing so it can
be unit-tested and reused when inputs change from turtlesim → UWB.

### Phase 0 — housekeeping (do first)
- `git init` in `~/honebot_ws` with a `.gitignore` for `build/ install/ log/`; commit current state.
  Suggest the owner push to a private GitHub repo as a backup.
- Optional: `honebot_bringup` package for top-level launch files.

### Phase 1 — obstacle safety stop (bench, uses existing hardware)
- New node `safety_gate` (in a new `honebot_safety` package or in `honebot_follow`): subscribes to the
  controller's raw command (`/cmd_vel_raw`), `/ultrasonic/range`, `/esp32/link_ok`; publishes `/cmd_vel`.
  Forces zero linear velocity when `range < stop_distance` (e.g. 0.4 m, param), when range data is stale
  (> 0.3 s), or when link_ok is false. Allows turning in place. Logs state transitions.
- Remap the turtlesim follower through it. Test: hand over the sensor → chasing turtle freezes; unplug
  USB → freezes.
- Unit-test the gate logic as pure functions.

### Phase 2 — motor/steering control on the ESP32 (needs ESC + servo; a bare servo is enough to start)
- Extend firmware: parse `C,<throttle>,<steer>` (−1..1), output 50 Hz servo PWM (1000–2000 µs, 1500 neutral)
  with the ESP32 LEDC peripheral on two GPIOs. **Neutral throttle whenever linkOk is false or no `C` for
  > 200 ms.** Clamp and rate-limit throttle on the ESP32 too. ESC arming sequence (neutral for ~2 s at boot).
- Optional RC receiver input on the ESP32 for manual override plus a kill channel (RC always wins).
- Pi side: a `cmd_vel_to_ackermann` node mapping Twist (v, ω) → throttle/steer using wheelbase and max steer
  angle (`δ = atan(L·ω/v)`), plus `/esp32/cmd` sending. Gamepad teleop via `joy` + `teleop_twist_joy`.
- Test on a stand with wheels off the ground first. Consider micro-ROS later only if the serial protocol
  becomes limiting; the line protocol is easy to debug.

### Phase 3 — remote monitoring and auto-start
- `ros-jazzy-foxglove-bridge` on the Pi; Foxglove desktop app on the Mac (`ws://<pi-ip>:8765`).
- systemd service that launches the bringup at boot (with clean SIGTERM shutdown already supported).
- Static IP / DHCP reservation for the Pi.

### Phase 4 — camera and recording
- Start with a USB webcam (`ros-jazzy-usb-cam` or `v4l2_camera`). Pi Camera Module 3 on Ubuntu + Pi 5
  needs Raspberry Pi's libcamera fork + `camera_ros`; harder — defer.
- Recording: `ros2 bag record` for sensor data (debugging/tuning); for footage, record video directly
  (or the action cam records on its own and the robot just keeps it framed).

### Phase 5 — person detection to aim the camera
- Small detector (YOLO nano class, via ONNX Runtime / NCNN on CPU) publishing the target's bearing and box.
  CPU-only will be a few FPS — fine for aiming, not for steering. The Raspberry Pi AI HAT+ (Hailo) is
  faster but its software is best supported on Raspberry Pi OS; on Ubuntu expect to build drivers.
- Gimbal control from bearing (later, hardware-dependent).

### Phase 6 — UWB following
- UWB tag on the skater, 2+ anchors on the robot → range and bearing (or relative x, y) to the skater.
- Node `uwb_node` publishing the target relative to the robot (e.g. `geometry_msgs/PointStamped` in
  `base_link`).
- Refactor the follower to work from a **relative** target (distance, bearing) instead of two world poses:
  add `follow_command_relative(distance, bearing, gains)` to `control.py` with tests; keep the turtlesim
  path working by converting poses to relative form. Choose a filming offset (e.g. 3 m behind, slightly
  to the side). Stale UWB → stop (same pattern as `leader_timeout`).
- Optional: Gazebo (Harmonic) simulation of an Ackermann car on a PC/VM for tuning before real runs.

### Phase 7 — integration and field testing
- Wireless kill switch (second ESP32, ESP-NOW) handled entirely on the ESP32 side.
- Test order: walking speed in an empty lot → jogging → rolling on the board → full speed. Tune gains,
  add speed-dependent follow distance, and log everything with `ros2 bag`.
- A single HC-SR04 (~4 m, 16 Hz, narrow) is only a low-speed safety layer; consider additional/longer-range
  sensors for higher speed.

### Open decisions for the owner
1. Chassis model (drives ESC, servo, size, top speed).
2. UWB module family.
3. Filming approach: action cam on a gimbal vs Pi camera.
4. Whether to install arduino-cli on the Pi for in-place flashing.
