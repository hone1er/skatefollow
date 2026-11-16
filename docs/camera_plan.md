# honebot camera: GoPro HERO9 Black video-in, pan servo, person detection (2026-11-13)

Software: ROS 2 package `honebot_camera` (`camera_node`, `camera_aim`, `gopro_ctl`), the `P,<pan>` line in
the ESP32 bridge and firmware 2.1.0. This file covers how it fits the hardware and how to wire it.

## 1. The one decision to make: how the GoPro is used

A GoPro in **USB webcam mode cannot record to its own SD card**. So there are two ways to run it:

| Mode | What the Pi gets | What the footage is | When to use |
|---|---|---|---|
| **A. Webcam mode** (`source:=gopro`) | 1080p30 H.264 stream over USB, ~0.3–0.5 s latency | The Pi saves the *same* 1080p30 stream (`record:=true`, zero CPU, `~/videos/*.ts`). No HyperSmooth, no 4K/60. | Development, tuning the aiming, and runs where 1080p30 footage is fine. One camera, one cable. |
| **B. Native recording** (`source:=/dev/video0`) | Nothing from the GoPro. A cheap USB webcam on the same pan platform feeds the detector. | GoPro's full quality (4K/5K, HyperSmooth), started by hand or the GoPro remote. | The real filming runs. Costs a webcam (~$20–70) and a second cable. |

The software is the same either way (`source` parameter). Build the pan mount so a small webcam can sit on
it next to the GoPro later; start with mode A because it needs nothing new.

**HERO9 details.** Webcam mode needs firmware ≥ 1.60 (update with the Quik app) and
Preferences → Connections → **USB Connection = GoPro Connect** (not MTP). Set Auto Power Off = Never.
Over USB the camera is a network adapter (see §3). In webcam mode it gets warm; leave the mount open at
the back and on the sides and keep the battery in (it buffers the Pi's limited USB current).

## 2. Wiring

### 2a. GoPro → Pi (video in)
```
GoPro HERO9 side USB-C port  ──USB-C→USB-A (or C) cable, ≤ 1 m──  Pi 5 USB 3 port (blue)
```
Nothing else. The GoPro appears as a USB network interface (`usb0` or `enx…`) and hands the Pi the
address `172.2X.1YZ.50` (this HERO9 gave `.54`); the camera is always `.51`. Ubuntu Server needs one netplan file to bring that
interface up (once, sudo): see §3.

**Power:** the GoPro charges from the Pi's port (up to ~1 A). With the Pi on the GPIO 5 V pins from the
5 V 5 A buck, add `usb_max_current_enable=1` to `/boot/firmware/config.txt` (already in the power plan) or
the Pi caps USB at 600 mA total and the camera will slowly drain. On the bench, the Pi's 27 W supply is fine.

### 2b. Pan servo → ESP32 (camera aiming)
The ESP32 already makes 50 Hz servo pulses for the two ESC channels; the pan servo is a third channel.

| Servo wire | Connect to | Notes |
|---|---|---|
| Signal (orange / yellow / white) | **ESP32 GPIO 27** | `PAN_PIN` in firmware 2.1.0. 3.3 V logic drives every hobby servo. |
| + (red) | **5–6 V servo supply** | Bench: the ESP32 `VIN`/5 V pin (USB power) is OK for a 9 g SG90/MG90S only. Robot: a separate **5 V/6 V 3 A UBEC/buck from the 3S pack** (~$5). Never the Pi's 5 V rail or USB. |
| − (brown / black) | **GND**, common with the ESP32 and the Pi | One ground for everything. |

Put a 470–1000 µF electrolytic across the servo's + and − near the servo (stall current spikes). The pan
channel is deliberately **not** part of the drive fail-safe: on a Pi/link loss the camera holds its last
position; it centres at boot and is slew-limited (`PAN_RATE_PER_S`, full travel in ~0.7 s).

Pins now in use on the ESP32: 2 LED · 5 trig · 18 echo · 25 left ESC · 26 right ESC · **27 pan servo**.

### 2c. Servo choice and the mount
- Camera + mount ≈ 160 g + 40 g on a 3–4 cm arm → a **standard-size metal-gear servo**: MG996R (~$8,
  ~10 kg·cm) or a digital DS3218 (~$12–15, 20 kg·cm, waterproof) at 5–6 V. A 9 g servo is for the bench.
- Pan axis **vertical through the camera's centre of mass**, servo shaft pointing up, GoPro's two-finger
  mount on an adapter screwed to the servo horn (print the adapter in PETG; the fingers' M5 thumb screw
  holds it). A foam/TPU pad between servo base and frame; **leave slack in the USB-C cable** for ±45°.
- 1000–2000 µs moves most servos ~90–120°, not 180°. Measure yours (pan_cmd ±1.0 → angle) and set
  `half_range_deg` = half of that travel (default 45). If it pans the wrong way: `pan_sign:=-1.0`.
- Tilt is not needed: at 3 m the GoPro's vertical field of view covers a standing skater and the ground.
  Mount it level or tilted up ~5°.

## 3. One-time Pi setup (sudo, owner types the password)
```bash
sudo cp ~/honebot_ws/src/honebot_camera/netplan/99-gopro-usb.yaml /etc/netplan/
sudo chmod 600 /etc/netplan/99-gopro-usb.yaml
sudo netplan apply
```
Plug in the camera (on, GoPro Connect), then:
```bash
ip -4 -brief addr                               # a usb0/enx* line with 172.2X.1YZ.50
ros2 run honebot_camera gopro_ctl find          # "GoPro link: enx… host 172.2X.1YZ.54 camera 172.2X.1YZ.51"
ros2 run honebot_camera gopro_ctl start 720 4   # camera shows a blue webcam icon; stream on UDP 8554
ros2 run honebot_camera gopro_ctl stop
```
If `find` sees the interface but `start` fails, open `http://172.2X.1YZ.51/gp/gpWebcam/STATUS` with curl
from the Pi to see what the camera says. The node tries the Open GoPro API (port 8080) first and falls back
to the HERO8/9 `/gp/gpWebcam/` API.

## 4. Running it
```bash
ros2 launch honebot_bringup bringup.launch.py camera:=true                 # everything + camera + aiming
ros2 launch honebot_bringup bringup.launch.py camera:=true record:=true    # also save the 1080p stream
ros2 launch honebot_camera camera.launch.py source:=/dev/video0            # webcam only, no bridge
ros2 launch honebot_camera camera.launch.py source:=~/clip.mp4 aim:=false  # test the detector on a file
```
In Foxglove (`ws://honebot.local:8765`): Image panel on `/camera/image/compressed` (green box = the chosen
skater, orange = other people, white line = centre), plots of `/camera/target` x (bearing error, rad),
`/camera/pan_angle`, `/camera/pan_cmd`, and `/camera/status` text (frames in, det/s, record size).

Live tuning: `ros2 param set /camera_aim kp 0.4` (0.3–0.8), `max_rate_deg_s`, `deadband_deg`,
`recenter_timeout`. The loop is open-loop and incremental: pan += kp × (measured bearing error); when the
skater is centred nothing moves, so the lens field of view (`hfov_deg`, 87 for Linear, 118 for Wide) only
changes how fast it converges.

Recorded `.ts` files play in VLC or the GoPro Player; to get `.mp4`: `ffmpeg -i x.ts -c copy x.mp4` on the
Mac (or `sudo apt install ffmpeg` on the Pi).

## 5. Detector
- **Default: OpenCV HOG** people detector. No download, ~8–10 detections/s at 640 px wide on the Pi 5
  (measured 2026-11-13 on a file). Misses people at odd angles and on skateboards at distance; good enough
  to see the aiming work.
- **Default since 2026-11-14: YOLOv8n ONNX** at 320 px input, **10–11 detections/s on the Pi 5 CPU through
  onnxruntime**. The launch files default `model_path` to `~/models/yolov8n.onnx` and fall back to HOG with a
  warning if the file is missing. The model was exported **on the Pi** (venv `~/yolo`, needs `python3-venv`):
  ```bash
  python3 -m venv ~/yolo && ~/yolo/bin/pip install ultralytics onnx onnxslim
  mkdir -p ~/models && cd ~/models && ~/yolo/bin/yolo export model=yolov8n.pt format=onnx opset=12 imgsz=320
  ```
  (The same commands work on the Mac, then `scp yolov8n.onnx honebot.local:~/models/`.)
  **OpenCV 4.6's `cv2.dnn` loads this file but fails inside `forward()`** (shape assertion in
  `shape_utils.hpp`, with opset 11 and 12 alike), so the Pi runs it through **onnxruntime 1.32**
  (`pip install --user --break-system-packages onnxruntime`, already installed). `detector:=auto` tries
  cv2.dnn on a warm-up frame and moves to onnxruntime by itself; `detector:=onnx`/`ort` force one.
  Measured 2026-11-14 on 40 live frames of an empty room (jacket on a door, tall air purifier):
  HOG 6 false positives, YOLOv8n 0. A bigger input (`imgsz=416`, `input_size:=416`) helps distant skaters at
  the cost of a few det/s.
- **Tracking (2026-11-14, `track.py`):** boxes are filtered for plausibility (aspect 0.6–6, ≥ 5 % of frame
  height, bottom edge not in the top third, size consistent with the UWB distance when available), then a
  robot-frame Kalman tracker needs 3 consecutive hits to confirm a target and coasts it through up to 1 s of
  misses. A new track is seeded nearest the **UWB** bearing, else on the biggest box. Everything is a live
  parameter (`track_*`, `min_aspect`, …; see the package README). Later the Raspberry Pi AI HAT+ (Hailo) can
  run a bigger model at 30+ FPS if distant or mid-trick skaters are still missed.

## 6. What `camera/bearing` is for
`camera_aim` publishes the skater's bearing in the robot frame (`pan + bearing in image`). When UWB is not
fitted yet, a small node can turn that into a `/target` point at an assumed distance so the follower can be
tested on vision alone at walking speed. Not written yet; UWB remains the plan for steering.

## 7. Parts to buy for this part
| Item | Price | Notes |
|---|---|---|
| Metal-gear standard servo (MG996R or DS3218) | $8–15 | 180° type; 1000–2000 µs ≈ 90–120° of travel |
| 5 V/6 V 3 A UBEC or buck for the servo | ~$5 | From the 3S pack; share ground |
| 470–1000 µF 16 V capacitor | <$1 | Across the servo supply |
| USB-C cable, ≤ 1 m, data-capable | $5 | GoPro ↔ Pi |
| (mode B) small USB webcam, e.g. Logitech C270/C920 | $20–70 | Only if you want native GoPro recording while the Pi still sees |

## 8. Verified 2026-11-13 with the HERO9 on the Pi
- Enumerates as `2672:0052 GoPro HERO9`, interface `enx023591c96b46`, Pi got **172.22.120.54** (not .50), camera
  172.22.120.51 answers both the legacy API and Open GoPro (webcam version 4). Webcam start via Open GoPro works.
- OpenCV capture of the UDP stream: with `probesize;32` it never decoded a frame ("non-existing PPS"); with
  `probesize;1000000|analyzeduration;1000000` + `nobuffer` the first 1280×720 frame arrives 1.3 s after open and
  the stream runs at a steady 29 fps. FFmpeg defaults take 6.5 s to open and then dump a backlog. The node uses
  the 1 MB probe setting.
