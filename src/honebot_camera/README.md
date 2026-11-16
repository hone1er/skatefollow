# honebot_camera

Video in (GoPro over USB, a webcam, a URL or a file), person detection, choice of which person is the
skater, and an open-loop pan-servo controller that keeps them centred. This subsystem **cannot move the
robot**; its outputs are one servo command and a bearing.

```
GoPro ──USB network, HTTP control, MPEG-TS over UDP──┐
/dev/videoN, rtsp://, file.mp4 ──────────────────────┤
                                                     ▼
   /target [UWB hint] ────────────────────────► camera_node ──► /camera/target [Vector3Stamped: bearing, elevation, size]
   /camera/pan_angle ─────────────────────────►     │      ──► /camera/image/compressed [JPEG + boxes, for Foxglove]
                                                    │      ──► /camera/status [String, 1 Hz]
                                                    ▼
   /target [UWB fallback] ────────────────────► camera_aim ──► /camera/pan_cmd [Float32 −1..1] ──► esp32_bridge ──P──► servo
                                                           ──► /camera/pan_angle [rad]  (fed back to camera_node)
                                                           ──► /camera/bearing  [rad, robot frame, only while seen]
```

## Files

| File | Role |
|---|---|
| `honebot_camera/camera_node.py` | Node `camera_node`: source handling, `FrameGrabber` thread, detector thread, `UdpTee` recorder, preview, status. |
| `honebot_camera/detector.py` | Pure numpy: `Detection`, `letterbox_params`, `decode_yolov8`, `nms`, `select_target`. Backends (need cv2): `HogPersonDetector`, `OnnxPersonDetector`, `OrtPersonDetector`, `make_detector`. |
| `honebot_camera/track.py` | Pure: `plausible()` box filters, `BearingKF` (bearing + rate Kalman filter), `TargetTracker` (confirm / associate / coast). |
| `honebot_camera/aim.py` | Pure: pixel ↔ bearing (pinhole), `AimGains`, `PanController`. |
| `honebot_camera/camera_aim.py` | Node `camera_aim`: wraps `PanController` at 20 Hz, live parameters. |
| `honebot_camera/gopro.py` | Finding the GoPro on its USB link, HTTP webcam start/stop (Open GoPro, then legacy fallback), stream URL. `gopro_ctl` CLI. |
| `launch/camera.launch.py` | Both nodes; `source`, `record`, `model_path`, `aim`, lens and servo arguments. |
| `netplan/99-gopro-usb.yaml` | Makes Ubuntu DHCP on the GoPro's USB interface without taking its default route (sudo install). |
| `test/` | 42 tests: aim 7, detector 11, gopro 5, track 19. |

## camera_node: threads and why

| Thread | Job | Why separate |
|---|---|---|
| `FrameGrabber` | `cap.read()` in a loop; keeps **only the newest frame** under a lock; reopens the source after ~1 s of failures; loops files | A 30 fps source feeding a 5–10 fps detector must drop frames, or latency grows without bound and the servo aims at the past |
| detector | waits for a new frame, resizes to `proc_width`, runs the backend, selects the target, publishes bearing, publishes a JPEG at `preview_hz` | Detection takes 100–200 ms; it must not block ROS callbacks |
| main (`rclpy.spin`) | parameters, subscriptions (`/target`, `/camera/pan_angle`), 1 Hz status timer | Standard |
| `UdpTee` (only with `record`) | receives the GoPro's UDP packets, appends them to a `.ts` file, forwards them to a local port OpenCV reads | Byte-level tee = recording with zero CPU and no re-encode |

Shutdown joins all threads before `rclpy.shutdown()`; tearing OpenCV down under a live capture crashed the
process otherwise. The thread classes keep their stop flag in `_stop_evt`, never `_stop`: `Thread.join()`
calls a private `_stop()` method, and shadowing it made every shutdown fail with exit code 1 (fixed
2026-11-14).

### Sources
`source` is `gopro` (find the camera, start webcam mode over HTTP, read `udp://0.0.0.0:8554`), a V4L2 index or
`/dev/videoN`, any URL FFmpeg opens, or a file (loops, for bench tests). Low-latency FFmpeg flags are set via
`OPENCV_FFMPEG_CAPTURE_OPTIONS` **before** `cv2` is imported, which is why the import is below the `os.environ` line.

### From detections to one target (`track.py`)
A detector only says where people are in *this* frame. Two things go wrong if that is used directly: a
one-frame false positive becomes the target instantly, and a missed detection (odd pose mid-trick) drops the
target so the next frame picks whoever is biggest. So every detector cycle goes through three stages:

1. **Plausibility** (`plausible`): aspect ratio within `min_aspect`..`max_aspect` (0.6..6, permissive enough
   for crouching and mid-trick, strict enough for poles and benches), height at least `min_height_frac` of the
   frame, bottom edge below `min_bottom_y_norm` (nothing stands in the top third of a level camera; a jump is
   still fine), and, when UWB is fresh, box height within `size_tol_low`..`size_tol_high` of what a
   `person_height_m` person at the UWB distance would measure. Rejected boxes are drawn grey in the preview.
2. **Association in the robot frame.** Each plausible box becomes a robot-frame bearing (commanded pan +
   image bearing), so the camera panning does not look like the skater moving. A 2-state Kalman filter
   (bearing, rate; `track_meas_std_deg`, `track_accel_std_deg_s2`) predicts where the target should be; the
   nearest candidate inside the gate is taken. The gate is `track_n_sigma` × innovation std, clamped to
   `track_gate_min_deg`..`track_gate_max_deg`, so it widens while coasting but never beyond the cap.
3. **Life cycle.** A new track needs `track_confirm_hits` consecutive hits (one miss resets it; flicker never
   confirms). A confirmed track survives misses for `track_coast_time` seconds, then is dropped and acquisition
   restarts, seeded by the candidate nearest the UWB bearing (within `track_hint_gate_deg`), else the biggest.

`camera/target` is published only on cycles where the **confirmed** track matched a detection, carrying the
filtered bearing instead of the raw box centre, so `camera_aim`'s "silence means not seen" contract is kept.
The status line shows `seen=<matched>/<processed>` per second and `track=none|acquiring k/N|locked|coasting Xs`;
the preview draws the prediction as a white circle while coasting. All tracker and plausibility parameters are
live-settable (`ros2 param set /camera_node track_confirm_hits 2`). `detector.select_target` remains as the
simple per-frame picker but is no longer used by the node.

### Output geometry
`x = pixel_to_bearing(x_norm, hfov)` = `atan(−x_norm · tan(hfov/2))`: a pinhole model, positive = left to match
ROS. `hfov_deg` is the lens field of view (GoPro Linear ≈ 87°, Wide ≈ 118°). Elevation uses the vertical
field of view derived from the aspect ratio. `z` is box height as a fraction of the frame (a crude distance proxy).

## camera_aim: open-loop pan

The servo gives no position feedback, so the controller tracks the angle it *commanded* and nudges it:

```
if a detection arrived:    pan += kp · bearing_err   (ignored inside deadband; slew-limited by max_rate)
elif lost > lost_timeout and UWB fresh:   move toward the UWB bearing at recenter_rate
elif lost > recenter_timeout:             move toward 0 at recenter_rate
pan = clamp(±half_range)
```
Because the step is proportional to the *measured* error, the loop converges to "target centred" regardless
of how accurate `hfov` or `kp` are; they only set convergence speed. Measurements older than
`measurement_max_age` are dropped so a stalled detector cannot keep pushing. `pan_sign` flips the servo
direction; `half_range_deg` is the mechanical travel. All live-settable: `ros2 param set /camera_aim kp 0.4`.

`/camera/bearing` = commanded pan + image bearing = the skater's bearing in the robot frame. It is
published only while the detector sees someone, so a consumer can use its silence as "no vision fix".

## Why it is built this way

- **Vision aims, UWB follows.** Pi-CPU detection is too slow and too brittle to steer a robot at 6 m/s,
  but plenty for keeping a wide lens roughly centred. The design keeps vision strictly out of the motion path.
- **Detector behind one interface** (`detect(frame_bgr) -> [Detection]`, `.name`). HOG works out of the
  box so the rest could be built and tested without downloading a model; YOLO slots in via `model_path`
  and is the default when `~/models/yolov8n.onnx` exists. YOLO tensor decoding is pure numpy and handles
  both `(1, 84, N)` and `(1, N, 84)` layouts.
- **`auto` tries cv2.dnn, then onnxruntime.** Each backend runs one warm-up frame in its constructor so a
  model that loads but cannot run fails at construction, where `make_detector` can switch backend and
  `camera_node` can drop to HOG. OpenCV 4.6 on the Pi loads YOLOv8 ONNX files but fails in `forward()`;
  onnxruntime runs them at ~90 ms a frame.
- **GoPro as a network device, controlled by HTTP.** That is simply what the camera is over USB; the
  netplan file exists because Ubuntu Server otherwise ignores the interface, and it must not take the
  camera's default route or the Pi loses internet.
- **Two GoPro APIs tried in order** (Open GoPro on :8080, then the HERO8/9 legacy API on :80) so the
  same code works across firmware generations. The HERO9 rejects a `port=` parameter, hence the special case.
- **Recording is a tee, not a re-encode.** The 1080p30 H.264 the camera already produces is written as-is;
  the Pi has no CPU to spare and no ffmpeg CLI installed.
- **Track in the robot frame, publish in the camera frame.** The skater's motion is smooth in the robot
  frame; in the image it is the sum of their motion and the servo's. Filtering in the robot frame keeps the
  Kalman model honest, and `tracked_image_bearing` subtracts the pan again for `camera_aim`.
- **Confirm before trusting, coast before giving up.** Phantoms flicker, people persist. Three hits in a row
  costs ~0.3 s of latency on acquisition and buys immunity to single-frame false positives; one second of
  coasting bridges the frames a detector loses during a trick without ever inventing a detection.

## Running and testing

```bash
ros2 run honebot_camera gopro_ctl find|start|stop|status            # bench checks without ROS nodes
ros2 launch honebot_camera camera.launch.py                           # GoPro, HOG, aiming
ros2 launch honebot_camera camera.launch.py source:=~/clip.mp4 aim:=false   # detector on a file
ros2 launch honebot_camera camera.launch.py model_path:=~/models/yolov8n.onnx
ros2 topic echo /camera/status
colcon test --packages-select honebot_camera && colcon test-result --verbose
```
Foxglove: Image panel on `/camera/image/compressed` (green = tracked, orange = other plausible people, grey =
rejected as implausible, white circle = predicted position while coasting, white line = image centre). Hardware notes, servo wiring and the model export command: `docs/camera_plan.md`.
