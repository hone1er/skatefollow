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
| `honebot_camera/aim.py` | Pure: pixel ↔ bearing (pinhole), `AimGains`, `PanController`. |
| `honebot_camera/camera_aim.py` | Node `camera_aim`: wraps `PanController` at 20 Hz, live parameters. |
| `honebot_camera/gopro.py` | Finding the GoPro on its USB link, HTTP webcam start/stop (Open GoPro, then legacy fallback), stream URL. `gopro_ctl` CLI. |
| `launch/camera.launch.py` | Both nodes; `source`, `record`, `model_path`, `aim`, lens and servo arguments. |
| `netplan/99-gopro-usb.yaml` | Makes Ubuntu DHCP on the GoPro's USB interface without taking its default route (sudo install). |
| `test/` | 19 tests: aim 7, detector 7, gopro 5. |

## camera_node: threads and why

| Thread | Job | Why separate |
|---|---|---|
| `FrameGrabber` | `cap.read()` in a loop; keeps **only the newest frame** under a lock; reopens the source after ~1 s of failures; loops files | A 30 fps source feeding a 5–10 fps detector must drop frames, or latency grows without bound and the servo aims at the past |
| detector | waits for a new frame, resizes to `proc_width`, runs the backend, selects the target, publishes bearing, publishes a JPEG at `preview_hz` | Detection takes 100–200 ms; it must not block ROS callbacks |
| main (`rclpy.spin`) | parameters, subscriptions (`/target`, `/camera/pan_angle`), 1 Hz status timer | Standard |
| `UdpTee` (only with `record`) | receives the GoPro's UDP packets, appends them to a `.ts` file, forwards them to a local port OpenCV reads | Byte-level tee = recording with zero CPU and no re-encode |

Shutdown joins all threads before `rclpy.shutdown()`; tearing OpenCV down under a live capture crashed the
process otherwise.

### Sources
`source` is `gopro` (find the camera, start webcam mode over HTTP, read `udp://0.0.0.0:8554`), a V4L2 index or
`/dev/videoN`, any URL FFmpeg opens, or a file (loops, for bench tests). Low-latency FFmpeg flags are set via
`OPENCV_FFMPEG_CAPTURE_OPTIONS` **before** `cv2` is imported, which is why the import is below the `os.environ` line.

### Picking the skater (`select_target`)
Among all detected people: the one nearest where UWB says the skater should be in the image (if within
`select_gate` × frame width), else nearest last frame's target, else the biggest box. UWB bearing is converted
into the camera frame by subtracting the current pan angle, which is why `camera_node` subscribes to
`/camera/pan_angle`.

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

## Running and testing

```bash
ros2 run honebot_camera gopro_ctl find|start|stop|status            # bench checks without ROS nodes
ros2 launch honebot_camera camera.launch.py                           # GoPro, HOG, aiming
ros2 launch honebot_camera camera.launch.py source:=~/clip.mp4 aim:=false   # detector on a file
ros2 launch honebot_camera camera.launch.py model_path:=~/models/yolov8n.onnx
ros2 topic echo /camera/status
colcon test --packages-select honebot_camera && colcon test-result --verbose
```
Foxglove: Image panel on `/camera/image/compressed` (green = tracked, orange = other people, white line =
image centre). Hardware notes, servo wiring and the model export command: `docs/camera_plan.md`.
