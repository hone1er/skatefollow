"""Camera node: GoPro (USB webcam mode) or any OpenCV source -> person detection -> target bearing.

Publishes
    camera/target             geometry_msgs/Vector3Stamped   x = bearing of the skater from the camera axis
                                                              (rad, +left), y = elevation (rad, +up),
                                                              z = box height as a fraction of the frame
                                                              (bigger = closer). Only when someone is seen.
    camera/image/compressed   sensor_msgs/CompressedImage    JPEG preview with boxes, preview_hz (Foxglove)
    camera/status             std_msgs/String                1 Hz: source, fps, detections, record file
Subscribes (both optional)
    target                    geometry_msgs/PointStamped     UWB target in base_link (from uwb_node): seeds the
                                                              track on the right person and sizes the boxes
    camera/pan_angle          std_msgs/Float32               current pan (rad) from camera_aim; needed to track
                                                              in the robot frame while the camera pans

Detections go through track.py before anything is published: implausible boxes are dropped, a new target
needs a few consecutive hits, and a confirmed target is predicted through missed frames. camera/target is
published only on cycles where the confirmed track was matched to a detection (so its silence still means
"not seen", which camera_aim relies on), with the filtered bearing rather than the raw box centre.

Sources (parameter `source`):
    'gopro'            find the camera on its USB network link, start webcam mode over HTTP, read the
                       MPEG-TS UDP stream (gopro_res 1080/720/480, gopro_fov 0 wide, 4 linear).
                       record:=true tees the raw H.264 stream to record_dir/<timestamp>.ts (no re-encode;
                       the GoPro itself cannot record to its card while in webcam mode).
    '/dev/video0', '0' a USB webcam (V4L2)
    'udp://...', 'rtsp://...', '/path/clip.mp4'   anything OpenCV/FFmpeg opens; files loop (bench testing)

Frames are grabbed in a thread that keeps only the newest one, so the detector (a few Hz on the Pi CPU)
never works on stale video. Detection runs in a second thread; ROS spins in the main thread.
"""

import math
import os
import socket
import threading
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, String

from honebot_camera.aim import bearing_to_x_norm, pixel_to_bearing, pixel_to_elevation, vfov_from_hfov
from honebot_camera.detector import make_detector
from honebot_camera.gopro import GOPRO_UDP_PORT, GoProControl, find_gopro, stream_url
from honebot_camera.track import (Candidate, PlausibilityParams, TargetTracker, TrackParams, expected_height_frac,
                                  plausible, tracked_image_bearing)

# Low-latency FFmpeg options for OpenCV's capture; must be in the environment before the capture opens.
os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS',
                      'fflags;nobuffer|flags;low_delay|probesize;1000000|analyzeduration;1000000')

import cv2  # noqa: E402  (after the env var on purpose)


class UdpTee(threading.Thread):
    """Receive the GoPro's UDP MPEG-TS, append it to a .ts file and forward it to a local port for OpenCV."""

    def __init__(self, listen_port, forward_port, path, logger):
        super().__init__(daemon=True, name='udp_tee')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
        self.sock.bind(('0.0.0.0', listen_port))
        self.sock.settimeout(0.5)
        self.forward = ('127.0.0.1', forward_port)
        self.path = path
        self.bytes = 0
        self.log = logger
        self._stop_evt = threading.Event()   # not _stop: Thread.join() calls a private _stop()

    def run(self):
        f = None
        try:
            f = open(self.path, 'ab')
        except OSError as e:
            self.log.error(f'cannot open recording file {self.path!r} ({e}); forwarding without recording')
        try:
            while not self._stop_evt.is_set():
                try:
                    data, _ = self.sock.recvfrom(65536)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if f is not None:
                    try:
                        f.write(data)
                        self.bytes += len(data)
                    except OSError as e:
                        self.log.error(f'recording write failed ({e}); stopping file write, stream continues')
                        try:
                            f.close()
                        except OSError:
                            pass
                        f = None
                try:
                    self.sock.sendto(data, self.forward)
                except OSError:
                    pass
        finally:
            if f is not None:
                try:
                    f.close()
                except OSError:
                    pass
            self.sock.close()

    def stop(self):
        self._stop_evt.set()


class FrameGrabber(threading.Thread):
    """Reads frames as fast as the source delivers them; keeps only the latest."""

    def __init__(self, open_fn, logger, loop_file=False):
        super().__init__(daemon=True, name='frame_grabber')
        self.open_fn = open_fn
        self.log = logger
        self.loop_file = loop_file
        self.lock = threading.Lock()
        self.frame = None
        self.seq = 0
        self.frames_in = 0
        self.connected = False
        self._stop_evt = threading.Event()   # not _stop: Thread.join() calls a private _stop()

    def run(self):
        cap = None
        failures = 0
        while not self._stop_evt.is_set():
            if cap is None:
                cap = self.open_fn()
                if cap is None or not cap.isOpened():
                    cap = None
                    self.connected = False
                    self._stop_evt.wait(2.0)
                    continue
                self.connected = True
                failures = 0
                self.log.info('video source opened')
            t_read = time.monotonic()
            ok, frame = cap.read()
            if not ok or frame is None:
                # A slow read (>= 1 s, e.g. FFmpeg UDP timeout=2s) means the stream already timed out;
                # reopen immediately instead of waiting 30 x 2 s = 60 s.
                failures += 30 if (time.monotonic() - t_read) >= 1.0 else 1
                if self.loop_file and failures < 3:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                if failures >= 30:            # ~a second of nothing (or a timed-out stream): reopen
                    self.log.warn('video source lost; reopening')
                    cap.release()
                    cap = None
                    self.connected = False
                else:
                    time.sleep(0.03)
                continue
            failures = 0
            with self.lock:
                self.frame = frame
                self.seq += 1
                self.frames_in += 1
        if cap is not None:
            cap.release()

    def latest(self, after_seq):
        with self.lock:
            if self.seq == after_seq or self.frame is None:
                return None, after_seq
            return self.frame, self.seq

    def pop_frames_in(self):
        with self.lock:
            n = self.frames_in
            self.frames_in = 0
            return n

    def stop(self):
        self._stop_evt.set()


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        p = self.declare_parameter
        self.source = str(p('source', 'gopro').value)
        self.frame_id = p('frame_id', 'camera_link').value
        self.hfov = math.radians(p('hfov_deg', 87.0).value)      # GoPro Linear 16:9 ~ 87, Wide ~ 118
        self.proc_width = int(p('proc_width', 640).value)          # frames are shrunk to this before detection
        self.detector_kind = p('detector', 'auto').value           # auto | hog | onnx
        self.model_path = os.path.expanduser(p('model_path', '').value)
        self.input_size = int(p('input_size', 320).value)
        self.conf = float(p('conf_threshold', 0.4).value)
        self.preview_hz = float(p('preview_hz', 5.0).value)       # 0 disables the JPEG preview
        self.jpeg_quality = int(p('jpeg_quality', 60).value)
        self.use_uwb_hint = p('use_uwb_hint', True).value
        self.uwb_timeout = float(p('uwb_timeout', 0.5).value)
        # Tracking (track.py); all live-settable, e.g. ros2 param set /camera_node track_confirm_hits 2
        self.track_params = TrackParams(
            confirm_hits=int(p('track_confirm_hits', 3).value),
            coast_time=float(p('track_coast_time', 1.0).value),
            gate_min=math.radians(float(p('track_gate_min_deg', 6.0).value)),
            gate_max=math.radians(float(p('track_gate_max_deg', 25.0).value)),
            n_sigma=float(p('track_n_sigma', 3.0).value),
            meas_std=math.radians(float(p('track_meas_std_deg', 1.0).value)),
            accel_std=math.radians(float(p('track_accel_std_deg_s2', 90.0).value)),
            hint_gate=math.radians(float(p('track_hint_gate_deg', 15.0).value)),
        )
        self.plaus = PlausibilityParams(
            min_aspect=float(p('min_aspect', 0.6).value),
            max_aspect=float(p('max_aspect', 6.0).value),
            min_height_frac=float(p('min_height_frac', 0.05).value),
            min_bottom_y_norm=float(p('min_bottom_y_norm', -0.3).value),
            person_height_m=float(p('person_height_m', 1.7).value),
            size_tol_low=float(p('size_tol_low', 0.4).value),
            size_tol_high=float(p('size_tol_high', 2.5).value),
        )
        self.tracker = TargetTracker(self.track_params)
        self.add_on_set_parameters_callback(self._on_params)
        # GoPro specifics
        self.gopro_ip = p('gopro_ip', '').value                    # empty = auto-detect
        self.gopro_res = int(p('gopro_res', 1080).value)
        self.gopro_fov = int(p('gopro_fov', 4).value)              # 0 wide, 2 narrow, 3 superview, 4 linear
        self.gopro_port = int(p('gopro_port', GOPRO_UDP_PORT).value)
        self.record = p('record', False).value
        self.record_dir = os.path.expanduser(p('record_dir', '~/videos').value)

        self.target_pub = self.create_publisher(Vector3Stamped, 'camera/target', 10)
        self.image_pub = self.create_publisher(CompressedImage, 'camera/image/compressed', 1)
        self.status_pub = self.create_publisher(String, 'camera/status', 10)
        self.create_subscription(PointStamped, 'target', self._on_uwb, 10)
        self.create_subscription(Float32, 'camera/pan_angle', self._on_pan, 10)

        self._uwb = None             # immutable (bearing, dist, stamp), swapped atomically across threads
        self.pan_angle = 0.0
        self._stats_lock = threading.Lock()
        self.track_text = 'none'
        self.tee = None
        self.gopro = None
        self.record_path = None
        self.det_count = 0
        self.det_time_acc = 0.0
        self.det_fps = 0.0
        self.last_n_dets = 0
        self.seen_count = 0          # cycles this second where the confirmed track matched a detection
        self.last_preview = 0.0
        self.have_target = False

        if self.model_path and not os.path.isfile(self.model_path):
            # The launch files default model_path to ~/models/yolov8n.onnx; without the file, run HOG.
            self.get_logger().warn(f'model_path {self.model_path!r} does not exist; using the HOG detector '
                                   f'(export a YOLO model: see docs/camera_plan.md section 5)')
            self.model_path = ''
        try:
            self.detector = make_detector(self.detector_kind, self.model_path, self.input_size, self.conf,
                                          log=self.get_logger().warn)
        except Exception as e:  # cv2.error, RuntimeError or ValueError: fall back so the camera path still works
            self.get_logger().error(f'Could not create detector {self.detector_kind!r} '
                                    f'(model {self.model_path!r}): {e}; using HOG')
            self.detector = make_detector('hog')
        self.get_logger().info(f'detector = {self.detector.name}, source = {self.source}, '
                               f'hfov = {math.degrees(self.hfov):.0f} deg')

        self.grabber = FrameGrabber(self._open_source, self.get_logger(),
                                    loop_file=os.path.isfile(self.source))
        self.grabber.start()
        self._stop_evt = threading.Event()   # not _stop: Thread.join() calls a private _stop()
        self.worker = threading.Thread(target=self._detect_loop, daemon=True, name='detector')
        self.worker.start()
        self.create_timer(1.0, self._status)

    # --- sources ----------------------------------------------------------------------
    def _open_source(self):
        if self.source == 'gopro':
            return self._open_gopro()
        src = self.source
        if src.isdigit():
            src = int(src)
        cap = cv2.VideoCapture(src)
        if isinstance(src, int) and cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            self.get_logger().warn(f'cannot open video source {self.source!r}', throttle_duration_sec=10.0)
        return cap

    def _open_gopro(self):
        ip = self.gopro_ip
        if not ip:
            found = find_gopro()
            if found is None:
                self.get_logger().warn('No GoPro USB link (no 172.2X.1YZ.x interface). Camera on, cable in, '
                                       'USB Connection = GoPro Connect, netplan file installed?',
                                       throttle_duration_sec=10.0)
                return None
            iface, host_ip, ip = found
            self.get_logger().info(f'GoPro on {iface}: host {host_ip}, camera {ip}')
        self.gopro = GoProControl(ip)
        try:
            self.get_logger().info(self.gopro.start_webcam(self.gopro_res, self.gopro_fov, self.gopro_port))
        except OSError as e:
            self.get_logger().warn(f'GoPro webcam start failed: {e}', throttle_duration_sec=10.0)
            return None
        url = stream_url('0.0.0.0', self.gopro_port)
        if self.record:
            if self.tee is None:
                os.makedirs(self.record_dir, exist_ok=True)
                self.record_path = os.path.join(self.record_dir,
                                                datetime.now().strftime('gopro_%Y%m%d_%H%M%S.ts'))
                try:
                    self.tee = UdpTee(self.gopro_port, self.gopro_port + 1, self.record_path, self.get_logger())
                    self.tee.start()
                    self.get_logger().info(f'recording raw stream to {self.record_path}')
                except OSError as e:
                    self.get_logger().error(f'cannot record ({e}); streaming without recording')
                    self.tee = None
            if self.tee is not None:
                url = stream_url('127.0.0.1', self.gopro_port + 1)
        return cv2.VideoCapture(url, cv2.CAP_FFMPEG)

    # --- parameters, hints -----------------------------------------------------------------
    _TRACK_PARAMS = {    # ROS name -> (dataclass attribute, converter)
        'track_confirm_hits': ('confirm_hits', int),
        'track_coast_time': ('coast_time', float),
        'track_gate_min_deg': ('gate_min', math.radians),
        'track_gate_max_deg': ('gate_max', math.radians),
        'track_n_sigma': ('n_sigma', float),
        'track_meas_std_deg': ('meas_std', math.radians),
        'track_accel_std_deg_s2': ('accel_std', math.radians),
        'track_hint_gate_deg': ('hint_gate', math.radians),
    }

    _POSITIVE_TRACK = {'track_gate_max_deg', 'track_n_sigma', 'track_meas_std_deg'}
    _POSITIVE_PLAUS = {'max_aspect', 'person_height_m', 'size_tol_high'}

    def _on_params(self, params):
        updates = []
        for prm in params:
            if prm.name in self._TRACK_PARAMS:
                attr, conv = self._TRACK_PARAMS[prm.name]
                if prm.name == 'track_confirm_hits':
                    val = int(prm.value)
                    if val < 1:
                        return SetParametersResult(successful=False, reason='track_confirm_hits must be >= 1')
                else:
                    raw = float(prm.value)
                    if not math.isfinite(raw) or (prm.name in self._POSITIVE_TRACK and raw <= 0.0) or raw < 0.0:
                        return SetParametersResult(successful=False, reason=f'{prm.name} must be finite and valid')
                    val = conv(raw)
                updates.append(('track', prm.name, attr, val))
            elif hasattr(self.plaus, prm.name):
                val = float(prm.value)
                if not math.isfinite(val):
                    return SetParametersResult(successful=False, reason=f'{prm.name} must be finite')
                if prm.name in self._POSITIVE_PLAUS and val <= 0.0:
                    return SetParametersResult(successful=False, reason=f'{prm.name} must be > 0')
                if prm.name != 'min_bottom_y_norm' and val < 0.0:
                    return SetParametersResult(successful=False, reason=f'{prm.name} must be >= 0')
                updates.append(('plaus', prm.name, prm.name, val))
            elif prm.name == 'use_uwb_hint':
                updates.append(('self', prm.name, 'use_uwb_hint', bool(prm.value)))
        for kind, pname, attr, val in updates:
            if kind == 'track':
                setattr(self.track_params, attr, val)
            elif kind == 'plaus':
                setattr(self.plaus, attr, val)
            else:
                setattr(self, attr, val)
            self.get_logger().info(f'{pname} = {val}')
        return SetParametersResult(successful=True)

    def _on_uwb(self, msg: PointStamped):
        x, y = msg.point.x, msg.point.y
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        self._uwb = (math.atan2(y, x), math.hypot(x, y), time.monotonic())

    def _on_pan(self, msg: Float32):
        v = float(msg.data)
        if math.isfinite(v):
            self.pan_angle = v

    def _uwb_sample(self):
        """Return (bearing, dist) if UWB hint is enabled and fresh, else (None, None)."""
        uwb = self._uwb
        if not self.use_uwb_hint or uwb is None or time.monotonic() - uwb[2] > self.uwb_timeout:
            return None, None
        return uwb[0], uwb[1]

    # --- detection loop -----------------------------------------------------------------
    def _detect_loop(self):
        seq = 0
        while not self._stop_evt.is_set() and rclpy.ok():
            frame, seq = self.grabber.latest(seq)
            if frame is None:
                time.sleep(0.005)
                continue
            try:
                self._process(frame)
            except Exception as e:  # keep the node alive; log once per second at most
                if not rclpy.ok():
                    break               # context torn down under us during shutdown: not an error
                self.get_logger().error(f'detection error: {e}', throttle_duration_sec=1.0)

    def _process(self, frame):
        h0, w0 = frame.shape[:2]
        if w0 > self.proc_width:
            frame = cv2.resize(frame, (self.proc_width, int(round(h0 * self.proc_width / w0))))
        h, w = frame.shape[:2]
        t0 = time.monotonic()
        dets = self.detector.detect(frame)
        dt = time.monotonic() - t0

        # 1. Plausibility: drop boxes that cannot be a person (and, with UWB, not a person at that distance).
        uwb_bearing, uwb_dist = self._uwb_sample()
        exp_h = (expected_height_frac(uwb_dist, self.hfov, w, h, self.plaus.person_height_m)
                 if uwb_dist is not None else None)
        pan = self.pan_angle                       # sampled once so every box in this frame uses the same pan
        plausible_dets, rejected = [], []
        for d in dets:
            (plausible_dets if plausible(d, w, h, self.plaus, exp_h) else rejected).append(d)
        # 2. Track in the robot frame: robot bearing = commanded pan + where the box sits in the image.
        cands = [Candidate(pan + pixel_to_bearing(2.0 * d.cx / w - 1.0, self.hfov), d) for d in plausible_dets]
        state = self.tracker.update(time.monotonic(), cands, uwb_bearing)

        target = None
        if state is None:
            track_text = 'none'
        elif not state.confirmed:
            track_text = f'acquiring {state.hits}/{self.track_params.confirm_hits}'
        elif state.matched:
            track_text = 'locked'
            target = state.det
        else:
            track_text = f'coasting {state.coast:.1f}s'
        self.have_target = target is not None
        with self._stats_lock:
            self.det_count += 1
            self.det_time_acc += dt
            self.last_n_dets = len(dets)
            self.track_text = track_text
            if target is not None:
                self.seen_count += 1
            det_fps = self.det_fps

        if target is not None:
            # Filtered bearing (less box jitter than the raw centre); elevation and size from the matched box.
            y_norm = 2.0 * target.cy / h - 1.0
            vfov = vfov_from_hfov(self.hfov, w, h)
            msg = Vector3Stamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.vector.x = tracked_image_bearing(state, pan)
            msg.vector.y = pixel_to_elevation(y_norm, vfov)
            msg.vector.z = target.height / h
            self.target_pub.publish(msg)

        if self.preview_hz > 0 and time.monotonic() - self.last_preview >= 1.0 / self.preview_hz:
            self.last_preview = time.monotonic()
            pred_x = None
            if state is not None and state.confirmed:
                diff = state.bearing - pan
                if abs(diff) < math.pi / 2.0 - 0.05:
                    xn = bearing_to_x_norm(diff, self.hfov)
                    pred_x = int((xn + 1.0) * 0.5 * w)
            self._publish_preview(frame, plausible_dets, rejected, target, pred_x, det_fps, pan, track_text)

    def _publish_preview(self, frame, dets, rejected, target, pred_x=None, det_fps=0.0, pan=0.0, track_text='none'):
        """green = tracked target, orange = other plausible people, grey = rejected as implausible,
        white circle = where the track predicts the target (visible while coasting)."""
        img = frame.copy()
        h, w = img.shape[:2]
        for d in rejected:
            cv2.rectangle(img, (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2)), (128, 128, 128), 1)
        for d in dets:
            color = (0, 255, 0) if d is target else (0, 160, 255)
            cv2.rectangle(img, (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2)), color, 2)
            cv2.putText(img, f'{d.score:.2f}', (int(d.x1), max(12, int(d.y1) - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        if pred_x is not None and 0 <= pred_x < w:
            cv2.circle(img, (pred_x, h // 2), 8, (255, 255, 255), 2)
        cv2.line(img, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        cv2.putText(img, f'{det_fps:.1f} det/s  pan {math.degrees(pan):+.0f} deg  {track_text}',
                    (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self.image_pub.publish(msg)

    # --- status ------------------------------------------------------------------------------
    def _status(self):
        with self._stats_lock:
            n = self.det_count
            seen = self.seen_count
            dt_acc = self.det_time_acc
            last_n_dets = self.last_n_dets
            track_text = self.track_text
            if n:
                det_fps = n / max(dt_acc, 1e-6) if dt_acc else 0.0
                self.det_fps = min(det_fps, float(n))   # bounded by frames actually processed
            self.det_count = 0
            self.seen_count = 0
            self.det_time_acc = 0.0
        frames_in = self.grabber.pop_frames_in()
        rec = ''
        if self.tee is not None:
            rec = f' rec={os.path.basename(self.record_path)} {self.tee.bytes / 1e6:.1f}MB'
        s = (f'source={self.source} {"connected" if self.grabber.connected else "NO VIDEO"} '
             f'frames_in={frames_in} det/s={n} people={last_n_dets} '
             f'seen={seen}/{n} track={track_text} '
             f'detector={self.detector.name}{rec}')
        self.status_pub.publish(String(data=s))
        self.get_logger().info(s, throttle_duration_sec=10.0)

    def shutdown(self):
        self._stop_evt.set()
        self.grabber.stop()
        if self.tee is not None:
            self.tee.stop()
        # Join the threads so OpenCV is not torn down underneath a running capture at interpreter exit.
        self.worker.join(timeout=2.0)
        self.grabber.join(timeout=3.0)
        if self.tee is not None:
            self.tee.join(timeout=2.0)
        if self.gopro is not None:
            try:
                self.get_logger().info(self.gopro.stop_webcam())
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
