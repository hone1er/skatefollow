"""Person detection for camera aiming.

Pure, numpy-only parts (unit-tested, no cv2 needed):
    Detection                 one box in pixels of the processed frame
    letterbox_params()        resize-with-padding geometry for a square model input
    decode_yolov8()           raw YOLOv8/YOLO11 ONNX output -> person Detections in frame pixels
    nms()                     greedy non-maximum suppression
    select_target()           which person is the skater: nearest to a hint (UWB), else nearest to
                              the previous target, else the biggest box

Backends (need cv2, created by the camera node):
    HogPersonDetector         OpenCV's built-in HOG+SVM people detector. No model file to download,
                              slow-ish and misses people at odd angles; fine to get everything else working.
    OnnxPersonDetector        YOLOv8n/YOLO11n ONNX via cv2.dnn (CPU). Much better. Export the model on the
                              Mac with:  pip install ultralytics && yolo export model=yolov8n.pt format=onnx
                              opset=12 imgsz=320   then copy yolov8n.onnx to the Pi (~12 MB).
    OrtPersonDetector         the same model through onnxruntime, for when OpenCV's dnn module cannot
                              run the file (detector:=ort after `pip install onnxruntime`).
    make_detector('auto')     tries cv2.dnn first and silently moves to onnxruntime if cv2.dnn fails on a
                              warm-up frame. OpenCV 4.6 (the Pi's apt version) loads YOLOv8 ONNX files but
                              fails inside forward() with a shape assertion, so on the Pi 'auto' ends up on
                              onnxruntime; a newer OpenCV may run it directly.

Pixel convention: x to the right, y down, origin top-left, as in OpenCV.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float

    @property
    def cx(self):
        return 0.5 * (self.x1 + self.x2)

    @property
    def cy(self):
        return 0.5 * (self.y1 + self.y2)

    @property
    def width(self):
        return max(0.0, self.x2 - self.x1)

    @property
    def height(self):
        return max(0.0, self.y2 - self.y1)

    @property
    def area(self):
        return self.width * self.height


def letterbox_params(width, height, size):
    """Scale and pad a width x height frame into a size x size square keeping aspect ratio.

    Returns (scale, pad_x, pad_y, new_w, new_h): resize to new_w x new_h, then pad with pad_x on the
    left/right and pad_y on top/bottom (floor; the remainder goes right/bottom).
    """
    scale = min(size / width, size / height)
    new_w = int(round(width * scale))
    new_h = int(round(height * scale))
    pad_x = (size - new_w) // 2
    pad_y = (size - new_h) // 2
    return scale, pad_x, pad_y, new_w, new_h


def iou(a, b):
    ix1, iy1 = max(a.x1, b.x1), max(a.y1, b.y1)
    ix2, iy2 = min(a.x2, b.x2), min(a.y2, b.y2)
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    union = a.area + b.area - inter
    return inter / union if union > 0 else 0.0


def nms(dets, iou_threshold=0.45):
    """Keep the highest-scoring boxes, dropping any that overlap a kept box by more than iou_threshold."""
    kept = []
    for d in sorted(dets, key=lambda d: d.score, reverse=True):
        if all(iou(d, k) <= iou_threshold for k in kept):
            kept.append(d)
    return kept


def decode_yolov8(output, scale, pad_x, pad_y, conf_threshold=0.4, person_class=0,
                  frame_width=None, frame_height=None):
    """Turn a YOLOv8-family ONNX output into person Detections in frame pixels.

    output: (1, 4 + nc, N) as exported by ultralytics (also accepts (4 + nc, N) or (1, N, 4 + nc)).
    Boxes are cx, cy, w, h in model-input pixels; scale/pad_x/pad_y from letterbox_params().
    """
    out = np.asarray(output, dtype=np.float32)
    if out.ndim == 3:
        out = out[0]
    # Anchors N are in the thousands (2100 at 320 px, 8400 at 640 px); channels 4+nc are <= 256.
    if out.shape[0] > 256 and out.shape[1] <= 256:    # (N, 4+nc) -> (4+nc, N)
        out = out.T
    if out.shape[0] < 5:
        return []
    scores = out[4 + person_class]
    keep = scores >= conf_threshold
    if not np.any(keep):
        return []
    cx, cy, w, h = out[0, keep], out[1, keep], out[2, keep], out[3, keep]
    sc = scores[keep]
    x1 = (cx - w / 2 - pad_x) / scale
    y1 = (cy - h / 2 - pad_y) / scale
    x2 = (cx + w / 2 - pad_x) / scale
    y2 = (cy + h / 2 - pad_y) / scale
    dets = []
    for i in range(len(sc)):
        a, b, c, d = float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i])
        if frame_width is not None:
            a, c = max(0.0, a), min(float(frame_width), c)
        if frame_height is not None:
            b, d = max(0.0, b), min(float(frame_height), d)
        if c > a and d > b:
            dets.append(Detection(a, b, c, d, float(sc[i])))
    return dets


def select_target(dets, prev_cx=None, hint_cx=None, gate_px=None):
    """Pick the skater among the detected people.

    Priority: the box nearest hint_cx (where UWB says the skater should appear) if it is within gate_px;
    else the box nearest prev_cx (what we tracked last frame) within gate_px; else the largest box.
    gate_px None = no gate. Returns a Detection or None.
    """
    if not dets:
        return None
    for ref in (hint_cx, prev_cx):
        if ref is None:
            continue
        best = min(dets, key=lambda d: abs(d.cx - ref))
        if gate_px is None or abs(best.cx - ref) <= gate_px:
            return best
    return max(dets, key=lambda d: d.area)


# --- backends (cv2) --------------------------------------------------------------------

class HogPersonDetector:
    """OpenCV built-in pedestrian detector. ~5-10 Hz at 640x360 on a Pi 5, rough boxes."""

    name = 'hog'

    def __init__(self, stride=8, scale=1.05, hit_threshold=0.0):
        import cv2
        self._cv2 = cv2
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.stride = (stride, stride)
        self.scale = scale
        self.hit_threshold = hit_threshold

    def detect(self, frame_bgr):
        rects, weights = self.hog.detectMultiScale(frame_bgr, winStride=self.stride, padding=(8, 8),
                                                   scale=self.scale, hitThreshold=self.hit_threshold)
        dets = []
        for (x, y, w, h), s in zip(rects, np.asarray(weights).reshape(-1)):
            # HOG boxes are padded generously; shrink to the person
            dets.append(Detection(x + 0.15 * w, y + 0.05 * h, x + 0.85 * w, y + 0.95 * h, float(s)))
        return nms(dets, 0.3)


class OnnxPersonDetector:
    """YOLOv8n / YOLO11n ONNX through cv2.dnn on the CPU."""

    name = 'onnx'

    def __init__(self, model_path, input_size=320, conf_threshold=0.4, iou_threshold=0.45, threads=0):
        import cv2
        self._cv2 = cv2
        if threads:
            cv2.setNumThreads(threads)
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.size = int(input_size)
        self.conf = conf_threshold
        self.iou = iou_threshold
        self._warm_up()

    def _warm_up(self):
        # A model that loads but cannot run should fail here, where make_detector can fall back,
        # not later in the detector thread.
        self.detect(np.zeros((self.size, self.size, 3), dtype=np.uint8))

    def detect(self, frame_bgr):
        cv2 = self._cv2
        h, w = frame_bgr.shape[:2]
        scale, pad_x, pad_y, new_w, new_h = letterbox_params(w, h, self.size)
        canvas = np.full((self.size, self.size, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv2.resize(frame_bgr, (new_w, new_h))
        blob = cv2.dnn.blobFromImage(canvas, 1.0 / 255.0, (self.size, self.size), swapRB=True, crop=False)
        self.net.setInput(blob)
        out = self.net.forward()
        dets = decode_yolov8(out, scale, pad_x, pad_y, self.conf, frame_width=w, frame_height=h)
        return nms(dets, self.iou)


class OrtPersonDetector:
    """Same YOLO model through onnxruntime (pip install onnxruntime) if cv2.dnn cannot load it."""

    name = 'ort'

    def __init__(self, model_path, input_size=320, conf_threshold=0.4, iou_threshold=0.45, threads=0):
        import cv2
        import onnxruntime as ort
        self._cv2 = cv2
        opts = ort.SessionOptions()
        if threads:
            opts.intra_op_num_threads = threads
        self.sess = ort.InferenceSession(model_path, opts, providers=['CPUExecutionProvider'])
        self.input_name = self.sess.get_inputs()[0].name
        self.size = int(input_size)
        self.conf = conf_threshold
        self.iou = iou_threshold
        self.detect(np.zeros((self.size, self.size, 3), dtype=np.uint8))   # warm-up, see OnnxPersonDetector

    def detect(self, frame_bgr):
        cv2 = self._cv2
        h, w = frame_bgr.shape[:2]
        scale, pad_x, pad_y, new_w, new_h = letterbox_params(w, h, self.size)
        canvas = np.full((self.size, self.size, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv2.resize(frame_bgr, (new_w, new_h))
        rgb = canvas[:, :, ::-1].astype(np.float32) / 255.0
        blob = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None])
        out = self.sess.run(None, {self.input_name: blob})[0]
        dets = decode_yolov8(out, scale, pad_x, pad_y, self.conf, frame_width=w, frame_height=h)
        return nms(dets, self.iou)


def _have_onnxruntime():
    try:
        import onnxruntime  # noqa: F401
        return True
    except ImportError:
        return False


def make_detector(kind, model_path='', input_size=320, conf_threshold=0.4, log=None):
    """Build a detector.

    kind: 'hog', 'onnx' (cv2.dnn), 'ort' (onnxruntime), or 'auto': hog when model_path is empty, otherwise
    cv2.dnn and, if that cannot load *or run* the model and onnxruntime is installed, onnxruntime.
    `log` (optional callable taking a string) is told about the fallback. Raises if nothing works, so the
    caller can decide what to do (camera_node drops to HOG).
    """
    if kind == 'auto':
        if not model_path:
            return HogPersonDetector()
        try:
            return OnnxPersonDetector(model_path, input_size, conf_threshold)
        except Exception as e:  # cv2.error (load or forward), OSError
            if not _have_onnxruntime():
                raise RuntimeError(f'cv2.dnn cannot run {model_path!r} ({str(e).splitlines()[-1][:100]}) and '
                                   f'onnxruntime is not installed (pip install onnxruntime)') from e
            if log:
                log(f'cv2.dnn cannot run {model_path!r}; using onnxruntime instead')
            return OrtPersonDetector(model_path, input_size, conf_threshold)
    if kind in ('onnx', 'ort'):
        if not model_path:
            raise ValueError(f"detector {kind!r} needs model_path")
        cls = OnnxPersonDetector if kind == 'onnx' else OrtPersonDetector
        return cls(model_path, input_size, conf_threshold)
    if kind == 'hog':
        return HogPersonDetector()
    raise ValueError(f"detector must be 'auto', 'hog', 'onnx' or 'ort', not {kind!r}")
