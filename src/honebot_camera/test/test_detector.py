import numpy as np
import pytest

from honebot_camera import detector

from honebot_camera.detector import (Detection, decode_yolov8, iou, letterbox_params, nms,
                                     select_target)


def test_letterbox_16_9_into_square():
    scale, pad_x, pad_y, new_w, new_h = letterbox_params(640, 360, 320)
    assert scale == 0.5
    assert (new_w, new_h) == (320, 180)
    assert pad_x == 0 and pad_y == 70


def test_letterbox_tall_frame():
    scale, pad_x, pad_y, new_w, new_h = letterbox_params(360, 640, 320)
    assert new_h == 320 and pad_y == 0 and pad_x == 70


def _yolo_output(rows, n=20, nc=80):
    """Build a (1, 4+nc, n) tensor with the given (cx, cy, w, h, person_score) rows, rest zero."""
    out = np.zeros((1, 4 + nc, n), dtype=np.float32)
    for i, (cx, cy, w, h, s) in enumerate(rows):
        out[0, :4, i] = (cx, cy, w, h)
        out[0, 4, i] = s
    return out


def test_decode_maps_boxes_back_to_frame_pixels():
    # frame 640x360 letterboxed to 320: scale 0.5, pad_y 70. A person at model (160, 160) 40x120.
    out = _yolo_output([(160, 160, 40, 120, 0.9), (50, 50, 10, 10, 0.1)])
    dets = decode_yolov8(out, 0.5, 0, 70, conf_threshold=0.4, frame_width=640, frame_height=360)
    assert len(dets) == 1                       # the 0.1 one is below threshold
    d = dets[0]
    assert abs(d.cx - 320) < 1e-3 and abs(d.cy - 180) < 1e-3
    assert abs(d.width - 80) < 1e-3 and abs(d.height - 240) < 1e-3
    assert d.score == np.float32(0.9)


def test_decode_clips_to_frame_and_accepts_transposed_layout():
    out = _yolo_output([(5, 160, 40, 120, 0.8)], n=300)       # hangs off the left edge
    dets = decode_yolov8(out[0].T, 0.5, 0, 70, frame_width=640, frame_height=360)   # (N, 84) layout
    assert len(dets) == 1 and dets[0].x1 == 0.0


def test_decode_ignores_other_classes():
    out = np.zeros((1, 84, 5), dtype=np.float32)
    out[0, :4, 0] = (100, 100, 20, 20)
    out[0, 4 + 2, 0] = 0.99                   # class 2 = car
    assert decode_yolov8(out, 1.0, 0, 0) == []


def test_nms_merges_overlapping_boxes():
    a = Detection(100, 100, 200, 300, 0.9)
    b = Detection(105, 110, 205, 310, 0.8)      # almost the same box
    c = Detection(400, 100, 480, 300, 0.7)      # someone else
    kept = nms([b, a, c], 0.45)
    assert kept == [a, c]
    assert iou(a, b) > 0.8 and iou(a, c) == 0.0


def test_select_target_prefers_hint_then_previous_then_biggest():
    left = Detection(50, 100, 110, 300, 0.9)      # cx 80, small
    big = Detection(300, 50, 460, 350, 0.9)       # cx 380, biggest
    right = Detection(540, 100, 600, 300, 0.9)    # cx 570
    dets = [left, big, right]
    assert select_target(dets) is big
    assert select_target(dets, prev_cx=560) is right
    assert select_target(dets, prev_cx=560, hint_cx=90) is left          # hint beats previous
    assert select_target(dets, prev_cx=560, hint_cx=60, gate_px=15) is right  # hint too far, prev wins
    assert select_target(dets, prev_cx=200, gate_px=20) is big           # nothing in gate: biggest
    assert select_target([]) is None


# --- make_detector('auto') fallback, with fake backends so no cv2/model is needed -------------

class _Boom:
    def __init__(self, *a, **k):
        raise RuntimeError('forward failed')


class _Fake:
    name = 'fake'

    def __init__(self, model_path, input_size, conf):
        self.args = (model_path, input_size, conf)


def test_auto_without_model_is_hog(monkeypatch):
    monkeypatch.setattr(detector, 'HogPersonDetector', lambda: 'hog')
    assert detector.make_detector('auto', '') == 'hog'


def test_auto_prefers_cv2_dnn_when_it_works(monkeypatch):
    monkeypatch.setattr(detector, 'OnnxPersonDetector', _Fake)
    monkeypatch.setattr(detector, 'OrtPersonDetector', _Boom)
    d = detector.make_detector('auto', 'm.onnx', 320, 0.4)
    assert isinstance(d, _Fake) and d.args == ('m.onnx', 320, 0.4)


def test_auto_falls_back_to_onnxruntime_when_cv2_fails(monkeypatch):
    monkeypatch.setattr(detector, 'OnnxPersonDetector', _Boom)
    monkeypatch.setattr(detector, 'OrtPersonDetector', _Fake)
    monkeypatch.setattr(detector, '_have_onnxruntime', lambda: True)
    msgs = []
    d = detector.make_detector('auto', 'm.onnx', 320, 0.4, log=msgs.append)
    assert isinstance(d, _Fake)
    assert msgs and 'onnxruntime' in msgs[0]


def test_auto_raises_when_cv2_fails_and_no_onnxruntime(monkeypatch):
    monkeypatch.setattr(detector, 'OnnxPersonDetector', _Boom)
    monkeypatch.setattr(detector, '_have_onnxruntime', lambda: False)
    with pytest.raises(RuntimeError, match='onnxruntime is not installed'):
        detector.make_detector('auto', 'm.onnx')
