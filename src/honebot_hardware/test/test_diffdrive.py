import math

from honebot_hardware.diffdrive import DiffDriveParams, twist_to_diffdrive

P = DiffDriveParams(track_width=0.40, max_wheel_speed=7.0)


def test_straight():
    l, r = twist_to_diffdrive(3.5, 0.0, P)
    assert math.isclose(l, 0.5) and math.isclose(r, 0.5)


def test_stopped_is_neutral():
    assert twist_to_diffdrive(0.0, 0.0, P) == (0.0, 0.0)


def test_left_turn_right_wheel_faster():
    l, r = twist_to_diffdrive(2.0, 1.0, P)
    assert r > l
    assert math.isclose(r - l, 1.0 * 0.40 / 7.0)


def test_turn_in_place_is_opposite_wheels():
    l, r = twist_to_diffdrive(0.0, 2.0, P)
    assert math.isclose(l, -r) and r > 0


def test_saturation_preserves_ratio():
    l, r = twist_to_diffdrive(10.0, 5.0, P)          # both would exceed 1.0
    assert max(abs(l), abs(r)) == 1.0
    raw_l, raw_r = (10.0 - 5.0 * 0.2) / 7.0, (10.0 + 5.0 * 0.2) / 7.0
    assert math.isclose(l / r, raw_l / raw_r)


def test_reverse():
    l, r = twist_to_diffdrive(-3.5, 0.0, P)
    assert math.isclose(l, -0.5) and math.isclose(r, -0.5)


def test_reversal_flags_and_bad_input():
    q = DiffDriveParams(left_reversed=True)
    l, r = twist_to_diffdrive(3.5, 0.0, q)
    assert l < 0 < r
    assert twist_to_diffdrive(math.nan, 0.0, P) == (0.0, 0.0)
