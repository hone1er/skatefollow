import math

from honebot_uwb.localize import (RangeFilter, UwbGeometry, aoa_position, to_distance_bearing,
                                  two_anchor_position)

G = UwbGeometry(baseline=0.30, anchor_x=0.0)


def ranges_for(x, y, g=G):
    rl = math.hypot(x - g.anchor_x, y - g.baseline / 2)
    rr = math.hypot(x - g.anchor_x, y + g.baseline / 2)
    return rl, rr


def test_tag_straight_ahead():
    x, y = two_anchor_position(*ranges_for(3.0, 0.0), G)
    assert math.isclose(x, 3.0, abs_tol=1e-6) and math.isclose(y, 0.0, abs_tol=1e-6)


def test_tag_ahead_left_and_right_round_trip():
    for tx, ty in [(3.0, 1.0), (3.0, -1.0), (0.5, 0.1), (10.0, -4.0)]:
        x, y = two_anchor_position(*ranges_for(tx, ty), G)
        assert math.isclose(x, tx, abs_tol=1e-6) and math.isclose(y, ty, abs_tol=1e-6)


def test_anchor_offset_is_added():
    g = UwbGeometry(baseline=0.30, anchor_x=0.2)
    x, y = two_anchor_position(*ranges_for(3.0, 0.5, g), g)
    assert math.isclose(x, 3.0, abs_tol=1e-6) and math.isclose(y, 0.5, abs_tol=1e-6)


def test_inconsistent_ranges_rejected():
    assert two_anchor_position(5.0, 1.0, G) is None       # differ by far more than the baseline
    assert two_anchor_position(-1.0, 2.0, G) is None      # no fix
    assert two_anchor_position(2.0, 0.0, G) is None


def test_noise_cannot_produce_nan():
    # ranges slightly too short for the geometry -> clamp onto the baseline, no NaN
    x, y = two_anchor_position(0.14, 0.16, G)
    assert math.isfinite(x) and math.isfinite(y) and x >= 0.0


def test_distance_bearing():
    d, b = to_distance_bearing(3.0, 3.0)
    assert math.isclose(d, math.hypot(3, 3)) and math.isclose(b, math.pi / 4)


def test_filter_kills_single_spike():
    f = RangeFilter(window=3, alpha=1.0)
    for r in [2.0, 2.0, 2.0]:
        f.update(r)
    assert f.update(9.0) == 2.0            # median of (2, 2, 9)
    assert f.update(2.1) == 2.1 or f.update(2.1) == 2.1


def test_filter_reports_a_dropout_instead_of_the_last_value():
    # "-1" is the board's "no fix this cycle". The filter must say so (None): if it handed back the last
    # estimate, the node would keep publishing a fresh-looking target and the robot would chase a ghost.
    f = RangeFilter(window=1, alpha=1.0, max_range=30.0)
    assert f.update(float('nan')) is None
    assert f.update(3.0) == 3.0
    assert f.update(-1.0) is None
    assert f.update(100.0) is None
    assert f.update(3.2) == 3.2               # a short gap keeps the history


def test_filter_forgets_after_a_long_dropout():
    f = RangeFilter(window=3, alpha=0.5, max_misses=3)
    for r in (2.0, 2.0, 2.0):
        f.update(r)
    for _ in range(3):
        assert f.update(-1.0) is None
    assert f.update(8.0) == 8.0               # re-acquired far away: not blended with the old 2.0


def test_bearing_filter_accepts_negative_angles():
    f = RangeFilter(window=1, alpha=1.0, min_range=-80.0, max_range=80.0)
    assert f.update(-30.0) == -30.0
    assert f.update(85.0) is None             # outside the anchor's usable angle


def test_filter_smooths():
    f = RangeFilter(window=1, alpha=0.5)
    f.update(2.0)
    assert math.isclose(f.update(4.0), 3.0)


def test_aoa_straight_ahead_and_left():
    assert aoa_position(3.0, 0.0, G) == (3.0, 0.0)
    x, y = aoa_position(2.0, math.radians(90), G)
    assert math.isclose(x, 0.0, abs_tol=1e-9) and math.isclose(y, 2.0)


def test_aoa_offset_and_rejects():
    g = UwbGeometry(anchor_x=0.2)
    assert aoa_position(1.0, 0.0, g) == (1.2, 0.0)
    assert aoa_position(-1.0, 0.0, G) is None
    assert aoa_position(99.0, 0.0, G, max_range=30.0) is None
    assert aoa_position(float('nan'), 0.0, G) is None


def test_two_anchor_rejects_non_finite():
    for rl, rr in [(math.nan, 2.0), (2.0, math.nan), (math.inf, 2.0), (2.0, -math.inf)]:
        assert two_anchor_position(rl, rr, G) is None
