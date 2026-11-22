import math

import pytest

from honebot_camera.detector import Detection
from honebot_camera.track import (BearingKF, Candidate, PlausibilityParams, TargetTracker, TrackParams,
                                  expected_height_frac, plausible, tracked_image_bearing)

W, H = 640, 360
HFOV = math.radians(87.0)
deg = math.radians


def box(cx, cy, w, h, score=0.9):
    return Detection(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2, score)


def person(cx=320, h=200):
    """A standing person box with feet in the lower half of the frame."""
    return box(cx, 180 + h / 4, h / 3, h)


# --- plausibility ---------------------------------------------------------------------------------

def test_standing_person_is_plausible():
    assert plausible(person(), W, H, PlausibilityParams())


def test_pole_and_bench_are_rejected_by_aspect():
    pole = box(300, 180, 12, 300)            # aspect 25
    bench = box(300, 300, 300, 60)           # aspect 0.2
    p = PlausibilityParams()
    assert not plausible(pole, W, H, p)
    assert not plausible(bench, W, H, p)


def test_crouching_and_wide_trick_poses_pass():
    crouch = box(300, 300, 120, 110)         # aspect ~0.9
    assert plausible(crouch, W, H, PlausibilityParams())


def test_speck_and_floating_box_are_rejected():
    p = PlausibilityParams()
    assert not plausible(box(300, 200, 5, 12), W, H, p)            # 3 % of frame height
    assert not plausible(box(300, 40, 30, 70), W, H, p)             # bottom edge at y=75 -> y_norm -0.58


def test_size_check_against_uwb_distance():
    p = PlausibilityParams()
    exp = expected_height_frac(6.0, HFOV, W, H)                    # person at 6 m: f ~337 px -> ~95 px of 360
    assert 0.2 < exp < 0.35
    ok = box(320, 220, 30, exp * H)
    crouch = box(320, 240, 40, 0.5 * exp * H)                      # half height still accepted
    tiny = box(320, 220, 8, 0.2 * exp * H)                         # far too small for 6 m
    huge = box(320, 200, 100, 3.0 * exp * H)                       # a person at 2 m when UWB says 6 m
    assert plausible(ok, W, H, p, exp)
    assert plausible(crouch, W, H, p, exp)
    assert not plausible(tiny, W, H, p, exp)
    assert not plausible(huge, W, H, p, exp)


def test_expected_height_scales_with_distance():
    assert expected_height_frac(2.0, HFOV, W, H) == pytest.approx(2 * expected_height_frac(4.0, HFOV, W, H))
    assert expected_height_frac(0.0, HFOV, W, H) == math.inf


# --- Kalman filter ---------------------------------------------------------------------------------

def test_kf_tracks_constant_velocity_and_predicts_ahead():
    kf = BearingKF(0.0, meas_std=deg(1), accel_std=deg(90))
    rate = deg(30)
    t = 0.0
    for _ in range(30):                          # 3 s at 10 Hz
        t += 0.1
        kf.predict(0.1)
        kf.update(rate * t)
    assert kf.v == pytest.approx(rate, abs=deg(3))
    kf.predict(0.5)                              # coast half a second
    assert kf.b == pytest.approx(rate * (t + 0.5), abs=deg(3))


def test_kf_uncertainty_grows_while_coasting():
    kf = BearingKF(0.0, meas_std=deg(1), accel_std=deg(90))
    for _ in range(10):
        kf.predict(0.1)
        kf.update(0.0)
    s0 = kf.innovation_std()
    for _ in range(10):
        kf.predict(0.1)
    assert kf.innovation_std() > 3 * s0


# --- tracker: confirmation ---------------------------------------------------------------------------

def run(tracker, frames, dt=0.1, hint=None):
    """frames: list of candidate lists. Returns the list of states."""
    out = []
    t = 0.0
    for cands in frames:
        out.append(tracker.update(t, cands, hint))
        t += dt
    return out


def test_single_frame_false_positive_is_never_confirmed():
    tr = TargetTracker(TrackParams(confirm_hits=3))
    states = run(tr, [[Candidate(deg(10), person())], [], [], []])
    assert states[0] is not None and not states[0].confirmed
    assert states[1] is None and states[2] is None


def test_flickering_detection_never_confirms():
    tr = TargetTracker(TrackParams(confirm_hits=3))
    c = [Candidate(deg(10), person())]
    states = run(tr, [c, [], c, [], c, []])
    assert not any(s is not None and s.confirmed for s in states)


def test_consecutive_hits_confirm():
    tr = TargetTracker(TrackParams(confirm_hits=3))
    c = [Candidate(deg(10), person())]
    states = run(tr, [c, c, c])
    assert [s.confirmed for s in states] == [False, False, True]
    assert states[-1].matched and states[-1].det is c[0].det


# --- tracker: association and coasting -------------------------------------------------------------

def confirmed_tracker(bearing=deg(0)):
    tr = TargetTracker(TrackParams(confirm_hits=3))
    run(tr, [[Candidate(bearing, person())]] * 3)
    assert tr.confirmed
    return tr


def test_far_detection_is_outside_the_gate():
    tr = confirmed_tracker(deg(0))
    s = tr.update(0.4, [Candidate(deg(20), person(500))])
    assert s.confirmed and not s.matched           # still our track, just a miss
    assert s.bearing == pytest.approx(0.0, abs=deg(1))


def test_coasting_survives_misses_then_reacquires_same_target_not_biggest():
    tr = confirmed_tracker(deg(0))
    t = 0.3
    for _ in range(5):                             # 0.5 s of misses
        s = tr.update(t, [])
        assert s is not None and s.confirmed and not s.matched
        t += 0.1
    assert s.coast == pytest.approx(0.5, abs=1e-6)
    small_near = Candidate(deg(2), person(330, h=100))
    big_far = Candidate(deg(22), person(520, h=300))
    s = tr.update(t, [big_far, small_near])
    assert s.matched and s.det is small_near.det


def test_track_is_dropped_after_coast_time_and_reseeds():
    tr = confirmed_tracker(deg(0))
    t = 0.3
    for _ in range(12):                            # 1.2 s > coast_time 1.0
        s = tr.update(t, [])
        t += 0.1
    assert s is None
    s = tr.update(t, [Candidate(deg(30), person())])
    assert s is not None and not s.confirmed and s.hits == 1


def test_gate_widens_while_coasting_but_is_capped():
    tr = confirmed_tracker(deg(0))
    g0 = tr.gate()
    t = 0.3
    for _ in range(8):
        tr.update(t, [])
        t += 0.1
    g1 = tr.gate()
    assert g1 > g0
    assert g1 <= TrackParams().gate_max + 1e-9


def test_moving_target_is_followed_by_prediction():
    tr = TargetTracker(TrackParams(confirm_hits=2))
    rate = deg(40)
    t = 0.0
    s = None
    for _ in range(20):
        s = tr.update(t, [Candidate(rate * t, person())])
        t += 0.1
    assert s.confirmed and s.matched
    # two misses: the estimate keeps moving, and a detection where the target *should* be is accepted
    s = tr.update(t, []); t += 0.1
    s = tr.update(t, []); t += 0.1
    assert s.bearing == pytest.approx(rate * (t - 0.1), abs=deg(3))
    s = tr.update(t, [Candidate(rate * t, person())])
    assert s.matched


# --- seeding with the UWB hint -----------------------------------------------------------------------

def test_hint_picks_nearest_candidate_over_biggest():
    tr = TargetTracker()
    big = Candidate(deg(-20), person(100, h=300))
    small = Candidate(deg(14), person(500, h=120))
    s = tr.update(0.0, [big, small], hint=deg(12))
    assert s.det is small.det


def test_hint_far_from_everyone_falls_back_to_biggest():
    tr = TargetTracker()
    big = Candidate(deg(-20), person(100, h=300))
    small = Candidate(deg(14), person(500, h=120))
    s = tr.update(0.0, [big, small], hint=deg(60))
    assert s.det is big.det


def test_tracked_image_bearing_removes_pan():
    tr = confirmed_tracker(deg(30))
    s = tr.update(0.4, [Candidate(deg(30), person())])
    assert tracked_image_bearing(s, deg(25)) == pytest.approx(deg(5), abs=deg(0.5))


def test_stale_track_resets_after_video_freeze():
    tr = confirmed_tracker(deg(0))                 # last match at t = 0.2, coast_time = 1.0
    # Video freezes for 3 s, then a random person appears near 0 deg: must NOT be immediately confirmed.
    s = tr.update(3.2, [Candidate(deg(1), person())])
    assert s is not None and not s.confirmed and s.hits == 1


def test_live_kf_param_updates_and_zero_meas_std_safe():
    tr = confirmed_tracker(deg(0))
    tr.p.meas_std = deg(5.0)
    tr.p.accel_std = deg(180.0)
    tr.update(0.3, [Candidate(deg(0), person())])
    assert tr.kf.r == pytest.approx(deg(5.0) ** 2)
    assert tr.kf.q == pytest.approx(deg(180.0) ** 2)
    kf = BearingKF(0.0, meas_std=0.0, accel_std=0.0)
    kf.update(0.1)                                 # must not ZeroDivisionError
    assert math.isfinite(kf.b)
