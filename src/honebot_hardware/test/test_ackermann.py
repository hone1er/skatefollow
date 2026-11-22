import math

from honebot_hardware.ackermann import AckermannParams, pulse_us, twist_to_ackermann

P = AckermannParams(wheelbase=0.33, max_steer_angle=0.45, max_speed=5.0, min_speed_for_steer=0.3)


def test_straight_ahead():
    thr, steer = twist_to_ackermann(2.5, 0.0, P)
    assert math.isclose(thr, 0.5)
    assert steer == 0.0


def test_stopped_and_straight_is_neutral():
    assert twist_to_ackermann(0.0, 0.0, P) == (0.0, 0.0)


def test_left_turn_steers_left():
    _, steer = twist_to_ackermann(2.0, 1.0, P)
    expected = math.atan(0.33 * 1.0 / 2.0) / 0.45
    assert steer > 0 and math.isclose(steer, expected)


def test_right_turn_steers_right():
    _, steer = twist_to_ackermann(2.0, -1.0, P)
    assert steer < 0


def test_reversing_flips_steering_direction():
    _, fwd = twist_to_ackermann(1.0, 1.0, P)
    _, rev = twist_to_ackermann(-1.0, 1.0, P)
    assert fwd > 0 and rev < 0 and math.isclose(fwd, -rev)


def test_turn_request_at_zero_speed_still_steers():
    thr, steer = twist_to_ackermann(0.0, 2.0, P)
    assert thr == 0.0
    assert steer == 1.0          # clamped to full lock, not NaN/inf


def test_outputs_are_clamped():
    thr, steer = twist_to_ackermann(50.0, 100.0, P)
    assert thr == 1.0 and steer == 1.0
    # Reversing with a negative omega: the wheels must steer the *other* way
    # (omega = v tan(delta) / L with v < 0), so steer is +1 here.
    thr, steer = twist_to_ackermann(-50.0, -100.0, P)
    assert thr == -1.0 and steer == 1.0


def test_non_finite_input_is_neutral():
    assert twist_to_ackermann(math.nan, 0.0, P) == (0.0, 0.0)
    assert twist_to_ackermann(1.0, math.inf, P) == (0.0, 0.0)


def test_reversal_flags():
    q = AckermannParams(steer_reversed=True, throttle_reversed=True)
    thr, steer = twist_to_ackermann(2.5, 1.0, q)
    assert thr < 0 and steer < 0


def test_pulse_us():
    assert pulse_us(0.0) == 1500
    assert pulse_us(1.0) == 2000
    assert pulse_us(-1.0) == 1000
    assert pulse_us(3.0) == 2000
    assert pulse_us(0.5) == 1750


def test_zero_min_speed_for_steer_does_not_divide_by_zero():
    q = AckermannParams(min_speed_for_steer=0.0)
    assert twist_to_ackermann(0.0, 1.0, q) == (0.0, 0.0)
