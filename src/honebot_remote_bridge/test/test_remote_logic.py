import math

from honebot_remote_bridge.remote_logic import (MANUAL_LIMIT, SRC_MANUAL, SRC_NONE, SRC_PI, Arbiter,
                                                arcade_mix, parse_remote_line, remote_to_wheels, shape_stick,
                                                wheels_to_twist)

RUN_FWD = 'state=RUN x=+1.00 y=+0.00 btn=0 robot=1 pi=1 failsafe=0 manual=1'


def test_parse_real_firmware_line():
    s = parse_remote_line('state=RUN x=+0.50 y=-0.20 btn=0 robot=1 pi=0 failsafe=0 manual=1\r\n')
    assert s.state == 'RUN' and s.run
    assert math.isclose(s.x, 0.5) and math.isclose(s.y, -0.2)
    assert not s.btn and s.robot and s.manual


def test_parse_ignores_banner_and_junk():
    assert parse_remote_line('honebot remote 1.0.0  centre x=1850 y=1902  stick=ok radio=ok') is None
    assert parse_remote_line('') is None
    assert parse_remote_line('ets Jun  8 2016 00:22:57') is None
    assert parse_remote_line('state=RUN x=abc y=0') is None
    assert parse_remote_line('state=MANUAL x=0 y=0') is None


def test_parse_clamps_out_of_range():
    s = parse_remote_line('state=RUN x=+3.00 y=-7.00')
    assert s.x == 1.0 and s.y == -1.0


def test_stop_and_fault_never_drive():
    for state in ('STOP', 'FAULT'):
        s = parse_remote_line(f'state={state} x=+1.00 y=+1.00 btn=0')
        assert remote_to_wheels(s) == (0.0, 0.0)
    assert remote_to_wheels(None) == (0.0, 0.0)


def test_deadband_matches_firmware():
    assert shape_stick(0.10) == 0.0
    assert shape_stick(-0.14) == 0.0
    assert shape_stick(1.0) == 1.0
    assert math.isclose(shape_stick(0.575), 0.5)
    assert math.isclose(shape_stick(-0.575), -0.5)


def test_full_forward_is_manual_limit_on_both_wheels():
    left, right = remote_to_wheels(parse_remote_line(RUN_FWD))
    assert math.isclose(left, MANUAL_LIMIT) and math.isclose(right, MANUAL_LIMIT)


def test_stick_left_turns_left():
    # firmware: y = +1 is stick pushed left -> right wheel faster -> positive (CCW) yaw
    left, right = remote_to_wheels(parse_remote_line('state=RUN x=+0.00 y=+1.00'))
    assert right > 0 > left
    _, angular = wheels_to_twist(left, right)
    assert angular > 0


def test_arcade_mix_scales_together_when_saturated():
    left, right = arcade_mix(1.0, 1.0, limit=1.0, turn_scale=0.6)
    assert math.isclose(right, 1.0)
    assert math.isclose(left, 0.4 / 1.6)


def test_wheels_to_twist_full_manual_forward_is_2_4_mps():
    linear, angular = wheels_to_twist(0.25, 0.25)
    assert math.isclose(linear, 2.4) and angular == 0.0


def test_wheels_to_twist_inverts_diffdrive():
    # honebot_hardware.diffdrive: left = (v - w*T/2)/S, right = (v + w*T/2)/S
    v, w, T, S = 1.2, 0.8, 0.39, 9.6
    left, right = (v - w * T / 2) / S, (v + w * T / 2) / S
    linear, angular = wheels_to_twist(left, right, T, S)
    assert math.isclose(linear, v) and math.isclose(angular, w)


def test_scale_keeps_the_path():
    linear, angular = wheels_to_twist(0.2, 0.1)
    sl, sa = wheels_to_twist(0.2, 0.1, scale=0.5)
    assert math.isclose(sl, linear * 0.5) and math.isclose(sa, angular * 0.5)
    assert math.isclose(linear / angular, sl / sa)   # same turn radius


def test_bad_numbers_are_neutral():
    assert wheels_to_twist(float('nan'), 0.1) == (0.0, 0.0)


# --- Arbiter: same decisions as drive::decide in the robot firmware ----------------------------------------

def _r(state, x=0.0, y=0.0):
    return parse_remote_line(f'state={state} x={x:+.2f} y={y:+.2f}')


def test_never_heard_remote_blocks_the_pi():
    a = Arbiter()
    assert a.decide(1.0, None, None, pi_time=1.0) == (SRC_NONE, 'never')


def test_stop_blocks_the_pi():
    a = Arbiter()
    assert a.decide(1.0, _r('STOP'), 1.0, pi_time=1.0) == (SRC_NONE, 'stop')
    assert a.decide(1.1, _r('FAULT'), 1.1, pi_time=1.1) == (SRC_NONE, 'stop')


def test_run_centred_lets_the_pi_drive():
    a = Arbiter()
    assert a.decide(1.0, _r('RUN'), 1.0, pi_time=1.0) == (SRC_PI, 'run')


def test_run_without_pi_commands_is_nobody():
    a = Arbiter()
    assert a.decide(1.0, _r('RUN'), 1.0, pi_time=None)[0] == SRC_NONE
    assert a.decide(2.0, _r('RUN'), 2.0, pi_time=1.0)[0] == SRC_NONE      # Pi command 1 s old


def test_stick_overrides_the_pi_then_hands_back_after_the_hold():
    a = Arbiter()
    assert a.decide(0.0, _r('RUN'), 0.0, 0.0)[0] == SRC_PI
    assert a.decide(0.1, _r('RUN', x=-0.8), 0.1, 0.1)[0] == SRC_MANUAL     # stick pushed: you drive
    assert a.decide(0.5, _r('RUN', x=-0.8), 0.5, 0.5)[0] == SRC_MANUAL
    assert a.decide(0.6, _r('RUN'), 0.6, 0.6)[0] == SRC_MANUAL             # released: still you (hold)
    assert a.decide(1.2, _r('RUN'), 1.2, 1.2)[0] == SRC_MANUAL             # 0.7 s after the last push
    assert a.decide(1.35, _r('RUN'), 1.35, 1.35)[0] == SRC_PI              # 0.85 s: the follower again


def test_stick_held_when_entering_run_does_not_drive():
    # the stick must be seen centred in this RUN before it may drive (loose wire / held stick guard)
    a = Arbiter()
    assert a.decide(0.0, _r('STOP', x=1.0), 0.0, 0.0)[0] == SRC_NONE
    assert a.decide(0.1, _r('RUN', x=1.0), 0.1, 0.1)[0] == SRC_PI
    assert a.decide(0.2, _r('RUN'), 0.2, 0.2)[0] == SRC_PI
    assert a.decide(0.3, _r('RUN', x=1.0), 0.3, 0.3)[0] == SRC_MANUAL


def test_lost_remote_stops_everything_and_rearms():
    a = Arbiter()
    a.decide(0.0, _r('RUN'), 0.0, 0.0)
    a.decide(0.1, _r('RUN', x=1.0), 0.1, 0.1)
    assert a.decide(0.7, _r('RUN', x=1.0), 0.1, 0.7) == (SRC_NONE, 'lost')  # last line 0.6 s ago
    # back in range with the stick still pushed: not manual until it has been centred again
    assert a.decide(0.8, _r('RUN', x=1.0), 0.8, 0.8)[0] == SRC_PI


def test_click_to_stop_during_manual():
    a = Arbiter()
    a.decide(0.0, _r('RUN'), 0.0, 0.0)
    a.decide(0.1, _r('RUN', y=1.0), 0.1, 0.1)
    assert a.decide(0.2, _r('STOP', y=1.0), 0.2, 0.2) == (SRC_NONE, 'stop')
