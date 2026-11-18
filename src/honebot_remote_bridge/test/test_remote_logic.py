import math

from honebot_remote_bridge.remote_logic import (MANUAL_LIMIT, arcade_mix, parse_remote_line,
                                                remote_to_wheels, shape_stick, wheels_to_twist)

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
