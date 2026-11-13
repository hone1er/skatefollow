import math

from honebot_safety.gate import (CMD_STALE, LINK_DOWN, OBSTACLE, OK, RANGE_INVALID, RANGE_STALE,
                                 SLOW, GateParams, gate_command, slow_factor)

P = GateParams(stop_distance=0.4, slow_distance=1.0, range_timeout=0.3, cmd_timeout=0.5)


def run(linear=1.0, angular=0.5, range_m=2.0, range_age=0.05, link_ok=True, link_age=0.05, cmd_age=0.05):
    return gate_command(linear, angular, range_m, range_age, link_ok, link_age, cmd_age, P)


def test_passes_command_when_everything_is_fine():
    assert run() == (1.0, 0.5, OK)


def test_infinite_range_means_clear():
    assert run(range_m=math.inf) == (1.0, 0.5, OK)


def test_link_down_stops_everything():
    assert run(link_ok=False) == (0.0, 0.0, LINK_DOWN)


def test_link_never_seen_or_stale_stops_everything():
    assert run(link_age=None) == (0.0, 0.0, LINK_DOWN)
    assert run(link_age=1.0) == (0.0, 0.0, LINK_DOWN)


def test_stale_or_missing_range_stops_everything():
    assert run(range_age=0.31) == (0.0, 0.0, RANGE_STALE)
    assert run(range_m=None, range_age=None) == (0.0, 0.0, RANGE_STALE)


def test_invalid_range_stops_everything():
    assert run(range_m=math.nan) == (0.0, 0.0, RANGE_INVALID)
    assert run(range_m=0.0) == (0.0, 0.0, RANGE_INVALID)


def test_silent_controller_stops_everything():
    assert run(cmd_age=0.6) == (0.0, 0.0, CMD_STALE)
    assert run(cmd_age=None) == (0.0, 0.0, CMD_STALE)


def test_obstacle_blocks_forward_but_keeps_steering():
    assert run(range_m=0.3) == (0.0, 0.5, OBSTACLE)


def test_obstacle_allows_reversing_and_stopping():
    assert run(linear=-0.5, range_m=0.3) == (-0.5, 0.5, OK)
    assert run(linear=0.0, range_m=0.3) == (0.0, 0.5, OK)


def test_slow_zone_scales_forward_speed_linearly():
    lin, ang, state = run(range_m=0.7)          # halfway between stop and slow
    assert state == SLOW
    assert math.isclose(lin, 0.5)
    assert ang == 0.5


def test_slow_factor_edges():
    assert slow_factor(0.4, P) == 0.0
    assert slow_factor(1.0, P) == 1.0
    assert slow_factor(5.0, P) == 1.0
    assert math.isclose(slow_factor(0.55, P), 0.25)
    no_slow_zone = GateParams(stop_distance=0.5, slow_distance=0.5)
    assert slow_factor(0.51, no_slow_zone) == 1.0


def test_priority_link_beats_obstacle():
    assert run(link_ok=False, range_m=0.1) == (0.0, 0.0, LINK_DOWN)
