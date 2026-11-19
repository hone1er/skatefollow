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


# ---- sight limit: never faster than the robot can stop within what the sensor sees ----------------
from honebot_safety.gate import SIGHT_LIMIT, rising_cap, sight_speed  # noqa: E402

SIGHT = GateParams(brake_decel=4.4, reaction_time=0.25, sight_distance=3.0)


def _run(linear, range_m, p=SIGHT, cap=None):
    return gate_command(linear, 0.0, range_m, 0.0, True, 0.0, 0.0, p, cap)


def test_sight_limit_is_off_by_default():
    assert sight_speed(1.0, GateParams()) == math.inf
    assert _run(9.0, 2.0, GateParams())[0] == 9.0


def test_sight_speed_stops_within_the_room_it_has():
    for rng in (0.6, 1.0, 1.95, 3.0, math.inf):
        v = sight_speed(rng, SIGHT)
        room = min(rng, SIGHT.sight_distance) - SIGHT.stop_distance
        assert math.isclose(SIGHT.reaction_time * v + v * v / (2 * SIGHT.brake_decel), room, rel_tol=1e-9)
    assert sight_speed(0.3, SIGHT) == 0.0
    # the two numbers the docs quote: ground return at 1.95 m, and "nothing seen"
    assert math.isclose(sight_speed(1.95, SIGHT), 2.75, abs_tol=0.05)
    assert math.isclose(sight_speed(math.inf, SIGHT), 3.81, abs_tol=0.05)


def test_sight_limit_caps_and_names_the_state():
    lin, _, state = _run(6.0, math.inf)
    assert math.isclose(lin, sight_speed(math.inf, SIGHT)) and state == SIGHT_LIMIT
    lin, _, state = _run(1.0, math.inf)
    assert lin == 1.0 and state == OK                    # below the limit: untouched
    lin, _, state = _run(-2.0, 0.2)
    assert lin == -2.0 and state == OK                   # reversing is never capped


def test_sight_limit_never_fights_the_follower_behind_a_skater():
    # When the skater is what the sensor sees, the follower (1.5 m gap, 0.5 s headway, same decel) must
    # already be at or below the gate's limit, or the two would tug against each other.
    def follower_speed(d):
        extra = d - 1.5
        return 0.0 if extra <= 0 else 4.4 * (-0.5 + math.sqrt(0.25 + 2 * extra / 4.4))
    d = 0.5
    while d <= SIGHT.sight_distance:
        assert sight_speed(d, SIGHT) >= follower_speed(d) - 1e-9, d
        d += 0.05


def test_cap_drops_at_once_and_rises_slowly():
    cap = None
    seen = []
    for i in range(100):                                   # 2 s at 50 Hz, reading flickers ground / nothing
        rng = 1.95 if i % 2 == 0 else math.inf
        cap = rising_cap(cap, sight_speed(rng, SIGHT), 0.02, SIGHT)
        seen.append(cap)
    low = sight_speed(1.95, SIGHT)
    assert max(seen[1:]) <= low + SIGHT.cap_rise_rate * 0.02 + 1e-9      # never surges toward the 'nothing' limit
    # steady "nothing": climbs at cap_rise_rate, reaching the higher limit after about a second
    for _ in range(100):
        cap = rising_cap(cap, sight_speed(math.inf, SIGHT), 0.02, SIGHT)
    assert math.isclose(cap, sight_speed(math.inf, SIGHT))
    assert rising_cap(cap, 0.5, 0.02, SIGHT) == 0.5                      # an obstacle: drops immediately


# --- blind check: no echo at all for blind_timeout -> no forward motion ------------------------------------------
from honebot_safety.gate import BLIND  # noqa: E402

PB = GateParams(blind_timeout=0.5)


def blind(linear, echo_age, range_m=math.inf, p=PB, angular=0.3, cmd_age=0.0):
    return gate_command(linear, angular, range_m, 0.0, True, 0.0, cmd_age, p, None, echo_age)


def test_blind_check_is_off_by_default():
    assert gate_command(1.0, 0.0, math.inf, 0.0, True, 0.0, 0.0, P, None, None)[2] == OK


def test_no_echo_for_longer_than_the_timeout_blocks_forward_but_keeps_steering():
    assert blind(1.0, 0.4) == (1.0, 0.3, OK)               # a few missed pings: still fine
    assert blind(1.0, 0.6) == (0.0, 0.3, BLIND)


def test_never_echoed_counts_as_blind():
    assert blind(1.0, None)[2] == BLIND                    # sensor unplugged at boot


def test_blind_allows_reversing_and_stopping():
    assert blind(-0.5, 5.0) == (-0.5, 0.3, OK)
    assert blind(0.0, 5.0)[2] == OK


def test_fresh_echo_clears_blind_and_obstacle_rules_still_apply():
    assert blind(1.0, 0.0, range_m=1.95)[2] == OK
    assert blind(1.0, 0.0, range_m=0.2)[2] == 'OBSTACLE'


def test_stop_states_beat_blind():
    assert blind(1.0, 5.0, cmd_age=None) == (0.0, 0.0, CMD_STALE)
