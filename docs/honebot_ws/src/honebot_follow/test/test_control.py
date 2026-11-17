import math

from honebot_follow.control import FollowGains, follow_command, wrap_angle

G = FollowGains()


def test_drives_forward_toward_leader_ahead():
    lin, ang, _ = follow_command(0, 0, 0, 5, 0, G)
    assert lin > 0
    assert abs(ang) < 1e-9


def test_stops_inside_follow_distance():
    lin, _, _ = follow_command(0, 0, 0, 1.0, 0, G)
    assert lin == 0.0


def test_turns_without_driving_when_leader_behind():
    lin, ang, _ = follow_command(0, 0, 0, -5, 0.01, G)
    assert lin == 0.0
    assert abs(ang) == G.max_angular


def test_turns_left_for_leader_on_left():
    _, ang, _ = follow_command(0, 0, 0, 3, 3, G)
    assert ang > 0


def test_speed_is_capped():
    lin, _, _ = follow_command(0, 0, 0, 100, 0, G)
    assert lin == G.max_linear


def test_wrap_angle():
    assert math.isclose(wrap_angle(3 * math.pi), math.pi, abs_tol=1e-9) or \
        math.isclose(wrap_angle(3 * math.pi), -math.pi, abs_tol=1e-9)
    assert math.isclose(wrap_angle(-math.pi / 2), -math.pi / 2)
