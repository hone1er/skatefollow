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


# --- relative-target form (what UWB will feed) --------------------------------

from honebot_follow.control import follow_command_relative, relative_target  # noqa: E402

A = FollowGains(ackermann=True, creep_speed=0.3)


def test_relative_target_from_poses():
    d, b = relative_target(0, 0, 0, 3, 3)
    assert math.isclose(d, math.hypot(3, 3))
    assert math.isclose(b, math.pi / 4)
    # follower facing +y, leader straight ahead of it
    d, b = relative_target(1, 1, math.pi / 2, 1, 5)
    assert math.isclose(d, 4) and math.isclose(b, 0, abs_tol=1e-9)


def test_pose_and_relative_forms_agree():
    lin1, ang1, d1 = follow_command(1, 2, 0.7, 4, -1, G)
    lin2, ang2, d2 = follow_command_relative(*relative_target(1, 2, 0.7, 4, -1), G)
    assert (lin1, ang1, d1) == (lin2, ang2, d2)


def test_relative_straight_ahead():
    lin, ang, _ = follow_command_relative(5.0, 0.0, G)
    assert lin > 0 and ang == 0.0


def test_relative_stops_inside_follow_distance():
    lin, _, _ = follow_command_relative(1.0, 0.3, G)
    assert lin == 0.0


def test_diff_drive_target_behind_turns_in_place():
    lin, ang, _ = follow_command_relative(5.0, math.pi - 0.01, G)
    assert lin == 0.0 and ang == G.max_angular


def test_ackermann_target_behind_creeps_forward_with_full_steer():
    lin, ang, _ = follow_command_relative(5.0, math.pi - 0.01, A)
    assert lin == A.creep_speed and ang == A.max_angular


def test_ackermann_does_not_creep_when_close():
    lin, _, _ = follow_command_relative(1.0, math.pi - 0.01, A)
    assert lin == 0.0


def test_ackermann_normal_following_unchanged():
    assert follow_command_relative(5.0, 0.0, A) == follow_command_relative(5.0, 0.0, G)
