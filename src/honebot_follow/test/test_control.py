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


# ---- real-robot options: speed-dependent gap, arcs instead of pivots, cornering cap -------------
from honebot_follow.control import safe_gap, speed_for_gap  # noqa: E402

ROBOT = FollowGains(follow_distance=1.5, max_linear=8.0, kp_angular=3.0, max_angular=3.0,
                    brake_decel=4.4, time_headway=0.5, max_lat_accel=4.0,
                    min_turn_radius=0.4, arc_speed=0.6, pivot_bearing=2.0, pivot_rate=0.8, stopped_turn_deadband=0.6)


def test_new_options_default_off():
    g = FollowGains()
    assert g.brake_decel == 0 and g.max_lat_accel == 0 and g.min_turn_radius == 0 and g.stopped_turn_deadband == 0
    assert safe_gap(5.0, g) == g.follow_distance


def test_safe_gap_and_inverse_agree():
    for v in (0.0, 1.0, 3.6, 5.4, 6.7):
        d = safe_gap(v, ROBOT)
        assert math.isclose(speed_for_gap(d, ROBOT), v, abs_tol=1e-9)
    assert speed_for_gap(1.0, ROBOT) == 0.0                      # inside follow_distance
    # gap covers the braking distance from that speed with the headway to spare
    assert safe_gap(6.7, ROBOT) > 6.7 ** 2 / (2 * 4.4) + 0.5 * 6.7


def test_speed_grows_with_distance_and_respects_max():
    v_prev = 0.0
    for d in (2.0, 4.0, 6.0, 8.0, 10.0):
        v, _, _ = follow_command_relative(d, 0.0, ROBOT)
        assert v > v_prev
        v_prev = v
    v, _, _ = follow_command_relative(50.0, 0.0, ROBOT)
    assert v == ROBOT.max_linear


def _simulate(leader_speed, stop_at, dt=0.05, t_end=30.0, accel=7.8, decel=4.9, delay=0.25):
    """Leader runs straight at leader_speed and stops dead at stop_at; the robot follows the law at 20 Hz with
    a reaction delay and limited acceleration/braking. Returns (steady gap before the stop, smallest gap)."""
    lead, rob, v = 12.0, 0.0, 0.0
    pending = []                                   # (time the command takes effect, command)
    steady = None
    min_gap = 1e9
    t = 0.0
    while t < t_end:
        if t < stop_at:
            lead += leader_speed * dt
        gap = lead - rob
        cmd, _, _ = follow_command_relative(gap, 0.0, ROBOT)
        pending.append((t + delay, cmd))
        while len(pending) > 1 and pending[1][0] <= t:
            pending.pop(0)
        target = pending[0][1] if pending[0][0] <= t else v
        if target > v:
            v = min(target, v + accel * dt)
        else:
            v = max(target, v - decel * dt)
        rob += v * dt
        if abs(t - (stop_at - dt)) < dt / 2:
            steady = gap
        min_gap = min(min_gap, lead - rob)
        t += dt
    return steady, min_gap


def test_follow_loop_is_stable_and_stops_in_time_at_15_mph():
    for mph in (8, 12, 15):
        v = mph * 0.44704
        steady, min_gap = _simulate(v, stop_at=20.0)
        # settles on the designed gap (no oscillation or run-away) ...
        assert math.isclose(steady, safe_gap(v, ROBOT), rel_tol=0.03), (mph, steady)
        # ... and when the skater stops dead the robot stops short of them
        assert min_gap > 1.0, (mph, min_gap)


def test_arc_instead_of_pivot_for_target_to_the_side():
    # 90 degrees to the left, far away: legacy law would pivot (linear 0); the robot law rolls on an arc
    v, w, _ = follow_command_relative(6.0, math.pi / 2, ROBOT)
    assert math.isclose(v, ROBOT.arc_speed)
    assert w > 0 and v / w >= ROBOT.min_turn_radius - 1e-9
    v0, w0, _ = follow_command_relative(6.0, math.pi / 2, FollowGains())
    assert abs(v0) < 1e-9 and w0 > 0


def test_pivot_allowed_when_target_is_behind():
    v, w, _ = follow_command_relative(6.0, 3.0, ROBOT)             # beyond pivot_bearing
    assert v == 0.0 and math.isclose(w, ROBOT.max_angular)


def test_cornering_cap_at_speed():
    v, w, _ = follow_command_relative(12.0, 0.5, ROBOT)
    assert v > 3.0 and w > 0
    assert abs(v * w) <= ROBOT.max_lat_accel + 1e-9


def test_no_pivot_for_small_bearings_when_stopped():
    v, w, _ = follow_command_relative(1.2, 0.4, ROBOT)              # inside follow distance, 23 deg off
    assert v == 0.0 and w == 0.0
    v, w, _ = follow_command_relative(1.2, 1.2, ROBOT)              # well off to the side: turn to face, gently
    assert v == 0.0 and 0 < w <= ROBOT.pivot_rate


def test_command_is_continuous_through_follow_distance():
    # UWB range noise of a few cm around follow_distance must not flip the command between two behaviours
    for bearing in (0.0, 0.3, 0.7, 1.0, 1.8, -1.0):
        prev = None
        d = 1.30
        while d <= 1.70:
            v, w, _ = follow_command_relative(d, bearing, ROBOT)
            if prev is not None:
                assert abs(v - prev[0]) < 0.05 and abs(w - prev[1]) < 0.1, (bearing, d, prev, (v, w))
            prev = (v, w)
            d += 0.01


# ---- offset follow: hold the skater at a bearing, so the robot rides beside their line -------------
def _simulate_2d(g, leader_path, t_end=25.0, dt=0.05, accel=7.8, decel=4.9):
    """Unicycle robot under the law, skater on leader_path(t) -> (x, y). Returns the trace of
    (t, robot x, y, heading, skater x, y, distance, bearing)."""
    rx, ry, rth, v = 0.0, 0.0, 0.0, 0.0
    trace = []
    t = 0.0
    while t < t_end:
        lx, ly = leader_path(t)
        dist, bearing = relative_target(rx, ry, rth, lx, ly)
        lin, ang, _ = follow_command_relative(dist, bearing, g)
        v = min(lin, v + accel * dt) if lin > v else max(lin, v - decel * dt)
        rth += ang * dt
        rx += v * math.cos(rth) * dt
        ry += v * math.sin(rth) * dt
        trace.append((t, rx, ry, rth, lx, ly, dist, bearing))
        t += dt
    return trace


OFFSET = FollowGains(follow_distance=1.5, max_linear=8.0, kp_angular=3.0, max_angular=3.0,
                     brake_decel=4.4, time_headway=0.5, max_lat_accel=4.0, min_turn_radius=0.4,
                     arc_speed=0.6, pivot_bearing=2.0, pivot_rate=0.8, stopped_turn_deadband=0.3,
                     follow_bearing=0.45)


def test_offset_follow_settles_beside_the_skaters_line():
    v = 3.6                                               # 8 mph, skater along +x from (6, 0)
    tr = _simulate_2d(OFFSET, lambda t: (6.0 + v * t, 0.0))
    _, rx, ry, rth, lx, ly, dist, bearing = tr[-1]
    assert math.isclose(bearing, 0.45, abs_tol=0.03)      # skater held at the chosen bearing
    assert abs(rth) < 0.03                                # robot runs parallel to the skater
    assert math.isclose(dist, safe_gap(v, OFFSET), rel_tol=0.05)
    assert math.isclose(-ry, dist * math.sin(0.45), rel_tol=0.1)   # robot is to the RIGHT of the line
    # and it got there without oscillating: bearing stays put over the last 5 s
    tail = [b for (t, *_rest, b) in tr if t > 20.0]
    assert max(tail) - min(tail) < 0.02


def test_offset_follow_passes_beside_a_skater_who_stops_dead():
    v = 5.4                                               # 12 mph, stops dead at t = 15 s
    tr = _simulate_2d(OFFSET, lambda t: (6.0 + v * min(t, 15.0), 0.0), t_end=30.0)
    lx, ly = tr[-1][4], tr[-1][5]
    closest = min(math.hypot(lx - x, ly - y) for (t, x, y, *_r) in tr if t >= 15.0)
    assert closest > 1.0
    # sideways miss distance of the robot's path from the skater at the moment of the stop
    at_stop = next(r for r in tr if r[0] >= 15.0)
    assert abs(at_stop[2] - ly) > 2.0


def test_offset_follow_holds_through_turns_both_ways():
    v, radius = 3.0, 12.0
    for sign in (1, -1):
        path = lambda t, s=sign: (radius * math.sin(v * t / radius), s * radius * (1 - math.cos(v * t / radius)) + 0.0)  # noqa: E731
        tr = _simulate_2d(OFFSET, lambda t, p=path: (p(t)[0] + 6.0, p(t)[1]), t_end=40.0)
        tail = [r for r in tr if r[0] > 25.0]
        assert all(abs(r[7] - 0.45) < 0.25 for r in tail), sign          # stays near the chosen bearing
        assert all(1.5 < r[6] < 12.0 for r in tail), sign                # neither closes in nor falls away


def test_offset_follow_gives_margin_when_braking_is_weak():
    # Braking at under half of what the gap law assumes, skater stops dead from 12 mph. The robot still
    # steers toward the skater in both cases; the offset run ends well clear, the straight-behind run does not.
    import dataclasses
    v = 5.4

    def closest(g):
        tr = _simulate_2d(g, lambda t: (6.0 + v * min(t, 15.0), 0.0), t_end=30.0, decel=2.0)
        lx, ly = tr[-1][4], tr[-1][5]
        return min(math.hypot(lx - x, ly - y) for (t, x, y, *_r) in tr if t >= 15.0)
    behind = closest(dataclasses.replace(OFFSET, follow_bearing=0.0))
    offset = closest(OFFSET)
    assert behind < 0.5 and offset > 1.0


# --- side shot ----------------------------------------------------------------------------------------------------
import pytest  # noqa: E402

from honebot_follow.control import SideShot, SideShotParams, follow_command_relative, speed_for_gap  # noqa: E402

SIDE = SideShotParams(side_distance=2.0, side_angle=math.radians(45))
GS = FollowGains(follow_distance=1.5, max_linear=7.5, max_angular=3.0, brake_decel=4.4, time_headway=0.5)


def _side_run(speed, seconds=20.0, turn=0.0, start=(-3.0, 0.0, 0.0), own=lambda v, w: (v, w)):
    """Skater from (0, 0) heading +x at `speed` (turning at `turn` rad/s); robot a unicycle that does exactly what
    it is told, controller and UWB at 20 Hz. own(v, w) -> what the robot reports as its own speed and yaw rate.
    Returns the history of (time, distance, angle round from behind the skater in degrees, + = their right)."""
    ctl = SideShot(SIDE)
    rx, ry, rth = start
    sx = sy = sth = 0.0
    dt, t, v, w, out = 0.05, 0.0, 0.0, 0.0, []
    while t < seconds:
        sth += turn * dt
        sx += speed * math.cos(sth) * dt
        sy += speed * math.sin(sth) * dt
        dx, dy = sx - rx, sy - ry
        d = math.hypot(dx, dy)
        b = math.atan2(dy, dx) - rth
        b = math.atan2(math.sin(b), math.cos(b))
        cmd = ctl.step(t, (d, b), t, *own(v, w), GS)
        v, w = cmd if cmd is not None else follow_command_relative(d, b, GS)[:2]
        rth += w * dt
        rx += v * math.cos(rth) * dt
        ry += v * math.sin(rth) * dt
        t += dt
        a = math.atan2(ry - sy, rx - sx) - (sth + math.pi)
        out.append((t, math.hypot(rx - sx, ry - sy), math.degrees(math.atan2(math.sin(a), math.cos(a))), ctl.active))
    return out


def test_side_shot_is_off_by_default_and_without_own_motion():
    assert SideShot(SideShotParams()).step(0.0, (3.0, 0.0), 0.0, 0.0, 0.0, GS) is None
    ctl = SideShot(SIDE)
    assert ctl.step(0.0, (3.0, 0.0), 0.0, None, 0.0, GS) is None       # no S line
    assert ctl.step(0.0, (3.0, 0.0), 0.0, 0.0, None, GS) is None       # no gyro


@pytest.mark.parametrize("speed", [1.5, 2.7, 5.4])        # ~3, 6 and 12 mph
def test_side_shot_settles_at_two_metres_and_45_degrees(speed):
    hist = _side_run(speed)
    late = [h for h in hist if h[0] > 12.0]
    assert all(h[3] for h in late)
    # one 20 Hz tick of lag shows at speed: 1.91 m / 48 deg at 6 mph, 1.82 m / 51 deg at 12 mph
    assert all(abs(h[1] - 2.0) < 0.25 for h in late)
    assert all(abs(h[2] - 45.0) < 8.0 for h in late)


def test_side_shot_holds_through_a_turn_toward_the_robot():
    hist = _side_run(2.2, seconds=25.0, turn=-0.3)          # skater curves right, toward the robot's side
    late = [h for h in hist if h[0] > 12.0]
    assert min(h[1] for h in late) > 1.5
    assert abs(sorted(h[2] for h in late)[len(late) // 2] - 45.0) < 10.0


def test_side_shot_tolerates_a_wrong_speed_scale():
    """max_wheel_speed is a no-load guess: the robot's own-speed report can be 15 % off. The correction is
    proportional, so the held point shifts: 2.2 m / 40 deg (report 15 % low) and 1.66 m / 58 deg (15 % high)
    at 6 mph. Still a side view; calibrating max_wheel_speed (docs/first_drive.md) removes it."""
    for k in (0.85, 1.15):
        late = [h for h in _side_run(2.7, own=lambda v, w: (v * k, w)) if h[0] > 12.0]
        assert all(abs(h[1] - 2.0) < 0.6 and abs(h[2] - 45.0) < 15.0 for h in late), k


def _settled(speed=2.7):
    """A controller that has seen the skater moving straight ahead-left at `speed` for 2 s, robot pacing them."""
    ctl = SideShot(SIDE)
    for i in range(41):
        t = 0.05 * i
        ctl.step(t, (2.0, math.radians(45)), t, speed, 0.0, GS)
    return ctl, t


def test_aim_point_behind_means_no_forward_motion():
    ctl, t = _settled()
    # robot drew level with the skater (they braked): aim point is behind us
    lin, ang = ctl.step(t + 0.05, (1.6, math.radians(90)), t + 0.05, 2.7, 0.0, GS)
    assert lin == 0.0


def test_too_close_never_forward_and_never_steers_toward_the_skater():
    ctl, t = _settled()
    lin, ang = ctl.step(t + 0.05, (1.2, math.radians(30)), t + 0.05, 2.7, 0.0, GS)   # skater ahead-left, 1.2 m
    assert lin == 0.0 and ang <= 0.0


def test_skater_in_our_path_caps_speed_with_the_braking_law():
    ctl, t = _settled()
    lin, _ = ctl.step(t + 0.05, (1.9, 0.05), t + 0.05, 2.7, 0.0, GS)   # swerved in front of us, in the lane
    assert lin <= speed_for_gap(1.9 * math.cos(0.05), GS) + 1e-9 < 1.0  # ~0.69 m/s, not the 2.7 it was doing


def test_side_mode_hysteresis_and_range():
    ctl, t = _settled(speed=1.0)
    assert ctl.active                                       # above side_enter_speed (0.8)
    ctl, t = _settled(speed=0.6)
    assert not ctl.active                                   # never went above 0.8
    ctl, t = _settled(speed=2.0)
    assert ctl.step(t + 0.05, (14.0, 0.3), t + 0.05, 2.0, 0.0, GS) is None   # beyond side_engage_range + 1.5


def test_follow_command_relative_stops_on_non_finite_or_negative():
    for d, b in [(math.nan, 0.0), (0.0, math.nan), (math.nan, math.nan),
                 (math.inf, 0.0), (-math.inf, 0.0), (5.0, math.inf), (-1.0, 0.0)]:
        assert follow_command_relative(d, b, G) == (0.0, 0.0, 0.0)
        assert follow_command_relative(d, b, ROBOT) == (0.0, 0.0, 0.0)


def test_side_shot_resets_on_non_finite_inputs():
    ctl, t = _settled(speed=2.0)
    assert ctl.active
    assert ctl.step(t + 0.05, (math.nan, 0.3), t + 0.05, 2.0, 0.0, GS) is None
    assert not ctl.active
    ctl, t = _settled(speed=2.0)
    assert ctl.step(t + 0.05, (2.0, math.inf), t + 0.05, 2.0, 0.0, GS) is None
    assert not ctl.active
    ctl, t = _settled(speed=2.0)
    assert ctl.step(t + 0.05, (2.0, 0.3), t + 0.05, math.nan, 0.0, GS) is None
    assert not ctl.active


def test_side_shot_ignores_bursty_timestamps_and_clamps_spikes():
    ctl = SideShot(SIDE)
    # First fix at t = 0.0, second fix 2 ms later with 10 cm UWB noise -> 50 m/s if differentiated!
    assert ctl.step(0.0, (3.0, 0.0), 0.0, 0.0, 0.0, GS) is None
    assert ctl.step(0.002, (3.1, 0.0), 0.002, 0.0, 0.0, GS) is None
    assert not ctl.has_vel and not ctl.active
    # Normal spacing (50 ms) with an impossible 5 m jump -> velocity magnitude clamped to 15 m/s
    ctl.step(0.05, (8.0, 0.0), 0.05, 0.0, 0.0, GS)
    assert ctl.has_vel and math.hypot(*ctl.vel) <= 15.0 + 1e-6
