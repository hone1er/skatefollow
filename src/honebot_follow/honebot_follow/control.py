"""Pure follow-me math, kept free of ROS so it can be unit-tested and reused.

Two entry points:
  follow_command_relative(distance, bearing, gains)  the real one: target given
                                                     relative to the robot, as
                                                     UWB will provide it
  follow_command(fx, fy, ftheta, lx, ly, gains)      turtlesim convenience:
                                                     two world poses -> relative
"""

import math
from dataclasses import dataclass


@dataclass
class FollowGains:
    follow_distance: float = 1.5   # how far behind the leader to stay (m)
    kp_linear: float = 1.0         # forward speed per metre of extra distance
    kp_angular: float = 4.0        # turn rate per radian of heading error
    max_linear: float = 2.0        # m/s
    max_angular: float = 4.0       # rad/s
    ackermann: bool = False        # car-like: cannot turn in place
    creep_speed: float = 0.3       # m/s forward while re-acquiring a target behind (ackermann only)
    # --- real-robot options; every one is OFF at 0 so the turtlesim behaviour is unchanged ---
    brake_decel: float = 0.0       # m/s^2 the robot can stop with. > 0 switches on the speed-dependent gap:
    #                                gap(v) = follow_distance + time_headway * v + v^2 / (2 * brake_decel)
    time_headway: float = 0.5      # s of travel kept in hand on top of the braking distance
    max_lat_accel: float = 0.0     # m/s^2 cap on v * omega (cornering); 0 = no cap
    min_turn_radius: float = 0.0   # m. > 0: turn on arcs of at least this radius instead of pivoting ...
    arc_speed: float = 0.6         # m/s kept while arcing toward a target off to the side
    pivot_bearing: float = 2.0     # rad: ... unless the target is further round than this (then pivot fast)
    pivot_rate: float = 0.8        # rad/s: gentle pivot always allowed in arc mode (little scrub at this rate)
    stopped_turn_deadband: float = 0.0   # rad: bearings below this are not turned toward by pivoting
    follow_bearing: float = 0.0    # rad: where to keep the skater relative to the robot's nose (+ = to the left).
    #                                0 = straight ahead (robot directly behind). 0.45 (26 deg) = the robot rides
    #                                behind and to the RIGHT of the skater's line: a 3/4 rear shot. It still steers
    #                                toward a skater who stops, but it buys margin when braking is worse than
    #                                assumed (see test_offset_follow_gives_margin_when_braking_is_weak).


def safe_gap(speed: float, g: 'FollowGains') -> float:
    """Distance to keep at `speed` so the robot can stop behind a skater who stops dead."""
    if g.brake_decel <= 0.0:
        return g.follow_distance
    v = max(0.0, speed)
    return g.follow_distance + g.time_headway * v + v * v / (2.0 * g.brake_decel)


def speed_for_gap(distance: float, g: 'FollowGains') -> float:
    """Inverse of safe_gap: the fastest speed that is still safe at this distance.

    Solving gap(v) = distance for v in closed form means the command never depends on the robot's own
    (delayed) speed estimate, so there is no feedback loop to go unstable at high speed."""
    extra = distance - g.follow_distance
    if extra <= 0.0:
        return 0.0
    a, t = g.brake_decel, g.time_headway
    return a * (-t + math.sqrt(t * t + 2.0 * extra / a))


def wrap_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def relative_target(fx, fy, ftheta, lx, ly):
    """World poses -> (distance, bearing) of the leader as seen from the follower.
    bearing is in the follower's frame: 0 = straight ahead, +ve = to the left."""
    dx, dy = lx - fx, ly - fy
    return math.hypot(dx, dy), wrap_angle(math.atan2(dy, dx) - ftheta)


def follow_command_relative(distance: float, bearing: float, g: FollowGains):
    """Return (linear, angular, distance) for a target at (distance, bearing)
    relative to the robot.

    - Turns toward the target proportionally to the bearing.
    - Drives forward proportionally to how far it is beyond follow_distance.
    - Scales forward speed by cos(bearing), so it slows down while turning and
      never drives away from a target that is behind it.
    - Never reverses (a car chasing a skater shouldn't).
    - ackermann=True: a car can't spin in place, so when the target is beyond
      follow_distance but off to the side/behind it creeps forward at
      creep_speed with full steering instead of stopping dead.
    - brake_decel > 0: the gap grows with speed (safe_gap / speed_for_gap).
    - min_turn_radius > 0: arcs instead of pivots; max_lat_accel > 0: cornering cap.
    """
    heading_error = wrap_angle(bearing - g.follow_bearing)     # steer to hold the skater at follow_bearing
    angular = clamp(g.kp_angular * heading_error, -g.max_angular, g.max_angular)

    gap = distance - g.follow_distance
    if g.brake_decel > 0.0:
        # Speed-dependent gap: drive at the fastest speed from which the robot can still stop in time.
        base = speed_for_gap(distance, g)
    else:
        base = g.kp_linear * gap
    linear = clamp(base * max(0.0, math.cos(heading_error)), 0.0, g.max_linear)

    arcs = g.min_turn_radius > 0.0 and abs(heading_error) <= g.pivot_bearing
    # Gentle pivot allowed in arc mode: grows from zero at the deadband, capped at pivot_rate. Used both when
    # stopped and as the yaw floor while rolling, so the command is continuous through follow_distance
    # (a noisy range must not flip between "pivot hard" and "barely turn").
    slow_pivot = min(g.pivot_rate, g.kp_angular * max(0.0, abs(heading_error) - g.stopped_turn_deadband))

    if gap <= 0.0:
        # Close enough: stop, and stop turning when right on top of the target
        # where the bearing is meaningless.
        linear = 0.0
        if distance < 0.2:
            angular = 0.0
        elif arcs:
            angular = clamp(angular, -slow_pivot, slow_pivot)
        elif abs(heading_error) < g.stopped_turn_deadband:
            angular = 0.0
    elif g.ackermann and linear < g.creep_speed:
        # Target is far but we're pointed away from it; a car has to roll to turn.
        linear = min(g.creep_speed, g.max_linear)
    elif arcs:
        # Skid steer scrubs its tyres when it pivots: keep rolling and turn on an arc instead.
        # (Target further round than pivot_bearing: a pivot is quicker than a long loop.)
        linear = max(linear, min(g.arc_speed, g.max_linear, base))
        w_max = max(linear / g.min_turn_radius, slow_pivot)
        angular = clamp(angular, -w_max, w_max)

    if g.max_lat_accel > 0.0 and linear > 1e-3:
        w_max = g.max_lat_accel / linear        # v * omega <= max_lat_accel
        angular = clamp(angular, -w_max, w_max)

    return linear, angular, distance


def follow_command(fx, fy, ftheta, lx, ly, g: FollowGains):
    """Turtlesim form: follower pose (fx, fy, ftheta) chasing leader (lx, ly)."""
    distance, bearing = relative_target(fx, fy, ftheta, lx, ly)
    return follow_command_relative(distance, bearing, g)
