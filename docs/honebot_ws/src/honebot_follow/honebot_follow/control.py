"""Pure follow-me math, kept free of ROS so it can be unit-tested and reused
later when the leader position comes from UWB instead of turtlesim."""

import math
from dataclasses import dataclass


@dataclass
class FollowGains:
    follow_distance: float = 1.5   # how far behind the leader to stay (m)
    kp_linear: float = 1.0         # forward speed per metre of extra distance
    kp_angular: float = 4.0        # turn rate per radian of heading error
    max_linear: float = 2.0        # m/s
    max_angular: float = 4.0       # rad/s


def wrap_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def follow_command(fx, fy, ftheta, lx, ly, g: FollowGains):
    """Return (linear, angular) velocity for a follower at (fx, fy, ftheta)
    chasing a leader at (lx, ly).

    - Turns toward the leader proportionally to the heading error.
    - Drives forward proportionally to how far it is beyond follow_distance.
    - Scales forward speed by cos(heading error), so it slows down while
      turning and never drives away from a leader that is behind it.
    - Never reverses (a car-like robot chasing a skater shouldn't).
    """
    dx, dy = lx - fx, ly - fy
    distance = math.hypot(dx, dy)

    heading_error = wrap_angle(math.atan2(dy, dx) - ftheta)
    angular = clamp(g.kp_angular * heading_error, -g.max_angular, g.max_angular)

    gap = distance - g.follow_distance
    linear = g.kp_linear * gap * max(0.0, math.cos(heading_error))
    linear = clamp(linear, 0.0, g.max_linear)

    # Close enough: stop and stop turning (avoids spinning in place on top of
    # the leader where the heading is meaningless).
    if gap <= 0.0:
        linear = 0.0
        if distance < 0.2:
            angular = 0.0

    return linear, angular, distance
