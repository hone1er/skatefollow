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
    """
    heading_error = wrap_angle(bearing)
    angular = clamp(g.kp_angular * heading_error, -g.max_angular, g.max_angular)

    gap = distance - g.follow_distance
    linear = g.kp_linear * gap * max(0.0, math.cos(heading_error))
    linear = clamp(linear, 0.0, g.max_linear)

    if gap <= 0.0:
        # Close enough: stop, and stop turning when right on top of the target
        # where the bearing is meaningless.
        linear = 0.0
        if distance < 0.2:
            angular = 0.0
    elif g.ackermann and linear < g.creep_speed:
        # Target is far but we're pointed away from it; a car has to roll to turn.
        linear = min(g.creep_speed, g.max_linear)

    return linear, angular, distance


def follow_command(fx, fy, ftheta, lx, ly, g: FollowGains):
    """Turtlesim form: follower pose (fx, fy, ftheta) chasing leader (lx, ly)."""
    distance, bearing = relative_target(fx, fy, ftheta, lx, ly)
    return follow_command_relative(distance, bearing, g)
