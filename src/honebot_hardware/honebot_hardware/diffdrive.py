"""Pure Twist -> differential-drive (left, right) conversion, free of ROS.

For two driven wheels `track_width` apart:
    v_left  = v - omega * track_width / 2
    v_right = v + omega * track_width / 2
Normalised by max_wheel_speed to -1..1. If either wheel would saturate, both
are scaled together so the turn ratio (and therefore the path) is preserved
and the robot just goes slower, rather than clipping one side and veering.
"""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class DiffDriveParams:
    track_width: float = 0.40        # m between the two driven wheels
    max_wheel_speed: float = 7.0     # m/s at command = 1.0 (25 km/h ~ 7 m/s); set from the real drivetrain
    left_reversed: bool = False
    right_reversed: bool = False


def twist_to_diffdrive(linear: float, angular: float, p: DiffDriveParams) -> Tuple[float, float]:
    """Return (left, right) wheel commands in -1..1 for Twist (linear.x, angular.z)."""
    if not (math.isfinite(linear) and math.isfinite(angular)) or p.max_wheel_speed <= 0:
        return 0.0, 0.0
    half = p.track_width / 2.0
    left = (linear - angular * half) / p.max_wheel_speed
    right = (linear + angular * half) / p.max_wheel_speed
    biggest = max(abs(left), abs(right))
    if biggest > 1.0:
        left /= biggest
        right /= biggest
    if p.left_reversed:
        left = -left
    if p.right_reversed:
        right = -right
    return left, right
