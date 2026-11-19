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
    # 4-wheel skid steer: geometric track is 0.277 m, but the tyres scrub when turning, so the robot turns
    # slower than the geometry says. 0.39 (= 0.277 x 1.4) is the starting guess; set it from the spin test.
    track_width: float = 0.39
    # m/s at command = 1.0. 9.6 = no-load top speed of the drivetrain (same number as FULL_SCALE_MPS in the
    # firmware). Calibrate: drive at a known command, measure the real speed, scale.
    max_wheel_speed: float = 9.6
    # Leave these False on the real robot and set motor direction in the firmware (THROTTLE_REVERSED /
    # STEER_REVERSED) or the ESC, so the handheld remote's stick drives the same way as the Pi.
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


def wheels_to_speed(left: float, right: float, p: DiffDriveParams) -> float:
    """Forward speed (m/s) from the wheel outputs the ESP32 reports in its S line: the inverse of
    twist_to_diffdrive's linear part, on the same scale, so it reads the speed the Pi asked for once the
    ESP32's ramps and limits have been applied. NaN/inf -> 0."""
    if not (math.isfinite(left) and math.isfinite(right)):
        return 0.0
    if p.left_reversed:
        left = -left
    if p.right_reversed:
        right = -right
    return (left + right) / 2.0 * p.max_wheel_speed
