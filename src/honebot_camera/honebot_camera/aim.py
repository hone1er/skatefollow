"""Pure math for aiming the camera pan servo at the skater (unit-tested, no ROS).

Angles follow ROS: radians, positive = left (counter-clockwise seen from above).
The pan servo is open loop: we only know the angle we commanded. The controller is
incremental: pan += kp * (bearing of the target relative to the camera axis). When the
target is centred the error is zero and the pan stops moving, whatever kp is, so the
exact lens field of view only changes how fast it converges.
"""

import math
from dataclasses import dataclass


def pixel_to_bearing(x_norm, hfov):
    """Horizontal angle of a pixel from the optical axis (pinhole model).

    x_norm: -1 at the left edge, 0 centre, +1 right edge. Returns radians, positive = left.
    """
    return math.atan(-x_norm * math.tan(hfov / 2.0))


def bearing_to_x_norm(bearing, hfov):
    """Inverse of pixel_to_bearing; result may be outside [-1, 1] when the angle is off-frame."""
    return -math.tan(bearing) / math.tan(hfov / 2.0)


def pixel_to_elevation(y_norm, vfov):
    """Vertical angle of a pixel: y_norm -1 at the top edge, +1 bottom. Returns radians, positive = up."""
    return math.atan(-y_norm * math.tan(vfov / 2.0))


def vfov_from_hfov(hfov, width, height):
    """Vertical field of view for a pinhole camera of the given aspect ratio."""
    return 2.0 * math.atan(math.tan(hfov / 2.0) * height / width)


@dataclass
class AimGains:
    # Same values as camera_aim.py's declared parameter defaults (sim/tests/test_sources.py checks).
    kp: float = 0.6               # fraction of the measured error applied per detection
    deadband: float = math.radians(1.0)       # rad: ignore errors smaller than this
    max_rate: float = math.radians(150.0)     # rad/s: pan slew limit while tracking (protects the camera)
    half_range: float = math.radians(70.0)    # angle at pan +-1 (firmware PAN_SPAN_US): measure it
    lost_timeout: float = 0.5     # s without a detection before we fall back to the UWB bearing
    recenter_timeout: float = 3.0  # s without a detection (and no UWB) before drifting back to centre
    recenter_rate: float = math.radians(30.0)  # rad/s when falling back / recentring


def _toward(current, target, max_step):
    delta = target - current
    if delta > max_step:
        delta = max_step
    elif delta < -max_step:
        delta = -max_step
    return current + delta


class PanController:
    """Holds the commanded pan angle and updates it from camera bearings (and a fallback bearing)."""

    def __init__(self, gains=None):
        self.g = gains or AimGains()
        self.pan = 0.0
        self.last_meas_time = None

    def update(self, now, dt, bearing_err=None, fallback_bearing=None):
        """Advance the pan by one step.

        now: seconds; dt: seconds since the previous update.
        bearing_err: target angle relative to the camera axis from the latest detection (None = none this
        step). fallback_bearing: where the target is in the robot frame from another sensor (UWB), or None.
        Returns the new pan angle (rad, robot frame).
        """
        g = self.g
        if not (math.isfinite(now) and math.isfinite(dt)):
            return self.pan
        if bearing_err is not None and not math.isfinite(bearing_err):
            bearing_err = None
        if fallback_bearing is not None and not math.isfinite(fallback_bearing):
            fallback_bearing = None
        dt = max(0.0, dt)
        if bearing_err is not None:
            self.last_meas_time = now
            step = g.kp * bearing_err if abs(bearing_err) > g.deadband else 0.0
            self.pan = _toward(self.pan, self.pan + step, g.max_rate * dt)
        else:
            age = math.inf if self.last_meas_time is None else now - self.last_meas_time
            if fallback_bearing is not None and age > g.lost_timeout:
                self.pan = _toward(self.pan, fallback_bearing, g.recenter_rate * dt)
            elif age > g.recenter_timeout:
                self.pan = _toward(self.pan, 0.0, g.recenter_rate * dt)
        self.pan = max(-g.half_range, min(g.half_range, self.pan))
        return self.pan

    def tracking(self, now):
        return self.last_meas_time is not None and now - self.last_meas_time <= self.g.lost_timeout

    def normalized(self, sign=1.0):
        """-1..1 servo command (1 = full left unless sign = -1)."""
        if self.g.half_range <= 0.0 or not math.isfinite(self.pan):
            return 0.0
        n = sign * self.pan / self.g.half_range
        return max(-1.0, min(1.0, n))
