"""Pure Twist -> Ackermann (throttle, steer) conversion, free of ROS for unit testing.

A car-like robot can't follow an arbitrary (v, omega) pair; it steers its front
wheels by an angle delta and the resulting turn rate is

    omega = v * tan(delta) / L        (L = wheelbase)

so the steering angle that realises a requested omega at speed v is

    delta = atan(L * omega / v)

At very low speed this blows up (any omega wants infinite steering), so below
min_speed_for_steer the speed is treated as that minimum. Normalised outputs
are in -1..1 and are what the ESP32 turns into 1000-2000 us servo pulses.
"""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class AckermannParams:
    wheelbase: float = 0.33          # m; set to the real chassis
    max_steer_angle: float = 0.45    # rad (~26 deg) at steer = +/-1
    max_speed: float = 5.0           # m/s at throttle = 1; set to the real chassis
    min_speed_for_steer: float = 0.3  # m/s; avoids atan blow-up near zero speed
    steer_reversed: bool = False     # flip if the car turns the wrong way
    throttle_reversed: bool = False  # flip if the car drives backward on +throttle


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def twist_to_ackermann(linear: float, angular: float, p: AckermannParams) -> Tuple[float, float]:
    """Return (throttle, steer), each in -1..1, for a Twist (linear.x, angular.z)."""
    if not (math.isfinite(linear) and math.isfinite(angular)):
        return 0.0, 0.0

    throttle = clamp(linear / p.max_speed, -1.0, 1.0) if p.max_speed > 0 else 0.0

    # Keep the sign of v: driving backward, the wheels must steer the other way
    # to produce the same turn direction.
    sign = -1.0 if linear < 0.0 else 1.0
    v_eff = sign * max(abs(linear), p.min_speed_for_steer)
    delta = math.atan(p.wheelbase * angular / v_eff)
    steer = clamp(delta / p.max_steer_angle, -1.0, 1.0) if p.max_steer_angle > 0 else 0.0

    if p.steer_reversed:
        steer = -steer
    if p.throttle_reversed:
        throttle = -throttle
    return throttle, steer


def pulse_us(norm: float, center_us: int = 1500, span_us: int = 500) -> int:
    """Normalised -1..1 -> servo pulse width in microseconds (1000..2000 by default)."""
    return int(round(center_us + clamp(norm, -1.0, 1.0) * span_us))
