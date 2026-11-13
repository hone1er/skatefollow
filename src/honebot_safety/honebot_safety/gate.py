"""Pure safety-gate logic, free of ROS so it can be unit-tested.

The gate sits between whatever produces velocity commands (the follower, a
gamepad, ...) and the motors. It can only make the robot *slower*: it never
adds motion of its own.

Rules, in priority order (first match wins):
  1. ESP32 link down / unknown           -> STOP (zero everything)
  2. Range reading stale or never seen   -> STOP
  3. Range reading invalid (NaN, <= 0)   -> STOP
  4. Controller command stale/never seen -> STOP
  5. Range < stop_distance               -> no forward motion (reverse and steering allowed)
  6. Range < slow_distance               -> forward speed scaled down linearly toward stop_distance
  7. otherwise                           -> pass through

A range of +inf means "nothing in range" (REP 117) and counts as clear.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

# State names, also published on /safety/state so they can be watched in Foxglove.
LINK_DOWN = 'LINK_DOWN'
RANGE_STALE = 'RANGE_STALE'
RANGE_INVALID = 'RANGE_INVALID'
CMD_STALE = 'CMD_STALE'
OBSTACLE = 'OBSTACLE'
SLOW = 'SLOW'
OK = 'OK'

STOP_STATES = (LINK_DOWN, RANGE_STALE, RANGE_INVALID, CMD_STALE)


@dataclass
class GateParams:
    stop_distance: float = 0.4    # m: forward motion forbidden closer than this
    slow_distance: float = 1.0    # m: forward speed tapers from here down to stop_distance
    range_timeout: float = 0.3    # s: range (and link_ok) older than this = stale
    cmd_timeout: float = 0.5      # s: controller command older than this = stale


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def slow_factor(range_m: float, p: GateParams) -> float:
    """0..1 multiplier for forward speed based on the obstacle distance."""
    if range_m <= p.stop_distance:
        return 0.0
    if p.slow_distance <= p.stop_distance or range_m >= p.slow_distance:
        return 1.0
    return (range_m - p.stop_distance) / (p.slow_distance - p.stop_distance)


def gate_command(linear: float, angular: float,
                 range_m: Optional[float], range_age: Optional[float],
                 link_ok: bool, link_age: Optional[float],
                 cmd_age: Optional[float],
                 p: GateParams) -> Tuple[float, float, str]:
    """Return (linear, angular, state).

    Ages are seconds since the last message; None means "never received".
    """
    if link_age is None or link_age > p.range_timeout or not link_ok:
        return 0.0, 0.0, LINK_DOWN
    if range_m is None or range_age is None or range_age > p.range_timeout:
        return 0.0, 0.0, RANGE_STALE
    if math.isnan(range_m) or range_m <= 0.0:
        return 0.0, 0.0, RANGE_INVALID
    if cmd_age is None or cmd_age > p.cmd_timeout:
        return 0.0, 0.0, CMD_STALE

    if linear <= 0.0:                      # reversing or stopped: the forward sensor is irrelevant
        return linear, angular, OK

    factor = slow_factor(range_m, p)
    if factor <= 0.0:
        return 0.0, angular, OBSTACLE
    if factor < 1.0:
        return linear * factor, angular, SLOW
    return linear, angular, OK
