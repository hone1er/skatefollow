"""Pure safety-gate logic, free of ROS so it can be unit-tested.

The gate sits between whatever produces velocity commands (the follower, a
gamepad, ...) and the motors. It can only make the robot *slower*: it never
adds motion of its own.

Rules, in priority order (first match wins):
  1. ESP32 link down / unknown           -> STOP (zero everything)
  2. Range reading stale or never seen   -> STOP
  3. Range reading invalid (NaN, <= 0)   -> STOP
  4. Controller command stale/never seen -> STOP
  5. No echo for blind_timeout           -> no forward motion (BLIND; only when blind_timeout > 0)
  6. Range < stop_distance               -> no forward motion (reverse and steering allowed)
  7. Range < slow_distance               -> forward speed scaled down linearly toward stop_distance
  8. forward speed above the sight limit -> capped to it (only when brake_decel > 0, see sight_speed)
  9. otherwise                           -> pass through

A range of +inf means "nothing in range" (REP 117) and counts as clear.

Blind check (rule 5): an unplugged or broken HC-SR04 never echoes, which looks exactly like +inf = clear road.
Mounted tilted up 12 deg at 10 cm, a working sensor sees the road at ~1.95 m on every ping, so "no echo at all for
blind_timeout seconds" means the sensor is dead, the ground fell away (a crest) or it points at the sky: in every
case it cannot see, so forward motion stops. A single missed ping does not trip it. Off (0) by default for
turtlesim and the bench; robot.yaml switches it on. Leave it off for a sensor that is meant never to see the
ground (a lidar tilted up): such a sensor needs its own health check.

Sight limit (rule 7): never drive faster than the robot can stop within what the sensor can see.
With the ultrasonic sensor that is about 2.7 m/s when it reads the ground ahead (~1.95 m) and about
3.8 m/s when it reads nothing (taken as sight_distance = 3 m of clear road). Faster needs a longer-range
sensor: the limit is the sensor's, not a tuning choice.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

# State names, also published on /safety/state so they can be watched in Foxglove.
LINK_DOWN = 'LINK_DOWN'
RANGE_STALE = 'RANGE_STALE'
RANGE_INVALID = 'RANGE_INVALID'
CMD_STALE = 'CMD_STALE'
BLIND = 'BLIND'
OBSTACLE = 'OBSTACLE'
SLOW = 'SLOW'
SIGHT_LIMIT = 'SIGHT_LIMIT'
OK = 'OK'

STOP_STATES = (LINK_DOWN, RANGE_STALE, RANGE_INVALID, CMD_STALE)


@dataclass
class GateParams:
    stop_distance: float = 0.4    # m: forward motion forbidden closer than this
    slow_distance: float = 1.0    # m: forward speed tapers from here down to stop_distance
    range_timeout: float = 0.3    # s: range (and link_ok) older than this = stale
    cmd_timeout: float = 0.5      # s: controller command older than this = stale
    # Sight limit, off while brake_decel is 0 (turtlesim and the bench tests keep the old behaviour).
    brake_decel: float = 0.0      # m/s^2 the robot can stop with; > 0 switches the sight limit on
    reaction_time: float = 0.25   # s from an obstacle appearing to the brakes biting (sensor 60 ms + gate + ramp)
    sight_distance: float = 3.0   # m of road taken as clear when the sensor reports nothing (+inf)
    cap_rise_rate: float = 1.0    # m/s^2: how fast the limit may rise again (it drops at once)
    # Blind check, off while 0: no echo (only +inf readings) for this long -> no forward motion.
    blind_timeout: float = 0.0    # s


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def slow_factor(range_m: float, p: GateParams) -> float:
    """0..1 multiplier for forward speed based on the obstacle distance."""
    if range_m <= p.stop_distance:
        return 0.0
    if p.slow_distance <= p.stop_distance or range_m >= p.slow_distance:
        return 1.0
    return (range_m - p.stop_distance) / (p.slow_distance - p.stop_distance)


def sight_speed(range_m: float, p: GateParams) -> float:
    """Fastest forward speed from which the robot can still stop short of stop_distance, given how far
    the sensor can vouch for: reaction_time * v + v^2 / (2 * brake_decel) = clear - stop_distance."""
    if p.brake_decel <= 0.0:
        return math.inf
    clear = min(range_m, p.sight_distance)
    room = clear - p.stop_distance
    if room <= 0.0:
        return 0.0
    a, t = p.brake_decel, p.reaction_time
    return a * (-t + math.sqrt(t * t + 2.0 * room / a))


def rising_cap(previous: Optional[float], new: float, dt: float, p: GateParams) -> float:
    """The sight limit drops immediately but rises only at cap_rise_rate, so a sensor that flickers
    between "ground at 2 m" and "nothing" does not make the robot surge and brake."""
    if previous is None or new <= previous or not math.isfinite(new):
        return new
    return min(new, previous + p.cap_rise_rate * max(0.0, dt))


def gate_command(linear: float, angular: float,
                 range_m: Optional[float], range_age: Optional[float],
                 link_ok: bool, link_age: Optional[float],
                 cmd_age: Optional[float],
                 p: GateParams, speed_cap: Optional[float] = None,
                 echo_age: Optional[float] = None) -> Tuple[float, float, str]:
    """Return (linear, angular, state).

    Ages are seconds since the last message; None means "never received".
    speed_cap: the (rate-limited) sight limit from rising_cap(); None = compute it from this reading.
    echo_age: seconds since the last finite range (a real echo); None = never. Only used when blind_timeout > 0.
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
    if p.blind_timeout > 0.0 and (echo_age is None or echo_age > p.blind_timeout):
        return 0.0, angular, BLIND

    factor = slow_factor(range_m, p)
    if factor <= 0.0:
        return 0.0, angular, OBSTACLE
    state = OK
    if factor < 1.0:
        linear, state = linear * factor, SLOW
    if p.brake_decel > 0.0:
        cap = sight_speed(range_m, p)
        if speed_cap is not None:
            cap = min(cap, speed_cap)
        if cap <= 0.0:
            return 0.0, angular, OBSTACLE
        if linear > cap:
            linear = cap
            if state == OK:
                state = SIGHT_LIMIT
    return linear, angular, state
