"""Pure logic for the handheld remote bridge, free of ROS (unit-tested in test/test_remote_logic.py).

Two ways in, one way out:

  * The remote board's USB serial printout (bench use: remote plugged into the Pi or Mac), one line every
    200 ms, printed by firmware/esp32_remote/esp32_remote.ino:
        state=RUN x=+0.50 y=-0.20 btn=0 robot=1 pi=1 failsafe=0 manual=1
    x = forward (+1 = stick pushed away from you), y = turn (+1 = stick pushed left),
    state = STOP | RUN | FAULT (stick fault at power-up; treated as STOP).
    `remote_to_wheels` turns that into the wheel commands the robot ESP32 would output, using the same
    rules and numbers as firmware/esp32_ultrasonic (shapeStick + arcadeMix, MANUAL_LIMIT, MANUAL_TURN_SCALE).

  * The robot ESP32's `S,<failsafe>,<left>,<right>` status line (via esp32_bridge -> esp32/outputs): the wheel
    outputs the robot is really producing, whoever drives it (remote stick or Pi), after ramps and limits.

Both give (left, right) wheel commands in -1..1; `wheels_to_twist` turns them into the robot's speed and turn
rate, which drives a turtle in turtlesim exactly as the real robot would move.
"""

import math
import re
from dataclasses import dataclass
from typing import Optional, Tuple

# Same numbers as firmware/esp32_ultrasonic/esp32_ultrasonic.ino. Keep in sync.
STICK_DEADBAND = 0.15       # STICK_DEADBAND
MANUAL_LIMIT = 0.25         # MANUAL_LIMIT: full stick forward = this wheel command (~2.4 m/s)
MANUAL_TURN_SCALE = 0.6     # MANUAL_TURN_SCALE
FULL_SCALE_MPS = 9.6        # FULL_SCALE_MPS: m/s at wheel command 1.0 (= esp32_bridge max_wheel_speed)
TRACK_WIDTH = 0.39          # esp32_bridge track_width (effective, skid-steer scrub included)

_FIELD = re.compile(r'(\w+)=([^\s]+)')


@dataclass
class RemoteState:
    state: str          # 'STOP', 'RUN' or 'FAULT'
    x: float            # forward, -1..1
    y: float            # turn, -1..1, + = left
    btn: bool
    robot: bool         # the remote hears the robot ESP32
    manual: bool        # the robot reports that the stick is driving it

    @property
    def run(self) -> bool:
        return self.state == 'RUN'


def parse_remote_line(line: str) -> Optional[RemoteState]:
    """Parse one status line from the remote board. Returns None for anything else (boot banner, junk)."""
    fields = dict(_FIELD.findall(line.strip()))
    if 'state' not in fields or 'x' not in fields or 'y' not in fields:
        return None
    state = fields['state'].upper()
    if state not in ('STOP', 'RUN', 'FAULT'):
        return None
    try:
        x = float(fields['x'])
        y = float(fields['y'])
    except ValueError:
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return RemoteState(
        state=state,
        x=max(-1.0, min(1.0, x)),
        y=max(-1.0, min(1.0, y)),
        btn=fields.get('btn') == '1',
        robot=fields.get('robot') == '1',
        manual=fields.get('manual') == '1',
    )


def shape_stick(v: float, deadband: float = STICK_DEADBAND) -> float:
    """Firmware drive::shapeStick: zero inside the deadband, then rescaled so the edge of it is 0 and full is 1."""
    a = abs(v)
    if a < deadband:
        return 0.0
    out = min(1.0, (a - deadband) / (1.0 - deadband))
    return -out if v < 0.0 else out


def arcade_mix(forward: float, turn: float, limit: float = MANUAL_LIMIT,
               turn_scale: float = MANUAL_TURN_SCALE) -> Tuple[float, float]:
    """Firmware drive::arcadeMix: stick (forward, turn) -> (left, right) wheel commands."""
    forward = max(-1.0, min(1.0, forward))
    turn = max(-1.0, min(1.0, turn)) * turn_scale
    left = forward - turn
    right = forward + turn
    big = max(abs(left), abs(right))
    if big > 1.0:
        left /= big
        right /= big
    return left * limit, right * limit


def remote_to_wheels(s: Optional[RemoteState]) -> Tuple[float, float]:
    """Wheel commands the robot ESP32 would output for this remote state (without its ramps).

    Only RUN drives; STOP, FAULT and "no remote" (None) give neutral, like the robot firmware.
    """
    if s is None or not s.run:
        return 0.0, 0.0
    return arcade_mix(shape_stick(s.x), shape_stick(s.y))


SRC_NONE, SRC_PI, SRC_MANUAL = 'none', 'pi', 'manual'


@dataclass
class ArbiterConfig:
    """Same rules and numbers as drive::decide in firmware/esp32_ultrasonic/drive_logic.h."""
    remote_timeout: float = 0.5     # REMOTE_TIMEOUT_MS
    manual_hold: float = 0.8        # MANUAL_HOLD_MS: after the stick is released, wait before the Pi drives again
    cmd_timeout: float = 0.5        # Pi command freshness (firmware: 200 ms on a 50 Hz serial line; ROS jitter here)
    stick_deadband: float = STICK_DEADBAND
    remote_required: bool = True    # REMOTE_REQUIRED


class Arbiter:
    """Who drives the robot: nobody, the Pi (follow-bot code) or the remote's stick. Port of drive::decide.

      1. remote STOP (or FAULT), or heard and now lost      -> nobody (the follower is stopped too)
      2. remote required but never heard                    -> nobody
      3. RUN and stick deflected, after it was seen centred in this RUN -> manual; stays manual until the
         stick has been centred for manual_hold
      4. Pi command fresh                                   -> Pi
    Used for source:=serial (remote board on USB, no robot ESP32). With source:=esp32 the robot ESP32's own
    firmware makes this decision and the bridge only mirrors the result.
    """

    def __init__(self, cfg: Optional[ArbiterConfig] = None):
        self.cfg = cfg or ArbiterConfig()
        self.centred_seen_in_run = False
        self.manual_active = False
        self.manual_until = 0.0
        self.prev_run = False

    def decide(self, now: float, remote: Optional[RemoteState], remote_time: Optional[float],
               pi_time: Optional[float]) -> Tuple[str, str]:
        """Returns (source, remote) with source in none|pi|manual and remote in never|lost|stop|run."""
        c = self.cfg
        if remote is None or remote_time is None:
            rstate = 'never'
        elif now - remote_time >= c.remote_timeout:
            rstate = 'lost'
        else:
            rstate = 'run' if remote.run else 'stop'

        run = rstate == 'run'
        if not run or not self.prev_run:        # leaving RUN or just entered it: the stick must be re-centred
            self.centred_seen_in_run = False
            self.manual_active = False
        self.prev_run = run

        if rstate in ('stop', 'lost') or (rstate == 'never' and c.remote_required):
            return SRC_NONE, rstate
        if run:
            centred = abs(remote.x) < c.stick_deadband and abs(remote.y) < c.stick_deadband
            if centred:
                self.centred_seen_in_run = True
            elif self.centred_seen_in_run:
                self.manual_active = True
                self.manual_until = now + c.manual_hold
            if self.manual_active and centred and now >= self.manual_until:
                self.manual_active = False
            if self.manual_active:
                return SRC_MANUAL, rstate
        if pi_time is not None and now - pi_time <= c.cmd_timeout:
            return SRC_PI, rstate
        return SRC_NONE, rstate


def wheels_to_twist(left: float, right: float, track_width: float = TRACK_WIDTH,
                    full_scale_mps: float = FULL_SCALE_MPS, scale: float = 1.0) -> Tuple[float, float]:
    """(left, right) wheel commands in -1..1 -> (linear m/s, angular rad/s, + = left).

    Inverse of honebot_hardware.diffdrive.twist_to_diffdrive (before its *_reversed flags, which the
    robot firmware's S line does not include either). `scale` shrinks both for a small turtlesim window.
    """
    if not (math.isfinite(left) and math.isfinite(right)) or track_width <= 0:
        return 0.0, 0.0
    v_left = left * full_scale_mps
    v_right = right * full_scale_mps
    linear = (v_left + v_right) / 2.0 * scale
    angular = (v_right - v_left) / track_width * scale
    return linear, angular
