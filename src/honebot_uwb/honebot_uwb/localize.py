"""Pure UWB geometry and filtering, free of ROS for unit testing.

Robot frame: x forward, y left, origin at base_link. Two anchors are mounted
on the robot, symmetric about the centre line, `baseline` metres apart and
`anchor_x` metres ahead of base_link:

      left anchor  (anchor_x, +baseline/2)
      right anchor (anchor_x, -baseline/2)

Alternatively an angle-of-arrival (AoA) anchor reports range and bearing from
one module; see aoa_position().

The skater's tag is at range r_left from the left anchor and r_right from the
right one. Two ranges give the tag position up to a front/back mirror; we
assume the tag is in front (x >= anchor_x), which holds because the robot's job
is to stay behind the skater. A third anchor would remove the ambiguity.
"""

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class UwbGeometry:
    baseline: float = 0.30      # m between the two anchors (wider = better bearing)
    anchor_x: float = 0.0       # m anchors sit ahead of base_link


def two_anchor_position(r_left: float, r_right: float, g: UwbGeometry,
                        tolerance: float = 0.25) -> Optional[Tuple[float, float]]:
    """(x, y) of the tag in the robot frame, or None if the ranges are not
    geometrically consistent (difference larger than the baseline + tolerance,
    or non-positive)."""
    if not (math.isfinite(r_left) and math.isfinite(r_right)):
        return None
    b = g.baseline
    if b <= 0.0 or r_left <= 0.0 or r_right <= 0.0:
        return None
    if abs(r_left - r_right) > b + tolerance:
        return None                      # impossible: the two ranges can't differ by more than the baseline

    # Anchors at (0, +b/2) and (0, -b/2) in the anchor frame:
    #   r_l^2 = x^2 + (y - b/2)^2,  r_r^2 = x^2 + (y + b/2)^2  ->  r_r^2 - r_l^2 = 2 b y
    y = (r_right ** 2 - r_left ** 2) / (2.0 * b)
    x_sq = r_left ** 2 - (y - b / 2.0) ** 2
    x = math.sqrt(x_sq) if x_sq > 0.0 else 0.0     # tiny negatives from noise -> on the baseline
    return x + g.anchor_x, y


def aoa_position(distance: float, bearing: float, g: UwbGeometry,
                 max_range: float = 50.0) -> Optional[Tuple[float, float]]:
    """(x, y) in the robot frame from an angle-of-arrival anchor that reports
    range and bearing directly (bearing in radians, +ve = left, 0 = straight
    ahead of the anchor). The anchor sits anchor_x ahead of base_link."""
    if not (math.isfinite(distance) and math.isfinite(bearing)):
        return None
    if distance <= 0.0 or distance > max_range or abs(bearing) > math.pi:
        return None
    return distance * math.cos(bearing) + g.anchor_x, distance * math.sin(bearing)


def to_distance_bearing(x: float, y: float) -> Tuple[float, float]:
    return math.hypot(x, y), math.atan2(y, x)


class RangeFilter:
    """Cleanup for one measured quantity (a range, or an AoA bearing with min/max set to suit):
    median-of-N kills single-sample spikes, then an exponential moving average smooths.

    A sample outside [min, max] or not finite is "no measurement this cycle": update() returns None,
    so the caller publishes nothing and the follower's target timeout can see the dropout. (Returning
    the last estimate here would keep a lost tag looking alive, and the robot would chase a ghost.)
    After max_misses of those in a row the history is cleared, so a re-acquired tag is not blended
    with where it was before the gap."""

    def __init__(self, window: int = 3, alpha: float = 0.5,
                 min_range: float = 0.1, max_range: float = 50.0, max_misses: int = 5):
        self.window = deque(maxlen=max(1, window))
        self.alpha = alpha
        self.min_range = min_range
        self.max_range = max_range
        self.max_misses = max_misses
        self.misses = 0
        self.value: Optional[float] = None

    def update(self, r: float) -> Optional[float]:
        if not math.isfinite(r) or r < self.min_range or r > self.max_range:
            self.misses += 1
            if self.misses >= self.max_misses:
                self.reset()
            return None
        self.misses = 0
        self.window.append(r)
        med = sorted(self.window)[len(self.window) // 2]
        self.value = med if self.value is None else self.alpha * med + (1 - self.alpha) * self.value
        return self.value

    def reset(self):
        self.window.clear()
        self.value = None
        self.misses = 0
