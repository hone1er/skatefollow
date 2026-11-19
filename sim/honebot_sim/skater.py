"""Skater scenarios: a path the skater rides, as a speed and turn-rate schedule integrated over time.

Each scenario is a list of segments (duration s, target speed m/s, yaw rate rad/s, accel/decel m/s^2) plus
where the robot starts and any obstacles. Speeds in mph are converted at the edges (1 mph = 0.447 m/s).
Skater accelerations: pushing ~1 m/s^2; a normal stop ~2 m/s^2; a foot-brake / bail "dead stop" 4-7 m/s^2.
"""

import math
from dataclasses import dataclass, field

MPH = 0.44704


@dataclass
class Segment:
    duration: float
    speed: float          # m/s to approach during the segment
    yaw_rate: float = 0.0  # rad/s while moving (+ = left)
    accel: float = 1.0    # m/s^2 used when speeding up
    decel: float = 2.0    # m/s^2 used when slowing down


@dataclass
class Scenario:
    name: str
    description: str
    segments: list
    robot_start: tuple = (-3.0, 0.0, 0.0)      # x, y, heading of the robot; the skater starts at (0, 0, 0)
    obstacles: list = field(default_factory=list)   # (x, y, size_x, size_y, height[, yaw])
    expect_keep_up: bool = True                # with the "today" settings, should the gap stay bounded?
    settle: float = 4.0                        # s at the end with the skater standing still
    side_check: bool = True                    # --side: judge the held distance/angle here (not where a box
    #                                            blocks the side position)

    @property
    def duration(self):
        return sum(s.duration for s in self.segments) + self.settle


class SkaterPath:
    """Integrates a Scenario into a pose at any time (precomputed at 100 Hz)."""

    def __init__(self, sc: Scenario, dt=0.01):
        self.dt = dt
        x = y = th = v = 0.0
        self.samples = [(0.0, x, y, th, v)]
        t = 0.0
        segs = list(sc.segments) + [Segment(sc.settle, 0.0, 0.0, 1.0, 2.0)]
        for s in segs:
            n = max(1, int(round(s.duration / dt)))
            for _ in range(n):
                if v < s.speed:
                    v = min(s.speed, v + s.accel * dt)
                else:
                    v = max(s.speed, v - s.decel * dt)
                if v > 0.05:
                    th += s.yaw_rate * dt
                x += v * math.cos(th) * dt
                y += v * math.sin(th) * dt
                t += dt
                self.samples.append((t, x, y, th, v))

    def at(self, t):
        i = min(len(self.samples) - 1, max(0, int(t / self.dt)))
        return self.samples[i][1:]


def _cruise(mph, decel, cruise=10.0):
    v = mph * MPH
    return [Segment(v / 1.0 + cruise, v, 0.0, 1.0, 2.0), Segment(v / decel + 0.5, 0.0, 0.0, 1.0, decel)]


SCENARIOS = {s.name: s for s in [
    Scenario("walk_stop", "walking pace 3 mph, then stop: the first thing to try in a car park",
             [Segment(12.0, 3 * MPH, 0, 0.8, 1.5), Segment(3.0, 0.0, 0, 0.8, 1.5)]),
    Scenario("cruise_stop_6", "push to 6 mph, cruise 10 s, foot-brake dead stop (4 m/s^2)", _cruise(6, 4.0)),
    Scenario("cruise_stop_8", "push to 8 mph, cruise, dead stop", _cruise(8, 4.0), expect_keep_up=False),
    Scenario("cruise_stop_12", "push to 12 mph, cruise, dead stop", _cruise(12, 4.0), expect_keep_up=False),
    Scenario("cruise_stop_15", "push to 15 mph, cruise, dead stop", _cruise(15, 4.0), expect_keep_up=False),
    Scenario("bail_8", "8 mph then a bail: the skater stops at 7 m/s^2", _cruise(8, 7.0, cruise=6.0),
             expect_keep_up=False),
    Scenario("carve_6", "6 mph slalom: S-curves of +/-0.6 rad/s",
             [Segment(6.0, 6 * MPH, 0.0)] + [Segment(2.5, 6 * MPH, w) for w in (0.6, -0.6, 0.6, -0.6, 0.6, -0.6)]
             + [Segment(3.0, 0.0, 0.0, 1.0, 2.0)]),
    Scenario("u_turn_5", "5 mph, then a 180 deg turn of ~3.5 m radius and back",
             [Segment(8.0, 5 * MPH, 0.0), Segment(math.pi / 0.64, 5 * MPH, 0.64), Segment(8.0, 5 * MPH, 0.0),
              Segment(2.0, 0.0, 0.0, 1.0, 2.0)]),
    Scenario("u_turn_right_5", "5 mph, then a 180 deg turn to the RIGHT (toward a side-view robot) and back",
             [Segment(8.0, 5 * MPH, 0.0), Segment(math.pi / 0.64, 5 * MPH, -0.64), Segment(8.0, 5 * MPH, 0.0),
              Segment(2.0, 0.0, 0.0, 1.0, 2.0)]),
    Scenario("stop_and_go", "4 mph push, coast, stop, again (x3)",
             [Segment(6.0, 4 * MPH), Segment(2.5, 0.0, 0, 1.0, 2.5)] * 3),
    Scenario("obstacle", "5 mph S-bend around a 0.5 m box on the straight line: the robot must not hit it",
             [Segment(3.0, 5 * MPH, 0.0), Segment(1.5, 5 * MPH, 0.5), Segment(3.0, 5 * MPH, -0.5),
              Segment(1.5, 5 * MPH, 0.5), Segment(6.0, 5 * MPH, 0.0), Segment(2.0, 0.0)],
             obstacles=[(9.0, 0.0, 0.5, 0.5, 0.5)], side_check=False),
    Scenario("standing_turns", "skater walks a slow circle around the robot (pivot and camera pan)",
             [Segment(25.0, 1.0, 0.33, 0.5, 1.0)], robot_start=(-2.5, 0.0, 0.0)),
    Scenario("playground", "free driving with the remote or the keyboard: the skater stands 8 m ahead, boxes "
             "to drive around; the follower drives whenever the stick is centred",
             [Segment(600.0, 0.0)], robot_start=(-8.0, 0.0, 0.0),
             obstacles=[(-4.0, 3.0, 0.5, 0.5, 0.5), (-4.0, -3.0, 0.5, 0.5, 0.5), (2.0, 4.0, 2.0, 0.4, 0.3),
                        (-10.0, -5.0, 0.6, 1.8, 0.5)], settle=0.0),
]}
