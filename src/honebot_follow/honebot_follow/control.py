"""Pure follow-me math, kept free of ROS so it can be unit-tested and reused.

Entry points:
  follow_command_relative(distance, bearing, gains)  the real one: target given
                                                     relative to the robot, as
                                                     UWB will provide it
  follow_command(fx, fy, ftheta, lx, ly, gains)      turtlesim convenience:
                                                     two world poses -> relative
  SideShot(params).step(...)                         side-view filming mode (stateful):
                                                     hold a point behind and beside the
                                                     skater; None = use follow_command_relative
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class FollowGains:
    follow_distance: float = 1.5   # how far behind the leader to stay (m)
    kp_linear: float = 1.0         # forward speed per metre of extra distance
    kp_angular: float = 4.0        # turn rate per radian of heading error
    max_linear: float = 2.0        # m/s
    max_angular: float = 4.0       # rad/s
    ackermann: bool = False        # car-like: cannot turn in place
    creep_speed: float = 0.3       # m/s forward while re-acquiring a target behind (ackermann only)
    # --- real-robot options; every one is OFF at 0 so the turtlesim behaviour is unchanged ---
    brake_decel: float = 0.0       # m/s^2 the robot can stop with. > 0 switches on the speed-dependent gap:
    #                                gap(v) = follow_distance + time_headway * v + v^2 / (2 * brake_decel)
    time_headway: float = 0.5      # s of travel kept in hand on top of the braking distance
    max_lat_accel: float = 0.0     # m/s^2 cap on v * omega (cornering); 0 = no cap
    min_turn_radius: float = 0.0   # m. > 0: turn on arcs of at least this radius instead of pivoting ...
    arc_speed: float = 0.6         # m/s kept while arcing toward a target off to the side
    pivot_bearing: float = 2.0     # rad: ... unless the target is further round than this (then pivot fast)
    pivot_rate: float = 0.8        # rad/s: gentle pivot always allowed in arc mode (little scrub at this rate)
    stopped_turn_deadband: float = 0.0   # rad: bearings below this are not turned toward by pivoting
    follow_bearing: float = 0.0    # rad: where to keep the skater relative to the robot's nose (+ = to the left).
    #                                0 = straight ahead (robot directly behind). 0.45 (26 deg) = the robot rides
    #                                behind and to the RIGHT of the skater's line: a 3/4 rear shot. It still steers
    #                                toward a skater who stops, but it buys margin when braking is worse than
    #                                assumed (see test_offset_follow_gives_margin_when_braking_is_weak).


def safe_gap(speed: float, g: 'FollowGains') -> float:
    """Distance to keep at `speed` so the robot can stop behind a skater who stops dead."""
    if g.brake_decel <= 0.0:
        return g.follow_distance
    v = max(0.0, speed)
    return g.follow_distance + g.time_headway * v + v * v / (2.0 * g.brake_decel)


def speed_for_gap(distance: float, g: 'FollowGains') -> float:
    """Inverse of safe_gap: the fastest speed that is still safe at this distance.

    Solving gap(v) = distance for v in closed form means the command never depends on the robot's own
    (delayed) speed estimate, so there is no feedback loop to go unstable at high speed."""
    extra = distance - g.follow_distance
    if extra <= 0.0:
        return 0.0
    a, t = g.brake_decel, g.time_headway
    return a * (-t + math.sqrt(t * t + 2.0 * extra / a))


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
    - brake_decel > 0: the gap grows with speed (safe_gap / speed_for_gap).
    - min_turn_radius > 0: arcs instead of pivots; max_lat_accel > 0: cornering cap.
    """
    heading_error = wrap_angle(bearing - g.follow_bearing)     # steer to hold the skater at follow_bearing
    angular = clamp(g.kp_angular * heading_error, -g.max_angular, g.max_angular)

    gap = distance - g.follow_distance
    if g.brake_decel > 0.0:
        # Speed-dependent gap: drive at the fastest speed from which the robot can still stop in time.
        base = speed_for_gap(distance, g)
    else:
        base = g.kp_linear * gap
    linear = clamp(base * max(0.0, math.cos(heading_error)), 0.0, g.max_linear)

    arcs = g.min_turn_radius > 0.0 and abs(heading_error) <= g.pivot_bearing
    # Gentle pivot allowed in arc mode: grows from zero at the deadband, capped at pivot_rate. Used both when
    # stopped and as the yaw floor while rolling, so the command is continuous through follow_distance
    # (a noisy range must not flip between "pivot hard" and "barely turn").
    slow_pivot = min(g.pivot_rate, g.kp_angular * max(0.0, abs(heading_error) - g.stopped_turn_deadband))

    if gap <= 0.0:
        # Close enough: stop, and stop turning when right on top of the target
        # where the bearing is meaningless.
        linear = 0.0
        if distance < 0.2:
            angular = 0.0
        elif arcs:
            angular = clamp(angular, -slow_pivot, slow_pivot)
        elif abs(heading_error) < g.stopped_turn_deadband:
            angular = 0.0
    elif g.ackermann and linear < g.creep_speed:
        # Target is far but we're pointed away from it; a car has to roll to turn.
        linear = min(g.creep_speed, g.max_linear)
    elif arcs:
        # Skid steer scrubs its tyres when it pivots: keep rolling and turn on an arc instead.
        # (Target further round than pivot_bearing: a pivot is quicker than a long loop.)
        linear = max(linear, min(g.arc_speed, g.max_linear, base))
        w_max = max(linear / g.min_turn_radius, slow_pivot)
        angular = clamp(angular, -w_max, w_max)

    if g.max_lat_accel > 0.0 and linear > 1e-3:
        w_max = g.max_lat_accel / linear        # v * omega <= max_lat_accel
        angular = clamp(angular, -w_max, w_max)

    return linear, angular, distance


def follow_command(fx, fy, ftheta, lx, ly, g: FollowGains):
    """Turtlesim form: follower pose (fx, fy, ftheta) chasing leader (lx, ly)."""
    distance, bearing = relative_target(fx, fy, ftheta, lx, ly)
    return follow_command_relative(distance, bearing, g)


# ------------------------------------------------------------------------------------------------- side shot
@dataclass
class SideShotParams:
    """Side-view filming: keep the robot side_distance from the skater, side_angle round from straight behind.

    Off while side_distance is 0. Needs the robot's own speed (esp32/speed, from the ESP32's S line) and yaw rate
    (esp32/yaw_rate, from its gyro G line): without both, or while the skater is slow or far, the caller falls
    back to follow_command_relative (the normal law, which keeps the stop-behind gap).
    """
    side_distance: float = 0.0       # m, skater to robot (UWB range). 0 = side mode off
    side_angle: float = 0.785        # rad round from directly behind the skater; + = robot on the skater's RIGHT
    side_kp: float = 1.0             # 1/s: correction speed per metre of position error
    side_max_correction: float = 1.5  # m/s cap on that correction (a far-off aim point must not cause a spin)
    side_max_steer: float = 0.7      # rad: largest heading change asked for at once
    side_kp_angular: float = 1.5     # rad/s per rad of heading change
    side_enter_speed: float = 0.8    # m/s skater speed to switch side mode on ...
    side_exit_speed: float = 0.4     # m/s ... and off again (stopped skater -> normal law)
    side_engage_range: float = 12.0  # m: side mode only within this range (+1.5 m to leave it)
    side_min_range: float = 1.5      # m: closer than this, never forward, and only steer away from the skater
    side_lane: float = 0.6           # m: skater within this of the robot's path -> normal braking law caps speed
    side_speed_tau: float = 0.3      # s: smoothing of the skater velocity estimate
    side_input_timeout: float = 0.3  # s: own speed / yaw rate older than this -> side mode off


class SideShot:
    """Side-view filming controller. Pure (no ROS); the follower node calls step() at its rate.

    How it works, all in a local odometry frame that the robot dead-reckons from its own speed and gyro yaw rate:
      1. each UWB fix puts the skater in that frame; their velocity is the smoothed change between fixes
         (relative motion alone cannot tell the skater's speed from the robot's);
      2. the aim point is side_distance from the skater, side_angle round from straight behind them,
         on their right for side_angle > 0 (it is never the skater itself);
      3. the robot asks for the skater's velocity plus a capped correction toward the aim point, steering
         toward that direction (capped) and driving at its forward component (never backwards).
    Safety rules on top, none of them depending on the estimate: closer than side_min_range -> no forward
    motion and no steering toward the skater; skater inside side_lane of the robot's path -> the normal
    stop-behind braking law caps the speed. The safety gate and firmware still apply downstream.
    """

    def __init__(self, p: SideShotParams):
        self.p = p
        self.reset()

    def reset(self):
        self.pose = (0.0, 0.0, 0.0)      # x, y, heading in the odometry frame
        self.last_t = None
        self.fix = None                  # (x, y, stamp) of the last UWB fix, odometry frame
        self.vel = (0.0, 0.0)            # skater velocity, odometry frame
        self.has_vel = False
        self.active = False

    def step(self, t: float, target: Optional[Tuple[float, float]], target_stamp: Optional[float],
             own_speed: Optional[float], own_yaw_rate: Optional[float],
             g: FollowGains) -> Optional[Tuple[float, float]]:
        """t: now (s). target: fresh (distance, bearing) in the robot frame, or None. target_stamp: when that fix
        arrived. own_speed / own_yaw_rate: m/s and rad/s, None when unavailable or stale.
        Returns (linear, angular) while side mode is active, else None."""
        p = self.p
        if p.side_distance <= 0.0 or target is None or own_speed is None or own_yaw_rate is None:
            self.reset()
            return None
        # 1. dead-reckon our own pose
        if self.last_t is not None:
            dt = t - self.last_t
            if dt <= 0.0 or dt > 0.5:
                self.reset()
            else:
                x0, y0, th = self.pose
                th += own_yaw_rate * dt
                self.pose = (x0 + own_speed * math.cos(th) * dt, y0 + own_speed * math.sin(th) * dt, th)
        self.last_t = t
        X, Y, TH = self.pose
        c, s = math.cos(TH), math.sin(TH)
        distance, bearing = target
        tx, ty = distance * math.cos(bearing), distance * math.sin(bearing)
        # 2. skater velocity from successive fixes
        if target_stamp is not None and (self.fix is None or target_stamp > self.fix[2]):
            fx, fy = X + tx * c - ty * s, Y + tx * s + ty * c
            if self.fix is not None:
                dts = target_stamp - self.fix[2]
                vx, vy = (fx - self.fix[0]) / dts, (fy - self.fix[1]) / dts
                a = min(1.0, dts / p.side_speed_tau) if p.side_speed_tau > 0.0 else 1.0
                if not self.has_vel:
                    a = 1.0
                self.vel = (self.vel[0] + a * (vx - self.vel[0]), self.vel[1] + a * (vy - self.vel[1]))
                self.has_vel = True
            self.fix = (fx, fy, target_stamp)
        if not self.has_vel:
            return None
        vsx, vsy = self.vel[0] * c + self.vel[1] * s, -self.vel[0] * s + self.vel[1] * c   # robot frame
        speed = math.hypot(vsx, vsy)
        if self.active:
            self.active = speed > p.side_exit_speed and distance < p.side_engage_range + 1.5
        else:
            self.active = speed > p.side_enter_speed and distance < p.side_engage_range
        if not self.active:
            return None
        # 3. aim point: behind the skater by cos(angle), beside them (their right for angle > 0) by sin(angle)
        ux, uy = vsx / speed, vsy / speed
        nx, ny = -uy, ux                                 # the skater's left
        back, side = p.side_distance * math.cos(p.side_angle), p.side_distance * math.sin(p.side_angle)
        ax, ay = tx - back * ux - side * nx, ty - back * uy - side * ny
        cx, cy = p.side_kp * ax, p.side_kp * ay
        cm = math.hypot(cx, cy)
        if cm > p.side_max_correction:
            cx, cy = cx * p.side_max_correction / cm, cy * p.side_max_correction / cm
        wx, wy = vsx + cx, vsy + cy
        heading = math.atan2(wy, wx)
        linear = clamp(math.hypot(wx, wy) * math.cos(heading), 0.0, g.max_linear)   # aim point behind: 0
        steer = clamp(heading, -p.side_max_steer, p.side_max_steer)
        angular = clamp(p.side_kp_angular * steer, -g.max_angular, g.max_angular)
        # safety rules that do not depend on the estimate
        if abs(ty) < p.side_lane and tx > 0.0:          # skater in our path: stop-behind braking law
            if g.brake_decel > 0.0:
                linear = min(linear, speed_for_gap(tx, g))
            else:
                linear = min(linear, max(0.0, g.kp_linear * (tx - g.follow_distance)))
        if distance < p.side_min_range:                 # too close: no forward, never steer toward them
            linear = 0.0
            if angular * ty > 0.0:
                angular = 0.0
        if g.max_lat_accel > 0.0 and linear > 1e-3:
            w_max = g.max_lat_accel / linear
            angular = clamp(angular, -w_max, w_max)
        return linear, angular
