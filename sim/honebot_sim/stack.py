"""The Pi's software, in-process, on the simulation clock.

Uses the robot's own pure modules (the same code the ROS nodes call) and mirrors each node's timer/timeout
plumbing, which is a few lines per node:

  uwb_node      _handle_aoa(): RangeFilter on range and bearing, aoa_position()      (honebot_uwb.localize)
  follower      _tick() 20 Hz: target older than target_timeout -> zero Twist, else SideShot.step() (side_distance
                > 0) or follow_command_relative(); own speed / yaw rate from esp32_bridge's S and G lines
  safety_gate   _tick() 50 Hz: rising_cap(sight_speed()), gate_command()           (honebot_safety.gate)
  esp32_bridge  heartbeat 10 Hz, C line 50 Hz via twist_to_diffdrive() (neutral when cmd_vel is older than
                cmd_timeout), P line 20 Hz; R lines -> range (+inf for -1) and link_ok
  camera_aim    _step() 20 Hz: PanController with the UWB fallback                  (honebot_camera.aim)

Every parameter (and every timer rate) comes from the node's own declare_parameter() default with robot.yaml on
top (config.load_node_params), so the sim has no copies of its own. sim/tests/test_sources.py hashes the node
functions mirrored here and fails when one changes, like tests/test_firmware.py does for the sketch.

Not exercised here: message transport, live parameter changes, serial port I/O and reconnects (they need ROS or
hardware), the camera node's frame grabbing, and its plausibility filters + Kalman tracker (honebot_camera.track,
pure and unit-tested but not wired in: detections go straight to camera_aim).
"""

import math

from honebot_camera.aim import AimGains, PanController
from honebot_follow.control import FollowGains, SideShot, SideShotParams, follow_command_relative
from honebot_hardware.diffdrive import DiffDriveParams, twist_to_diffdrive, wheels_to_speed
from honebot_safety.gate import GateParams, gate_command, rising_cap, sight_speed
from honebot_uwb.localize import RangeFilter, UwbGeometry, aoa_position


def _pick(cls, params):
    names = cls.__dataclass_fields__.keys()
    return cls(**{k: v for k, v in params.items() if k in names})


class PiStack:
    """Parameters: cfg.params (each node's own declare_parameter() defaults with robot.yaml on top), then
    `overrides` ({node: {param: value}}, from --set / profiles). Nothing is defaulted here: a parameter the node
    does not declare raises KeyError instead of silently taking a sim-only value."""

    NODES = ("follower", "safety_gate", "uwb_node", "esp32_bridge", "camera_aim")

    def __init__(self, cfg, overrides=None):
        ov = overrides or {}
        unknown = set(ov) - set(self.NODES)
        if unknown:
            raise KeyError("no such node in the sim's Pi stack: %s" % sorted(unknown))
        for n, params in ov.items():
            bad = set(params) - set(cfg.params[n])
            if bad:      # ROS would ignore an undeclared parameter; here a typo would silently test nothing
                raise KeyError("%s does not declare %s" % (n, sorted(bad)))
        P = {n: dict(cfg.params[n], **ov.get(n, {})) for n in self.NODES}
        self.params = P
        fol, gat, uwb, brg, aim = (P[n] for n in self.NODES)
        self.gains = _pick(FollowGains, fol)
        self.target_timeout = fol["target_timeout"]
        self.side = SideShot(_pick(SideShotParams, fol))
        self.side_input_timeout = fol["side_input_timeout"]
        self.drive_mode = brg["drive_mode"]
        self.own_speed = None       # (m/s, t) esp32/speed, from S lines
        self.own_yaw = None         # (rad/s, t) esp32/yaw_rate, from usable G lines
        self.gate = _pick(GateParams, gat)
        self.dd = _pick(DiffDriveParams, brg)
        self.bridge_cmd_timeout = brg["cmd_timeout"]
        # timer rates of the nodes (Hz), used by sim.py's scheduler
        self.rates = dict(follower=fol["rate_hz"], gate=gat["rate_hz"], heartbeat=brg["heartbeat_hz"],
                          cmd=brg["cmd_rate_hz"], pan=brg["pan_rate_hz"], aim=aim["rate_hz"])
        # uwb_node
        self.geom = UwbGeometry(baseline=uwb["baseline"], anchor_x=uwb["anchor_x"])
        self.max_range = uwb["max_range"]
        self.max_bearing = uwb["max_bearing_deg"]
        self.bearing_sign = uwb["bearing_sign"]
        self.f_range = RangeFilter(alpha=uwb["filter_alpha"], max_range=self.max_range)
        self.f_bearing = RangeFilter(alpha=uwb["bearing_alpha"], min_range=-self.max_bearing,
                                     max_range=self.max_bearing)
        # camera_aim
        self.pan = PanController(AimGains(kp=aim["kp"], deadband=math.radians(aim["deadband_deg"]),
                                          max_rate=math.radians(aim["max_rate_deg_s"]),
                                          half_range=math.radians(aim["half_range_deg"]),
                                          lost_timeout=aim["lost_timeout"], recenter_timeout=aim["recenter_timeout"],
                                          recenter_rate=math.radians(aim["recenter_rate_deg_s"])))
        self.pan_half_range = math.radians(aim["half_range_deg"])
        self.pan_sign = aim["pan_sign"]
        self.use_uwb_fallback = aim["use_uwb_fallback"]
        self.aim_uwb_timeout = aim["uwb_timeout"]
        self.aim_meas_max_age = aim["measurement_max_age"]
        # state (None = never received), all times on the sim clock
        self.target = None          # (distance, bearing)
        self.target_t = None
        self.cmd_raw = (0.0, 0.0)
        self.cmd_raw_t = None
        self.cmd_vel = (0.0, 0.0)
        self.cmd_vel_t = None
        self.range_m = None
        self.range_t = None
        self.echo_t = None          # last finite range (safety_gate.echo_stamp)
        self.link_ok = False
        self.link_t = None
        self.speed_cap = None
        self.gate_state = None
        self.uwb_bearing = None
        self.uwb_t = None
        self.meas = None
        self.last_aim_t = 0.0
        self.pan_cmd = None         # esp32_bridge sends no P line until camera_aim has published once

    # ---- uwb_node --------------------------------------------------------------------------------------
    def on_uwb_line(self, t, rng, bearing_deg):
        if not (math.isfinite(rng) and self.f_range.min_range <= rng <= self.f_range.max_range
                and math.isfinite(bearing_deg) and abs(bearing_deg) <= self.max_bearing):
            self.f_range.update(math.nan)
            self.f_bearing.update(math.nan)
            return
        r = self.f_range.update(rng)
        b = self.f_bearing.update(bearing_deg)
        if r is None or b is None:
            return
        pos = aoa_position(r, math.radians(b) * self.bearing_sign, self.geom, self.max_range)
        if pos is None:
            return
        x, y = pos
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        self.target = (math.hypot(x, y), math.atan2(y, x))      # follower._on_target
        self.target_t = t
        self.uwb_bearing, self.uwb_t = math.atan2(y, x), t      # camera_aim._on_uwb

    # ---- follower ------------------------------------------------------------------------------------
    def _fresh(self, sample, t):
        return None if sample is None or t - sample[1] > self.side_input_timeout else sample[0]

    def follower_tick(self, t):
        lin = ang = 0.0
        if self.target is not None and t - self.target_t <= self.target_timeout:
            side = self.side.step(t, self.target, self.target_t, self._fresh(self.own_speed, t),
                                  self._fresh(self.own_yaw, t), self.gains)
            if side is not None:
                lin, ang = side
            else:
                lin, ang, _ = follow_command_relative(self.target[0], self.target[1], self.gains)
        elif self.target is not None:
            self.side.reset()
        self.cmd_raw, self.cmd_raw_t = (lin, ang), t

    # ---- esp32_bridge: incoming lines ------------------------------------------------------------------
    def on_r_line(self, t, dist, link):
        if math.isfinite(dist) and dist >= 0.0:
            self.range_m = dist
        elif dist == -1.0:
            self.range_m = math.inf
        else:
            self.range_m = math.nan
        self.range_t = t
        if math.isfinite(self.range_m) and self.range_m > 0.0:   # finite reading = an echo (-1 = none)
            self.echo_t = t
        self.link_ok, self.link_t = link, t

    def on_s_line(self, t, a, b):
        if not (math.isfinite(a) and math.isfinite(b)):
            return
        if self.drive_mode == "diff":
            v = wheels_to_speed(a, b, self.dd)
            if math.isfinite(v):
                self.own_speed = (v, t)

    def on_g_line(self, t, yaw, usable):
        if usable and math.isfinite(yaw):
            self.own_yaw = (yaw, t)

    # ---- safety_gate -------------------------------------------------------------------------------------
    def gate_tick(self, t):
        p = self.gate
        dt = 1.0 / self.rates["gate"]          # safety_gate passes 1 / rate_hz to rising_cap
        age = lambda s: None if s is None else t - s
        range_age, link_age = age(self.range_t), age(self.link_t)
        fresh_sensor = (self.link_ok and link_age is not None and link_age <= p.range_timeout
                        and self.range_m is not None and range_age is not None and range_age <= p.range_timeout
                        and not math.isnan(self.range_m) and self.range_m > 0.0)
        if p.brake_decel > 0.0:
            if fresh_sensor:
                self.speed_cap = rising_cap(self.speed_cap, sight_speed(self.range_m, p), dt, p)
            else:
                self.speed_cap = 0.0
        else:
            self.speed_cap = None
        lin, ang, state = gate_command(self.cmd_raw[0], self.cmd_raw[1], self.range_m, range_age,
                                       self.link_ok, link_age, age(self.cmd_raw_t), p, self.speed_cap,
                                       age(self.echo_t))
        self.cmd_vel, self.cmd_vel_t = (lin, ang), t
        self.gate_state = state

    # ---- esp32_bridge: outgoing lines -----------------------------------------------------------------
    def bridge_cmd_line(self, t):
        if self.cmd_vel_t is None or t - self.cmd_vel_t > self.bridge_cmd_timeout:
            a = b = 0.0
        else:
            a, b = twist_to_diffdrive(self.cmd_vel[0], self.cmd_vel[1], self.dd)
        return "C,%.3f,%.3f" % (a, b)

    def bridge_pan_line(self):
        """None until camera_aim has published (the bridge sends nothing before that)."""
        return None if self.pan_cmd is None else "P,%.3f" % self.pan_cmd

    # ---- camera_aim --------------------------------------------------------------------------------------
    def on_detection(self, t, bearing_err):
        if math.isfinite(bearing_err):
            self.meas = (bearing_err, t)

    def aim_tick(self, t):
        dt = t - self.last_aim_t
        self.last_aim_t = t
        err = None
        if self.meas is not None:
            b, mt = self.meas
            self.meas = None
            if t - mt <= self.aim_meas_max_age:
                err = b
        fb = None
        if self.use_uwb_fallback and self.uwb_t is not None and t - self.uwb_t <= self.aim_uwb_timeout:
            fb = self.uwb_bearing
        self.pan.update(t, dt, err, fb)
        self.pan_cmd = self.pan.normalized(self.pan_sign)
