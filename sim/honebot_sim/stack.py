"""The Pi's software, in-process, on the simulation clock.

Uses the robot's own pure modules (the same code the ROS nodes call) and mirrors each node's timer/timeout
plumbing, which is a few lines per node:

  uwb_node      _handle_aoa(): RangeFilter on range and bearing, aoa_position()      (honebot_uwb.localize)
  follower      _tick() 20 Hz: target older than target_timeout -> zero Twist, else follow_command_relative()
  safety_gate   _tick() 50 Hz: rising_cap(sight_speed()), gate_command()           (honebot_safety.gate)
  esp32_bridge  heartbeat 10 Hz, C line 50 Hz via twist_to_diffdrive() (neutral when cmd_vel is older than
                cmd_timeout), P line 20 Hz; R lines -> range (+inf for -1) and link_ok
  camera_aim    _step() 20 Hz: PanController with the UWB fallback                  (honebot_camera.aim)

Not exercised here (they need ROS): message transport, parameter handling, serial port I/O and reconnects,
the camera node's frame grabbing and its Kalman tracker (detections go straight to camera_aim).
"""

import math

from honebot_camera.aim import AimGains, PanController
from honebot_follow.control import FollowGains, follow_command_relative
from honebot_hardware.diffdrive import DiffDriveParams, twist_to_diffdrive
from honebot_safety.gate import GateParams, gate_command, rising_cap, sight_speed
from honebot_uwb.localize import RangeFilter, UwbGeometry, aoa_position


def _pick(cls, params):
    names = cls.__dataclass_fields__.keys()
    return cls(**{k: v for k, v in params.items() if k in names})


class PiStack:
    def __init__(self, cfg, overrides=None):
        y = cfg.yaml
        ov = overrides or {}
        fol = dict(y["follower"]); fol.update(ov.get("follower", {}))
        gat = dict(y["safety_gate"]); gat.update(ov.get("safety_gate", {}))
        uwb = dict(y["uwb_node"]); uwb.update(ov.get("uwb_node", {}))
        brg = dict(y["esp32_bridge"]); brg.update(ov.get("esp32_bridge", {}))
        self.gains = _pick(FollowGains, fol)
        self.target_timeout = fol.get("target_timeout", 0.5)
        self.gate = _pick(GateParams, gat)
        self.dd = _pick(DiffDriveParams, brg)
        self.bridge_cmd_timeout = brg.get("cmd_timeout", 0.3)
        # uwb_node
        self.geom = UwbGeometry(baseline=uwb.get("baseline", 0.3), anchor_x=uwb.get("anchor_x", 0.0))
        alpha = uwb.get("filter_alpha", 0.5)
        self.max_range = uwb.get("max_range", 30.0)
        self.max_bearing = uwb.get("max_bearing_deg", 80.0)
        self.bearing_sign = uwb.get("bearing_sign", 1.0)
        self.f_range = RangeFilter(alpha=alpha, max_range=self.max_range)
        self.f_bearing = RangeFilter(alpha=uwb.get("bearing_alpha", 0.6), min_range=-self.max_bearing,
                                     max_range=self.max_bearing)
        # camera_aim's declared parameter defaults (camera_aim.py), not AimGains' own defaults
        self.pan = PanController(AimGains(kp=0.6, deadband=math.radians(1.0), max_rate=math.radians(150.0),
                                          half_range=math.radians(45.0), lost_timeout=0.5, recenter_timeout=3.0,
                                          recenter_rate=math.radians(30.0)))
        self.aim_meas_max_age = 0.3
        # state (None = never received), all times on the sim clock
        self.target = None          # (distance, bearing)
        self.target_t = None
        self.cmd_raw = (0.0, 0.0)
        self.cmd_raw_t = None
        self.cmd_vel = (0.0, 0.0)
        self.cmd_vel_t = None
        self.range_m = None
        self.range_t = None
        self.link_ok = False
        self.link_t = None
        self.speed_cap = None
        self.gate_state = None
        self.uwb_bearing = None
        self.uwb_t = None
        self.meas = None
        self.last_aim_t = 0.0
        self.pan_cmd = 0.0

    # ---- uwb_node --------------------------------------------------------------------------------------
    def on_uwb_line(self, t, rng, bearing_deg):
        r = self.f_range.update(rng)
        b = self.f_bearing.update(bearing_deg)
        if r is None or b is None:
            return
        pos = aoa_position(r, math.radians(b) * self.bearing_sign, self.geom, self.max_range)
        if pos is None:
            return
        x, y = pos
        self.target = (math.hypot(x, y), math.atan2(y, x))      # follower._on_target
        self.target_t = t
        self.uwb_bearing, self.uwb_t = math.atan2(y, x), t      # camera_aim._on_uwb

    # ---- follower ------------------------------------------------------------------------------------
    def follower_tick(self, t):
        lin = ang = 0.0
        if self.target is not None and t - self.target_t <= self.target_timeout:
            lin, ang, _ = follow_command_relative(self.target[0], self.target[1], self.gains)
        self.cmd_raw, self.cmd_raw_t = (lin, ang), t

    # ---- esp32_bridge: incoming lines ------------------------------------------------------------------
    def on_r_line(self, t, dist, link):
        self.range_m = dist if dist >= 0.0 else math.inf
        self.range_t = t
        self.link_ok, self.link_t = link, t

    # ---- safety_gate -------------------------------------------------------------------------------------
    def gate_tick(self, t, dt):
        p = self.gate
        if p.brake_decel > 0.0 and self.range_m is not None and self.range_m == self.range_m:
            self.speed_cap = rising_cap(self.speed_cap, sight_speed(self.range_m, p), dt, p)
        else:
            self.speed_cap = None
        age = lambda s: None if s is None else t - s
        lin, ang, state = gate_command(self.cmd_raw[0], self.cmd_raw[1], self.range_m, age(self.range_t),
                                       self.link_ok, age(self.link_t), age(self.cmd_raw_t), p, self.speed_cap)
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
        return "P,%.3f" % self.pan_cmd

    # ---- camera_aim --------------------------------------------------------------------------------------
    def on_detection(self, t, bearing_err):
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
        fb = self.uwb_bearing if (self.uwb_t is not None and t - self.uwb_t <= 0.5) else None
        self.pan.update(t, dt, err, fb)
        self.pan_cmd = self.pan.normalized(1.0)
