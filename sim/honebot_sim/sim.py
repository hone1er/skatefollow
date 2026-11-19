"""The closed loop: MuJoCo physics + emulated ESP32 + the Pi's software + sensors + a scripted skater.

Everything runs on one clock (simulation time), stepped at physics_dt (2 ms):
  every step      ESP32 updateOutputs() -> ESC voltages -> motor torques -> mj_step
  60 ms           HC-SR04 ping -> R line -> bridge -> gate input            (10 ms for the TFmini option)
  1/heartbeat_hz  bridge heartbeat H                                        (the ROS nodes' own rate parameters:
  1/rate_hz       follower; safety gate; camera_aim                          PiStack.rates, from the node sources
  1/cmd_rate_hz   bridge C line;  1/pan_rate_hz  bridge P line               and robot.yaml)
  1/uwb_hz        UWB fix -> uwb_node filters -> target
  1/cam_hz        camera frame -> detection (after latency) -> camera_aim

Faults for the safety tests (Sim.faults, a set; add or remove at any time during a run):
  "pi_dead"   the Pi (bridge) stops sending anything: no H, C or P lines  -> ESP32 watchdog fail-safe
  "uwb_dead"  the UWB board stops reporting                                -> follower target timeout
  "us_dead"   the HC-SR04 never echoes (unplugged / broken): R,-1 lines    -> the gate sees +inf = clear
  "rx_dead"   the ESP32's R lines stop reaching the Pi (Pi still sends)    -> gate RANGE_STALE / LINK_DOWN
  "status_dead"  the ESP32's S and G lines stop reaching the Pi           -> side shot falls back to normal law
  "follower_dead"  the follower node stops publishing                     -> gate CMD_STALE
  "gate_dead"      the safety gate stops publishing                       -> bridge sends neutral after cmd_timeout
The handheld remote is Sim.esp.remote: "run" | "stop" | "lost" (packets stopped) | "none" (never heard).
"""

import math
from dataclasses import dataclass, field

import mujoco
import numpy as np

from .firmware import Esp32
from .model import Motor, build_mjcf
from .sensors import Detector, RangeSensor, Uwb
from .skater import SkaterPath
from .stack import PiStack


SIDE_SETTLE = 8.0       # s: side-view samples start here (skater's push to 12 mph is 5.4 s, then the approach)


@dataclass
class Metrics:
    duration: float = 0.0
    min_gap: float = math.inf            # robot body to skater centre, metres
    min_gap_moving: float = math.inf     # same, only while the robot moves (> 0.1 m/s): what the robot did, not
    #                                      the scripted skater walking past a robot that stands still
    skater_contact: bool = False
    obstacle_contact: bool = False
    tipped: bool = False
    max_speed: float = 0.0
    max_gap_moving: float = 0.0          # largest gap while the skater moves
    final_gap: float = math.nan
    final_speed: float = math.nan
    lost_time: float = 0.0               # s with no fresh UWB target at the follower
    framed: float = 0.0                  # s with the skater inside the GoPro frame
    visible_time: float = 0.0            # s the metric was evaluated
    gate_time: dict = field(default_factory=dict)
    max_current: float = 0.0
    failsafe_time: float = 0.0
    lost_moving: float = 0.0             # lost_time while the skater moves (> 0.3 m/s)
    framed_moving: float = 0.0           # framed / visible_time, counted only while the skater moves
    visible_moving: float = 0.0
    # side view, sampled at 20 Hz while the skater moves (> 1 m/s), from t = SIDE_SETTLE on (the robot starts 3 m
    # directly behind and needs the skater's push + a few seconds to reach the side): distance skater -> robot
    # (UWB anchor) and the angle round from directly behind the skater (+ = robot on the skater's right), degrees
    side_dist: list = field(default_factory=list)
    side_angle: list = field(default_factory=list)

    @staticmethod
    def _pct(v, q):
        if not v:
            return math.nan
        s = sorted(v)
        return s[min(len(s) - 1, int(q * len(s)))]

    def side(self, q=0.5):
        """(distance m, angle deg) at quantile q of the side-view samples."""
        return self._pct(self.side_dist, q), self._pct(self.side_angle, q)

    @property
    def framing_moving(self):
        return self.framed_moving / self.visible_moving if self.visible_moving > 0 else 0.0

    @property
    def framing(self):
        return self.framed / self.visible_time if self.visible_time > 0 else 0.0


class Sim:
    def __init__(self, cfg, scenario, seed=0, stack_overrides=None, firmware_overrides=None, log=False):
        self.cfg = cfg
        self.sc = scenario
        self.path = SkaterPath(scenario)
        fw = dict(cfg.firmware)
        fw.update(firmware_overrides or {})
        self.pi = PiStack(cfg, stack_overrides)
        self.xml = build_mjcf(cfg, scenario.obstacles, params=self.pi.params)   # overrides reach the model too
        self.m = mujoco.MjModel.from_xml_string(self.xml)
        self.d = mujoco.MjData(self.m)
        self.rng = np.random.default_rng(seed)
        self.esp = Esp32(fw, remote=cfg.remote)
        self.motors = [Motor(cfg.motor), Motor(cfg.motor)]
        self.rs = RangeSensor(cfg, self.rng)
        self.uwb = Uwb(cfg, self.rng)
        self.det = Detector(cfg, self.rng)
        mid = lambda kind, name: mujoco.mj_name2id(self.m, kind, name)
        self.chassis = mid(mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self.skater = mid(mujoco.mjtObj.mjOBJ_BODY, "skater")
        self.pan_body = mid(mujoco.mjtObj.mjOBJ_BODY, "pan")
        self.cam = mid(mujoco.mjtObj.mjOBJ_CAMERA, "gopro")
        self.anchor = mid(mujoco.mjtObj.mjOBJ_SITE, "uwb")
        self.tag = mid(mujoco.mjtObj.mjOBJ_SITE, "tag")
        self.jFL = self.m.jnt_dofadr[mid(mujoco.mjtObj.mjOBJ_JOINT, "wheel_FL")]
        self.jFR = self.m.jnt_dofadr[mid(mujoco.mjtObj.mjOBJ_JOINT, "wheel_FR")]
        self.jpan = self.m.jnt_qposadr[mid(mujoco.mjtObj.mjOBJ_JOINT, "pan")]
        self.robot_geoms = {i for i in range(self.m.ngeom) if self.m.geom_bodyid[i] not in (0, self.skater)}
        self.skater_geoms = {i for i in range(self.m.ngeom) if self.m.geom_bodyid[i] == self.skater}
        self.obstacle_geoms = {i for i in range(self.m.ngeom)
                               if (mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith("obstacle")}
        # place the robot
        x, y, th = scenario.robot_start
        self.d.qpos[0:3] = (x, y, 0.002)
        self.d.qpos[3:7] = (math.cos(th / 2), 0, 0, math.sin(th / 2))
        self._move_skater(0.0)
        mujoco.mj_forward(self.m, self.d)
        self.t = 0.0
        self.ms = 0
        self.next = {k: 0.0 for k in ("us", "hb", "gate", "cmd", "fol", "aim", "pan", "uwb", "cam", "log", "status", "side")}
        self.faults = set()
        self.metrics = Metrics()
        self.log = [] if log else None
        self.skater_state = (0.0, 0.0, 0.0, 0.0)
        self.remote_input = None
        # gyro (MPU6050) as the firmware sees it: 200 Hz, 0.05 rad/s noise, a constant bias the firmware calibrates
        # away during the arming window; faults for testing: sign (mounted upside down), stuck (reads 0), dead
        self.gyro_fault = cfg.overrides.get("gyro_fault", "none")
        self.gyro_bias = float(self.rng.normal(0.0, 0.03))
        self.next_gyro = 0.0
        self.pwm_hold = (0.0, 0.0)            # servo PWM: the ESCs see a new pulse width once per 20 ms frame
        self.next_pwm = 0.0

    # ----------------------------------------------------------------------------------------------------
    def _move_skater(self, t):
        x, y, th, v = self.path.at(t)
        self.skater_state = (x, y, th, v)
        self.d.mocap_pos[0] = (x, y, 0.0)
        self.d.mocap_quat[0] = (math.cos(th / 2), 0, 0, math.sin(th / 2))

    def robot_pose(self):
        q = self.d.qpos
        w, x, y, z = q[3:7]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return q[0], q[1], yaw

    def _gap(self):
        """Horizontal distance from the skater's centre to the robot's footprint (tub + wheels)."""
        g = self.cfg.geom
        rx, ry, yaw = self.robot_pose()
        sx, sy = self.skater_state[0], self.skater_state[1]
        dx, dy = sx - rx, sy - ry
        lx = math.cos(yaw) * dx + math.sin(yaw) * dy
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
        ex = max(g.body_x0 - lx, 0.0, lx - max(g.body_x1, g.wheel_r))
        hw = g.track / 2 + g.wheel_w / 2
        ey = max(abs(ly) - hw, 0.0)
        return math.hypot(ex, ey)

    def step(self):
        cfg, m, d, pi, esp = self.cfg, self.m, self.d, self.pi, self.esp
        t = self.t
        nm = self.next
        # ---- sensors and Pi software at their own rates -------------------------------------------------
        rate = pi.rates
        pi_alive = "pi_dead" not in self.faults
        if t >= nm["us"]:
            nm["us"] += self.rs.period
            dist = -1.0 if "us_dead" in self.faults else self.rs.measure(m, d, self.chassis)
            r, _, _ = esp.status_lines(dist)
            parts = r.split(",")
            if pi_alive and "rx_dead" not in self.faults:
                pi.on_r_line(t, float(parts[1]), parts[2] == "1")
        if t >= nm["hb"]:
            nm["hb"] += 1.0 / rate["heartbeat"]
            if pi_alive:
                esp.line("H", self.ms)
        if t >= nm["uwb"]:
            nm["uwb"] += self.uwb.period
            rng, brg = self.uwb.measure(m, d, self.chassis, self.anchor, self.tag)
            if "uwb_dead" not in self.faults:
                pi.on_uwb_line(t, float("%.2f" % rng), float("%.1f" % brg))
        if t >= nm["cam"]:
            nm["cam"] += self.det.period
            self.det.capture(t, m, d, self.pan_body, self.cam, self.skater)
        e = self.det.ready(t)
        if e is not None:
            pi.on_detection(t, e)
        if t >= nm["fol"]:
            nm["fol"] += 1.0 / rate["follower"]
            if "follower_dead" not in self.faults:
                pi.follower_tick(t)
        if t >= nm["aim"]:
            nm["aim"] += 1.0 / rate["aim"]
            pi.aim_tick(t)
        if t >= nm["pan"]:
            nm["pan"] += 1.0 / rate["pan"]
            line = pi.bridge_pan_line()
            if line is not None and pi_alive:
                esp.line(line, self.ms)
        if t >= nm["gate"]:
            nm["gate"] += 1.0 / rate["gate"]
            if "gate_dead" not in self.faults:
                pi.gate_tick(t)
        if t >= nm["cmd"]:
            nm["cmd"] += 1.0 / rate["cmd"]
            if pi_alive:
                esp.line(pi.bridge_cmd_line(t), self.ms)
        # ---- ESP32 and drivetrain -------------------------------------------------------------------------
        if self.remote_input is not None:      # real remote over USB, or the keyboard: stands in for the radio
            esp.remote, fwd, turn = self.remote_input.poll()
            if esp.remote in ("run", "stop"):  # a fresh packet; when lost the firmware keeps the last one
                esp.stick = (fwd, turn)
        if t >= self.next_gyro:
            self.next_gyro += esp.k["GYRO_PERIOD_MS"] / 1000.0
            w = float(d.qvel[5]) + self.gyro_bias + float(self.rng.normal(0.0, 0.05))
            w = {"sign": -w, "stuck": 0.0, "dead": None}.get(self.gyro_fault, w)
            esp.gyro_read(w if w is None else w * esp.k["GYRO_SIGN"], self.ms)
        esp.update(self.ms)
        if t >= self.next["status"]:          # S (+ G when the yaw loop is compiled in) every STATUS_PERIOD_MS
            self.next["status"] += esp.k["STATUS_PERIOD_MS"] / 1000.0
            if pi_alive and not ({"rx_dead", "status_dead"} & self.faults):
                _, s_line, _ = esp.status_lines(None)
                p = s_line.split(",")
                pi.on_s_line(t, float(p[2]), float(p[3]))
                if esp.k["YAW_LOOP_ENABLED"]:
                    g = esp.g_line(self.ms).split(",")
                    pi.on_g_line(t, float(g[1]), g[2] == "1")
        if t >= self.next_pwm:
            self.next_pwm += 1.0 / esp.k["SERVO_FREQ_HZ"]
            self.pwm_hold = esp.wheel_outputs()
        ua, ub = self.pwm_hold
        wl, wr = d.qvel[self.jFL], d.qvel[self.jFR]
        d.ctrl[0] = self.motors[0].torque(ua, wl)
        d.ctrl[1] = self.motors[1].torque(ub, wr)
        d.ctrl[2] = esp.out_pan * pi.pan_half_range         # servo: +1 = half_range_deg left (camera_aim)
        # air drag on the body (Cd*A ~0.03 m^2: 1.5 N at 20 mph), applied at the CG
        vx, vy = d.qvel[0], d.qvel[1]
        sp = math.hypot(vx, vy)
        d.xfrc_applied[self.chassis, 0:2] = (-0.6 * self.cfg.cda * sp * vx, -0.6 * self.cfg.cda * sp * vy)
        self._move_skater(t)
        mujoco.mj_step(m, d)
        self.t += m.opt.timestep
        self.ms = int(round(self.t * 1000))
        self._measure(t)

    def _measure(self, t):
        mt, d, dt = self.metrics, self.d, self.m.opt.timestep
        gap = self._gap()
        mt.duration = self.t
        mt.min_gap = min(mt.min_gap, gap)
        sv = self.skater_state[3]
        if sv > 0.3:
            mt.max_gap_moving = max(mt.max_gap_moving, gap)
        speed = math.hypot(d.qvel[0], d.qvel[1])
        mt.max_speed = max(mt.max_speed, speed)
        if speed > 0.1:
            mt.min_gap_moving = min(mt.min_gap_moving, gap)
        mt.max_current = max(mt.max_current, abs(self.motors[0].current), abs(self.motors[1].current))
        zaxis = d.xmat[self.chassis].reshape(3, 3)[:, 2]
        if zaxis[2] < math.cos(math.radians(45)):
            mt.tipped = True
        for i in range(d.ncon):
            c = d.contact[i]
            pair = {c.geom1, c.geom2}
            if pair & self.robot_geoms and pair & self.skater_geoms:
                mt.skater_contact = True
            if pair & self.robot_geoms and pair & self.obstacle_geoms:
                mt.obstacle_contact = True
        skater_moving = self.skater_state[3] > 0.3
        if self.pi.target_t is None or t - self.pi.target_t > self.pi.target_timeout:
            mt.lost_time += dt
            if skater_moving:
                mt.lost_moving += dt
        if self.esp.failsafe and t > 2.5:
            mt.failsafe_time += dt
        st = self.pi.gate_state or "none"
        mt.gate_time[st] = mt.gate_time.get(st, 0.0) + dt
        # framing: is the skater inside the GoPro's horizontal field of view right now?
        if t > 2.0:
            cmat = d.cam_xmat[self.cam].reshape(3, 3)
            v = d.xpos[self.skater] + np.array([0, 0, 1.0]) - d.cam_xpos[self.cam]
            err = math.atan2(float(v @ -cmat[:, 0]), float(v @ -cmat[:, 2]))
            mt.visible_time += dt
            inside = abs(err) < math.radians(self.cfg.sensors.cam_hfov_deg) / 2 and float(v @ -cmat[:, 2]) > 0
            if inside:
                mt.framed += dt
            if skater_moving:
                mt.visible_moving += dt
                mt.framed_moving += dt if inside else 0.0
        mt.final_gap = gap
        mt.final_speed = speed
        if sv > 1.0 and t >= SIDE_SETTLE and t >= self.next["side"]:
            self.next["side"] = t + 0.05
            sx, sy, sth = self.skater_state[0], self.skater_state[1], self.skater_state[2]
            ax, ay = d.site_xpos[self.anchor][0], d.site_xpos[self.anchor][1]
            mt.side_dist.append(math.hypot(ax - sx, ay - sy))
            a = math.atan2(ay - sy, ax - sx) - (sth + math.pi)
            mt.side_angle.append(math.degrees(math.atan2(math.sin(a), math.cos(a))))
        if self.log is not None and t >= self.next["log"]:
            self.next["log"] += 0.05
            rx, ry, yaw = self.robot_pose()
            sx, sy, sth, sv = self.skater_state
            self.log.append(dict(t=round(t, 3), robot_x=rx, robot_y=ry, robot_yaw=yaw, robot_speed=speed,
                                 skater_x=sx, skater_y=sy, skater_speed=sv, gap=gap,
                                 cmd_lin=self.pi.cmd_raw[0], cmd_ang=self.pi.cmd_raw[1],
                                 gate_lin=self.pi.cmd_vel[0], gate=st, range=self.pi.range_m,
                                 out_l=self.esp.out_a, out_r=self.esp.out_b, failsafe=int(self.esp.failsafe),
                                 i_l=self.motors[0].current, i_r=self.motors[1].current,
                                 pan=float(d.qpos[self.jpan]), uwb_ok=int(not (self.pi.target_t is None or
                                                                                 t - self.pi.target_t > self.pi.target_timeout))))

    def run(self, until=None, callback=None, extend=30.0):
        """Run to the scenario's end; then keep going (up to `extend` s) until the robot has stopped for 1 s,
        so a robot that fell behind gets the chance to catch up and stop before it is judged."""
        hard = (self.sc.duration if until is None else until)
        limit = hard + (extend if until is None else 0.0)
        still = 0.0
        while self.t < limit:
            self.step()
            if callback is not None and callback(self) is False:
                break
            sp = math.hypot(self.d.qvel[0], self.d.qvel[1])
            still = still + self.m.opt.timestep if sp < 0.05 else 0.0
            if self.t >= hard and still >= 1.0:
                break
        return self.metrics
