"""Sensor models: what the robot's software actually gets to see.

HC-SR04  rays fanned over the +/-15 deg beam from the CAD mount (102 mm high, tilted 12 deg up); the nearest hit
         is the echo (the ground ahead at ~1.95 m, an obstacle, the skater). No echo within the 25 ms timeout
         (4.29 m) -> -1, like the firmware. One ping per 60 ms, small noise, occasional lost echo.
TFmini   option for the speed study: 12 m, 2 deg beam, horizontal, 100 Hz.
UWB AoA  range (3-D, anchor to tag) and horizontal bearing, Gaussian noise, dropouts, a range bias and much
         worse bearing when anything blocks the line of sight. Output = the `A,<range>,<bearing_deg>` the
         board prints, fed through uwb_node's own filter code.
Camera   person detector at ~10 Hz with ~0.1 s latency: a detection when the skater is inside the GoPro's
         field of view and not hidden, with bearing noise; misses at the measured rate.
"""

import math

import mujoco
import numpy as np

from .model import ROBOT_GROUP

_GROUPS = np.array([1 if i != ROBOT_GROUP else 0 for i in range(6)], np.uint8)


def ray(m, d, origin, direction, max_dist):
    """Distance to the first non-robot geom along direction (unit), or None. Returns (dist, geom_id)."""
    gid = np.zeros(1, np.int32)
    dist = mujoco.mj_ray(m, d, np.asarray(origin, float), np.asarray(direction, float), _GROUPS, 1, -1, gid)
    if dist < 0 or dist > max_dist:
        return None, -1
    return dist, int(gid[0])


def _body_frame(d, body_id):
    return d.xpos[body_id].copy(), d.xmat[body_id].reshape(3, 3).copy()


class RangeSensor:
    def __init__(self, cfg, rng, kind=None):
        s = cfg.sensors
        self.kind = kind or s.forward_sensor
        g = cfg.geom
        self.origin = np.array(g.us_origin)
        if self.kind == "tfmini":
            self.period, self.max_range, self.noise, self.miss = 0.01, 12.0, 0.02, 0.005
            tilt, half, n_ring = math.radians(1.0), math.radians(1.0), 1   # tilted up by half its beam: never sees flat ground
        else:
            self.period, self.max_range, self.noise, self.miss = s.us_period, s.us_max_range, s.us_noise, s.us_miss
            tilt, half, n_ring = g.us_tilt, g.us_half, 3
        self.rng = rng
        dirs = []
        for k in range(n_ring + 1):                        # concentric rings over the cone, beam edge included
            a = half * k / n_ring
            n = 1 if k == 0 else 8 * k
            for j in range(n):
                ph = 2 * math.pi * j / n
                el, az = tilt + a * math.sin(ph), a * math.cos(ph)
                dirs.append((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
        self.dirs = np.array(dirs)

    def measure(self, m, d, body_id):
        """Distance in metres or -1 (no echo), in the robot's frame of reference."""
        if self.rng.random() < self.miss:
            return -1.0
        pos, R = _body_frame(d, body_id)
        o = pos + R @ self.origin
        best = None
        for v in self.dirs:
            dist, _ = ray(m, d, o, R @ v, self.max_range)
            if dist is not None and (best is None or dist < best):
                best = dist
        if best is None:
            return -1.0
        return max(0.02, best + self.rng.normal(0.0, self.noise))


class Uwb:
    def __init__(self, cfg, rng):
        s = cfg.sensors
        self.s, self.rng = s, rng
        self.period = 1.0 / s.uwb_hz

    def measure(self, m, d, body_id, anchor_site, tag_site):
        """(range_m, bearing_deg) as the AoA anchor prints it (+ = left), or (-1, 0) for no fix."""
        s, rng = self.s, self.rng
        pos, R = _body_frame(d, body_id)
        a = d.site_xpos[anchor_site].copy()
        t = d.site_xpos[tag_site].copy()
        v = t - a
        rng3 = float(np.linalg.norm(v))
        local = R.T @ v
        bearing = math.degrees(math.atan2(local[1], local[0]))
        hit, _ = ray(m, d, a, v / max(rng3, 1e-9), rng3 - 0.35)    # anything between anchor and skater?
        nlos = hit is not None
        if rng.random() < (0.3 if nlos else s.uwb_dropout):
            return -1.0, 0.0
        r = rng3 + rng.normal(0.0, s.uwb_range_noise) + (s.uwb_nlos_bias if nlos else 0.0)
        # AoA arrays get worse toward the edge of their cone: noise grows as 1/cos(bearing)
        sig = s.uwb_bearing_noise_deg * (4.0 if nlos else 1.0) / max(0.25, math.cos(math.radians(bearing)))
        return r, bearing + rng.normal(0.0, sig)


class Detector:
    """Camera + YOLO: returns bearing errors relative to the camera axis, delivered after the latency."""

    def __init__(self, cfg, rng):
        s = cfg.sensors
        self.s, self.rng = s, rng
        self.period = 1.0 / s.cam_hz
        self.half_fov = math.radians(s.cam_hfov_deg) / 2
        self.queue = []                       # (deliver_time, bearing_err)

    def capture(self, now, m, d, pan_body, cam_id, skater_body):
        """Take a frame now; maybe queue a detection."""
        cpos = d.cam_xpos[cam_id].copy()
        cmat = d.cam_xmat[cam_id].reshape(3, 3)
        fwd = -cmat[:, 2]                     # MuJoCo cameras look along -z
        target = d.xpos[skater_body] + np.array([0, 0, 1.0])
        v = target - cpos
        dist = float(np.linalg.norm(v))
        # bearing in the camera's horizontal plane (+ = left)
        left = -cmat[:, 0]
        err = math.atan2(float(v @ left), float(v @ fwd))
        if abs(err) > self.half_fov * 0.97 or float(v @ fwd) <= 0:
            return
        hit, gid = ray(m, d, cpos, v / dist, dist - 0.3)
        if hit is not None:
            return                            # hidden behind an obstacle
        if self.rng.random() > self.s.cam_p_detect:
            return
        self.queue.append((now + self.s.cam_latency, err + math.radians(self.rng.normal(0, self.s.cam_bearing_noise_deg))))

    def ready(self, now):
        out = [e for t, e in self.queue if t <= now]
        self.queue = [(t, e) for t, e in self.queue if t > now]
        return out[-1] if out else None
