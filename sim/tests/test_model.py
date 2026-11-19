"""The simulated robot matches the CAD and the firmware; sensors and motor behave as specified."""

import math

import mujoco
import numpy as np
import pytest

from honebot_sim.config import load_config, load_mass_cg
from honebot_sim.model import Motor, build_mjcf
from honebot_sim.sensors import RangeSensor, Uwb


@pytest.fixture(scope="module")
def cfg():
    return load_config()


def test_geometry_from_cad(cfg):
    g = cfg.geom
    assert g.track == pytest.approx(0.277, abs=1e-3)
    assert g.wheelbase == pytest.approx(0.200, abs=1e-3)
    assert g.wheel_r == pytest.approx(0.045)
    assert g.us_origin[2] == pytest.approx(0.103, abs=0.003)
    assert math.degrees(g.us_tilt) == pytest.approx(12.0)


def test_mass_and_cg_match_the_cad_report(cfg):
    mass, cg = load_mass_cg()
    m = mujoco.MjModel.from_xml_string(build_mjcf(cfg))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    robot = [b for b in range(m.nbody) if b not in (0, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "skater"))]
    total = sum(m.body_mass[b] for b in robot)
    com = sum(m.body_mass[b] * d.xipos[b] for b in robot) / total
    assert total == pytest.approx(mass, rel=1e-3)
    assert np.allclose(com, cg, atol=1e-3)


def test_motor_top_speed_matches_firmware_full_scale(cfg):
    v = Motor(cfg.motor).no_load_speed(cfg.geom.wheel_r)
    assert v == pytest.approx(cfg.firmware["FULL_SCALE_MPS"], rel=0.03)


def test_motor_current_limit_and_braking(cfg):
    mo = Motor(cfg.motor)
    mo.torque(1.0, 0.0)
    assert mo.current == pytest.approx(cfg.motor.current_limit)
    t = mo.torque(0.0, 100.0)            # throttle released at speed: complementary PWM brakes
    assert t < 0 and mo.current < 0


def _world(cfg, obstacles=()):
    m = mujoco.MjModel.from_xml_string(build_mjcf(cfg, obstacles))
    d = mujoco.MjData(m)
    d.mocap_pos[0] = (100, 100, 0)
    mujoco.mj_forward(m, d)
    return m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")


def test_hcsr04_sees_the_ground_at_about_195_cm(cfg):
    m, d, ch = _world(cfg)
    rs = RangeSensor(cfg, np.random.default_rng(0))
    rs.miss = 0.0
    r = np.median([rs.measure(m, d, ch) for _ in range(20)])
    assert 1.8 < r < 2.1                 # the CAD check's ground return (beam edge 3 deg below horizontal)


def test_hcsr04_sees_a_box_and_tfmini_sees_far(cfg):
    m, d, ch = _world(cfg, [(1.2, 0.0, 0.2, 1.0, 0.6)])
    rs = RangeSensor(cfg, np.random.default_rng(0))
    rs.miss = 0.0
    assert rs.measure(m, d, ch) == pytest.approx(1.1 - 0.085, abs=0.05)
    m, d, ch = _world(cfg, [(8.0, 0.0, 0.2, 1.0, 0.6)])
    lidar = RangeSensor(cfg, np.random.default_rng(0), kind="tfmini")
    lidar.miss = 0.0
    assert lidar.measure(m, d, ch) == pytest.approx(7.9 - 0.085, abs=0.1)


def test_uwb_noise_and_bearing(cfg):
    m, d, ch = _world(cfg)
    d.mocap_pos[0] = (4.0, 3.0, 0.0)
    mujoco.mj_forward(m, d)
    u = Uwb(cfg, np.random.default_rng(1))
    a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "uwb")
    t = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tag")
    got = [u.measure(m, d, ch, a, t) for _ in range(400)]
    fixes = [g for g in got if g[0] > 0]
    assert 0.9 < len(fixes) / len(got) < 1.0
    rng = np.mean([f[0] for f in fixes])
    brg = np.mean([f[1] for f in fixes])
    assert rng == pytest.approx(math.dist((0.06, 0, cfg.sensors.uwb_anchor_z), (4, 3, cfg.sensors.uwb_tag_z)), abs=0.03)
    assert brg == pytest.approx(math.degrees(math.atan2(3.0, 4.0 - 0.06)), abs=1.0)
