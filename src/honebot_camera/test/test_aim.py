import math

from honebot_camera.aim import (AimGains, PanController, bearing_to_x_norm, pixel_to_bearing,
                                pixel_to_elevation, vfov_from_hfov)

HFOV = math.radians(90.0)


def test_pixel_bearing_signs_and_edges():
    assert pixel_to_bearing(0.0, HFOV) == 0.0
    assert math.isclose(pixel_to_bearing(-1.0, HFOV), math.radians(45.0))    # left edge -> +45 deg (left)
    assert math.isclose(pixel_to_bearing(1.0, HFOV), -math.radians(45.0))
    assert math.isclose(pixel_to_elevation(-1.0, HFOV), math.radians(45.0))  # top edge -> up


def test_bearing_round_trip():
    for x in (-0.9, -0.3, 0.0, 0.42, 1.0):
        assert math.isclose(bearing_to_x_norm(pixel_to_bearing(x, HFOV), HFOV), x, abs_tol=1e-9)


def test_vfov_for_16_9():
    v = vfov_from_hfov(HFOV, 1920, 1080)
    assert math.isclose(math.degrees(v), 58.7, abs_tol=0.1)


def test_pan_converges_on_a_target_to_the_left():
    c = PanController(AimGains(kp=0.5, deadband=0.0, max_rate=10.0))
    true_bearing = 0.4                      # target 0.4 rad left of the robot's nose
    for i in range(30):
        err = true_bearing - c.pan           # what the camera would measure
        c.update(now=i * 0.1, dt=0.1, bearing_err=err)
    assert math.isclose(c.pan, true_bearing, abs_tol=1e-3)
    assert c.normalized() > 0               # left = positive command


def test_pan_rate_limit_and_range_limit():
    c = PanController(AimGains(kp=1.0, deadband=0.0, max_rate=1.0, half_range=math.radians(45)))
    c.update(now=0.0, dt=0.1, bearing_err=1.0)
    assert math.isclose(c.pan, 0.1)                          # 1 rad/s * 0.1 s
    for i in range(1, 100):
        c.update(now=i * 0.1, dt=0.1, bearing_err=1.0)
    assert math.isclose(c.pan, math.radians(45))
    assert c.normalized() == 1.0
    assert c.normalized(sign=-1.0) == -1.0


def test_deadband_holds_still():
    c = PanController(AimGains(deadband=0.05))
    c.update(0.0, 0.1, bearing_err=0.3)
    before = c.pan
    c.update(0.1, 0.1, bearing_err=0.02)
    assert c.pan == before


def test_lost_target_falls_back_to_uwb_then_recentres():
    g = AimGains(kp=1.0, deadband=0.0, max_rate=10.0, lost_timeout=0.5, recenter_timeout=3.0,
                 recenter_rate=1.0)
    c = PanController(g)
    c.update(0.0, 0.1, bearing_err=0.3)
    assert math.isclose(c.pan, 0.3) and c.tracking(0.0)
    # just lost: hold
    c.update(0.3, 0.1, bearing_err=None, fallback_bearing=-0.5)
    assert math.isclose(c.pan, 0.3)
    # lost > 0.5 s with UWB: move toward UWB at 1 rad/s
    c.update(0.7, 0.1, bearing_err=None, fallback_bearing=-0.5)
    assert math.isclose(c.pan, 0.2) and not c.tracking(0.7)
    # lost but no UWB and under recenter_timeout: hold
    c.update(1.0, 0.1, bearing_err=None)
    assert math.isclose(c.pan, 0.2)
    # lost > 3 s, no UWB: drift back to centre
    c.update(3.2, 0.1, bearing_err=None)
    assert math.isclose(c.pan, 0.1)
    c.update(3.3, 0.5, bearing_err=None)
    assert c.pan == 0.0


def test_pan_controller_ignores_non_finite_inputs_and_zero_half_range():
    c = PanController()
    c.update(0.0, 0.05, bearing_err=math.nan)
    assert c.pan == 0.0
    c.update(1.0, 0.05, bearing_err=None, fallback_bearing=math.nan)
    assert c.pan == 0.0
    c.update(2.0, 0.05, bearing_err=math.inf, fallback_bearing=-math.inf)
    assert c.pan == 0.0
    c.update(math.nan, 0.05, bearing_err=0.2)
    assert c.pan == 0.0
    c0 = PanController(AimGains(half_range=0.0))
    assert c0.normalized() == 0.0
