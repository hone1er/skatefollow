"""honebot_sim/stack.py behaves like the nodes it mirrors (their code is hash-pinned in test_sources.py)."""

import math

import pytest

from honebot_sim.config import load_config
from honebot_sim.stack import PiStack


@pytest.fixture(scope="module")
def cfg():
    return load_config()


def test_rates_are_the_nodes_rates(cfg):
    pi = PiStack(cfg)
    p = cfg.params
    assert pi.rates == dict(follower=p["follower"]["rate_hz"], gate=p["safety_gate"]["rate_hz"],
                            heartbeat=p["esp32_bridge"]["heartbeat_hz"], cmd=p["esp32_bridge"]["cmd_rate_hz"],
                            pan=p["esp32_bridge"]["pan_rate_hz"], aim=p["camera_aim"]["rate_hz"])


def test_r_line_no_echo_is_plus_inf(cfg):
    pi = PiStack(cfg)
    pi.on_r_line(1.0, -1.0, True)
    assert pi.range_m == math.inf and pi.link_ok


def test_bridge_sends_neutral_when_cmd_vel_is_stale(cfg):
    pi = PiStack(cfg)
    assert pi.bridge_cmd_line(0.0) == "C,0.000,0.000"                    # nothing from the gate yet
    pi.cmd_vel, pi.cmd_vel_t = (1.0, 0.0), 1.0
    assert pi.bridge_cmd_line(1.0) != "C,0.000,0.000"
    assert pi.bridge_cmd_line(1.0 + pi.bridge_cmd_timeout + 0.01) == "C,0.000,0.000"


def test_no_pan_line_before_camera_aim_has_run(cfg):
    pi = PiStack(cfg)
    assert pi.bridge_pan_line() is None
    pi.aim_tick(0.05)
    assert pi.bridge_pan_line() == "P,0.000"


def test_uwb_no_fix_publishes_nothing_and_follower_times_out(cfg):
    pi = PiStack(cfg)
    for i in range(5):
        pi.on_uwb_line(0.05 * i, 5.0, 0.0)
    pi.follower_tick(0.25)
    assert pi.cmd_raw[0] > 0.0
    t_fix = pi.target_t
    pi.on_uwb_line(0.3, -1.0, 0.0)                                       # board reports "no fix"
    assert pi.target_t == t_fix
    pi.follower_tick(t_fix + pi.target_timeout + 0.01)
    assert pi.cmd_raw == (0.0, 0.0)


def test_gate_stops_on_stale_range(cfg):
    pi = PiStack(cfg)
    pi.on_r_line(0.0, 2.0, True)
    pi.cmd_raw, pi.cmd_raw_t = (1.0, 0.0), 0.0
    pi.gate_tick(0.01)
    assert pi.cmd_vel[0] > 0.0
    pi.cmd_raw_t = 1.0
    pi.gate_tick(1.0)
    assert pi.cmd_vel == (0.0, 0.0) and pi.gate_state in ("LINK_DOWN", "RANGE_STALE")
