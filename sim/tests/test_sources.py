"""One source of truth: every number and every piece of logic the simulator (or another copy) uses must either be
read from its owner or be pinned here against it.

  owner                                      copies checked here
  node declare_parameter() defaults          the pure modules' dataclass defaults (FollowGains, GateParams,
                                             DiffDriveParams, AimGains); the sim reads the node defaults directly
  node callback / timer code (ROS)           honebot_sim/stack.py mirrors it: hashed below, like the sketch's
                                             functions in test_firmware.py
  firmware .ino constants                    robot.yaml (max_wheel_speed, track_width), honebot_remote_bridge's
                                             remote_logic.py constants and ArbiterConfig
  firmware drive::decide (drive_logic.h)     remote_logic.Arbiter (a Python port): compared decision by decision
  cad/check_design.py drivetrain model       sim MotorParams (Kv, resistance, ESC current limit); the belt ratio
                                             is read from the CAD PARAMS
"""

import ast
import ctypes
import hashlib
import math
import random
import re
import sys

import pytest

from honebot_sim.config import (NODE_SOURCES, REPO, load_cad_model, load_config, load_firmware_constants,
                                load_node_defaults, load_robot_yaml)

FW = load_firmware_constants()
sys.path.insert(0, str(REPO / "src" / "honebot_remote_bridge"))


# --------------------------------------------------------------------------------- node defaults, one copy only
def test_node_defaults_are_parsed():
    want = {"follower": {"rate_hz", "target_timeout", "follow_distance", "max_linear"},
            "safety_gate": {"rate_hz", "range_timeout", "cmd_timeout", "brake_decel"},
            "uwb_node": {"anchor_x", "filter_alpha", "bearing_alpha", "max_bearing_deg"},
            "esp32_bridge": {"heartbeat_hz", "cmd_rate_hz", "pan_rate_hz", "cmd_timeout", "track_width"},
            "camera_aim": {"rate_hz", "kp", "half_range_deg", "uwb_timeout", "measurement_max_age", "pan_sign"}}
    for node, keys in want.items():
        assert keys <= set(load_node_defaults(node)), node


def test_sim_uses_node_defaults_with_robot_yaml_on_top():
    cfg = load_config()
    y = load_robot_yaml()
    for node in NODE_SOURCES:
        expect = load_node_defaults(node)
        expect.update(y.get(node, {}))
        assert cfg.params[node] == expect, node
    # a value only robot.yaml sets, and one only the node declares
    assert cfg.params["uwb_node"]["anchor_x"] == y["uwb_node"]["anchor_x"]
    assert cfg.params["safety_gate"]["range_timeout"] == load_node_defaults("safety_gate")["range_timeout"]


def test_stack_rejects_parameters_no_node_declares():
    from honebot_sim.stack import PiStack
    cfg = load_config()
    with pytest.raises(KeyError):
        PiStack(cfg, {"follower": {"max_lineer": 2.0}})
    with pytest.raises(KeyError):
        PiStack(cfg, {"folower": {"max_linear": 2.0}})
    assert PiStack(cfg, {"follower": {"max_linear": 2.0}}).gains.max_linear == 2.0


def _dataclass_vs_node(cls, node, convert=None):
    d = load_node_defaults(node)
    convert = convert or {}
    for name, f in cls.__dataclass_fields__.items():
        pname, fn = convert.get(name, (name, lambda v: v))
        assert pname in d, "%s has no parameter %s for %s.%s" % (node, pname, cls.__name__, name)
        assert fn(d[pname]) == pytest.approx(f.default), "%s.%s default %r != %s's %s %r" % (
            cls.__name__, name, f.default, node, pname, d[pname])


def test_follow_gains_defaults_match_the_follower_node():
    from honebot_follow.control import FollowGains
    _dataclass_vs_node(FollowGains, "follower")


def test_gate_params_defaults_match_the_safety_gate_node():
    from honebot_safety.gate import GateParams
    _dataclass_vs_node(GateParams, "safety_gate")


def test_diffdrive_defaults_match_the_bridge_node():
    from honebot_hardware.diffdrive import DiffDriveParams
    _dataclass_vs_node(DiffDriveParams, "esp32_bridge")


def test_aim_gains_defaults_match_the_camera_aim_node():
    from honebot_camera.aim import AimGains
    rad = math.radians
    _dataclass_vs_node(AimGains, "camera_aim", {"deadband": ("deadband_deg", rad),
                                                "max_rate": ("max_rate_deg_s", rad),
                                                "half_range": ("half_range_deg", rad),
                                                "recenter_rate": ("recenter_rate_deg_s", rad)})


# ------------------------------------------------------------------------ node code mirrored in honebot_sim/stack.py
# If one of these hashes changes, the node changed: update honebot_sim/stack.py (or sim.py's scheduler) to match,
# then put the new hash here.
MIRRORED_NODE_CODE = {
    ("follower", "_on_target"): "953d5b711012",
    ("follower", "_tick"): "31f86476248e",
    ("safety_gate", "_on_cmd"): "47994b036247",
    ("safety_gate", "_on_range"): "f9df425a6044",
    ("safety_gate", "_on_link"): "3116ad3352ce",
    ("safety_gate", "_age"): "b8a62ad45191",
    ("safety_gate", "_tick"): "f21b91f50c9d",
    ("uwb_node", "_handle"): "6438ba5681ae",
    ("uwb_node", "_handle_aoa"): "f7d2ac4b4f2b",
    ("esp32_bridge", "_send_cmd"): "eba39aa0ad05",
    ("esp32_bridge", "_send_pan"): "824f91bee311",
    ("esp32_bridge", "_on_pan"): "9d596d8520c6",
    ("esp32_bridge", "_handle"): "cd519fad2c87",
    ("esp32_bridge", "_publish_range"): "8a01f16cd88b",
    ("camera_aim", "_on_target"): "d4d5fad5c672",
    ("camera_aim", "_on_uwb"): "5e6825906f69",
    ("camera_aim", "_step"): "c51e8c7cf917",
}


def _method_source(node, name):
    src = (REPO / NODE_SOURCES[node]).read_text()
    for n in ast.walk(ast.parse(src)):
        if isinstance(n, ast.FunctionDef) and n.name == name:
            return ast.get_source_segment(src, n)
    raise KeyError("%s has no method %s" % (node, name))


@pytest.mark.parametrize("node,name", sorted(MIRRORED_NODE_CODE))
def test_mirrored_node_code_unchanged(node, name):
    h = hashlib.sha1(_method_source(node, name).encode()).hexdigest()[:12]
    assert h == MIRRORED_NODE_CODE[(node, name)], (
        "%s.%s changed in %s: update sim/honebot_sim/stack.py to match and set its hash to %s in this test"
        % (node, name, NODE_SOURCES[node], h))


# ------------------------------------------------------------------------------------ firmware <-> Pi numbers
def test_bridge_full_scale_matches_firmware():
    """The Pi turns m/s into wheel commands with max_wheel_speed; the firmware turns them back (ramps, yaw loop)
    with FULL_SCALE_MPS."""
    assert load_robot_yaml()["esp32_bridge"]["max_wheel_speed"] == pytest.approx(FW["FULL_SCALE_MPS"])


def test_remote_bridge_constants_match_firmware():
    import honebot_remote_bridge.remote_logic as rl
    assert rl.STICK_DEADBAND == pytest.approx(FW["STICK_DEADBAND"])
    assert rl.MANUAL_LIMIT == pytest.approx(FW["MANUAL_LIMIT"])
    assert rl.MANUAL_TURN_SCALE == pytest.approx(FW["MANUAL_TURN_SCALE"])
    assert rl.FULL_SCALE_MPS == pytest.approx(FW["FULL_SCALE_MPS"])
    assert rl.TRACK_WIDTH == pytest.approx(load_robot_yaml()["esp32_bridge"]["track_width"])
    c = rl.ArbiterConfig()
    assert c.remote_timeout == pytest.approx(FW["REMOTE_TIMEOUT_MS"] / 1000.0)
    assert c.manual_hold == pytest.approx(FW["MANUAL_HOLD_MS"] / 1000.0)
    assert c.stick_deadband == pytest.approx(FW["STICK_DEADBAND"])
    assert c.remote_required == FW["REMOTE_REQUIRED"]


def test_remote_bridge_stick_mixing_matches_compiled_firmware():
    import honebot_remote_bridge.remote_logic as rl
    from honebot_sim.firmware import lib
    L = lib()
    rnd = random.Random(1)
    for _ in range(500):
        x, y = rnd.uniform(-1.1, 1.1), rnd.uniform(-1.1, 1.1)
        assert rl.shape_stick(x) == pytest.approx(L.hb_shape_stick(x, FW["STICK_DEADBAND"]), abs=1e-6)
        a, b = ctypes.c_float(), ctypes.c_float()
        L.hb_arcade_mix(rl.shape_stick(x), rl.shape_stick(y), FW["MANUAL_LIMIT"], FW["MANUAL_TURN_SCALE"],
                        ctypes.byref(a), ctypes.byref(b))
        assert rl.arcade_mix(rl.shape_stick(x), rl.shape_stick(y)) == pytest.approx((a.value, b.value), abs=1e-6)


def test_remote_bridge_arbiter_matches_compiled_drive_decide():
    """remote_logic.Arbiter is a Python port of drive::decide. Drive both with the same random remote / stick /
    Pi-command history (no arming window and the heartbeat always fresh: the Arbiter has neither) and compare
    every decision. Times avoid the exact timeout boundaries, where the two compare with < vs <=."""
    import honebot_remote_bridge.remote_logic as rl
    from honebot_sim.firmware import lib
    L = lib()
    src_name = {0: rl.SRC_NONE, 1: rl.SRC_PI, 2: rl.SRC_MANUAL}
    rem_name = {0: "never", 1: "lost", 2: "stop", 3: "run"}
    cmd_timeout_ms = 500
    for seed in range(20):
        rnd = random.Random(seed)
        arb = rl.Arbiter(rl.ArbiterConfig(cmd_timeout=cmd_timeout_ms / 1000.0))
        st = L.hb_state_new()
        now = 1
        remote = None              # (run, x, y) of the last packet
        remote_ms = pi_ms = None
        try:
            for _ in range(600):
                now += rnd.choice((3, 7, 13, 37, 101, 251))
                if rnd.random() < 0.6:                       # a remote packet arrives
                    run = rnd.random() < 0.8
                    centred = rnd.random() < 0.5
                    x = rnd.uniform(-0.1, 0.1) if centred else rnd.choice((-1, 1)) * rnd.uniform(0.2, 1.0)
                    y = rnd.uniform(-0.1, 0.1) if centred else rnd.uniform(-1.0, 1.0)
                    remote, remote_ms = (run, x, y), now
                if rnd.random() < 0.5:                       # a Pi command arrives
                    pi_ms = now
                if remote_ms is not None and abs((now - remote_ms) - FW["REMOTE_TIMEOUT_MS"]) < 2:
                    continue
                if pi_ms is not None and abs((now - pi_ms) - cmd_timeout_ms) < 2:
                    continue
                run, x, y = remote if remote else (False, 0.0, 0.0)
                r_out, link = ctypes.c_int(), ctypes.c_int()
                src = L.hb_decide(st, int(FW["REMOTE_REQUIRED"]), 0, 10 ** 6, cmd_timeout_ms,
                                  int(FW["REMOTE_TIMEOUT_MS"]), int(FW["MANUAL_HOLD_MS"]), FW["STICK_DEADBAND"],
                                  now, 1, now, int(pi_ms is not None), pi_ms or 0, int(remote is not None),
                                  remote_ms or 0, int(run), x, y, ctypes.byref(r_out), ctypes.byref(link))
                rs = None if remote is None else rl.RemoteState("RUN" if run else "STOP", x, y, False, True, False)
                got = arb.decide(now / 1000.0, rs, None if remote_ms is None else remote_ms / 1000.0,
                                 None if pi_ms is None else pi_ms / 1000.0)
                assert got == (src_name[src], rem_name[r_out.value]), (seed, now)
        finally:
            L.hb_state_free(st)


# ---------------------------------------------------------------------------------------- drivetrain numbers
def test_motor_numbers_match_the_cad_drivetrain_model():
    """cad/check_design.py's chosen drivetrain line: bldc("**5010 370KV ... (chosen)**", kv, R, imax, ratio, ...)."""
    text = (REPO / "cad" / "check_design.py").read_text()
    m = re.search(r'bldc\("\*\*5010[^"]*\(chosen\)\*\*"[^,]*,\s*([\d.]+),\s*([\d.]+),\s*([\d.]+),', text)
    assert m, "chosen drivetrain line not found in cad/check_design.py"
    kv, r, imax = (float(v) for v in m.groups())
    cfg = load_config()
    assert (cfg.motor.kv, cfg.motor.resistance, cfg.motor.current_limit) == (kv, r, imax)
    assert "11.1" in text and cfg.motor.battery_v == 11.1


def test_belt_ratio_comes_from_the_cad():
    P = load_cad_model().info["P"]
    assert load_config().motor.ratio == pytest.approx(P["pul_w_t"] / P["pul_m_t"])
