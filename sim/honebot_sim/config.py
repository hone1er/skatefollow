"""Every number the simulator uses, read from the project's own sources so the sim cannot drift from them.

  geometry        cad/fusion/HonebotChassis/HonebotChassis.py  (build_model(): wheels, sensor, mast, camera)
  mass and CG     cad/report/check_report.md                   ("Mass 4.02 kg, CG (-71, 0, 81) mm", written by
                                                                 cad/check_design.py from the same CAD model)
  Pi software     each node's declare_parameter() defaults       (src/honebot_*/..., see NODE_SOURCES)
                  + src/honebot_bringup/config/robot.yaml      (follower, safety_gate, uwb_node, esp32_bridge)
  firmware        firmware/esp32_ultrasonic/esp32_ultrasonic.ino (const values: limits, ramps, timeouts)
  belt ratio      CAD PARAMS pul_w_t / pul_m_t                  (56T / 28T)
  motor           cad/check_design.py drivetrain model           (Kv 370, 0.15 ohm, 30 A, 3S 11.1 V; pinned by
                                                                 sim/tests/test_sources.py)

Only numbers no file holds yet live here (tyre friction, sensor noise, skater behaviour); each says why.
"""

import ast
import importlib.util
import math
import re
import sys
import types
from dataclasses import dataclass, field
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
SRC_PACKAGES = ("honebot_follow", "honebot_safety", "honebot_hardware", "honebot_uwb", "honebot_camera")


def add_src_paths():
    """Make the robot's pure-Python modules importable without ROS (they are plain packages under src/)."""
    for pkg in SRC_PACKAGES:
        p = str(REPO / "src" / pkg)
        if p not in sys.path:
            sys.path.insert(0, p)


add_src_paths()


# ---------------------------------------------------------------------------------------------------------- CAD
def load_cad_model():
    """Run the Fusion script's build_model() without Fusion (the same trick cad/check_design.py uses)."""
    for n in ("adsk", "adsk.core", "adsk.fusion"):
        sys.modules.setdefault(n, types.ModuleType(n))
    path = REPO / "cad" / "fusion" / "HonebotChassis" / "HonebotChassis.py"
    spec = importlib.util.spec_from_file_location("honebot_cad_for_sim", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.build_model()


def load_mass_cg(report=None):
    """(mass kg, (x, y, z) m) from the CAD check report. Raises if the line is missing."""
    report = Path(report or REPO / "cad" / "report" / "check_report.md")
    m = re.search(r"Mass ([\d.]+) kg, CG \((-?[\d.]+), (-?[\d.]+), (-?[\d.]+)\) mm", report.read_text())
    if not m:
        raise ValueError("no 'Mass ... CG (...)' line in %s: run cad/check_design.py" % report)
    return float(m.group(1)), tuple(float(m.group(i)) / 1000.0 for i in (2, 3, 4))


# -------------------------------------------------------------------------------------------------- robot.yaml
def load_robot_yaml(path=None):
    data = yaml.safe_load(Path(path or REPO / "src" / "honebot_bringup" / "config" / "robot.yaml").read_text())
    return {node: v.get("ros__parameters", {}) for node, v in data.items()}


# ------------------------------------------------------------------------------------------ node param defaults
# The ROS nodes the simulator mirrors (honebot_sim/stack.py). Their declare_parameter() defaults are THE defaults:
# the sim reads them from the node source instead of keeping its own copies, then applies robot.yaml on top,
# exactly as `ros2 launch` does.
NODE_SOURCES = {
    "follower": "src/honebot_follow/honebot_follow/follower.py",
    "safety_gate": "src/honebot_safety/honebot_safety/safety_gate.py",
    "uwb_node": "src/honebot_uwb/honebot_uwb/uwb_node.py",
    "esp32_bridge": "src/honebot_hardware/honebot_hardware/esp32_bridge.py",
    "camera_aim": "src/honebot_camera/honebot_camera/camera_aim.py",
    "camera_node": "src/honebot_camera/honebot_camera/camera_node.py",   # only hfov_deg is used (sensors.py)
}


def load_node_defaults(node):
    """{param: default} for every `declare_parameter('name', <literal>)` (or its `p(...)` alias) in the node.
    Parameters whose default is not a literal (follower's `leader_timeout` alias) are skipped."""
    tree = ast.parse((REPO / NODE_SOURCES[node]).read_text())
    out = {}
    for n in ast.walk(tree):
        if not isinstance(n, ast.Call) or len(n.args) < 2:
            continue
        f = n.func
        name = f.attr if isinstance(f, ast.Attribute) else f.id if isinstance(f, ast.Name) else None
        if name not in ("declare_parameter", "p"):
            continue
        if not (isinstance(n.args[0], ast.Constant) and isinstance(n.args[0].value, str)):
            continue
        try:
            out.setdefault(n.args[0].value, ast.literal_eval(n.args[1]))
        except ValueError:
            pass
    return out


def load_node_params(robot_yaml=None):
    """{node: params} as the running robot would have them: node defaults, then robot.yaml on top."""
    y = load_robot_yaml() if robot_yaml is None else robot_yaml
    out = {}
    for node in NODE_SOURCES:
        p = load_node_defaults(node)
        p.update(y.get(node, {}))
        out[node] = p
    return out


# ---------------------------------------------------------------------------------------------------- firmware
FIRMWARE_DIR = REPO / "firmware" / "esp32_ultrasonic"


def load_firmware_constants(path=None):
    """`const <type> NAME = <literal>;` lines of the robot sketch -> {NAME: value} (numbers and bools only)."""
    text = Path(path or FIRMWARE_DIR / "esp32_ultrasonic.ino").read_text()
    out = {}
    for name, val in re.findall(r"^const\s+(?:unsigned\s+long|unsigned|long|float|double|int|bool|uint32_t)\s+"
                                r"([A-Z_][A-Z0-9_]*)\s*=\s*([^;]+);", text, flags=re.M):
        v = val.strip()
        if v in ("true", "false"):
            out[name] = v == "true"
            continue
        try:
            out[name] = float(v.rstrip("fFuUlL")) if re.fullmatch(r"-?[\d.]+[fFuUlL]*", v) else None
        except ValueError:
            out[name] = None
        if out[name] is None:
            del out[name]       # derived expressions (ACCEL_RATE_PER_S = ...) are recomputed by the emulator
    return out


# -------------------------------------------------------------------------------------------- assembled config
@dataclass
class RobotGeometry:
    """Metres, robot frame = base_link: x forward from the FRONT axle, y left, z up from the ground."""
    wheel_r: float
    wheel_w: float
    axle_z: float
    wheelbase: float
    track: float                 # wheel centre to wheel centre
    body_x0: float               # tub extents
    body_x1: float
    body_hw: float
    body_z0: float
    body_z1: float
    mass: float
    cg: tuple
    wheel_mass: float            # wheel + bearings + pulley (CAD hardware masses)
    us_origin: tuple             # HC-SR04 transducer centre
    us_tilt: float               # rad, nose up
    us_half: float               # rad, beam half-angle
    pan_axis_x: float
    lens_z: float
    mast_x: float
    mast_top: float


@dataclass
class MotorParams:
    """One 5010 outrunner per side through the 2:1 belt (cad/check_design.py drivetrain section)."""
    kv: float = 370.0            # rpm/V, MAD 5010 EEE 370KV
    resistance: float = 0.15     # ohm phase-to-phase, typical 5010 [M]
    current_limit: float = 30.0  # A, ESC
    battery_v: float = 11.1      # 3S nominal (11.1-12.6 V)
    ratio: float = 2.0           # wheel pulley / motor pulley teeth; load_config() takes it from the CAD PARAMS
    efficiency: float = 0.9      # belt + bearings
    rotor_inertia: float = 2.5e-5   # kg m^2, ~40 g bell at 25 mm radius [M]
    esc_deadband: float = 0.0    # fraction of throttle the ESC ignores around neutral (measure: first_drive.md)
    no_load_current: float = 0.5 # A, 5010-class outrunner at ~10 V [datasheet-class]
    rolling_crr: float = 0.015   # urethane wheel on asphalt


@dataclass
class SensorParams:
    """Noise and rates. Datasheet-style guesses until the real parts are measured; all are CLI-tunable."""
    us_period: float = 0.060          # s, firmware MEASURE_PERIOD_MS (overwritten from the .ino)
    us_max_range: float = 4.29        # m, ECHO_TIMEOUT_US 25 ms there and back
    us_noise: float = 0.005           # m, measured stdev 1 mm on the bench; 5 mm outdoors
    us_miss: float = 0.02             # chance of no echo on a ping
    uwb_hz: float = 20.0              # MaUWB AoA update rate [datasheet-class]
    uwb_range_noise: float = 0.05     # m (spec +/-10 cm)
    uwb_bearing_noise_deg: float = 3.0
    uwb_dropout: float = 0.03
    uwb_nlos_bias: float = 0.3        # m added when the line of sight is blocked
    uwb_tag_z: float = 1.0            # tag on the skater's belt/chest
    uwb_anchor_z: float = 0.18        # anchor on the mast plate
    cam_hz: float = 10.0              # YOLOv8n on the Pi: 10-11 det/s measured
    cam_latency: float = 0.10         # s, frame -> detection
    cam_p_detect: float = 0.95        # recall measured 100 % indoors; 95 % for motion blur outdoors
    cam_bearing_noise_deg: float = 0.5
    cam_hfov_deg: float = 87.0        # GoPro Linear; load_config() takes camera_node's hfov_deg
    forward_sensor: str = "hcsr04"    # or "tfmini": 12 m, 2 deg beam, 100 Hz (what 12-15 mph needs)


@dataclass
class SimConfig:
    geom: RobotGeometry
    motor: MotorParams
    sensors: SensorParams
    yaml: dict                        # robot.yaml as written (node -> params)
    firmware: dict
    params: dict = field(default_factory=dict)   # node -> node defaults + robot.yaml: what the sim's Pi stack uses
    friction: float = 0.8             # urethane on asphalt slides at ~0.7-0.9 g (cad report); a GUESS until the
    #                                   spin test on the real robot. Scrub in turns depends on it directly.
    cda: float = 0.03                 # m^2, drag area of the body + mast + GoPro (estimate)
    physics_dt: float = 0.002
    remote: str = "run"               # handheld kill switch: run | stop | none (never heard)
    overrides: dict = field(default_factory=dict)


def load_config(**overrides) -> SimConfig:
    M = load_cad_model()
    P = M.info["P"]
    mass, cg = load_mass_cg()
    us = M.info["us"]
    cam = M.info["camera"]
    wheel_mass = 0.0
    for p in M.parts:
        if p["name"] in ("wheel_FL", "spacer_FL"):
            wheel_mass += p["mass"] or 0.0
    wheel_mass += 0.04            # printed wheel pulley (PAHT-CF, from the CAD volume, rounded)
    track = 2 * (P["wheel_inner_y"] + P["wheel_w"] / 2)
    g = RobotGeometry(
        wheel_r=P["wheel_d"] / 2000, wheel_w=P["wheel_w"] / 1000, axle_z=P["axle_z"] / 1000,
        wheelbase=P["wheelbase"] / 1000, track=track / 1000,
        body_x0=P["tub_x0"] / 1000, body_x1=P["tub_x1"] / 1000, body_hw=P["tub_hw"] / 1000,
        body_z0=P["floor_z0"] / 1000, body_z1=(P["tub_top"] + 5) / 1000,
        mass=mass, cg=cg, wheel_mass=wheel_mass,
        us_origin=tuple(v / 1000 for v in us["c"]), us_tilt=math.radians(us["tilt"]), us_half=math.radians(us["half"]),
        pan_axis_x=cam["pan_axis"][0] / 1000, lens_z=cam["lens"][2] / 1000,
        mast_x=M.info["mast"]["x"] / 1000, mast_top=cam["bottom"] / 1000)
    fw = load_firmware_constants()
    sensors = SensorParams(us_period=fw["MEASURE_PERIOD_MS"] / 1000.0,
                           us_max_range=fw["ECHO_TIMEOUT_US"] * 1e-6 * 343.0 / 2.0)
    y = load_robot_yaml()
    params = load_node_params(y)
    sensors.cam_hfov_deg = params["camera_node"]["hfov_deg"]
    cfg = SimConfig(geom=g, motor=MotorParams(esc_deadband=fw["ESC_DEADBAND"], ratio=P["pul_w_t"] / P["pul_m_t"]),
                    sensors=sensors, yaml=y, firmware=fw, params=params)
    for k, v in overrides.items():
        for obj in (cfg, cfg.motor, cfg.sensors):
            if hasattr(obj, k):
                setattr(obj, k, v)
                break
        else:
            cfg.overrides[k] = v
    return cfg
