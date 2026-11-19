"""MuJoCo model of the robot (from the CAD numbers), the skater, the ground and obstacles; plus the motor model.

Physics that matter for following, and how they are modelled:
  - mass 4.02 kg at the CAD centre of gravity, 4 wheels on hinge joints, contact friction with the ground
    (skid-steer scrub, sliding, pitch/tip-over all come out of the contact solver, not from formulas);
  - per side ONE motor drives the front wheel; the rear wheel is tied to it 1:1 by a joint equality (the belt);
  - motor = brushed-equivalent DC model of a 370 KV outrunner on an AM32 ESC in bidirectional mode: the PWM
    command sets the VOLTAGE (not the speed), current = (u V - back-EMF) / R limited to the ESC's 30 A, so
    acceleration, top speed and braking (complementary PWM brakes actively) follow from the motor, not from
    a scripted speed;
  - camera pan = hinge with a position-servo actuator.
"""

import math

import numpy as np

ROBOT_GROUP = 1        # geom group of the robot's own geoms (sensor rays skip it)


def _f(*v):
    return " ".join("%.6g" % x for x in v)


def build_mjcf(cfg, obstacles=(), friction=None):
    g = cfg.geom
    mu = cfg.friction if friction is None else friction
    mw = g.wheel_mass
    wheels = [("FL", 0.0, g.track / 2), ("FR", 0.0, -g.track / 2),
              ("RL", -g.wheelbase, g.track / 2), ("RR", -g.wheelbase, -g.track / 2)]
    # chassis body = everything except the 4 wheel assemblies; place its CG so the WHOLE robot's CG is the CAD one
    m_pan = 0.21                       # GoPro 0.158 + servo horn/buckle, on the pan joint
    m_ch = g.mass - 4 * mw - m_pan
    cg = np.array(g.cg)
    wsum = sum(np.array([x, y, g.axle_z]) * mw for _, x, y in wheels)
    wsum = wsum + np.array([g.pan_axis_x, 0.0, g.lens_z]) * m_pan
    cg_ch = (g.mass * cg - wsum) / m_ch
    lx, ly, lz = g.body_x1 - g.body_x0, 2 * g.body_hw, g.body_z1 - g.body_z0
    I = [m_ch / 12 * (ly ** 2 + lz ** 2), m_ch / 12 * (lx ** 2 + lz ** 2), m_ch / 12 * (lx ** 2 + ly ** 2)]
    bx, bz = (g.body_x0 + g.body_x1) / 2, (g.body_z0 + g.body_z1) / 2
    ox, oz = g.us_origin[0], g.us_origin[2]
    wr, whw = g.wheel_r, g.wheel_w / 2
    Iw_axis = 0.5 * mw * wr ** 2
    Iw_cross = mw * (3 * wr ** 2 + g.wheel_w ** 2) / 12
    arm = cfg.motor.rotor_inertia * cfg.motor.ratio ** 2      # motor rotor seen at the front wheel
    # urethane rolling resistance (Crr ~0.015) + bearings, as a constant drag torque per wheel
    rr = cfg.motor.rolling_crr * g.mass * 9.81 / 4 * wr + 0.0005
    # pan servo travel = camera_aim's half_range_deg (the owner sets it from the measured travel), + a hair of slack
    pan_lim = math.radians(cfg.params["camera_aim"]["half_range_deg"]) + 0.015

    out = ['<mujoco model="honebot">',
           '  <compiler angle="radian" autolimits="true"/>',
           '  <option timestep="%g" integrator="implicitfast" cone="elliptic" impratio="10"/>' % cfg.physics_dt,
           '  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="4096"/></visual>',
           '  <asset>',
           '    <texture name="grid" type="2d" builtin="checker" rgb1=".32 .33 .35" rgb2=".29 .30 .32" width="512" height="512"/>',
           '    <material name="asphalt" texture="grid" texrepeat="40 40" reflectance="0"/>',
           '    <texture name="sky" type="skybox" builtin="gradient" rgb1=".55 .7 .9" rgb2=".95 .95 .97" width="256" height="256"/>',
           '  </asset>',
           '  <default>',
           '    <geom friction="%s" condim="3" solref="0.004 1" solimp="0.95 0.99 0.001"/>' % _f(mu, 0.005, 0.0005),
           '  </default>',
           '  <worldbody>',
           '    <light pos="0 0 30" dir="0 0 -1" directional="true" diffuse=".8 .8 .8"/>',
           '    <geom name="ground" type="plane" size="200 200 0.1" material="asphalt"/>']
    for i, ob in enumerate(obstacles):
        out.append('    <geom name="obstacle_%d" type="box" pos="%s" size="%s" euler="0 0 %g" rgba=".95 .45 .1 1"/>'
                   % (i, _f(ob[0], ob[1], ob[4] / 2), _f(ob[2] / 2, ob[3] / 2, ob[4] / 2), ob[5] if len(ob) > 5 else 0.0))
    # robot (body frame = base_link at ground level under the front axle; free joint)
    out += ['    <body name="chassis" pos="0 0 0">',
            '      <freejoint name="root"/>',
            '      <inertial pos="%s" mass="%g" diaginertia="%s"/>' % (_f(*cg_ch), m_ch, _f(*I)),
            '      <geom name="tub" type="box" group="%d" pos="%s" size="%s" rgba=".12 .12 .13 1"/>'
            % (ROBOT_GROUP, _f(bx, 0, bz), _f(lx / 2, ly / 2, lz / 2)),
            '      <geom name="shell" type="box" group="%d" pos="%s" size="%s" rgba="1 .48 .1 1" contype="0" conaffinity="0" mass="0"/>'
            % (ROBOT_GROUP, _f(bx, 0, g.body_z1 - 0.004), _f(lx / 2 + 0.002, ly / 2 + 0.002, 0.005)),
            '      <geom name="mast" type="box" group="%d" pos="%s" size="0.011 0.011 %g" rgba=".1 .1 .1 1" mass="0"/>'
            % (ROBOT_GROUP, _f(g.mast_x, 0, (g.body_z1 + g.mast_top) / 2), (g.mast_top - g.body_z1) / 2),
            '      <geom name="hcsr04" type="box" group="%d" pos="%s" size="0.01 0.023 0.01" rgba=".2 .4 .9 1" mass="0" contype="0" conaffinity="0"/>'
            % (ROBOT_GROUP, _f(ox, 0, oz)),
            '      <site name="us" pos="%s"/>' % _f(*g.us_origin),
            '      <site name="uwb" pos="%s"/>' % _f(cfg.params["uwb_node"]["anchor_x"], 0, cfg.sensors.uwb_anchor_z),
            '      <body name="pan" pos="%s">' % _f(g.pan_axis_x, 0, g.mast_top),
            '        <joint name="pan" type="hinge" axis="0 0 1" range="-%g %g" damping="0.05" armature="0.002"/>' % (pan_lim, pan_lim),
            '        <geom name="gopro" type="box" group="%d" pos="0 0 %g" size="0.017 0.036 0.028" rgba=".15 .15 .15 1" mass="%g"/>'
            % (ROBOT_GROUP, g.lens_z - g.mast_top, m_pan),
            '        <camera name="gopro" pos="%s" xyaxes="0 -1 0 0 0 1" fovy="%g"/>'
            % (_f(0.022, 0, g.lens_z - g.mast_top), math.degrees(2 * math.atan(math.tan(math.radians(cfg.sensors.cam_hfov_deg) / 2) * 9 / 16))),
            '      </body>',
            '      <camera name="chase" pos="-2.2 0 1.1" xyaxes="0 -1 0 0.45 0 1" mode="trackcom"/>']
    for name, x, y in wheels:
        out += ['      <body name="wheel_%s" pos="%s">' % (name, _f(x, y, g.axle_z)),
                '        <joint name="wheel_%s" type="hinge" axis="0 1 0" damping="0.0002" frictionloss="%g" armature="%g"/>'
                % (name, rr, arm if name[0] == "F" else 0.0),
                '        <inertial pos="0 0 0" mass="%g" diaginertia="%s"/>' % (mw, _f(Iw_cross, Iw_axis, Iw_cross)),
                '        <geom name="tyre_%s" type="cylinder" group="%d" size="%g %g" euler="1.5707963 0 0" rgba=".85 .85 .8 1" mass="0"/>'
                % (name, ROBOT_GROUP, wr, whw),
                '      </body>']
    out += ['    </body>',
            # skater: kinematic (mocap) body moved along the scenario path; collides with the robot
            '    <body name="skater" mocap="true" pos="5 0 0">',
            '      <geom name="board" type="box" pos="0 0 0.06" size="0.41 0.11 0.02" rgba=".4 .25 .12 1"/>',
            '      <geom name="legs" type="capsule" fromto="0 0 0.1 0 0 0.95" size="0.14" rgba=".2 .25 .5 1"/>',
            '      <geom name="torso" type="capsule" fromto="0 0 1.0 0 0 1.45" size="0.2" rgba=".85 .2 .2 1"/>',
            '      <geom name="head" type="sphere" pos="0 0 1.65" size="0.11" rgba=".9 .75 .6 1"/>',
            '      <site name="tag" pos="0 0 %g"/>' % cfg.sensors.uwb_tag_z,
            '    </body>',
            '  </worldbody>',
            '  <equality>',
            '    <joint joint1="wheel_RL" joint2="wheel_FL" polycoef="0 1 0 0 0" solref="0.004 1"/>',
            '    <joint joint1="wheel_RR" joint2="wheel_FR" polycoef="0 1 0 0 0" solref="0.004 1"/>',
            '  </equality>',
            '  <actuator>',
            '    <motor name="motor_L" joint="wheel_FL" ctrlrange="-50 50"/>',
            '    <motor name="motor_R" joint="wheel_FR" ctrlrange="-50 50"/>',
            '    <position name="pan_servo" joint="pan" kp="3" kv="0.15" ctrlrange="-%g %g"/>' % (pan_lim, pan_lim),
            '  </actuator>',
            '  <sensor>',
            '    <framepos name="pos" objtype="body" objname="chassis"/>',
            '  </sensor>',
            '</mujoco>']
    return "\n".join(out)


class Motor:
    """AM32 ESC (bidirectional, complementary PWM) + 370 KV outrunner + 2:1 belt, seen at the wheel.

    u: -1..1 throttle from the servo pulse. The ESC applies u * V_batt across the motor, so
        I = clamp((u V - w_m / Kv) / R, +/- I_max),   torque_wheel = Kt I ratio efficiency
    At u = 0 the motor is shorted through the low-side FETs: it brakes in proportion to its speed, as an AM32
    in complementary-PWM mode does."""

    def __init__(self, p):
        self.p = p
        self.kv = p.kv * 2 * math.pi / 60.0          # rad/s per volt
        self.kt = 1.0 / self.kv                      # N m per amp (SI motor constant)
        self.current = 0.0

    def torque(self, u, wheel_omega):
        p = self.p
        if abs(u) < p.esc_deadband:
            u = 0.0
        w_m = wheel_omega * p.ratio
        i = (u * p.battery_v - w_m / self.kv) / p.resistance
        i = max(-p.current_limit, min(p.current_limit, i))
        self.current = i
        # iron/eddy losses: the no-load current makes no torque (opposes rotation)
        if abs(w_m) > 1.0:
            i = i - math.copysign(p.no_load_current, w_m)
        return self.kt * i * p.ratio * p.efficiency

    def no_load_speed(self, wheel_r):
        """m/s at full throttle with no load (the firmware's FULL_SCALE_MPS should match)."""
        return self.p.kv * self.p.battery_v / self.p.ratio * 2 * math.pi / 60.0 * wheel_r
