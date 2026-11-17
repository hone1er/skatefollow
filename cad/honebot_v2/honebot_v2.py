"""honebot V2 — Fusion 360 script (docs/robot_design_v2.md).

A camera robot that films a skater the way a skate filmer does: two belt-driven rear wheels, a servo-steered front
wheel (turns 90 deg for turning in place), a telescoping / folding camera mast with the pan servo, GoPro, vision
camera and UWB anchor on one pan stage, an LD19 lidar on the nose and the HC-SR04 under it.

Coordinates (mm): x forward, y left, z up. Ground z = 0, rear axle at x = 0.

Bodies are named by what to do with them:
  "... (print)"           print it (material in the name or in robot_design_v2.md)
  "... (buy, reference)"  bought part, drawn to check fit; don't print
  "... (hardware)"        screws, bolts, bearings, drawn where they go

Moving joints (Assemble > Drive Joints, or drag):
  rear wheels (each linked to its motor pulley 36:20), steering fork (linked 1:1 to the steering servo pulley),
  front wheel, mast fold hinge, mast telescope (slider), camera pan.

Everything marked MEASURE in P is from a listing or a typical part: measure yours and edit before printing.
Run: Fusion 360 > Utilities > Add-Ins > Scripts > + (pick this folder) > Run.
"""
import math
import traceback

try:
    import adsk.core
    import adsk.fusion
except ImportError:          # lets the layout and checks run outside Fusion
    adsk = None

P = dict(
    # ---- build toggles ---------------------------------------------------------------------------------------
    joints=True,                # components + moving joints + motion links
    hardware=True,              # draw bolts, nuts, bearings, spacer tubes
    belts=True,                 # draw the two drive belts and the steering belt (reference)
    build_lid=True,
    mast_extension=90.0,        # inner mast top above the sleeve top as drawn: 30, 60 or 90 (pin holes); fold at 30

    # ---- print fits (PAHT-CF / PETG on a tuned printer; adjust after a test print) ---------------------------
    fit_press=0.15,             # bearing seats: hole = bearing OD + this (snug press, not a hammer fit)
    fit_shaft=0.15,             # printed shafts in bearing bores: shaft = bore - this (slide fit)
    fit_slide=0.30,             # per side, sliding parts (telescoping mast, hinge knuckle)
    fit_free=0.50,              # per side, parts that must never touch (hinge cheeks, steering sweep is separate)
    fit_servo=0.40,             # per side, servo body pockets
    m2_tap=1.8, m25_tap=2.2, m3_tap=2.6,          # self-tapping pilot holes
    m3_clear=3.4, m4_clear=4.5, m5_clear=5.5, m8_clear=8.4,
    insert_m3=(4.0, 5.7),       # heat-set insert hole: diameter, depth
    insert_m4=(5.6, 8.0),
    insert_m5=(6.4, 9.0),
    min_gap=3.0,                # clearance the checks demand between moving and fixed parts

    # ---- wheels and axles ------------------------------------------------------------------------------------
    wheel=(90.0, 52.0),         # longboard wheel diameter, width (78A), 608 bearings + 10 mm skate spacer inside
    axle_z=45.0,
    wheelbase=280.0,            # rear axle to steering axis
    rear_wheel_inner_y=95.0,    # inner face of each rear wheel
    steer_trail=6.0,            # front axle this far behind the steering axis: it trails if the servo goes limp
    bearing_608=(8.0, 22.0, 7.0),
    bearing_6802=(15.0, 24.0, 5.0),   # steering head (2x)
    spacer_tube=(16.0, 8.4, 11.5),  # OD, ID, nose OD: rear wheel spacer, wall to the inner 608. 16 mm OD spreads the axle
                                # clamp on the printed wall; the 11.5 mm x 1.5 nose touches only the bearing's inner race
                                # (tighten the axle bolt to ~2 N*m + threadlocker)
    axle_nut=(13.0, 6.5),       # M8 nut captured in the axle boss (flats top/bottom), bolt goes in from outside the wheel
    stem_bolt=(8.0, 13.0, 5.0), # M8 low-head socket cap (DIN 7984) down the hollow stem into a nut in the crown: steel
                                # takes the bending from wheel strikes; the 6 x 2.5 magnet sits in its hex socket

    # ---- chassis tub (PAHT-CF) --------------------------------------------------------------------------------
    tub_rear_x=-50.0,
    floor_z=(38.0, 42.0),       # floor underside 38 mm above the ground
    tub_top_z=137.0,
    wall_t=4.0,
    motor_wall_y=(68.0, 75.0),  # rear-zone side walls carry the motors and the axle stubs: 7 mm thick
    front_half_w=77.0,          # outer half width ahead of the rear wheels
    rear_zone_x1=50.0,          # rear zone (motor walls) ends here
    axle_boss=(30.0, 57.0),     # diameter, inner face y (head of the M8 axle bolt bears here)
    corner_r=8.0,
    rear_deck_x1=-22.0,         # full-width deck at the rear carrying the mast hinge (lid starts here)
    rear_deck_z0=125.0,

    # ---- drive (each side): H5055 -> HTD-3M 20T -> 36T printed pulley bolted to the wheel ----------------------
    belt_pitch=3.0,             # HTD-3M
    drive_teeth=(20, 36),
    drive_belt=(75, 10.0),      # closed belt teeth (3M-225), width
    motor_angle_deg=65.0,       # motor centre direction from the axle, above horizontal, forward
    tension_slot=3.0,           # +/- along the belt line
    motor_can_d=49.5,           # Flipsky H5055: body 49.5 x 55.5, 8 mm shaft, 475 g (listing) — MEASURE
    motor_len=55.5,
    motor_shaft=(8.0, 25.0),    # diameter, length past the mounting face — MEASURE
    motor_pcd=30.0,             # 4 x M4 mounting pattern — MEASURE (common 5055 patterns: 25 or 30 mm)
    motor_boss_d=13.0,          # raised bearing boss on the mounting face — MEASURE (slot clears it)
    motor_pocket=3.0,           # the motor face sits in a 3 mm pocket in the wall (locates it, and gains shaft length).
                                # MEASURE: the first 3 mm of the motor at its face must not rotate and no wires exit there
    motor_screw_head=(7.6, 2.2, 1.5),   # M4 button head (ISO 7380): diameter, height, counterbore depth
    motor_pulley=(22.0, 7.0),   # bought HTD-3M 20T 8 mm bore: length, set-screw hub length (hub faces out)
    pulley_gap=2.0,             # wall to wheel pulley (M8 washer)
    wheel_pulley=dict(flange=1.5, teeth_w=11.0, disc_t=5.0, disc_d=62.0, bolt_pcd=46.0, bolt_hole=4.5, bore=17.5),

    # ---- battery (4S 3000-3300 mAh) ----------------------------------------------------------------------------
    battery=(140.0, 45.0, 36.0),    # X, Y, Z — MEASURE your pack
    battery_x0=-45.0,
    battery_slop=1.5,           # per side in the bay
    strap_w=20.0,

    # ---- electronics -------------------------------------------------------------------------------------------
    bulkhead_x=(100.0, 104.0),  # power bulkhead: ESP32 in front, buck + BEC behind, IMU on the back face
    esp32_board=(55.0, 28.0, 1.6),
    pi_board=(85.0, 56.0),
    pi_holes=(58.0, 49.0, 3.5), # pattern X, Y, inset from the board corner
    pi_origin=(116.0, 13.0),    # board corner (x, y), left side, USB/Ethernet end toward the back
    pi_standoff=(6.0, 6.0),     # boss diameter, height
    vesc=(84.0, 58.0, 28.0),    # Flipsky Mini Dual FSESC 4.20 envelope — MEASURE; held by zip ties, so size can vary
    vesc_origin=(118.0, -71.0),
    imu_holes=15.0,

    # ---- steering ----------------------------------------------------------------------------------------------
    steer_belt=(140, 9.0),      # GT2 closed loop 280 mm, 9 mm wide
    gt2_pitch=2.0,
    steer_teeth=48,             # both steering pulleys (1:1); big enough for the horn screws inside the teeth
    servo_horn_pcd=18.0,        # 4 screw holes of the 25T aluminium disc horn — MEASURE
    steer_tension=3.0,          # +/- slide of the steering servo along the belt (M3 bolts + nuts in slots)
    steer_belt_z=(107.5, 118.5),   # toothed band of both steering pulleys
    servo=dict(body=(40.5, 20.0, 38.0), tab_len=54.5, tab_t=2.5, tab_from_top=10.5, hole_pitch=49.5, hole_sep=10.0,
               spline_off=10.0, spline_h=4.0),   # standard servo — MEASURE yours (steering and pan)
    fork_leg=(8.0, 22.0),       # thickness, width
    fork_crown_z=(96.0, 106.0),
    stem_d=15.0,
    stem_collar=(24.0, 6.0),    # printed clamp collar above the upper bearing (OD, height), M3 pinch screw
    steer_magnet=(6.0, 2.5),    # diametric magnet in the stem top, read by an AS5600 on a bridge (steering angle to the ESP32)
    as5600_gap=1.5,             # magnet to sensor chip
    head_z=(121.0, 149.0),      # bearing housing bottom (lower 6802) .. nose deck top (upper 6802)
    nose_deck_z=(137.0, 149.0),
    nose_x1=335.0,
    nose_half_w=48.0,
    cheek_y=(44.0, 48.0),
    hc_sr04=dict(board=(45.0, 20.0), can_d=16.0, can_pitch=26.0, can_h=12.0),
    lidar=dict(size=(38.6, 38.6, 34.8), x=312.0, holes=(28.0, 28.0)),   # LDROBOT LD19 — MEASURE (holes M2.5)

    # ---- camera mast -------------------------------------------------------------------------------------------
    mast_x=-40.0,               # mast centre line
    sleeve=(30.0, 3.0, 150.0),  # outer square, wall, length
    sleeve_z0=156.0,            # sleeve bottom when upright (on the hinge pedestal); high enough that the folded
                                # pan stage clears the lid and the nose
    hinge_pivot=(7.0, 7.0),     # pivot from the sleeve's front face, above its bottom (it folds forward over the lid;
                                # equal values make the front-bottom edge an arc round the pivot)
    hinge_lock=(6.0, 10.0),     # lock bolt from the sleeve's rear face, above its bottom
    inner_mast=(160.0, 16.0),   # length, cable bore diameter
    mast_pin_pitch=30.0,
    mast_min_overlap=60.0,
    head_h=30.0,
    pan_plate=(20.0, 150.0, 5.0),   # reach ahead of the pan axis, Y, thickness (45 mm behind the axis)
    gopro_prongs=dict(t=3.0, gap=3.2, h=15.0, hole=5.3, hole_z=8.5),
    webcam=dict(holes=28.0, lens_d=14.0),           # small USB camera module — MEASURE
    uwb=dict(holes=(52.0, 32.0), board=(60.0, 40.0)),   # Makerfabs MaUWB_AOA Gen2 anchor — MEASURE

    # ---- masses for the CG check (g) ------------------------------------------------------------------------------
    mass=dict(motor=475, battery=330, wheel=100, pi=60, vesc=90, esp32=15, buck_bec=50, servo=60, lidar=45,
              hc_sr04=10, gopro=160, webcam=30, uwb=25, chassis=650, lid=90, fork=60, mast=150, pan_stage=60,
              hardware=200, pulleys_belts=120),
)


# ---------------------------------------------------------------------------
# Geometry (pure Python, no Fusion) — used by the build and by the checks
# ---------------------------------------------------------------------------
def pd(teeth, pitch):
    return teeth * pitch / math.pi


def belt_cc(belt_teeth, pitch, z1, z2):
    """Centre distance of an open belt with this many teeth."""
    L = belt_teeth * pitch
    d1, d2 = pd(z1, pitch), pd(z2, pitch)
    b = L - math.pi * (d1 + d2) / 2
    k = (d2 - d1) ** 2
    return (b + math.sqrt(b * b - 2 * k)) / 4


def rot(pt, c, a_deg):
    a = math.radians(a_deg)
    dx, dy = pt[0] - c[0], pt[1] - c[1]
    return (c[0] + dx * math.cos(a) - dy * math.sin(a), c[1] + dx * math.sin(a) + dy * math.cos(a))


def tire_plan_radius(f, p, z):
    """Largest plan-view distance from the steering axis to the front tire / fork at height z (for any steer angle)."""
    R = p['wheel'][0] / 2
    hw = p['wheel'][1] / 2
    t = p['steer_trail']
    r = 0.0
    if 0 <= z <= 2 * R + 0.01:
        half = math.sqrt(max(0.0, R * R - (z - p['axle_z']) ** 2))
        r = math.hypot(t + half, hw)
    lt, lw = p['fork_leg']
    if p['axle_z'] - lw / 2 <= z <= f['crown_z'][1]:
        r = max(r, math.hypot(t + lw / 2, f['fork_leg_y'][1]))
    if f['crown_z'][0] <= z <= f['crown_z'][1]:
        r = max(r, math.hypot(f['crown_x'], f['fork_leg_y'][1]))
    return r


def layout(p=P):
    f = {}
    R = p['wheel'][0] / 2
    az = p['axle_z']
    f['R'] = R
    # ---- rear drive
    wy0 = p['rear_wheel_inner_y']
    f['rear_wheel_y'] = (wy0, wy0 + p['wheel'][1])
    wp = p['wheel_pulley']
    mw0, mw1 = p['motor_wall_y']
    f['wheel_pulley_y'] = (mw1 + p['pulley_gap'], wy0)
    t0 = f['wheel_pulley_y'][0] + wp['flange']
    f['drive_teeth_y'] = (t0, t0 + wp['teeth_w'])
    f['wheel_disc_y'] = (wy0 - wp['disc_t'], wy0)
    bc = (f['drive_teeth_y'][0] + f['drive_teeth_y'][1]) / 2
    bw = p['drive_belt'][1]
    f['drive_belt_y'] = (bc - bw / 2, bc + bw / 2)
    zm, zw = p['drive_teeth']
    f['pd_motor'], f['pd_wheel'] = pd(zm, p['belt_pitch']), pd(zw, p['belt_pitch'])
    f['ratio'] = zw / zm
    cc = belt_cc(p['drive_belt'][0], p['belt_pitch'], zm, zw)
    f['drive_cc'] = cc
    a = math.radians(p['motor_angle_deg'])
    f['u'] = (math.cos(a), math.sin(a))                           # axle -> motor
    f['motor'] = (cc * math.cos(a), az + cc * math.sin(a))
    f['motor_face_y'] = mw0 + p['motor_pocket']                   # motor face in a pocket in the wall
    f['motor_y'] = (f['motor_face_y'] - p['motor_len'], f['motor_face_y'])
    pl, ph = p['motor_pulley']
    f['motor_pulley_y'] = (bc - (pl - ph) / 2, bc + (pl - ph) / 2 + ph)
    f['motor_shaft_y'] = (f['motor_face_y'], f['motor_face_y'] + p['motor_shaft'][1])
    f['hub_centre_y'] = f['motor_pulley_y'][1] - ph / 2
    # axle bolt: head on the boss's inner face, nyloc outside the wheel
    # axle bolt: from outside the wheel (head + washer outboard), into an M8 nut captured in the boss's inner face
    an_af, an_t = p['axle_nut']
    f['axle_nut_y'] = (p['axle_boss'][1], p['axle_boss'][1] + an_t)          # nut pocket, open to the cavity
    f['axle_washer_y'] = (f['rear_wheel_y'][1], f['rear_wheel_y'][1] + 1.6)
    f['axle_head_y'] = (f['axle_washer_y'][1], f['axle_washer_y'][1] + 5.3)
    need = f['axle_head_y'][0] - (f['axle_nut_y'][0] - 2.0)                  # 2 mm through the nut
    f['rear_axle_bolt'] = int(math.ceil(need / 5.0) * 5)
    f['axle_end_y'] = f['axle_head_y'][0] - f['rear_axle_bolt']
    f['spacer_y'] = (mw1, wy0)
    f['overall_width'] = 2 * f['axle_head_y'][1]
    # ---- chassis
    fz0, fz1 = p['floor_z']
    f['floor_top'] = fz1
    f['tub_x'] = (p['tub_rear_x'], None)
    # ---- steering / front
    xs = p['wheelbase']
    f['steer_x'] = xs
    f['front_axle'] = (xs - p['steer_trail'], az)
    lt, lw = p['fork_leg']
    gap = 2.5
    f['fork_leg_y'] = (p['wheel'][1] / 2 + gap, p['wheel'][1] / 2 + gap + lt)
    f['crown_z'] = p['fork_crown_z']
    f['crown_x'] = 14.0
    f['fork_spacer'] = (p['wheel'][1] / 2 + 0.1, f['fork_leg_y'][0])   # inner-race spacers on the legs
    # sweep: what the turning fork + tire need, by height band
    f['sweep_low'] = max(tire_plan_radius(f, p, z) for z in [i * 0.5 for i in range(0, 220)])
    f['tub_front_x'] = xs - f['sweep_low'] - p['min_gap'] - 0.5     # outer face of the front wall
    f['tub_x'] = (p['tub_rear_x'], f['tub_front_x'])
    sz0, sz1 = p['steer_belt_z']
    f['steer_pd'] = pd(p['steer_teeth'], p['gt2_pitch'])
    f['steer_cc'] = p['steer_belt'][0] * p['gt2_pitch'] / 2 - math.pi * f['steer_pd'] / 2
    f['servo_x'] = xs - f['steer_cc']                              # steering servo spline axis
    sv = p['servo']
    f['servo_body_cx'] = f['servo_x'] - sv['spline_off']           # spline at the front end of the body
    f['servo_top_z'] = sz0 - 1.5 - 2.5 - 3.0 - sv['spline_h']      # flange, adapter, horn, spline
    f['servo_tab_z'] = (f['servo_top_z'] - sv['tab_from_top'] - sv['tab_t'], f['servo_top_z'] - sv['tab_from_top'])
    f['servo_body_z'] = (f['servo_top_z'] - sv['body'][2], f['servo_top_z'])
    f['shelf_z'] = (f['servo_tab_z'][0] - 6.0, f['servo_tab_z'][0])
    hz0, hz1 = p['head_z']
    b6 = p['bearing_6802']
    f['bearing_low_z'] = (hz0, hz0 + b6[2])
    f['bearing_up_z'] = (hz1 - b6[2], hz1)
    sc_od, sc_h = p['stem_collar']
    f['collar_z'] = (hz1 + 0.2, hz1 + 0.2 + sc_h)
    f['stem_z'] = (sz1 + 1.5, f['collar_z'][1] + 0.5)
    sb_d, sb_hd, sb_hh = p['stem_bolt']
    f['stem_bolt_head_z'] = (f['stem_z'][1], f['stem_z'][1] + sb_hh)
    f['crown_nut_z'] = (f['crown_z'][0], f['crown_z'][0] + 6.5)
    f['stem_bolt'] = int(math.ceil((f['stem_z'][1] - f['crown_z'][0]) / 5.0) * 5)     # tip flush with the crown bottom
    f['as5600_z'] = f['stem_bolt_head_z'][1] + p['as5600_gap']        # magnet flush in the bolt's hex socket
    # ---- mast
    sw, st, sl = p['sleeve']
    xm = p['mast_x']
    f['sleeve_x'] = (xm - sw / 2, xm + sw / 2)
    z0 = p['sleeve_z0']
    f['sleeve_z'] = (z0, z0 + sl)
    f['pivot'] = (f['sleeve_x'][1] - p['hinge_pivot'][0], z0 + p['hinge_pivot'][1])
    f['lock'] = (f['sleeve_x'][0] + p['hinge_lock'][0], z0 + p['hinge_lock'][1])
    f['sleeve_bottom_t'] = 18.0                                   # solid bottom block (pivot + lock + cable hole)
    il, ib = p['inner_mast']
    f['inner_w'] = sw - 2 * st - 2 * p['fit_slide']
    ext = p['mast_extension']
    f['inner_top'] = f['sleeve_z'][1] + ext
    f['inner_z'] = (f['inner_top'] - il, f['inner_top'])
    f['inner_overlap'] = f['sleeve_z'][1] - max(f['inner_z'][0], z0 + f['sleeve_bottom_t'])
    f['mast_ext_max'] = il - p['mast_min_overlap']
    f['mast_ext_min'] = il - (sl - f['sleeve_bottom_t'] - 1.0)
    f['head_z'] = (f['inner_top'], f['inner_top'] + p['head_h'])
    ht = f['head_z'][1]
    f['pan_servo_top'] = ht + sv['tab_t'] + sv['tab_from_top']
    f['pan_spline_top'] = f['pan_servo_top'] + sv['spline_h']
    f['horn_z'] = (f['pan_spline_top'], f['pan_spline_top'] + 3.0)
    f['adapter_z'] = (f['horn_z'][1], f['horn_z'][1] + 6.0)
    f['plate_z'] = (f['adapter_z'][1], f['adapter_z'][1] + p['pan_plate'][2])
    f['lens_z'] = f['plate_z'][1] + p['gopro_prongs']['hole_z'] + 30.0   # HERO9: lens ~30 mm above the finger pivot
    f['pan_axis'] = (xm, 0.0)
    # pin holes in the inner mast (every pitch, from the top down), sleeve pin near its top
    f['sleeve_pin_z'] = f['sleeve_z'][1] - 12.0
    # ---- battery bay
    bx, by, bz = p['battery']
    s_ = p['battery_slop']
    f['battery_box'] = (p['battery_x0'], -by / 2, p['battery_x0'] + bx, by / 2)
    f['battery_z'] = (fz1, fz1 + bz)
    f['bay'] = (p['tub_rear_x'] + p['wall_t'] - 0.5, -by / 2 - s_, p['battery_x0'] + bx + s_, by / 2 + s_)
    # ---- lid
    f['lid_z'] = (p['tub_top_z'], p['tub_top_z'] + 3.0)
    f['lid_x'] = (p['rear_deck_x1'], f['tub_front_x'] - 8.0)
    # insert bosses for the lid (x, y)
    xr, xf = p['rear_deck_x1'] + 6.0, f['tub_front_x'] - p['wall_t'] - 6.0
    yi_f = p['front_half_w'] - p['wall_t']
    f['lid_bosses'] = [(xr, s * (mw0 - 3.5)) for s in (1, -1)] + [(p['rear_zone_x1'] + 8.0, s * (yi_f - 3.5)) for s in (1, -1)] + \
                      [(xf, s * (yi_f - 3.5)) for s in (1, -1)]       # r 4.5: each boss sinks 1 mm into its wall
    # ---- mass / CG
    m = p['mass']
    items = []
    mx, mz = f['motor']
    items += [(m['motor'], mx, mz)] * 2
    items += [(m['battery'], f['battery_box'][0] + bx / 2, fz1 + bz / 2)]
    items += [(m['wheel'], 0.0, az)] * 2 + [(m['wheel'], f['front_axle'][0], az)]
    items += [(m['pi'], p['pi_origin'][0] + 42, 60), (m['vesc'], p['vesc_origin'][0] + 42, 56), (m['esp32'], 107, 82),
              (m['buck_bec'], 95, 100), (m['servo'], f['servo_body_cx'], 85), (m['lidar'], p['lidar']['x'], 167),
              (m['hc_sr04'], 330, 124), (m['chassis'], 120, 90), (m['lid'], 95, 138), (m['fork'], xs, 90),
              (m['hardware'], 60, 70), (m['pulleys_belts'], 20, 80)]
    mast_cz = (f['sleeve_z'][0] + f['head_z'][1]) / 2
    items += [(m['mast'], xm, mast_cz), (m['servo'], xm, f['head_z'][1]), (m['pan_stage'], xm, f['plate_z'][1]),
              (m['gopro'], xm, f['lens_z']), (m['webcam'], xm + 20, f['plate_z'][1] + 18), (m['uwb'], xm + 20, f['plate_z'][1] + 22)]
    M = sum(i[0] for i in items)
    f['mass'] = M
    f['cg'] = (sum(i[0] * i[1] for i in items) / M, sum(i[0] * i[2] for i in items) / M)
    f['rear_load'] = 1 - f['cg'][0] / xs
    return f


# ---------------------------------------------------------------------------
# Sketch helpers (shared with the v5 script)
# ---------------------------------------------------------------------------
def rrect_segs(r4, radii):
    """Rectangle (u0, v0, u1, v1) with corner radii (u0v0, u1v0, u1v1, u0v1); 0 = sharp."""
    u0, v0, u1, v1 = r4
    if isinstance(radii, (int, float)):
        radii = (radii,) * 4
    r0, r1, r2, r3 = radii
    k = 1 - 1 / math.sqrt(2)
    raw = [('L', (u0 + r0, v0), (u1 - r1, v0)),
           ('A', (u1 - r1, v0), (u1 - r1 * k, v0 + r1 * k), (u1, v0 + r1)),
           ('L', (u1, v0 + r1), (u1, v1 - r2)),
           ('A', (u1, v1 - r2), (u1 - r2 * k, v1 - r2 * k), (u1 - r2, v1)),
           ('L', (u1 - r2, v1), (u0 + r3, v1)),
           ('A', (u0 + r3, v1), (u0 + r3 * k, v1 - r3 * k), (u0, v1 - r3)),
           ('L', (u0, v1 - r3), (u0, v0 + r0)),
           ('A', (u0, v0 + r0), (u0 + r0 * k, v0 + r0 * k), (u0 + r0, v0))]
    return [s for s in raw if math.hypot(s[-1][0] - s[1][0], s[-1][1] - s[1][1]) > 1e-4]


def capsule_segs(c1, r1, c2, r2):
    """Two circles joined by their outer tangents (a tapered link or a round-ended slot)."""
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy)
    ex, ey = dx / d, dy / d
    nx, ny = -ey, ex
    a = (r1 - r2) / d
    b = math.sqrt(max(0.0, 1 - a * a))
    w1 = (a * ex + b * nx, a * ey + b * ny)
    w2 = (a * ex - b * nx, a * ey - b * ny)
    t1a = (c1[0] + r1 * w1[0], c1[1] + r1 * w1[1]); t2a = (c2[0] + r2 * w1[0], c2[1] + r2 * w1[1])
    t1b = (c1[0] + r1 * w2[0], c1[1] + r1 * w2[1]); t2b = (c2[0] + r2 * w2[0], c2[1] + r2 * w2[1])
    return [('L', t1a, t2a), ('A', t2a, (c2[0] + r2 * ex, c2[1] + r2 * ey), t2b),
            ('L', t2b, t1b), ('A', t1b, (c1[0] - r1 * ex, c1[1] - r1 * ey), t1a)]


def slot_segs(rect):
    """Round-ended slot filling a bounding box, along its long side."""
    u0, v0, u1, v1 = rect
    if u1 - u0 >= v1 - v0:
        r = (v1 - v0) / 2; v = (v0 + v1) / 2
        return capsule_segs((u0 + r, v), r, (u1 - r, v), r)
    r = (u1 - u0) / 2; u = (u0 + u1) / 2
    return capsule_segs((u, v0 + r), r, (u, v1 - r), r)


def poly_segs(pts):
    return [('L', pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]


def hex_pts(c, af, rot=0.0):
    r = af / math.sqrt(3)
    return [(c[0] + r * math.cos(math.radians(rot + 60 * i)), c[1] + r * math.sin(math.radians(rot + 60 * i))) for i in range(6)]


# ---------------------------------------------------------------------------
# Fusion (API works in cm)
# ---------------------------------------------------------------------------
def cm(v):
    return v / 10.0


def _pt(sk, x, y, z=0.0):
    """Model mm -> sketch space, snapped onto the sketch plane."""
    pt = sk.modelToSketchSpace(adsk.core.Point3D.create(cm(x), cm(y), cm(z)))
    return adsk.core.Point3D.create(pt.x, pt.y, 0.0)


def _profiles(sk):
    c = adsk.core.ObjectCollection.create()
    for pr in sk.profiles:
        c.add(pr)
    return c


def _arc_points(seg, n=12):
    """Points along a 3-point arc segment (model u, v), start to end inclusive."""
    (x1, y1), (x2, y2), (x3, y3) = seg[1], seg[2], seg[3]
    d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    ux = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2)) / d
    uy = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1)) / d
    r = math.hypot(x1 - ux, y1 - uy)
    a1, a2, a3 = (math.atan2(y - uy, x - ux) for x, y in (seg[1], seg[2], seg[3]))
    d2 = (a2 - a1) % (2 * math.pi); d3 = (a3 - a1) % (2 * math.pi)
    sweep = d3 if d2 < d3 else d3 - 2 * math.pi
    return [(ux + r * math.cos(a1 + sweep * k / n), uy + r * math.sin(a1 + sweep * k / n)) for k in range(n + 1)]


def _draw_path(sk, segs, m):
    """Draw a closed outline; consecutive curves share sketch points so it forms a profile.
    Arcs are true 3-point arcs; if Fusion refuses one, that arc becomes a fine polyline."""
    lines = sk.sketchCurves.sketchLines
    arcs = sk.sketchCurves.sketchArcs
    first = prev = None
    n = len(segs)

    def pick(ent, tgt):
        a_, b_ = ent.startSketchPoint, ent.endSketchPoint
        return (b_, a_) if b_.geometry.distanceTo(tgt) <= a_.geometry.distanceTo(tgt) else (a_, b_)

    for i, seg in enumerate(segs):
        tgt = _pt(sk, *m(*seg[-1]))
        p0 = prev if prev is not None else _pt(sk, *m(*seg[1]))
        p1 = first if (i == n - 1 and first is not None) else tgt
        if seg[0] == 'L':
            end, start = pick(lines.addByTwoPoints(p0, p1), tgt)
        else:
            try:
                end, start = pick(arcs.addByThreePoints(p0, _pt(sk, *m(*seg[2])), p1), tgt)
            except Exception:
                q = _arc_points(seg)
                ln = lines.addByTwoPoints(p0, _pt(sk, *m(*q[1])))
                start = ln.startSketchPoint
                for pt_ in q[2:-1]:
                    ln = lines.addByTwoPoints(ln.endSketchPoint, _pt(sk, *m(*pt_)))
                ln = lines.addByTwoPoints(ln.endSketchPoint, p1)
                end = ln.endSketchPoint
        if first is None:
            first = start
        prev = end


LAST_FEATURE = ['(none yet)']   # reported if the script fails, so the failing feature is named


class Builder:
    def __init__(self, root):
        self.root = root
        self.ext = root.features.extrudeFeatures
        self.planes = root.constructionPlanes
        self._plane_cache = {}
        O = adsk.fusion.FeatureOperations
        self.ops = dict(new=O.NewBodyFeatureOperation, join=O.JoinFeatureOperation, cut=O.CutFeatureOperation)

    # -- planes -------------------------------------------------------------
    def plane(self, axis, dist, name=None):
        """Plane normal to `axis` ('x', 'y' or 'z') at model coordinate `dist` (mm)."""
        base = {'x': self.root.yZConstructionPlane, 'y': self.root.xZConstructionPlane,
                'z': self.root.xYConstructionPlane}[axis]
        if abs(dist) < 1e-9:
            return base
        key = (axis, round(dist, 4))
        if key in self._plane_cache:
            return self._plane_cache[key]

        def make(d):
            pin = self.planes.createInput()
            pin.setByOffset(base, adsk.core.ValueInput.createByReal(cm(d)))
            return self.planes.add(pin)
        pl = make(dist)
        o = pl.geometry.origin
        got = {'x': o.x, 'y': o.y, 'z': o.z}[axis] * 10.0
        if abs(got - dist) > 0.01:          # Fusion's offset direction isn't guaranteed: flip
            pl.deleteMe()
            pl = make(-dist)
        pl.name = name or f'{axis.upper()} = {dist:.1f}'
        pl.isLightBulbOn = False
        self._plane_cache[key] = pl
        return pl

    # -- sketches -------------------------------------------------------------
    def sketch(self, name, plane, m, paths=(), circles=()):
        LAST_FEATURE[0] = f'{self.root.name} / {name}'
        sk = self.root.sketches.add(plane)
        sk.name = name
        sk.isComputeDeferred = True
        for segs in paths:
            _draw_path(sk, segs, m)
        for (u, v, r) in circles:
            sk.sketchCurves.sketchCircles.addByCenterRadius(_pt(sk, *m(u, v)), cm(r))
        sk.isComputeDeferred = False
        return sk

    def _finish(self, op, feat, name):
        if op == 'new':
            body = feat.bodies.item(0)
            body.name = name
            return body
        return None

    # -- extrudes on the XY plane between z0 and z1 -------------------------
    def xy(self, op, name, z0, z1, paths=(), circles=(), body=None, rects=()):
        paths = list(paths) + [rrect_segs(r, 0.0) for r in rects]
        sk = self.sketch(name, self.root.xYConstructionPlane, lambda u, v: (u, v, 0.0), paths, circles)
        inp = self.ext.createInput(_profiles(sk), self.ops[op])
        inp.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cm(z1 - z0)))
        if z0:
            inp.startExtent = adsk.fusion.OffsetStartDefinition.create(adsk.core.ValueInput.createByReal(cm(z0)))
        if body is not None and op != 'new':
            inp.participantBodies = [body]
        return self._finish(op, self.ext.add(inp), name)

    # -- extrudes on a side plane (normal to X or Y) between a0 and a1 ------
    def side(self, op, name, axis, a0, a1, paths=(), circles=(), body=None, rects=()):
        """axis 'y': sketch (u, v) = (x, z), extruded from y=a0 to y=a1.
        axis 'x': sketch (u, v) = (y, z), extruded from x=a0 to x=a1."""
        mid, t = (a0 + a1) / 2, abs(a1 - a0)
        pl = self.plane(axis, mid)
        m = (lambda u, v: (u, mid, v)) if axis == 'y' else (lambda u, v: (mid, u, v))
        paths = list(paths) + [rrect_segs(r, 0.0) for r in rects]
        sk = self.sketch(name, pl, m, paths, circles)
        inp = self.ext.createInput(_profiles(sk), self.ops[op])
        inp.setSymmetricExtent(adsk.core.ValueInput.createByReal(cm(t)), True)
        if body is not None and op != 'new':
            inp.participantBodies = [body]
        return self._finish(op, self.ext.add(inp), name)


# ---------------------------------------------------------------------------
# Bodies
# ---------------------------------------------------------------------------
class Assembly:
    """Components + as-built joints, so the mechanism can be moved in Fusion (Assemble > Drive Joints)."""

    def __init__(self, root, enabled=True):
        self.root = root
        self.enabled = enabled
        self.occ = {}
        self.notes = []
        self.joints = {}

    def comp(self, name):
        """New component at the origin; returns (occurrence, Builder for it)."""
        occ = self.root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        try:
            occ.component.name = name
        except Exception:
            pass
        self.occ[name] = occ
        return occ, Builder(occ.component)

    @staticmethod
    def circle_edge(body, center, r):
        """The circular edge of `body` with this centre (mm, model) and radius (mm)."""
        c = adsk.core.Point3D.create(cm(center[0]), cm(center[1]), cm(center[2]))
        best = None
        for e in body.edges:
            g = e.geometry
            if g.objectType != adsk.core.Circle3D.classType():
                continue
            if abs(g.radius - cm(r)) < 0.002:
                d = g.center.distanceTo(c)
                if best is None or d < best[0]:
                    best = (d, e)
        if best is None:
            raise ValueError(f'no circular edge r={r} on {body.name}')
        return best[1]

    def revolute(self, name, moving, fixed, body, body_occ, center_xz_or_xyz, r, axis_point=None):
        if not self.enabled:
            return None
        try:
            e = self.circle_edge(body, center_xz_or_xyz, r).createForAssemblyContext(body_occ)
            geo = adsk.fusion.JointGeometry.createByCurve(e, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
            inp = self.root.asBuiltJoints.createInput(moving, fixed, geo)
            inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
            j = self.root.asBuiltJoints.add(inp)
            j.name = name
            self.joints[name] = j
            return j
        except Exception as ex:   # keep building; report at the end
            self.notes.append(f'joint "{name}" not created ({ex})')
            return None

    def rigid(self, name, a, b):
        if not self.enabled:
            return None
        try:
            inp = self.root.asBuiltJoints.createInput(a, b, None)
            inp.setAsRigidJointMotion()
            j = self.root.asBuiltJoints.add(inp)
            j.name = name
            return j
        except Exception as ex:
            self.notes.append(f'joint "{name}" not created ({ex})')
            return None

    def link(self, name, j1, j2, v1_deg, v2_deg):
        """Motion link j1 -> j2 (Fusion API since Nov 2025). Falls back to a note with manual steps."""
        if not (self.enabled and j1 and j2):
            return
        try:
            mi = self.root.motionLinks.createInput(j1, j2)
            mi.valueOne = adsk.core.ValueInput.createByString(f'{v1_deg} deg')
            mi.valueTwo = adsk.core.ValueInput.createByString(f'{v2_deg} deg')
            ml = self.root.motionLinks.add(mi)
            ml.name = name
        except Exception as ex:
            self.notes.append(f'motion link "{name}" not created ({ex}). Add it by hand: Assemble > Motion Link, '
                              f'pick the hip joint then the knee joint, {v1_deg} deg : {v2_deg} deg, tick Reverse if the '
                              'axle swings forward/back instead of staying under the hip')




# ---------------------------------------------------------------------------
# V2 helpers
# ---------------------------------------------------------------------------
def teeth_grooves(c, teeth, pitch, offset, groove_r, inset):
    """Belt grooves approximated by circles on the pulley rim (print, test with your belt)."""
    od = pd(teeth, pitch) - 2 * offset
    rc = od / 2 - inset
    return od, [(c[0] + rc * math.cos(2 * math.pi * i / teeth), c[1] + rc * math.sin(2 * math.pi * i / teeth), groove_r)
                for i in range(teeth)]


HTD3M = dict(offset=0.381, groove_r=0.9, inset=0.35)
GT2 = dict(offset=0.254, groove_r=0.6, inset=0.15)


def Y(s, a0, a1):
    """Mirror a y-range to the right side (s = -1)."""
    return (a0, a1) if s > 0 else (-a1, -a0)


def rect_c(cx, cy, w, h):
    return (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def hw_cyl_y(b, name, x, z, y0, y1, r):
    return b.side('new', name, 'y', y0, y1, circles=[(x, z, r)])


def hw_hex_y(b, name, x, z, y0, y1, af):
    return b.side('new', name, 'y', y0, y1, paths=[poly_segs(hex_pts((x, z), af, 30.0))])


def hw_cyl_z(b, name, x, y, z0, z1, r):
    return b.xy('new', name, z0, z1, circles=[(x, y, r)])


def belt_ring_y(b, name, y0, y1, c1, r1, c2, r2):
    ring = b.side('new', name, 'y', y0, y1, paths=[capsule_segs(c1, r1 + 1.6, c2, r2 + 1.6)])
    b.side('cut', 'Belt inside', 'y', y0 - 1, y1 + 1, body=ring, paths=[capsule_segs(c1, r1 - 0.8, c2, r2 - 0.8)])
    return ring


def belt_ring_z(b, name, z0, z1, c1, r1, c2, r2):
    ring = b.xy('new', name, z0, z1, paths=[capsule_segs(c1, r1 + 1.4, c2, r2 + 1.4)])
    b.xy('cut', 'Belt inside', z0 - 1, z1 + 1, body=ring, paths=[capsule_segs(c1, r1 - 0.6, c2, r2 - 0.6)])
    return ring


class Assembly2(Assembly):
    def slider(self, name, moving, fixed, body, body_occ, center, r):
        if not self.enabled:
            return None
        try:
            e = self.circle_edge(body, center, r).createForAssemblyContext(body_occ)
            geo = adsk.fusion.JointGeometry.createByCurve(e, adsk.fusion.JointKeyPointTypes.CenterKeyPoint)
            inp = self.root.asBuiltJoints.createInput(moving, fixed, geo)
            inp.setAsSliderJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
            j = self.root.asBuiltJoints.add(inp)
            j.name = name
            self.joints[name] = j
            return j
        except Exception as ex:
            self.notes.append(f'joint "{name}" not created ({ex})')
            return None

    def link(self, name, j1, j2, v1_deg, v2_deg):
        if not (self.enabled and j1 and j2):
            return
        try:
            mi = self.root.motionLinks.createInput(j1, j2)
            mi.valueOne = adsk.core.ValueInput.createByString(f'{v1_deg} deg')
            mi.valueTwo = adsk.core.ValueInput.createByString(f'{v2_deg} deg')
            ml = self.root.motionLinks.add(mi)
            ml.name = name
        except Exception as ex:
            self.notes.append(f'motion link "{name}" not created ({ex}). Add it by hand: Assemble > Motion Link, '
                              f'{v1_deg} deg : {v2_deg} deg')


# ---------------------------------------------------------------------------
# Chassis (grounded): tub, rear deck + hinge, bulkhead, mounts, nose, steering head
# ---------------------------------------------------------------------------
def build_chassis(b, f, p):
    wt = p['wall_t']
    fz0, fz1 = p['floor_z']
    top = p['tub_top_z']
    xr, xf = f['tub_x']
    mw0, mw1 = p['motor_wall_y']
    hw = p['front_half_w']
    rz1 = p['rear_zone_x1']
    cr = p['corner_r']
    az = p['axle_z']
    mx, mz = f['motor']
    ux, uz = f['u']
    sv = p['servo']
    xs = f['steer_x']
    xm = p['mast_x']

    # ---- shell: solid, then hollowed (interior features are joined after the cavity cuts)
    tub = b.xy('new', 'Chassis tub (print, PAHT-CF)', fz0, top, paths=[rrect_segs((xr, -mw1, rz1, mw1), (cr, 0, 0, cr))])
    b.xy('join', 'Tub front zone', fz0, top, body=tub, paths=[rrect_segs((rz1 - 2, -hw, xf, hw), (0, cr, cr, 0))])
    b.xy('cut', 'Cavity (rear zone)', fz1, top + 1, body=tub, paths=[rrect_segs((xr + wt, -mw0, rz1 + 2, mw0), 3.0)])
    b.xy('cut', 'Cavity (front zone)', fz1, top + 1, body=tub, paths=[rrect_segs((rz1, -(hw - wt), xf - wt, hw - wt), 3.0)])

    # ---- joins ------------------------------------------------------------------------------------------------
    ab_d, ab_y = p['axle_boss']
    for s in (1, -1):
        b.side('join', 'Axle boss', 'y', *Y(s, ab_y, mw0 + 0.5), body=tub,
               paths=[rrect_segs((-ab_d / 2, fz1 - 0.5, ab_d / 2, az + ab_d / 2), (0, 0, ab_d / 2, ab_d / 2))])
        b.side('join', 'Axle boss foot (room for the captured nut)', 'y', *Y(s, ab_y, mw1), body=tub,
               rects=[(-ab_d / 2, az - p['axle_nut'][0] / 2 - 2.5, ab_d / 2, fz0 + 0.5)])
    # battery guide rails
    bay = f['bay']
    for s in (1, -1):
        ry0, ry1 = Y(s, bay[3], bay[3] + 2.5)
        b.xy('join', 'Battery guide rail', fz1 - 0.5, fz1 + 8, body=tub, rects=[(bay[0] + 2, ry0, bay[2], ry1)])
    # rear deck (full width) carrying the mast hinge
    b.xy('join', 'Rear deck (mast hinge base)', p['rear_deck_z0'], top, body=tub,
         rects=[(xr + 1, -mw0 - 0.5, p['rear_deck_x1'], mw0 + 0.5)])
    sx0, sx1 = f['sleeve_x']
    sw = p['sleeve'][0]
    b.xy('join', 'Hinge pedestal', top - 0.5, p['sleeve_z0'], body=tub, rects=[(sx0 - 1, -sw / 2, sx1 + 3, sw / 2)])
    cy0 = sw / 2 + p['fit_free']
    for s in (1, -1):
        b.side('join', 'Hinge cheek', 'y', *Y(s, cy0, cy0 + 7.0), body=tub,
               paths=[rrect_segs((sx0 - 3, top - 0.5, sx1 + 3, f['lock'][1] + 9.0), (0, 0, 7.0, 7.0))])
    # door insert bosses on the rear wall
    for s in (1, -1):
        b.side('join', 'Door insert boss', 'x', xr + wt - 0.5, xr + wt + 7.0, body=tub, circles=[(s * 33.0, 62.0, 4.5)])
    # power bulkhead: ESP32 in front, buck + BEC + IMU behind
    bx0, bx1 = p['bulkhead_x']
    b.side('join', 'Power bulkhead', 'x', bx0, bx1, body=tub, rects=[(-(hw - wt) - 0.5, fz1 - 0.5, hw - wt + 0.5, 118.0)])
    # ESP32: stands in a groove on its antenna (short) edge, a zip tie round the top; header pins face forward, free
    ew = p['esp32_board'][1]
    eb0 = -10.0 - ew
    b.side('join', 'ESP32 foot', 'x', bx1 - 0.5, bx1 + 9.0, body=tub, rects=[(eb0 - 3.0, fz1 - 0.5, eb0 + ew + 3.0, 52.0)])
    imh = p['imu_holes'] / 2
    b.side('join', 'IMU bosses', 'x', bx0 - 6.0, bx0 + 0.5, body=tub, circles=[(-imh, 110.0, 3.5), (imh, 110.0, 3.5)])
    # Pi 5 standoffs
    pox, poy = p['pi_origin']
    hx_, hy_, hin = p['pi_holes']
    pi_h = [(pox + hin + i * hx_, poy + hin + j * hy_) for i in (0, 1) for j in (0, 1)]
    sd, sh = p['pi_standoff']
    b.xy('join', 'Pi 5 standoffs', fz1 - 0.5, fz1 + sh, body=tub, circles=[(x, y, sd / 2) for x, y in pi_h])
    # steering servo shelf, cantilevered from the front wall
    scx = f['servo_body_cx']
    tl = sv['tab_len']
    sz0_, sz1_ = f['shelf_z']
    ts = p['steer_tension']
    b.xy('join', 'Steering servo shelf', sz0_, sz1_, body=tub, rects=[(scx - tl / 2 - ts - 3.0, -14.0, xf - wt + 1.0, 14.0)])
    for s in (1, -1):
        ry0, ry1 = Y(s, 11.2, 14.0)
        b.xy('join', 'Shelf rib', sz1_ - 0.5, sz1_ + 8.0, body=tub, rects=[(scx - tl / 2 - ts - 3.0, ry0, xf - wt + 1.0, ry1)])
    # insert bosses inside the front wall for the bolt-on nose (4 x M4)
    nose_bolts = [(sy * 32.0, z_) for sy in (1, -1) for z_ in (110.0, 130.0)]
    f['nose_bolts'] = nose_bolts
    b.side('join', 'Nose insert bosses', 'x', xf - wt - 8.0, xf - wt + 0.5, body=tub, circles=[(y_, z_, 5.0) for y_, z_ in nose_bolts])
    # lid insert bosses
    b.xy('join', 'Lid insert bosses', top - 14.0, top, body=tub, circles=[(x, y, 4.5) for x, y in f['lid_bosses']])

    # ---- cuts (after every join) -------------------------------------------------------------------------------
    # rear axle stubs
    for s in (1, -1):
        b.side('cut', 'Rear axle hole (M8)', 'y', *Y(s, ab_y - 1, mw1 + 1), body=tub, circles=[(0.0, az, p['m8_clear'] / 2)])
        b.side('cut', 'Captured axle nut (M8, flats top/bottom)', 'y', *Y(s, ab_y - 1, f['axle_nut_y'][1]), body=tub,
               paths=[poly_segs(hex_pts((0.0, az), p['axle_nut'][0] + 0.25, 0.0))])
        b.side('cut', 'Axle bolt tail relief', 'y', *Y(s, f['axle_end_y'] - 2.0, ab_y + 0.5), body=tub, circles=[(0.0, az, 5.0)])
    # motor mounts: slotted along the belt for tension
    sl = p['tension_slot']
    pcd = p['motor_pcd'] / 2
    a0 = math.degrees(math.atan2(uz, ux)) + 45.0     # screws 45 deg off the belt line
    holes = [(mx + pcd * math.cos(math.radians(a0 + 90 * i)), mz + pcd * math.sin(math.radians(a0 + 90 * i))) for i in range(4)]
    f['motor_screws'] = holes

    def slot(c, r):
        return capsule_segs((c[0] - sl * ux, c[1] - sl * uz), r, (c[0] + sl * ux, c[1] + sl * uz), r)
    hd, hh, hc = p['motor_screw_head']
    for s in (1, -1):
        b.side('cut', 'Motor face pocket (locates the motor, slotted for tension)', 'y', *Y(s, mw0 - 1, f['motor_face_y']), body=tub,
               paths=[slot((mx, mz), p['motor_can_d'] / 2 + 0.5)])
        b.side('cut', 'Motor screw slots (M4)', 'y', *Y(s, mw0 - 1, mw1 + 1), body=tub,
               paths=[slot(h, p['m4_clear'] / 2) for h in holes] + [slot((mx, mz), p['motor_boss_d'] / 2 + 0.5)])
        b.side('cut', 'Motor screw heads (button head counterbore)', 'y', *Y(s, mw1 - hc, mw1 + 1), body=tub,
               paths=[slot(h, hd / 2 + 0.4) for h in holes])
    # hinge: pivot (M5) and lock (M4) through both cheeks, cable hole down through the pedestal and deck
    b.side('cut', 'Hinge pivot (M5)', 'y', -cy0 - 8, cy0 + 8, body=tub, circles=[(f['pivot'][0], f['pivot'][1], p['m5_clear'] / 2)])
    b.side('cut', 'Hinge lock (M4)', 'y', -cy0 - 8, cy0 + 8, body=tub, circles=[(f['lock'][0], f['lock'][1], p['m4_clear'] / 2)])
    # battery door opening + inserts, power switch, rear exhaust vents
    b.side('cut', 'Battery door opening', 'x', xr - 1, xr + wt + 1, body=tub, rects=[(-25.5, fz1, 25.5, fz1 + 40.0)])
    ins_d, ins_l = p['insert_m3']
    b.side('cut', 'Door inserts (M3)', 'x', xr - 1, xr + ins_l, body=tub, circles=[(33.0, 62.0, ins_d / 2), (-33.0, 62.0, ins_d / 2)])
    b.side('cut', 'Power switch (12 mm)', 'x', xr - 1, xr + wt + 1, body=tub, circles=[(48.0, 104.0, 6.1)])
    b.side('cut', 'Rear exhaust vents', 'x', xr - 1, xr + wt + 1, body=tub,
           paths=[slot_segs((y_, 92.0, y_ + 4.0, 118.0)) for y_ in (-60.0, -50.0, -40.0, -30.0, -20.0, 16.0, 26.0)])
    # battery strap slots (velcro under the floor, round the pack)
    sw_ = p['strap_w']
    b.xy('cut', 'Battery strap slots', fz0 - 1, fz1 + 1, body=tub,
         rects=[(x0, Y(s, bay[3] + 3.0, bay[3] + 6.5)[0], x0 + sw_ + 2.0, Y(s, bay[3] + 3.0, bay[3] + 6.5)[1])
                for x0 in (-12.0, 58.0) for s in (1, -1)])
    # bulkhead: wire passes, ESP32 grooves, IMU pilots, zip-tie slots for buck + BEC
    b.side('cut', 'Pi plug window (USB/Ethernet plugs and cables reach back over the battery)', 'x', bx0 - 1, bx1 + 1, body=tub,
           paths=[rrect_segs((10.0, fz1 - 0.6, hw - wt - 3.0, 86.0), (0, 0, 5, 5))])
    b.side('cut', 'Bulkhead wire passes', 'x', bx0 - 1, bx1 + 1, body=tub,
           paths=[rrect_segs((-66.0, fz1 - 0.6, -44.0, fz1 + 14.0), (0, 0, 4, 4)), rrect_segs((-44.0, 104.0, 4.0, 119.0), (4, 4, 0, 0))])
    b.side('cut', 'ESP32 groove', 'x', bx1 + 4.0, bx1 + 6.2, body=tub, rects=[(eb0 - 0.3, 49.5, eb0 + ew + 0.3, 53.0)])
    b.side('cut', 'ESP32 zip-tie slots', 'x', bx0 - 1, bx1 + 1, body=tub, rects=[rect_c(eb0 - 3.5, 95.0, 3.0, 6.0), rect_c(eb0 + ew + 3.5, 95.0, 3.0, 6.0)])
    b.side('cut', 'IMU pilot holes (M2.5)', 'x', bx0 - 7.0, bx0 - 1.0, body=tub,
           circles=[(-imh, 110.0, p['m25_tap'] / 2), (imh, 110.0, p['m25_tap'] / 2)])
    b.xy('cut', 'Buck + BEC zip-tie slots (floor, right of the battery)', fz0 - 1, fz1 + 1, body=tub,
         rects=[rect_c(x_, y_, 6.0, 2.6) for x_ in (72.0, 90.0) for y_ in (-66.0, -36.0)])
    # Pi standoff pilots (M2.5 self-tapping)
    b.xy('cut', 'Pi 5 standoff pilots (M2.5)', fz1 - 2.0, fz1 + sh + 1, body=tub, circles=[(x, y, p['m25_tap'] / 2) for x, y in pi_h])
    # VESC zip-tie slots in the floor (board size can vary)
    vox, voy = p['vesc_origin']
    vx, vy, _ = p['vesc']
    b.xy('cut', 'VESC zip-tie slots', fz0 - 1, fz1 + 1, body=tub,
         rects=[rect_c(x_, y_, 3.0, 6.0) for x_ in (vox - 2.5, vox + vx + 2.5) for y_ in (voy + 15.0, voy + vy - 15.0)])
    # floor zip-tie anchors along the wire runs
    ties = [(208.0, -30.0), (208.0, 30.0), (60.0, 50.0), (110.0, 60.0)]
    b.xy('cut', 'Wire tie slots', fz0 - 1, fz1 + 1, body=tub,
         rects=[rect_c(x_, y_ + d_, 6.0, 2.6) for x_, y_ in ties for d_ in (-4.0, 4.0)])
    # steering servo pocket (+ fit), cable slit at the rear end, tab pilots; belt window in the front wall
    bl, bw_, _ = sv['body']
    fs = p['fit_servo']
    b.xy('cut', 'Steering servo pocket (slides +/- for belt tension)', sz0_ - 1, sz1_ + 9.0, body=tub,
         rects=[(scx - bl / 2 - fs - ts, -bw_ / 2 - fs, scx + bl / 2 + fs + ts, bw_ / 2 + fs)])
    b.xy('cut', 'Servo cable slit', sz0_ - 1, sz1_ + 1, body=tub, rects=[(scx - bl / 2 - fs - ts - 4.0, -2.5, scx - bl / 2, 2.5)])
    hp = sv['hole_pitch'] / 2
    b.xy('cut', 'Servo tab slots (M3 bolt + nut)', sz0_ - 1, sz1_ + 1, body=tub,
         paths=[capsule_segs((scx + i * hp - ts, j * sv['hole_sep'] / 2), p['m3_clear'] / 2, (scx + i * hp + ts, j * sv['hole_sep'] / 2), p['m3_clear'] / 2)
                for i in (-1, 1) for j in (-1, 1)])
    sz0, sz1 = p['steer_belt_z']
    b.side('cut', 'Steering belt window', 'x', xf - wt - 1, xf + 1, body=tub, rects=[(-21.0, sz0 - 3.0, 21.0, sz1 + 3.0)])
    # front intake vents (cool air in at the front, out over the motors and the rear)
    b.side('cut', 'Front intake vents', 'x', xf - wt - 1, xf + 1, body=tub,
           paths=[slot_segs((y_, fz1 + 8.0, y_ + 4.0, 98.0)) for y_ in (-62.0, -54.0, -46.0, -38.0, -30.0, -22.0, 18.0, 26.0, 34.0, 42.0, 50.0, 58.0)])
    i4d, i4l = p['insert_m4']
    b.side('cut', 'Nose inserts (M4)', 'x', xf - i4l, xf + 1, body=tub, circles=[(y_, z_, i4d / 2) for y_, z_ in f['nose_bolts']])
    b.side('cut', 'Lidar + HC-SR04 + AS5600 cable passes', 'x', xf - wt - 1, xf + 1, body=tub, circles=[(10.0, 130.0, 4.5), (-10.0, 130.0, 4.5)])
    # lid inserts
    b.xy('cut', 'Lid inserts (M3)', top - ins_l, top + 1, body=tub, circles=[(x, y, ins_d / 2) for x, y in f['lid_bosses']])
    return tub



def build_nose(b, f, p):
    """Bolt-on nose (the tub + nose would not fit the print bed in one piece): flange on the front wall, deck,
    cheeks, steering head (2 x 6802), AS5600 bridge, lidar pad and the HC-SR04 bracket."""
    xf = f['tub_x'][1]
    xs = f['steer_x']
    nz0, nz1 = p['nose_deck_z']
    nhw = p['nose_half_w']
    hz0, hz1 = p['head_z']
    sz0, sz1 = p['steer_belt_z']
    fl0 = xf + 0.2                                       # 0.2 mm off the wall face so it seats on the bolts
    nose = b.side('new', 'Nose + steering head (print, PAHT-CF)', 'x', fl0, fl0 + 6.0, paths=[rrect_segs((-nhw, 100.0, nhw, nz1), 4.0)])
    b.xy('join', 'Nose deck', nz0, nz1, body=nose, paths=[rrect_segs((fl0 + 5.5, -nhw, p['nose_x1'], nhw), (0, 10, 10, 0))])
    c0, c1 = p['cheek_y']
    for s in (1, -1):
        b.side('join', 'Nose cheek', 'y', *Y(s, c0, c1), body=nose, rects=[(fl0 + 5.5, 110.0, xs + 20.0, nz0 + 0.5)])
    b.xy('join', 'Steering head boss', hz0, nz0 + 0.5, body=nose, circles=[(xs, 0.0, 18.0)])
    ab0 = f['as5600_z'] + 1.0 + 1.6
    f['as5600_plate_z'] = (ab0, ab0 + 3.0)
    b.xy('join', 'AS5600 bridge posts', nz1 - 0.5, ab0 + 0.5, body=nose, circles=[(xs - 8.0, 17.0, 3.5), (xs - 8.0, -17.0, 3.5)])
    b.xy('join', 'AS5600 bridge', ab0, ab0 + 3.0, body=nose, paths=[rrect_segs((xs - 14.0, -20.5, xs + 10.0, 20.5), 3.0)])
    hb = p['hc_sr04']['board']
    b.side('join', 'HC-SR04 bracket', 'x', p['nose_x1'] - 4.0, p['nose_x1'], body=nose,
           rects=[(-hb[0] / 2 - 4.0, 110.0, hb[0] / 2 + 4.0, nz0 + 1.0)])
    # cuts
    b.side('cut', 'Nose bolts (M4)', 'x', fl0 - 1, fl0 + 7.0, body=nose, circles=[(y_, z_, p['m4_clear'] / 2) for y_, z_ in f['nose_bolts']])
    b.side('cut', 'Steering belt window', 'x', fl0 - 1, fl0 + 7.0, body=nose, rects=[(-21.0, sz0 - 3.0, 21.0, sz1 + 3.0)])
    b.side('cut', 'Cable passes', 'x', fl0 - 1, fl0 + 7.0, body=nose, circles=[(10.0, 130.0, 4.5), (-10.0, 130.0, 4.5)])
    # steering head: bearing seats (snug), stem clearance; HC-SR04 can holes; lidar pilots; cable holes into the tub
    b6 = p['bearing_6802']
    rs = (b6[1] + p['fit_press']) / 2
    b.xy('cut', 'Lower head bearing seat (6802)', hz0 - 1, hz0 + b6[2], body=nose, circles=[(xs, 0.0, rs)])
    b.xy('cut', 'Upper head bearing seat (6802)', hz1 - b6[2], hz1 + 1, body=nose, circles=[(xs, 0.0, rs)])
    b.xy('cut', 'Stem clearance', hz0 - 1, hz1 + 1, body=nose, circles=[(xs, 0.0, b6[0] / 2 + 1.0)])
    hc = p['hc_sr04']
    b.side('cut', 'HC-SR04 can holes', 'x', p['nose_x1'] - 5, p['nose_x1'] + 1, body=nose,
           circles=[(-hc['can_pitch'] / 2, 124.0, hc['can_d'] / 2 + 0.3), (hc['can_pitch'] / 2, 124.0, hc['can_d'] / 2 + 0.3)])
    ld = p['lidar']
    lh = ld['holes']
    b.xy('cut', 'Lidar pilots (M2.5)', nz1 - 8.0, nz1 + 1, body=nose,
         circles=[(ld['x'] + i * lh[0] / 2, j * lh[1] / 2, p['m25_tap'] / 2) for i in (-1, 1) for j in (-1, 1)])
    b.xy('cut', 'AS5600 board screws (M2 self-tapping) + wire hole', f['as5600_plate_z'][0] - 1, f['as5600_plate_z'][1] + 1, body=nose,
         circles=[(xs - 2.0, 8.5, p['m2_tap'] / 2), (xs - 2.0, -8.5, p['m2_tap'] / 2), (xs - 10.0, 0.0, 2.5)])
    return nose


def build_door(b, f, p):
    xr = f['tub_x'][0]
    fz1 = p['floor_z'][1]
    d = b.side('new', 'Battery door (print, PETG)', 'x', xr - 3.0, xr, paths=[rrect_segs((-38.0, fz1 - 2.0, 38.0, fz1 + 46.0), 4.0)])
    b.side('cut', 'Door screws (M3)', 'x', xr - 4, xr + 1, body=d, circles=[(33.0, 62.0, p['m3_clear'] / 2), (-33.0, 62.0, p['m3_clear'] / 2)])
    b.side('cut', 'Door vents', 'x', xr - 4, xr + 1, body=d, paths=[slot_segs((y_, fz1 + 6.0, y_ + 3.5, fz1 + 36.0)) for y_ in (-20.0, -12.0, -4.0, 4.0, 12.0)])
    return d


def build_lid(b, f, p):
    z0, z1 = f['lid_z']
    lx0, lx1 = f['lid_x']
    mw1 = p['motor_wall_y'][1]
    hw = p['front_half_w']
    lid = b.xy('new', 'Lid (print, PETG)', z0, z1, rects=[(lx0, -mw1, p['rear_zone_x1'], mw1)])
    b.xy('join', 'Lid front', z0, z1, body=lid, paths=[rrect_segs((p['rear_zone_x1'] - 2, -hw, lx1, hw), (0, 0, 0, 0))])
    b.xy('cut', 'Motor exhaust vents', z0 - 1, z1 + 1, body=lid,
         paths=[slot_segs((x_, s * 18.0 if s > 0 else -62.0, x_ + 3.5, 62.0 if s > 0 else -18.0)) for x_ in (8.0, 16.0, 24.0, 32.0, 40.0, 48.0) for s in (1, -1)])
    pcx = p['pi_origin'][0] + p['pi_board'][0] / 2
    pcy = p['pi_origin'][1] + p['pi_board'][1] / 2
    b.xy('cut', 'Pi fan vent', z0 - 1, z1 + 1, body=lid, paths=[slot_segs((pcx - 15.0 + 7.5 * i, pcy - 15.0, pcx - 11.5 + 7.5 * i, pcy + 15.0)) for i in range(5)])
    b.xy('cut', 'Lid screws (M3)', z0 - 1, z1 + 1, body=lid, circles=[(x, y, p['m3_clear'] / 2) for x, y in f['lid_bosses']])
    b.xy('cut', 'Finger notch', z0 - 1, z1 + 1, body=lid, paths=[slot_segs((lx1 - 6.0, -15.0, lx1 + 1.0, 15.0))])
    b.xy('cut', 'Mast cable notch (lid lifts off round the cables)', z0 - 1, z1 + 1, body=lid, paths=[rrect_segs((lx0 - 1.0, -8.0, lx0 + 14.0, 8.0), (0, 4, 4, 0))])
    return lid


def build_reference(b, f, p):
    """Bought parts fixed to the chassis: motors (stator), battery, boards, servo, sensors, belts, axle hardware."""
    out = []
    mx, mz = f['motor']
    for s, side in ((1, 'L'), (-1, 'R')):
        out.append(b.side('new', f'Motor {side} (Flipsky H5055 200KV, buy, reference)', 'y', *Y(s, *f['motor_y']),
                          circles=[(mx, mz, p['motor_can_d'] / 2)]))
    bb = f['battery_box']
    out.append(b.xy('new', 'Battery 4S (buy, reference)', *f['battery_z'], rects=[bb]))
    pox, poy = p['pi_origin']
    sd, sh = p['pi_standoff']
    fz1 = p['floor_z'][1]
    out.append(b.xy('new', 'Raspberry Pi 5 (reference)', fz1 + sh, fz1 + sh + 1.6, rects=[(pox, poy, pox + p['pi_board'][0], poy + p['pi_board'][1])]))
    vox, voy = p['vesc_origin']
    vx, vy, vz = p['vesc']
    out.append(b.xy('new', 'VESC dual (buy, reference)', fz1 + 1.0, fz1 + 1.0 + vz, rects=[(vox, voy, vox + vx, voy + vy)]))
    bx1 = p['bulkhead_x'][1]
    eb = p['esp32_board']
    out.append(b.side('new', 'ESP32 DevKit (reference)', 'x', bx1 + 4.3, bx1 + 4.3 + eb[2], rects=[(-10.0 - eb[1], 50.0, -10.0, 50.0 + eb[0])]))
    sv = p['servo']
    scx = f['servo_body_cx']
    bl, bw_, bh = sv['body']
    sz = f['servo_body_z']
    s_ = b.xy('new', 'Steering servo (buy, reference)', sz[0], sz[1], rects=[rect_c(scx, 0.0, bl, bw_)])
    b.xy('join', 'Tabs', *f['servo_tab_z'], body=s_, rects=[rect_c(scx, 0.0, sv['tab_len'], bw_)])
    b.xy('join', 'Spline', sz[1] - 0.5, sz[1] + sv['spline_h'], body=s_, circles=[(f['servo_x'], 0.0, 2.9)])
    out.append(s_)
    ld = p['lidar']
    nz1 = p['nose_deck_z'][1]
    out.append(b.xy('new', 'Lidar LD19 (buy, reference)', nz1, nz1 + ld['size'][2], rects=[rect_c(ld['x'], 0.0, ld['size'][0], ld['size'][1])]))
    hc = p['hc_sr04']
    xn = p['nose_x1']
    h_ = b.side('new', 'HC-SR04 (reference)', 'x', xn - 5.6, xn - 4.0, rects=[rect_c(0.0, 124.0, *hc['board'])])
    b.side('join', 'Cans', 'x', xn - 4.5, xn - 4.0 + hc['can_h'], body=h_,
           circles=[(-hc['can_pitch'] / 2, 124.0, hc['can_d'] / 2), (hc['can_pitch'] / 2, 124.0, hc['can_d'] / 2)])
    out.append(h_)
    if p['belts']:
        r1, r2 = f['pd_wheel'] / 2, f['pd_motor'] / 2
        for s, side in ((1, 'L'), (-1, 'R')):
            out.append(belt_ring_y(b, f'Drive belt {side} (HTD-3M {p["drive_belt"][0] * 3:.0f} mm x {p["drive_belt"][1]:.0f}, buy)',
                                   *Y(s, *f['drive_belt_y']), (0.0, p['axle_z']), r1, (mx, mz), r2))
        rp = f['steer_pd'] / 2
        bz0, bz1 = p['steer_belt_z']
        bc = (bz0 + bz1) / 2
        out.append(belt_ring_z(b, f'Steering belt (GT2 {p["steer_belt"][0] * 2:.0f} mm x {p["steer_belt"][1]:.0f}, buy)',
                               bc - p['steer_belt'][1] / 2, bc + p['steer_belt'][1] / 2, (f['servo_x'], 0.0), rp, (f['steer_x'], 0.0), rp))
    if p['hardware']:
        az = p['axle_z']
        for s, side in ((1, 'L'), (-1, 'R')):
            out.append(hw_cyl_y(b, f'Rear axle bolt {side} (M8 x {f["rear_axle_bolt"]}, from outside, hardware)', 0.0, az,
                                *Y(s, f['axle_end_y'], f['axle_head_y'][0]), 4.0))
            out.append(hw_hex_y(b, f'Rear axle bolt head {side} (hardware)', 0.0, az, *Y(s, *f['axle_head_y']), 13.0))
            out.append(hw_cyl_y(b, f'Rear axle washer {side} (M8, hardware)', 0.0, az, *Y(s, *f['axle_washer_y']), 8.0))
            nut = b.side('new', f'Rear axle nut {side} (M8, captured, hardware)', 'y', *Y(s, f['axle_nut_y'][0] + 0.2, f['axle_nut_y'][1]),
                         paths=[poly_segs(hex_pts((0.0, az), p['axle_nut'][0], 0.0))])
            b.side('cut', 'Thread', 'y', *Y(s, f['axle_nut_y'][0] - 1, f['axle_nut_y'][1] + 1), body=nut, circles=[(0.0, az, 4.0)])
            out.append(nut)
            sy0, sy1 = f['spacer_y']
            sp = b.side('new', f'Spacer {side} (print, PAHT-CF, or turn in aluminium; 16/8.4 x {sy1 - sy0:.0f}, 11.5 nose)', 'y', *Y(s, sy0, sy1 - 1.5),
                        circles=[(0.0, az, p['spacer_tube'][0] / 2)])
            b.side('join', 'Nose (inner race only)', 'y', *Y(s, sy1 - 1.7, sy1), body=sp, circles=[(0.0, az, p['spacer_tube'][2] / 2)])
            b.side('cut', 'Bore', 'y', *Y(s, f['spacer_y'][0] - 1, f['spacer_y'][1] + 1), body=sp, circles=[(0.0, az, p['spacer_tube'][1] / 2)])
            out.append(sp)
            for i, (hx, hz) in enumerate(f['motor_screws']):
                hd, hh, hc = p['motor_screw_head']
                yh = p['motor_wall_y'][1] - hc
                out.append(hw_cyl_y(b, f'Motor screw {side}{i + 1} (M4 x 8 button head, hardware)', hx, hz, *Y(s, yh - 8.0, yh), 2.0))
                out.append(hw_cyl_y(b, f'Motor screw head {side}{i + 1} (hardware)', hx, hz, *Y(s, yh, yh + hh), hd / 2))
        b6 = p['bearing_6802']
        hz0, hz1 = p['head_z']
        for nm, (z0, z1) in (('lower', (hz0, hz0 + b6[2])), ('upper', (hz1 - b6[2], hz1))):
            br = hw_cyl_z(b, f'Head bearing {nm} (6802, hardware)', f['steer_x'], 0.0, z0, z1, b6[1] / 2)
            b.xy('cut', 'Bore', z0 - 1, z1 + 1, body=br, circles=[(f['steer_x'], 0.0, b6[0] / 2)])
            out.append(br)
        sx1 = f['sleeve_x'][1]
        cy1 = p['sleeve'][0] / 2 + p['fit_free'] + 7.0
        out.append(hw_cyl_y(b, 'Hinge pivot bolt (M5 x 50 + nyloc, hardware)', *f['pivot'], -cy1 - 3.0, cy1 + 6.0, 2.5))
        out.append(hw_cyl_y(b, 'Hinge lock bolt (M4 x 45, hardware)', *f['lock'], -cy1 - 3.0, cy1 + 5.0, 2.0))
    return out


def build_rear_wheel(b, f, p, s, side):
    out = []
    az = p['axle_z']
    R = p['wheel'][0] / 2
    wh = b.side('new', f'Rear wheel {side} (90 mm longboard, buy, reference)', 'y', *Y(s, *f['rear_wheel_y']), circles=[(0.0, az, R)])
    b.side('cut', 'Bearing bore', 'y', *Y(s, f['rear_wheel_y'][0] - 1, f['rear_wheel_y'][1] + 1), body=wh, circles=[(0.0, az, 11.0)])
    out.append(wh)
    wp = p['wheel_pulley']
    y0, y1 = f['wheel_pulley_y']
    t0, t1 = f['drive_teeth_y']
    od, grooves = teeth_grooves((0.0, az), p['drive_teeth'][1], p['belt_pitch'], **HTD3M)
    pu = b.side('new', f'Wheel pulley {side} (print, PAHT-CF; HTD-3M {p["drive_teeth"][1]}T)', 'y', *Y(s, t0, t1), circles=[(0.0, az, od / 2)])
    b.side('join', 'Inner flange', 'y', *Y(s, y0, t0 + 0.2), body=pu, circles=[(0.0, az, od / 2 + 2.0)])
    b.side('join', 'Outer flange', 'y', *Y(s, t1 - 0.2, f['wheel_disc_y'][0] + 0.2), body=pu, circles=[(0.0, az, od / 2 + 2.0)])
    b.side('join', 'Wheel adapter disc', 'y', *Y(s, *f['wheel_disc_y']), body=pu, circles=[(0.0, az, wp['disc_d'] / 2)])
    b.side('cut', 'Teeth', 'y', *Y(s, t0, t1), body=pu, circles=grooves)
    bp = wp['bolt_pcd'] / 2
    b.side('cut', 'Spacer clearance + wheel bolts (3 x M4)', 'y', *Y(s, y0 - 1, y1 + 1), body=pu,
           circles=[(0.0, az, wp['bore'] / 2)] + [(bp * math.cos(math.radians(90 + 120 * i)), az + bp * math.sin(math.radians(90 + 120 * i)), wp['bolt_hole'] / 2) for i in range(3)])
    out.append(pu)
    return out


def build_motor_pulley(b, f, p, s, side):
    mx, mz = f['motor']
    y0, y1 = f['motor_pulley_y']
    ph = p['motor_pulley'][1]
    od, grooves = teeth_grooves((mx, mz), p['drive_teeth'][0], p['belt_pitch'], **HTD3M)
    pu = b.side('new', f'Motor pulley {side} (HTD-3M {p["drive_teeth"][0]}T 8 mm bore, buy, reference)', 'y', *Y(s, y0, y1 - ph), circles=[(mx, mz, od / 2 + 1.5)])
    b.side('join', 'Hub', 'y', *Y(s, y1 - ph - 0.2, y1), body=pu, circles=[(mx, mz, 8.0)])
    b.side('join', 'Motor shaft (rotor)', 'y', *Y(s, f['motor_shaft_y'][0], y0 + 0.2), body=pu, circles=[(mx, mz, p['motor_shaft'][0] / 2)])
    return pu


def build_fork(b, f, p):
    xs = f['steer_x']
    xa, az = f['front_axle']
    lt, lw = p['fork_leg']
    ly0, ly1 = f['fork_leg_y']
    cz0, cz1 = f['crown_z']
    fork = b.xy('new', 'Steering fork (print, PAHT-CF)', cz0, cz1, paths=[rrect_segs((xs - f['crown_x'], -ly1, xs + f['crown_x'], ly1), 6.0)])
    for s in (1, -1):
        b.side('join', 'Fork leg', 'y', *Y(s, ly0, ly1), body=fork, paths=[capsule_segs((xa, az), lw / 2, (xs, cz1 - lw / 2), lw / 2)])
    for s in (1, -1):
        b.side('join', 'Inner-race spacer', 'y', *Y(s, *f['fork_spacer']), body=fork, circles=[(xa, az, p['spacer_tube'][2] / 2)])
    # steering pulley (GT2, printed with the fork), shoulder on the lower bearing's inner race, stem
    sz0, sz1 = p['steer_belt_z']
    od, grooves = teeth_grooves((xs, 0.0), p['steer_teeth'], p['gt2_pitch'], **GT2)
    b.xy('join', 'Pulley flange', cz1 - 0.5, sz0, body=fork, circles=[(xs, 0.0, od / 2 + 1.5)])
    b.xy('join', f'Steering pulley ({p["steer_teeth"]}T GT2)', sz0, sz1, body=fork, circles=[(xs, 0.0, od / 2)])
    b.xy('join', 'Pulley flange', sz1, sz1 + 1.5, body=fork, circles=[(xs, 0.0, od / 2 + 1.5)])
    hz0 = p['head_z'][0]
    b.xy('join', 'Bearing shoulder', sz1 + 1.0, hz0, body=fork, circles=[(xs, 0.0, 9.0)])
    sr = (p['stem_d'] - p['fit_shaft']) / 2
    b.xy('join', 'Stem', hz0 - 0.5, f['stem_z'][1], body=fork, circles=[(xs, 0.0, sr)])
    # cuts
    b.xy('cut', 'Steering pulley teeth', sz0, sz1, body=fork, circles=grooves)
    b.xy('cut', 'Stem bolt bore (M8)', f['crown_z'][0] - 1, f['stem_z'][1] + 1, body=fork, circles=[(xs, 0.0, p['m8_clear'] / 2)])
    b.xy('cut', 'Crown nut (M8, captured from below)', f['crown_nut_z'][0] - 1, f['crown_nut_z'][1], body=fork,
         paths=[poly_segs(hex_pts((xs, 0.0), p['axle_nut'][0] + 0.25, 0.0))])
    b.side('cut', 'Front axle (M8)', 'y', -ly1 - 1, ly1 + 1, body=fork, circles=[(xa, az, p['m8_clear'] / 2)])
    return fork


def build_front_wheel(b, f, p):
    xa, az = f['front_axle']
    hw = p['wheel'][1] / 2
    wh = b.side('new', 'Front wheel (90 mm longboard, buy, reference)', 'y', -hw, hw, circles=[(xa, az, p['wheel'][0] / 2)])
    b.side('cut', 'Bearing bore', 'y', -hw - 1, hw + 1, body=wh, circles=[(xa, az, 11.0)])
    return wh


def build_servo_pulley(b, f, p):
    x = f['servo_x']
    sz0, sz1 = p['steer_belt_z']
    od, grooves = teeth_grooves((x, 0.0), p['steer_teeth'], p['gt2_pitch'], **GT2)
    a0 = sz0 - 1.5 - 2.5
    pu = b.xy('new', f'Steering servo pulley (print, PAHT-CF; {p["steer_teeth"]}T GT2)', a0, sz0 - 1.5 + 0.2, circles=[(x, 0.0, 15.0)])
    b.xy('join', 'Flange', sz0 - 1.5, sz0, body=pu, circles=[(x, 0.0, od / 2 + 1.5)])
    b.xy('join', 'Teeth band', sz0, sz1, body=pu, circles=[(x, 0.0, od / 2)])
    b.xy('join', 'Flange', sz1, sz1 + 1.5, body=pu, circles=[(x, 0.0, od / 2 + 1.5)])
    b.xy('cut', 'Teeth', sz0, sz1, body=pu, circles=grooves)
    hr = p['servo_horn_pcd'] / 2
    b.xy('cut', 'Horn screws (M3, from the top) + spline screw', a0 - 1, sz1 + 2, body=pu,
         circles=[(x + hr * math.cos(math.radians(45 + 90 * i)), hr * math.sin(math.radians(45 + 90 * i)), p['m3_clear'] / 2 - 0.1) for i in range(4)] +
                 [(x, 0.0, 3.2)])
    return pu, a0


def build_sleeve(b, f, p):
    sw, st, sl = p['sleeve']
    sx0, sx1 = f['sleeve_x']
    z0, z1 = f['sleeve_z']
    px, pz = f['pivot']
    xm = p['mast_x']
    s_ = b.xy('new', 'Mast sleeve (print, PAHT-CF)', pz, z1, paths=[rrect_segs((sx0, -sw / 2, sx1, sw / 2), 3.0)])
    # bottom block: front-bottom edge is an arc round the pivot, so the sleeve never digs into the pedestal as it folds
    rp = sx1 - px
    b.side('join', 'Knuckle', 'y', -sw / 2, sw / 2, body=s_, paths=[rrect_segs((sx0, z0, sx1, pz + 1.0), (0, rp, 0, 0))])
    b.xy('cut', 'Inner mast bore', z0 + f['sleeve_bottom_t'], z1 + 1, body=s_, paths=[rrect_segs((sx0 + st, -sw / 2 + st, sx1 - st, sw / 2 - st), 1.5)])
    bt = f['sleeve_bottom_t']
    b.side('cut', 'Cable window (front face; cables loop down through the lid notch)', 'x', sx1 - st - 1, sx1 + 1, body=s_,
           rects=[(-7.0, z0 + bt + 2.0, 7.0, z0 + bt + 27.0)])
    b.side('cut', 'Pivot (M5)', 'y', -sw / 2 - 1, sw / 2 + 1, body=s_, circles=[(px, pz, p['m5_clear'] / 2)])
    b.side('cut', 'Lock (M4)', 'y', -sw / 2 - 1, sw / 2 + 1, body=s_, circles=[(f['lock'][0], f['lock'][1], p['m4_clear'] / 2)])
    b.side('cut', 'Height pin (M4)', 'y', -sw / 2 - 1, sw / 2 + 1, body=s_, circles=[(xm, f['sleeve_pin_z'], p['m4_clear'] / 2)])
    return s_


def build_inner_mast(b, f, p):
    xm = p['mast_x']
    iw = f['inner_w']
    iz0, iz1 = f['inner_z']
    sv = p['servo']
    m = b.xy('new', 'Inner mast + pan servo head (print, PAHT-CF)', iz0, iz1, paths=[rrect_segs(rect_c(xm, 0.0, iw, iw), 1.5)])
    hz0, hz1 = f['head_z']
    scx = xm - sv['spline_off']
    tl = sv['tab_len']
    b.xy('join', 'Servo head', hz0 - 0.5, hz1, body=m, paths=[rrect_segs((scx - tl / 2 - 2.0, -16.0, scx + tl / 2 + 2.0, 16.0), 3.0)])
    bl, bw_, bh = sv['body']
    fs = p['fit_servo']
    body_bottom = f['pan_servo_top'] - bh
    b.xy('cut', 'Cable bore', iz0 - 1, body_bottom + 0.5, body=m, circles=[(xm, 0.0, p['inner_mast'][1] / 2)])
    b.xy('cut', 'Pan servo pocket', body_bottom - 1.0, hz1 + 1, body=m, rects=[(scx - bl / 2 - fs, -bw_ / 2 - fs, scx + bl / 2 + fs, bw_ / 2 + fs)])
    b.xy('cut', 'Servo cable slit', body_bottom - 1.0, hz1 + 1, body=m, rects=[(scx - bl / 2 - fs - 4.0, -2.5, scx - bl / 2, 2.5)])
    hp = sv['hole_pitch'] / 2
    b.xy('cut', 'Servo tab pilots (M3 self-tapping)', hz1 - 10.0, hz1 + 1, body=m,
         circles=[(scx + i * hp, j * sv['hole_sep'] / 2, p['m3_tap'] / 2) for i in (-1, 1) for j in (-1, 1)])
    b.side('cut', 'Camera cable window (into the bore)', 'x', xm - iw / 2 - 1, xm, body=m, rects=[(-6.0, iz1 - 22.0, 6.0, iz1 - 6.0)])
    pins = [f['inner_top'] - (e + 12.0) for e in (30.0, 60.0, 90.0)]
    b.side('cut', 'Height pin holes (M4)', 'y', -iw / 2 - 1, iw / 2 + 1, body=m, circles=[(xm, z, p['m4_clear'] / 2) for z in pins])
    return m, body_bottom


def build_pan_stage(b, f, p):
    xm = p['mast_x']
    a0, a1 = f['adapter_z']
    z0, z1 = f['plate_z']
    pl = p['pan_plate']
    # 1) horn adapter: screwed to the disc horn first (screws reachable), carries 4 M3 inserts for the plate
    ad = b.xy('new', 'Pan horn adapter (print, PAHT-CF)', a0, a0 + 2.0, circles=[(xm, 0.0, 15.0)])
    b.xy('join', 'Adapter arms', a0 + 1.5, a1, body=ad, paths=[rrect_segs((xm - 36.0, -20.0, xm + 18.0, 20.0), 5.0)])
    hr = p['servo_horn_pcd'] / 2
    b.xy('cut', 'Horn screws (M3) + spline screw', a0 - 1, a1 + 1, body=ad,
         circles=[(xm + hr * math.cos(math.radians(45 + 90 * i)), hr * math.sin(math.radians(45 + 90 * i)), p['m3_clear'] / 2) for i in range(4)] +
                 [(xm, 0.0, 3.2)])
    plate_pts = [(xm - 30.0, 14.0), (xm - 30.0, -14.0), (xm + 12.0, 15.0), (xm + 12.0, -15.0)]
    di, dl = p['insert_m3']
    b.xy('cut', 'Plate inserts (M3, short)', a1 - min(dl, a1 - a0 - 0.8), a1 + 1, body=ad, circles=[(x, y, di / 2) for x, y in plate_pts])
    # 2) camera plate: GoPro prongs, vision camera and UWB anchor brackets; 4 x M3 into the adapter
    st = b.xy('new', 'Pan camera plate (print, PETG)', z0, z1, paths=[rrect_segs((xm - 34.0, -pl[1] / 2, xm + pl[0], pl[1] / 2), 6.0)])
    g = p['gopro_prongs']
    yc = [-(g['t'] + g['gap']), 0.0, g['t'] + g['gap']]
    for y_ in yc:
        b.side('join', 'GoPro prong', 'y', y_ - g['t'] / 2, y_ + g['t'] / 2, body=st,
               paths=[rrect_segs((xm - 7.5, z1 - 0.5, xm + 7.5, z1 + g['h']), (0, 0, 7.5, 7.5))])
    wc = p['webcam']
    b.side('join', 'Webcam bracket', 'x', xm + 15.0, xm + 19.0, body=st, rects=[(38.0, z1 - 0.5, 72.0, z1 + 38.0)])
    ub = p['uwb']
    b.side('join', 'UWB anchor bracket', 'x', xm + 15.0, xm + 19.0, body=st, rects=[(-74.0, z1 - 0.5, -38.0, z1 + 58.0)])
    # cuts
    b.side('cut', 'GoPro thumb screw (M5)', 'y', -10.0, 10.0, body=st, circles=[(xm, z1 + g['h'] - 7.5, g['hole'] / 2)])
    b.xy('cut', 'Plate screws (M3)', z0 - 1, z1 + 1, body=st, circles=[(x, y, p['m3_clear'] / 2) for x, y in plate_pts])
    b.xy('cut', 'Cable notches (cables drop beside the servo head)', z0 - 1, z1 + 1, body=st, circles=[(xm - 18.0, 26.0, 6.0), (xm - 18.0, -26.0, 6.0)])
    wcy, wcz = 55.0, z1 + 19.0
    hw_ = wc['holes'] / 2
    b.side('cut', 'Webcam holes (M2 self-tapping) + lens', 'x', xm + 14.0, xm + 20.0, body=st,
           circles=[(wcy + i * hw_, wcz + j * hw_, p['m2_tap'] / 2) for i in (-1, 1) for j in (-1, 1)] + [(wcy, wcz, wc['lens_d'] / 2)])
    uy, uz = -56.0, z1 + 29.0
    hx_, hy_ = ub['holes']
    b.side('cut', 'UWB anchor holes (M3 self-tapping)', 'x', xm + 14.0, xm + 20.0, body=st,
           circles=[(uy + i * hy_ / 2, uz + j * hx_ / 2, p['m3_tap'] / 2) for i in (-1, 1) for j in (-1, 1)])
    gp = b.xy('new', 'GoPro HERO9 (reference)', z1 + 13.0, z1 + 68.0, rects=[rect_c(xm, 0.0, 33.6, 71.0)])
    return ad, st, gp


def build(root, p=P):
    f = layout(p)
    asm = Assembly2(root, p['joints'])
    out = dict(f=f, bodies=[], asm=asm)
    ch_occ, b = asm.comp('Chassis')
    tub = build_chassis(b, f, p)
    out['bodies'] += [tub, build_nose(b, f, p), build_door(b, f, p)] + build_reference(b, f, p)
    try:
        ch_occ.isGrounded = True
    except Exception:
        pass
    if p['build_lid']:
        lid_occ, b = asm.comp('Lid')
        out['bodies'].append(build_lid(b, f, p))
        asm.rigid('Lid on chassis', lid_occ, ch_occ)
    az = p['axle_z']
    for s, side in ((1, 'L'), (-1, 'R')):
        w_occ, b = asm.comp(f'Rear wheel {side}')
        wb = build_rear_wheel(b, f, p, s, side)
        out['bodies'] += wb
        jw = asm.revolute(f'Rear wheel {side}', w_occ, ch_occ, wb[0], w_occ, (0.0, s * f['rear_wheel_y'][0], az), p['wheel'][0] / 2)
        m_occ, b = asm.comp(f'Motor pulley {side}')
        mp = build_motor_pulley(b, f, p, s, side)
        out['bodies'].append(mp)
        mx, mz = f['motor']
        jm = asm.revolute(f'Motor {side}', m_occ, ch_occ, mp, m_occ, (mx, s * f['motor_pulley_y'][1], mz), 8.0)
        asm.link(f'Drive belt {side} ({p["drive_teeth"][1]}:{p["drive_teeth"][0]})', jw, jm, 360, round(360 * f['ratio'], 1))
    fk_occ, b = asm.comp('Steering fork')
    fork = build_fork(b, f, p)
    out['bodies'].append(fork)
    if p['hardware']:
        xa, az_ = f['front_axle']
        ly1 = f['fork_leg_y'][1]
        L_ = int(math.ceil((2 * ly1 + 1.6 + 8.0 + 1.5) / 5.0) * 5)
        out['bodies'] += [hw_cyl_y(b, f'Front axle bolt (M8 x {L_}, hardware)', xa, az_, -ly1, -ly1 + L_, 4.0),
                          hw_hex_y(b, 'Front axle bolt head (hardware)', xa, az_, -ly1 - 5.3, -ly1, 13.0),
                          hw_hex_y(b, 'Front axle nyloc (hardware)', xa, az_, ly1 + 1.6, ly1 + 9.6, 13.0)]
        xs_ = f['steer_x']
        sb_d, sb_hd, sb_hh = p['stem_bolt']
        h0, h1 = f['stem_bolt_head_z']
        out['bodies'] += [hw_cyl_z(b, f'Stem bolt (M8 x {f["stem_bolt"]} low head, 10.9, hardware)', xs_, 0.0, h0 - f['stem_bolt'], h0, 4.0),
                          hw_cyl_z(b, 'Stem bolt head (hardware)', xs_, 0.0, h0, h1, sb_hd / 2),
                          b.xy('new', 'Crown nut (M8, captured, hardware)', f['crown_nut_z'][0] + 0.2, f['crown_nut_z'][1],
                               paths=[poly_segs(hex_pts((xs_, 0.0), p['axle_nut'][0], 0.0))]),
                          hw_cyl_z(b, 'Steering magnet (6 x 2.5 diametric, in the bolt socket, hardware)', xs_, 0.0, h1 - p['steer_magnet'][1], h1, 3.0)]
    # stem collar: clamps the stem above the upper bearing (stem can't drop out), M3 pinch screw across a slit
    co, ch = p['stem_collar']
    cz0, cz1 = f['collar_z']
    sr = (p['stem_d'] - p['fit_shaft']) / 2
    col = b.xy('new', 'Stem collar (print, PAHT-CF)', cz0, cz1, circles=[(f['steer_x'], 0.0, co / 2)])
    b.xy('cut', 'Stem bore', cz0 - 1, cz1 + 1, body=col, circles=[(f['steer_x'], 0.0, sr + 0.05)])
    b.xy('cut', 'Clamp slit', cz0 - 1, cz1 + 1, body=col, rects=[(f['steer_x'] + sr - 1.0, -0.75, f['steer_x'] + co / 2 + 1.0, 0.75)])
    b.side('cut', 'Pinch screw (M3)', 'y', -co / 2 - 1, co / 2 + 1, body=col, circles=[(f['steer_x'] + (sr + co / 2) / 2, (cz0 + cz1) / 2, p['m3_clear'] / 2)])
    out['bodies'].append(col)
    jf = asm.revolute('Steering', fk_occ, ch_occ, fork, fk_occ, (f['steer_x'], 0.0, f['stem_z'][1]), (p['stem_d'] - p['fit_shaft']) / 2)
    fw_occ, b = asm.comp('Front wheel')
    fw = build_front_wheel(b, f, p)
    out['bodies'].append(fw)
    asm.revolute('Front wheel', fw_occ, fk_occ, fw, fw_occ, (f['front_axle'][0], p['wheel'][1] / 2, az), p['wheel'][0] / 2)
    sp_occ, b = asm.comp('Steering servo pulley')
    sp, a0 = build_servo_pulley(b, f, p)
    out['bodies'].append(sp)
    js = asm.revolute('Steering servo', sp_occ, ch_occ, sp, sp_occ, (f['servo_x'], 0.0, a0), 15.0)
    asm.link('Steering belt (1:1)', js, jf, 360, 360)
    # mast: sleeve folds on the hinge, inner mast slides in the sleeve, pan stage turns on the servo
    sl_occ, b = asm.comp('Mast sleeve')
    sleeve = build_sleeve(b, f, p)
    out['bodies'].append(sleeve)
    asm.revolute('Mast fold', sl_occ, ch_occ, sleeve, sl_occ, (f['pivot'][0], p['sleeve'][0] / 2, f['pivot'][1]), p['m5_clear'] / 2)
    im_occ, b = asm.comp('Inner mast')
    inner, body_bottom = build_inner_mast(b, f, p)
    out['bodies'].append(inner)
    sv = p['servo']
    ps = b.xy('new', 'Pan servo (buy, reference)', body_bottom, f['pan_servo_top'], rects=[rect_c(p['mast_x'] - sv['spline_off'], 0.0, sv['body'][0], sv['body'][1])])
    b.xy('join', 'Tabs', f['head_z'][1], f['head_z'][1] + sv['tab_t'], body=ps, rects=[rect_c(p['mast_x'] - sv['spline_off'], 0.0, sv['tab_len'], sv['body'][1])])
    out['bodies'].append(ps)
    if p['hardware']:
        out['bodies'].append(hw_cyl_y(b, 'Height pin (M4 x 40 + nyloc, hardware)', p['mast_x'], f['sleeve_pin_z'], -20.0, 20.0, 2.0))
    asm.slider('Mast height', im_occ, sl_occ, inner, im_occ, (p['mast_x'], 0.0, f['inner_z'][0]), p['inner_mast'][1] / 2)
    pn_occ, b = asm.comp('Pan stage')
    adapter, stage, gopro = build_pan_stage(b, f, p)
    out['bodies'] += [adapter, stage, gopro]
    asm.revolute('Camera pan', pn_occ, im_occ, adapter, pn_occ, (p['mast_x'], 0.0, f['adapter_z'][0]), 15.0)
    return out


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        out = build(design.rootComponent)
        f, asm = out['f'], out['asm']
        app.activeViewport.fit()
        msg = (f'honebot V2 built: {len(out["bodies"])} bodies in {len(asm.occ)} components, {len(asm.joints)} moving joints.\n\n'
               f'Wheelbase {P["wheelbase"]:.0f} mm, track {2 * (f["rear_wheel_y"][0] + P["wheel"][1] / 2):.0f} mm, '
               f'overall width {f["overall_width"]:.0f} mm. Drive {P["drive_teeth"][0]}:{P["drive_teeth"][1]} HTD-3M, '
               f'belt c-c {f["drive_cc"]:.1f} mm. Steering belt c-c {f["steer_cc"]:.1f} mm.\n'
               f'GoPro lens ~{f["lens_z"]:.0f} mm above the ground (mast extension {P["mast_extension"]:.0f} mm, '
               f'{f["mast_ext_min"]:.0f}-{f["mast_ext_max"]:.0f} mm in 30 mm pin steps).\n'
               f'Mass ~{f["mass"] / 1000:.2f} kg, centre of mass {f["rear_load"] * 100:.0f}% on the rear axle, {f["cg"][1]:.0f} mm up.\n\n'
               'Try the motion: Assemble > Drive Joints (rear wheels, Steering, Front wheel, Mast fold, Mast height, Camera pan). '
               'The chassis is grounded.\n')
        if asm.notes:
            msg += '\nNotes:\n- ' + '\n- '.join(asm.notes[:8]) + '\n'
        msg += '\nPrint the "(print ...)" bodies (right-click > Save As Mesh). Docs: docs/robot_design_v2.md, cad/honebot_v2/README.md.'
        ui.messageBox(msg)
    except Exception:
        if ui:
            ui.messageBox('honebot V2 script failed while building:\n  {}\n\n{}'.format(LAST_FEATURE[0], traceback.format_exc()))
