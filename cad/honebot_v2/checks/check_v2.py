"""honebot V2 review: motion sweeps, clearances, fits, stability, stress, belts, cooling.

Usage: python3 check_v2.py ../honebot_v2.py
Pure geometry from layout(); no Fusion needed. Prints the numbers and FAILs; exits 1 if anything fails.
Material allowables are conservative printed values (PAHT-CF: 50 MPa in the layer plane, 25 MPa across layers;
PETG 35 / 15 MPa). Load cases: 5 g vertical bump, 0.8 g cornering/braking grip, 1.5 g fore-aft rocking at the mast.
"""
import math
import sys

ns = {}
exec(compile(open(sys.argv[1]).read(), sys.argv[1], 'exec'), ns)
P, layout = ns['P'], ns['layout']
f = layout(P)
ok = True
g = 9.81
MU = 0.8


def fail(msg):
    global ok
    ok = False
    print('  FAIL', msg)


def gap(name, value, need=None):
    need = P['min_gap'] if need is None else need
    print(f'  {name}: {value:.1f} mm' + ('' if value >= need else f'   <-- need {need:.1f}'))
    if value < need:
        fail(f'{name} {value:.1f} < {need:.1f}')


R = P['wheel'][0] / 2
az = P['axle_z']
mx, mz = f['motor']
mr = P['motor_can_d'] / 2
xr, xf = f['tub_x']
top = P['tub_top_z']
fz0, fz1 = P['floor_z']

print('== Layout ==')
print(f'  length {xr:.0f}..{P["nose_x1"] + P["hc_sr04"]["can_h"] - 4:.0f} mm, overall width {f["overall_width"]:.0f} mm, '
      f'track {2 * (f["rear_wheel_y"][0] + P["wheel"][1] / 2):.0f} mm, wheelbase {P["wheelbase"]:.0f} mm (axle to axle {f["front_axle"][0]:.0f})')
print(f'  drive: HTD-3M {P["drive_teeth"][0]}:{P["drive_teeth"][1]} ({f["ratio"]:.2f}:1), belt {P["drive_belt"][0]}T, c-c {f["drive_cc"]:.1f} mm, '
      f'motor centre ({mx:.1f}, {mz:.1f})')
print(f'  steering: GT2 {P["steer_teeth"]}:{P["steer_teeth"]} belt {P["steer_belt"][0]}T, c-c {f["steer_cc"]:.1f} mm, servo spline at x {f["servo_x"]:.1f}')
print(f'  mass {f["mass"] / 1000:.2f} kg, CG x {f["cg"][0]:.0f} z {f["cg"][1]:.0f} (mast at {P["mast_extension"]:.0f} mm), '
      f'{f["rear_load"] * 100:.0f} % on the rear wheels')

print('\n== Rear drive clearances ==')
gap('motor can to lid underside', top - (mz + mr))
gap('motor can to battery top', (mz - mr) - f['battery_z'][1])
gap('motor can to axle boss', math.hypot(mx, mz - az) - mr - P['axle_boss'][0] / 2)
gap('motor can to bulkhead', P['bulkhead_x'][0] - (mx + mr))
gap('motor can to rear deck', (mx - mr) - P['rear_deck_x1'])
gap('motors apart (inner ends)', 2 * f['motor_y'][0], 10.0)
rb = f['pd_motor'] / 2 + 1.6
gap('drive belt to tub wall (y)', f['drive_belt_y'][0] - P['motor_wall_y'][1], 2.0)
hd, hh, hc = P['motor_screw_head']
head_top = P['motor_wall_y'][1] - hc + hh
gap('motor screw heads (button, 1.5 mm counterbore) to belt (y)', f['drive_belt_y'][0] - head_top, 1.5)
od_m = ns['pd'](P['drive_teeth'][0], P['belt_pitch']) - 2 * 0.381
r_head_in = P['motor_pcd'] / 2 - hd / 2
if r_head_in < od_m / 2 + 1.5 + 1.0:
    gap('motor screw heads to motor pulley flange (y, they overlap radially)', f['motor_pulley_y'][0] - head_top, 0.5)
else:
    print(f'  motor screw heads clear the pulley flange radially ({r_head_in - od_m / 2 - 1.5:.1f} mm)')
gap('shaft past the pulley hub centre (set screw on the shaft) — MEASURE', f['motor_shaft_y'][1] - f['hub_centre_y'], 0.5)
print(f'  motor wall under the motor: {P["motor_wall_y"][1] - f["motor_face_y"]:.0f} mm (face in a {P["motor_pocket"]:.0f} mm pocket)')
gap('drive belt to front-zone wall (x, at the belt plane)', (P['rear_zone_x1'] - 2) - (mx + rb))
gap('motor pulley hub to wheel tire', math.hypot(mx, mz - az) - R - 8.0)
gap('wheel pulley flange to tub wall (y, M8 washer there)', f['wheel_pulley_y'][0] - P['motor_wall_y'][1], 1.5)
gap('rear tire to tub side (y)', f['rear_wheel_y'][0] - P['front_half_w'])
wrap = math.pi - 2 * math.asin((f['pd_wheel'] - f['pd_motor']) / 2 / f['drive_cc'])
teeth_in_mesh = P['drive_teeth'][0] * wrap / (2 * math.pi)
print(f'  20T wrap {math.degrees(wrap):.0f} deg -> {teeth_in_mesh:.1f} teeth in mesh (>= 6 wanted)')
if teeth_in_mesh < 6:
    fail('too few teeth in mesh on the motor pulley')
print(f'  belt tension: motor slides +/-{P["tension_slot"]:.0f} mm along the belt = +/-{2 * P["tension_slot"]:.0f} mm of belt length')

print('\n== Steering sweep (fork + tire turning +/-90 deg) ==')
xs = f['steer_x']
sweep = f['sweep_low']
print(f'  sweep radius below the crown: {sweep:.1f} mm (tire {P["wheel"][0]:.0f} x {P["wheel"][1]:.0f}, trail {P["steer_trail"]:.0f})')
gap('front wall to sweep', (xs - xf) - sweep)
crown_r = math.hypot(f['crown_x'], f['fork_leg_y'][1])
gap('crown corners to nose cheeks (y)', P['cheek_y'][0] - crown_r)
gap('crown top to cheek/bracket bottom (z)', 110.0 - f['crown_z'][1])
gap('crown corners to HC-SR04 bracket (x)', (P['nose_x1'] - 4.0) - (xs + crown_r))
od_s = ns['pd'](P['steer_teeth'], P['gt2_pitch']) - 2 * 0.254
gap('pulley flange to head boss (z)', P['head_z'][0] - (P['steer_belt_z'][1] + 1.5), 0.8)
gap('stem top (magnet) to AS5600 chip', f['as5600_z'] - f['stem_z'][1], 0.5)
gap('stem collar to AS5600 bridge posts', math.hypot(8.0, 17.0) - 3.5 - P['stem_collar'][0] / 2)
ld_ = P['lidar']
gap('AS5600 bridge to lidar (x)', (ld_['x'] - ld_['size'][0] / 2) - (xs + 10.0), 1.0)
gap('pulley flange to nose root (x)', (xs - od_s / 2 - 1.5) - (xf + 10.0))
gap('steering belt to belt window (y)', 21.0 - (od_s / 2 + 1.5), 1.5)
sb_z0 = f['stem_bolt_head_z'][0] - f['stem_bolt']
gap('stem bolt tip above the front tire', sb_z0 - P['wheel'][0], 2.0)
gap('crown nut pocket floor (skin under the M8 nut at the crown bottom)', f['crown_nut_z'][0] - f['crown_z'][0] + 0.0001, 0.0)
print(f'  steering servo slides +/-{P["steer_tension"]:.0f} mm on M3 bolts in slots = +/-{2 * P["steer_tension"]:.0f} mm of belt')
gap('servo tabs to front wall', (xf - P['wall_t']) - (f['servo_body_cx'] + P['servo']['tab_len'] / 2))
gap('fork leg bottom above ground', az - P['fork_leg'][1] / 2, 25.0)

print('\n== Electronics placement ==')
pox, poy = P['pi_origin']
gap('Pi 5 to steering servo (y)', poy - (P['servo']['body'][1] / 2 + P['fit_servo']), 2.0)
gap('VESC to steering servo (y)', -(P['vesc_origin'][1] + P['vesc'][1]) - (P['servo']['body'][1] / 2 + P['fit_servo']), 2.0)
gap('VESC top to servo shelf', f['shelf_z'][0] - (fz1 + 1.0 + P['vesc'][2]))
print(f'  Pi USB/Ethernet end faces the bulkhead; the bulkhead window (y 10..{P["front_half_w"] - P["wall_t"] - 3:.0f}, up to z 86) lets plugs and '
      f'cables run back {pox - (mx + mr):.0f} mm over the battery to the motors')
if pox - (mx + mr) < 40:
    fail('less than 40 mm of plug room behind the Pi')
gap('ESP32 (right of the centre line) to the Pi window edge (y)', 10.0 - (-10.0), 5.0)
gap('Pi to front wall (x)', (xf - P['wall_t']) - (pox + P['pi_board'][0]), 2.0)
gap('Pi fan to lid (z, cooler ~14 mm tall)', top - (fz1 + P['pi_standoff'][1] + 1.6 + 14.0), 10.0)
gap('ESP32 top (USB end) to lid', top - (50.0 + P['esp32_board'][0]), 15.0)
gap('battery to bulkhead (x)', P['bulkhead_x'][0] - f['bay'][2])

print('\n== Mast: fold and slide ==')
px, pz = f['pivot']
sw, st, sl = P['sleeve']
xm = P['mast_x']
ext_fold = 30.0      # lowest pin hole
z_shift = ext_fold - P['mast_extension']
sv = P['servo']
pl = P['pan_plate']
pt_ = f['plate_z'][1] + z_shift


def fold(pt_xz, a_deg=90.0):
    a = math.radians(a_deg)
    dx, dz = pt_xz[0] - px, pt_xz[1] - pz
    return (px + dx * math.cos(a) + dz * math.sin(a), pz - dx * math.sin(a) + dz * math.cos(a))


pts = []
sx0, sx1 = f['sleeve_x']
for z in (pz + 0.1, f['sleeve_z'][1]):
    pts += [((sx0, z), 'sleeve'), ((sx1, z), 'sleeve')]
it = f['inner_top'] + z_shift
iw = f['inner_w']
pts += [((xm - iw / 2, it), 'inner mast'), ((xm + iw / 2, it), 'inner mast')]
hx0 = xm - sv['spline_off'] - sv['tab_len'] / 2 - 2
hx1 = xm - sv['spline_off'] + sv['tab_len'] / 2 + 2
for z in (it, it + P['head_h'] + sv['tab_t'] + sv['tab_from_top']):
    pts += [((hx0, z), 'servo head'), ((hx1, z), 'servo head')]
pts += [((xm - 34.0, pt_), 'pan plate'), ((xm + pl[0], pt_), 'pan plate'),
        ((xm + 19.0, pt_ + 58.0), 'UWB bracket'), ((xm + 19.0, pt_ + 38.0), 'webcam bracket'),
        ((xm + 7.5, pt_ + 15.0), 'GoPro prongs')]
nz1 = P['nose_deck_z'][1]
ld = P['lidar']
lx0 = ld['x'] - ld['size'][0] / 2


def floor_under(x):
    if x < P['rear_deck_x1']:
        return top
    if x < f['lid_x'][1]:
        return f['lid_z'][1]
    xs_ = f['steer_x']
    if xs_ - 14.0 <= x <= xs_ + 10.0:
        return f['as5600_z'] + 1.0 + 1.6 + 3.0
    if x < lx0:
        return nz1
    return nz1 + ld['size'][2]


worst = None
for (q, name) in pts:
    for a in range(0, 91, 5):
        fx, fz = fold(q, a)
        if name == 'sleeve' and q[1] < pz + 5:
            continue
        c = fz - floor_under(fx)
        if worst is None or c < worst[0]:
            worst = (c, name, a, fx, fz)
print(f'  fold at the lowest pin (extension {ext_fold:.0f} mm), GoPro removed (one thumb screw): '
      f'lowest point {worst[1]} at {worst[2]} deg, x {worst[3]:.0f} z {worst[4]:.0f}')
gap('folded mast over the lid / nose / lidar', worst[0], 1.0)
tip = max(fold(q)[0] for q, n in pts)
print(f'  folded tip x {tip:.0f} (lidar starts at x {lx0:.0f}); folded height {max(fold(q)[1] for q, n in pts):.0f} mm, '
      f'lidar top {nz1 + ld["size"][2]:.0f} mm')
print(f'  upright: lens {f["lens_z"]:.0f} mm at extension {P["mast_extension"]:.0f}; range '
      f'{f["lens_z"] + f["mast_ext_min"] - P["mast_extension"]:.0f}-{f["lens_z"] + f["mast_ext_max"] - P["mast_extension"]:.0f} mm')
ov = P['inner_mast'][0] - 90.0
gap('inner mast overlap in the sleeve at the top pin (90)', ov, P['mast_min_overlap'])
gap('inner mast bottom above the sleeve bottom block at the lowest pin (30)', (f['sleeve_z'][1] + 30.0 - P['inner_mast'][0]) - (f['sleeve_z'][0] + f['sleeve_bottom_t']), 1.0)
gap('sleeve cable window above the pivot bolt', (f['sleeve_z'][0] + f['sleeve_bottom_t'] + 2.0) - (f['pivot'][1] + P['m5_clear'] / 2), 3.0)
print(f'  folded size (GoPro off, mast at pin 30): {f["tub_x"][0]:.0f}..{P["nose_x1"] + P["hc_sr04"]["can_h"] - 4:.0f} x '
      f'{f["overall_width"]:.0f} wide x {max(max(fold(q)[1] for q, n in pts), nz1 + ld["size"][2]):.0f} tall')
gap('sleeve front-bottom arc vs pedestal while folding', (f['sleeve_x'][1] - px) - (pz - f['sleeve_z'][0]), 0.0)
gap('lock bolt to sleeve rear face', f['lock'][0] - f['sleeve_x'][0] - P['m4_clear'] / 2, 2.5)
gap('pivot to sleeve front face', f['sleeve_x'][1] - f['pivot'][0] - P['m5_clear'] / 2, 2.5)

print('\n== Fits (print tolerances) ==')
fits = [('6802 head bearing seats', f'bore {P["bearing_6802"][1] + P["fit_press"]:.2f} for a 24.00 OD (snug press)'),
        ('steering stem', f'{P["stem_d"] - P["fit_shaft"]:.2f} in a 15.00 bore (slide)'),
        ('mast telescope', f'{f["inner_w"]:.2f} square in a {P["sleeve"][0] - 2 * P["sleeve"][1]:.2f} opening ({P["fit_slide"]:.2f}/side)'),
        ('hinge knuckle', f'{P["sleeve"][0]:.1f} between cheeks {P["sleeve"][0] + 2 * P["fit_free"]:.1f} apart ({P["fit_free"]:.2f}/side)'),
        ('servo pockets', f'+{P["fit_servo"]:.2f}/side, cable slit 5 x 4 mm at the cable end'),
        ('clearance holes', f'M3 {P["m3_clear"]}, M4 {P["m4_clear"]}, M5 {P["m5_clear"]}, M8 {P["m8_clear"]}'),
        ('heat-set inserts', f'M3 {P["insert_m3"][0]} x {P["insert_m3"][1]} deep, M5 {P["insert_m5"][0]} x {P["insert_m5"][1]}'),
        ('self-tapping pilots', f'M2 {P["m2_tap"]}, M2.5 {P["m25_tap"]}, M3 {P["m3_tap"]}'),
        ('battery bay', f'{P["battery_slop"]:.1f}/side around the pack, guide rails'),
        ('wheel pulley', f'bore {P["wheel_pulley"]["bore"]:.1f} over the {P["spacer_tube"][0]:.0f} mm spacer (never touches it)'),
        ('spacer noses', f'{P["spacer_tube"][2]:.1f} mm: clear of the 608 seals and outer races, clamp the inner races only'),
        ('fork inner-race spacers', f'0.1 mm from the wheel bearings: the axle nut clamps the inner races, not the wheel')]
for a, b_ in fits:
    print(f'  {a}: {b_}')
if P['fit_press'] < 0.1 or P['fit_slide'] < 0.2 or P['fit_servo'] < 0.3:
    fail('a fit is tighter than the printer can hold')

print('\n== Stability ==')
m = f['mass'] / 1000
h = f['cg'][1] / 1000
cgx = f['cg'][0] / 1000
L = f['front_axle'][0] / 1000
Rl = 1 - cgx / L
k = MU * h / L
brake = MU * Rl / (1 + k)
lift = cgx / h
yw = (f['rear_wheel_y'][0] + P['wheel'][1] / 2) / 1000
# tip line: rear contact (0, -yw) to front contact (L, 0); distance from CG (cgx, 0)
dx, dy = L, yw
d_line = abs(dy * cgx - dx * 0 + dx * (-yw) * 0 - dy * 0 + (L * 0 - 0)) if False else abs((L - 0) * (-yw - 0) - (0 - cgx) * (0 + yw)) / math.hypot(L, yw)
tip = d_line / h
print(f'  CG {h * 1000:.0f} mm up, {Rl * 100:.0f} % on the rear axle')
print(f'  braking (rear wheels only): {brake:.2f} g -> stop from 15 mph in {6.7 ** 2 / (2 * brake * g) + 6.7 * 0.25:.1f} m')
print(f'  front lifts under acceleration at {lift:.2f} g (grip allows {MU * Rl / (1 - k):.2f} g)')
print(f'  side tip about the rear-to-front wheel line: {tip:.2f} g (grip {MU:.1f} g) -> ESP32 lateral cap 0.4 g')
if tip < 0.6:
    fail('side tip under 0.6 g')
if lift < 0.45:
    fail('front lifts under 0.45 g (the ESP32 caps acceleration at 0.3 g)')
print(f'  ground clearance: floor {fz0:.0f} mm, axle nuts {az - 6.5:.0f} mm, fork legs {az - P["fork_leg"][1] / 2:.0f} mm; '
      f'breakover {math.degrees(2 * math.atan(fz0 / (f["front_axle"][0] / 2))):.0f} deg, '
      f'departure {math.degrees(math.atan(fz0 / -xr)):.0f} deg')

print('\n== Stress (5 g bump, 0.8 g grip) ==')
Wr = m * g * Rl / 2               # static per rear wheel (N)
Wf = m * g * (1 - Rl)             # front wheel
d_root = 6.47


def sig_bend(M, d):
    return 32 * M / (math.pi * (d / 1000) ** 3) / 1e6


lev = (f['rear_wheel_y'][0] + P['wheel'][1] / 2) - P['motor_wall_y'][1]
M_ax = 5 * Wr * lev / 1000
s_ax = sig_bend(M_ax, d_root)
print(f'  rear axle M8 (8.8, yield 640): {5 * Wr:.0f} N at {lev:.0f} mm -> {s_ax:.0f} MPa')
if s_ax > 320:
    fail('rear axle bolt over half of yield')
boss_len = P['motor_wall_y'][1] - P['axle_boss'][1]
p_boss = (M_ax / (2 / 3 * boss_len / 1000)) / (8 * boss_len) if boss_len else 0
print(f'  axle boss ({boss_len:.0f} mm long): bearing pressure on the printed hole {p_boss:.1f} MPa (PAHT-CF ~50)')
M_w = M_ax
sec = 60.0 * P['motor_wall_y'][1] ** 0 * (P['motor_wall_y'][1] - P['motor_wall_y'][0]) ** 2 / 6
s_wall = M_w * 1000 / sec
print(f'  side wall at the axle (60 mm strip, {P["motor_wall_y"][1] - P["motor_wall_y"][0]:.0f} mm thick): {s_wall:.1f} MPa (+ boss and floor share it)')
if s_wall > 25:
    fail('side wall at the axle over 25 MPa')
F_m = 5 * P['mass']['motor'] / 1000 * g
t_mw = P['motor_wall_y'][1] - f['motor_face_y']
s_mw = (F_m * P['motor_len'] / 2) / (50.0 * t_mw ** 2 / 6)
print(f'  motor wall ({t_mw:.0f} mm under the motor, {t_mw - P["motor_screw_head"][2]:.1f} mm under the screw heads), 5 g bump on the motor: {s_mw:.1f} MPa')
if s_mw > 15:
    fail('motor wall over 15 MPa')
T0 = 40.0
s_shaft = sig_bend(2 * T0 * ((f['drive_belt_y'][0] + f['drive_belt_y'][1]) / 2 - P['motor_wall_y'][0] + 5.0) / 1000, P['motor_shaft'][0])
print(f'  motor shaft Ø8: belt preload 2 x {T0:.0f} N -> {s_shaft:.0f} MPa (steel, fine)')
Fwheel = MU * Wr * 2
belt_pull = Fwheel / 2 * R / (f['pd_wheel'] / 2)
print(f'  drive belt pull at the grip limit: {belt_pull:.0f} N per side (HTD-3M 10 mm: ~250 N allowable, estimate)')
bolt_sh = belt_pull * f['pd_wheel'] / 2 / (P['wheel_pulley']['bolt_pcd'] / 2) / 3
print(f'  wheel pulley bolts (3 x M4): {bolt_sh:.0f} N each in shear; printed disc bearing {bolt_sh / (4 * P["wheel_pulley"]["disc_t"]):.1f} MPa')
# fork legs: lateral grip at the contact patch, lever from the ground to the crown
Fl = MU * Wf * 2.0               # include 2x for kerb strikes
Mleg = Fl * f['crown_z'][0] / 1000
I_leg = 2 * P['fork_leg'][1] * P['fork_leg'][0] ** 3 / 12 + 2 * P['fork_leg'][1] * P['fork_leg'][0] * f['fork_leg_y'][0] ** 2
s_leg = Mleg * 1000 * f['fork_leg_y'][1] / I_leg
print(f'  fork legs: {Fl:.0f} N sideways at the ground, {Mleg:.2f} N*m at the crown -> {s_leg:.1f} MPa')
if s_leg > 25:
    fail('fork legs over 25 MPa')
span = (P['head_z'][1] - P['bearing_6802'][2] / 2) - (P['head_z'][0] + P['bearing_6802'][2] / 2)
Mh = Fl * ((P['head_z'][0] + P['head_z'][1]) / 2) / 1000
Fb = Mh / (span / 1000)
print(f'  head bearings {span:.0f} mm apart: {Fb:.0f} N radial each under that side load (6802 static rating ~1300 N)')
if Fb > 650:
    fail('head bearings over half their static rating')
lever_nose = xs - xf
F_n = 5 * Wf
Z_deck = 2 * P['nose_half_w'] * (P['nose_deck_z'][1] - P['nose_deck_z'][0]) ** 2 / 6 + 2 * 4.0 * (P['nose_deck_z'][0] - 110.0) ** 2 / 6
s_nose = F_n * lever_nose / Z_deck
print(f'  nose (deck + cheeks) at the front wall: {F_n:.0f} N at {lever_nose:.0f} mm -> {s_nose:.1f} MPa')
# steering torque
T_scrub = MU * Wf * 0.008
T_trail = MU * Wf * P['steer_trail'] / 1000
# frontal wheel strike at 3 g: moment at the lower head bearing, carried by the M8 stem bolt (steel core)
F_hit = 3 * m * g
M_hit = F_hit * (P['head_z'][0] + P['bearing_6802'][2] / 2 - P['axle_z']) / 1000
s_bolt = sig_bend(M_hit, 6.47)
s_pr = 32 * M_hit / (math.pi * ((P['stem_d'] / 1000) ** 4 - (P['m8_clear'] / 1000) ** 4) / (P['stem_d'] / 1000)) / 1e6
print(f'  frontal wheel strike (3 g, {F_hit:.0f} N at the axle): {M_hit:.1f} N*m at the lower head bearing -> M8 stem bolt {s_bolt:.0f} MPa '
      f'(10.9 yields at 940); the printed stem alone would see {s_pr:.0f} MPa across its layers')
if s_bolt > 600:
    fail('stem bolt near yield in a 3 g strike')
print(f'  steering torque needed: ~{T_scrub + T_trail:.2f} N*m (scrub at rest + trail); a 20 kg*cm servo gives 1.96')
F_sb = 1.96 / (od_s / 2 / 1000)
print(f'  steering belt at servo stall: {F_sb:.0f} N (GT2 9 mm: ~100 N working): it can SKIP on a wheel strike, so the AS5600 on the '
      f'stem reads the real angle and the ESP32 stops on a mismatch')
# mast
m_top = (P['mass']['servo'] + P['mass']['pan_stage'] + P['mass']['gopro'] + P['mass']['webcam'] + P['mass']['uwb']) / 1000
h_top = (f['lens_z'] - pz) / 1000
M_mast = m_top * g * 1.5 * h_top + P['mass']['mast'] / 1000 * g * 1.5 * h_top / 2
F_bolts = M_mast / (abs(px - f['lock'][0]) / 1000)
print(f'  mast: {M_mast:.2f} N*m at the hinge (1.5 g rocking) -> {F_bolts:.0f} N on the pivot/lock couple '
      f'({F_bolts / (P["m5_clear"] * sw):.1f} MPa on the printed holes)')
I_sl = (sw ** 4 - (sw - 2 * st) ** 4) / 12
print(f'  sleeve tube: {M_mast * 1000 * sw / 2 / I_sl:.1f} MPa; cheeks (2 x 7 mm) sideways at 0.8 g: '
      f'{m_top * g * 0.8 * h_top * 1000 / (2 * 36.0 * 7.0 ** 2 / 6):.1f} MPa')
print(f'  inner mast in the sleeve: contact force {M_mast / (P["mast_min_overlap"] / 1000):.0f} N at min overlap')

# acceleration / braking caps enforced by VESC current limits
Kt = 60 / (2 * math.pi * 200)                      # H5055 200KV, N*m/A
F_per_A = Kt * f['ratio'] / (R / 1000)              # wheel force per motor amp
I_acc = 0.3 * m * g / 2 / F_per_A
I_brk = brake * m * g / 2 / F_per_A
print(f'\n== Acceleration caps ==\n  {F_per_A:.2f} N at the wheel per motor amp: 0.3 g needs {I_acc:.1f} A per motor; '
      f'full rear-wheel braking ({brake:.2f} g) {I_brk:.1f} A')
print(f'  VESC Tool: motor current max ~{I_acc * 1.3:.0f} A, motor current max brake ~-{I_brk:.0f} A per motor caps both in hardware '
      f'(the front cannot lift below {lift:.2f} g), plus the same ramps on the ESP32')
print('\n== Cooling (front intake -> electronics -> motors -> lid and rear vents) ==')
a_in = 12 * 4.0 * (98.0 - (fz1 + 8.0))
a_lid = 12 * 3.5 * 44.0 + 5 * 3.5 * 30.0
a_rear = 7 * 4.0 * 26.0 + 5 * 3.5 * 30.0
print(f'  intake {a_in:.0f} mm^2, exhaust {a_lid + a_rear:.0f} mm^2 (lid {a_lid:.0f}, rear {a_rear:.0f})')
Q = 3.0 * a_in * 1e-6
heat = 6.0 + 4.0 + 2.0 + 2 * 5.0
print(f'  ~{heat:.0f} W inside (Pi 6, VESC 4, buck/BEC 2, motors 2 x 5); at 3 m/s ram air {Q * 1000:.1f} L/s -> '
      f'+{heat / (1.2 * 1005 * Q):.0f} K; parked, the Pi fan alone (~2 L/s) -> +{heat / (1.2 * 1005 * 0.002):.0f} K')

print('\nV2 CHECKS PASSED' if ok else '\nV2 CHECKS FAILED')
sys.exit(0 if ok else 1)
