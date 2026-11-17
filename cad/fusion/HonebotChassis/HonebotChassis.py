"""honebot chassis: Fusion 360 script that generates the whole robot (printed parts + reference hardware).

Run it in Fusion 360: Utilities > Add-Ins > Scripts and Add-Ins > "+" (or "Create" > pick this folder),
then select HonebotChassis and click Run. A new design document is created; nothing existing is touched.

This one file is the ONLY geometry source. `cad/check_design.py` imports it (without Fusion) and runs the
clearance / camera-view / hole / structural checks on exactly the same primitives that Fusion builds.

Conventions (all numbers below are millimetres; Fusion's API works in cm, the builder converts):
  x = forward (driving direction), y = left, z = up.  Origin: midpoint of the DRIVE axle line, on the ground.
  Drive wheels are at the FRONT (x = 0), the caster trails at the REAR (negative x). See docs/chassis_cad.md
  for why (braking: weight transfers onto the driven wheels, and front-wheel braking is directionally stable).

Data quality tags used in PARAMS comments:
  [V]  verified from a retrieved source (see docs/chassis_cad.md "Sources")
  [M]  MEASURE with calipers before printing (owner has the part, or it varies by seller)
  [D]  design choice (free to change; re-run cad/check_design.py afterwards)
Parts tagged [M] are held by slots, a 10 mm hole grid or zip-tie slots, never by fixed holes.
"""

import math
import traceback

# ----------------------------------------------------------------------------------------------------------
# Parameters
# ----------------------------------------------------------------------------------------------------------
PARAMS = {
    # fit / tolerance policy (loose on purpose; FDM on a Bambu H2C, PAHT-CF / PETG)
    "clr_m2": 2.4, "clr_m25": 2.9, "clr_m3": 3.4, "clr_m4": 4.5, "clr_m5": 5.5, "clr_m8": 8.5,   # [D] through-holes
    "ins_m3": 4.0, "ins_m3_depth": 6.5, "ins_m4": 5.6, "ins_m4_depth": 9.0, "ins_m5": 6.4, "ins_m5_depth": 10.0,
    # [D] heat-set inserts (check insert maker's chart)
    "slip": 0.6,          # [D] gap for parts that slide into each other (mast in socket, deck between walls)
    "cut_over": 0.6,      # [D] cut tools overshoot faces by this much (robust booleans)
    # drive wheels: 90 x 52 mm longboard wheels with 608 bearings [M: core holes / bearing inset]
    "wheel_d": 90.0, "wheel_w": 52.0, "axle_z": 45.0,
    # tub (main printed body, PAHT-CF)
    "tub_x0": -242.0, "tub_x1": 58.0, "tub_hw": 62.0, "wall": 4.0, "floor_z0": 28.0, "floor_t": 4.0,
    "tub_top": 140.0, "front_t": 6.0, "rear_t": 6.0,
    # battery: 3S 5000 mAh hard case. Tunnel fits <= 145 x 50 x 30 [M]
    "bat_l": 145.0, "bat_w": 50.0, "bat_h": 30.0, "bat_x0": -141.0,
    # electronics deck (removable tray, PETG or PAHT-CF)
    "deck_z0": 79.0, "deck_t": 3.0, "grid_pitch": 10.0,
    # drive: 5008/5010-class outrunner (envelope, [M]) -> HTD-3M 9 mm belt -> printed 48T wheel pulley
    # reference motor: MAD 5010 EEE V2.0 370KV - O56 x 32.7, 5 mm shaft, base M3 on D25/D30, prop M3 on D12/D18
    # [V, retailer listings]; envelope below leaves margin. Any motor with M3 prop holes on D12 fits the pulley.
    "motor_env_d": 62.0, "motor_env_l": 33.0, "motor_shaft_d": 5.0, "motor_prop_pcd": 12.0,
    "motor_dx": -40.0,                              # [D] motor centre 40 mm behind the axle (can clears the idler-pressed 1:1 run)
    "pul_m_t": 28, "pul_w_t": 56, "belt_len": 258.0,   # [D] HTD-3M teeth (PD = 3*T/pi [V]); 258 mm = 86T belt
    "belt_t": 2.0, "belt_pretension_n": 30.0,           # [D] belt back thickness over the tooth tips; set tension
    "axle_sleeve_d": 16.0,                              # [D] stock tubes nested: 12 x 2 inside 16 x 2 (E355 steel)
    "axle_sleeve_E": 210e3, "axle_sleeve_yield": 355.0,
    # 4-wheel skid steer: rear axle, 1:1 belt from the front wheel pulley to the rear wheel, 608 idler tensioner
    "pul_r_t": 36, "belt2_len": 510.0, "belt2_plane_y": 84.0,  # 1:1 front -> rear belt: HTD 510-3M-09 (170T, stock)
    "belt2_take_up": 2.0,       # [D] belt length the idler takes up at nominal; slot covers 0.3 ... 4.5 mm
    "belt2_take_up_range": (0.3, 4.5),   # +/-0.3 % print shrink on 200 mm centres is ~ +/-1.2 mm of belt path
    "idler_r": 11.0,            # 608-2RS outer radius
    # prototype kit (hidden "Prototype kit" group): printed axle sleeves + TPU belts, to drive before buying metal
    "tpu_E": 600.0,             # [V/M] Bambu "TPU for AMS" (68D) TDS: 1190 MPa X-Y, 600 MPa Z; the lower value is used
    "tpu_strength": 22.4,       # [V] same TDS, tensile strength X-Y (belts print flat, so the loop runs in X-Y)
    "tpu_T0": 15.0,             # [D] TPU belt pretension per strand, N; belts print short by T0 / EA so they sit on pitch
    "belt_w": 9.0,
    "motor_plate": 66.0, "motor_plate_t": 8.0, "motor_bolt_r": 27.0, "tension_travel": 8.0,
    "esp_l": 55.0, "esp_w": 28.5,                  # [M] ESP32 DevKit PCB (38-pin DevKit V1 class)
    # front: HC-SR04 bracket tilted up so the beam's lower edge meets the ground beyond slow_distance
    "us_z": 102.0, "us_tilt_deg": 12.0, "us_beam_half_deg": 15.0,   # [M] beam half-angle: measure (see docs)
    # camera mast (removable), standard servo pan head, GoPro HERO9
    "pi_x0": -100.0,       # [D] Pi 5 rear (USB) edge; board is 85 long [V]
    "mast_x": 38.0, "mast_sq": 22.0, "mast_wall": 3.0, "servo_seat_z": 195.0,
    "gopro_w": 71.0, "gopro_h": 55.0, "gopro_d": 33.6, "gopro_mass": 0.158,      # [V]
    "gopro_lens_dy": 17.0, "gopro_lens_dz": 6.0, "gopro_lens_protrude": 5.0,    # [M] approx. lens centre offset
    "pan_half_range_deg": 45.0, "cam_tilt_deg": 0.0,                             # [D] matches camera_aim default
    # standard servo body (MG996R class) [V body 40.7 x 19.7 x 42.9, ears 54 wide]; ear seat height [M]
    "servo_l": 40.7, "servo_w": 19.7, "servo_h": 42.9, "servo_ear_span": 54.0, "servo_ear_z": 27.0,
    "servo_shaft_off": 10.0,                       # [M] shaft offset from body centre along its length
    # 9 g servo (SG90 class) for the adapter [V body 22.2 x 11.8, ears ~32]
    "sg90_l": 22.2, "sg90_w": 11.8, "sg90_shaft_off": 5.9,
}

# component colours (r, g, b)
COL = {
    "frame": (40, 40, 44), "shell": (255, 122, 26), "accent": (20, 190, 180), "light": (235, 235, 235),
    "mast": (30, 30, 34), "hw_dark": (60, 60, 60), "hw_metal": (170, 170, 175), "pcb_green": (30, 120, 60),
    "pcb_blue": (30, 70, 170), "battery": (230, 200, 40), "wheel": (245, 245, 240), "keepout": (255, 40, 40),
    "belt": (25, 25, 25),
}

# ----------------------------------------------------------------------------------------------------------
# Primitive helpers (pure python; shared by Fusion builder and checker)
#   box: oriented box  {"t":"box","c":centre,"s":(len_u,len_v,len_w),"u":dir,"v":dir}  (w = u x v)
#   cyl: cylinder      {"t":"cyl","a":p0,"b":p1,"r":radius}
# ----------------------------------------------------------------------------------------------------------
X, Y, Z = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
# motor-plate bolts (x, z unit offsets): front, top, rear -> all three heads sit above the deck, reachable
MOTOR_PLATE_BOLTS = ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0))


def box(x0, x1, y0, y1, z0, z1):
    return {"t": "box", "c": ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
            "s": (abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)), "u": X, "v": Y}


def obox(c, lu, lv, lw, u, v):
    return {"t": "box", "c": tuple(c), "s": (lu, lv, lw), "u": tuple(u), "v": tuple(v)}


def cyl(a, b, r):
    return {"t": "cyl", "a": tuple(a), "b": tuple(b), "r": r}


def sph(c, r):
    return {"t": "sph", "c": tuple(c), "r": r}


def tor(c, R, r):
    """Torus around a vertical (z) axis through c: ring radius R, tube radius r. Needs R >= r (no spindle torus)."""
    assert R >= r, "spindle torus (R %.2f < r %.2f): use a smaller edge radius or a larger corner radius" % (R, r)
    return {"t": "tor", "c": tuple(c), "R": R, "r": r}


def pdiff(a, bs):
    """a minus every primitive in bs (used for edge-rounding cutters and true belt wrap arcs)."""
    return {"t": "diff", "a": a, "b": list(bs)}


def round_edge_cut(p0, p1, inward_a, inward_b, R):
    """Cutter that rounds a straight convex edge p0-p1 with radius R. inward_a/b: unit vectors pointing into
    the solid along its two faces. Returns diff(square prism along the edge, cylinder)."""
    axis = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    L = math.sqrt(sum(v * v for v in axis))
    u = norm(axis)
    mid = vmul(vadd(p0, p1), 0.5)
    c = vadd(mid, vadd(vmul(inward_a, R / 2 - 0.3), vmul(inward_b, R / 2 - 0.3)))
    sq = obox(c, L + 1.0, R + 0.6, R + 0.6, u, inward_a)
    cc0 = vadd(p0, vadd(vmul(inward_a, R), vmul(inward_b, R)))
    cc1 = vadd(p1, vadd(vmul(inward_a, R), vmul(inward_b, R)))
    return pdiff(sq, [cyl(vadd(cc0, vmul(u, -1.0)), vadd(cc1, vmul(u, 1.0)), R)])


def chamfer_cut(p0, p1, inward_a, inward_b, c):
    """45-degree chamfer of size c on a convex edge p0-p1 (bed-side edges; prints without support)."""
    axis = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    L = math.sqrt(sum(v * v for v in axis))
    u = norm(axis)
    v = norm(vadd(inward_a, inward_b))
    return obox(vmul(vadd(p0, p1), 0.5), L + 2.0, c * math.sqrt(2), c * math.sqrt(2), u, v)


def belt_loop(c_big, r_big, c_small, r_small, t, y, w):
    """Open-belt loop in the x-z plane at y: tangent strands + wrap arcs. Radii are the pulley tip radii; the belt
    back is t thick outside them."""
    (wx, wz), (mx_, mz_) = c_big, c_small
    dx, dz = mx_ - wx, mz_ - wz
    C = math.hypot(dx, dz)
    d = (dx / C, dz / C)
    pr = (-d[1], d[0])
    rb, rs = r_big + t / 2, r_small + t / 2           # belt mid-line radii
    sb = (rb - rs) / C
    cb = math.sqrt(max(0.0, 1 - sb * sb))
    out = []
    for sg in (1, -1):
        n = (sb * d[0] + sg * cb * pr[0], sb * d[1] + sg * cb * pr[1])     # outward normal: n.d = (rb - rs) / C
        tb = (wx + rb * n[0], wz + rb * n[1])
        ts = (mx_ + rs * n[0], mz_ + rs * n[1])
        L = math.hypot(ts[0] - tb[0], ts[1] - tb[1])
        u = ((ts[0] - tb[0]) / L, 0.0, (ts[1] - tb[1]) / L)
        cen = ((tb[0] + ts[0]) / 2, y, (tb[1] + ts[1]) / 2)
        out.append(obox(cen, L + 0.6, t, w, u, (n[0], 0.0, n[1])))
    big = 4 * r_big
    for (cx, cz, r, sgn) in ((wx, wz, r_big, 1), (mx_, mz_, r_small, -1)):
        # chord through the tangent points sits at (X - c).d = r_mid * sb ; unwrapped side: + for big, - for small
        rm = r + t / 2
        off = rm * sb + sgn * big / 2
        hc = (cx + d[0] * off, y, cz + d[1] * off)
        half = obox(hc, big, big * 2, w + 2, (d[0], 0.0, d[1]), (pr[0], 0.0, pr[1]))
        ring_out = cyl((cx, y - w / 2, cz), (cx, y + w / 2, cz), r + t)
        ring_in = cyl((cx, y - w / 2 - 1, cz), (cx, y + w / 2 + 1, cz), r)
        out.append(pdiff(ring_out, [ring_in, half]))
    return out


def belt_path(circles):
    """Closed belt around circles [(cx, cz, rho)] listed in counter-clockwise travel order (x right, z up).
    rho > 0: the belt wraps the circle (pulley); rho < 0: the belt's back runs over it (idler). Returns
    (length, strands [(p_a, p_b, u, r)], arcs [(cx, cz, rho, th_in, dth)]) where r is the right-hand normal."""
    n = len(circles)
    strands, rr = [], []
    for i in range(n):
        (ax, az_, ra), (bx, bz, rb) = circles[i], circles[(i + 1) % n]
        dx, dz = bx - ax, bz - az_
        L = math.hypot(dx, dz)
        psi = math.atan2(dz, dx) - math.acos(max(-1.0, min(1.0, (ra - rb) / L)))
        r = (math.cos(psi), math.sin(psi)); u = (-r[1], r[0])
        pa = (ax + ra * r[0], az_ + ra * r[1]); pb = (bx + rb * r[0], bz + rb * r[1])
        strands.append((pa, pb, u, r)); rr.append(r)
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b, _, _ in strands)
    arcs = []
    for i in range(n):
        cx, cz, rho = circles[i]
        r_in, r_out = rr[i - 1], rr[i]
        t_in, t_out = math.atan2(r_in[1], r_in[0]), math.atan2(r_out[1], r_out[0])
        d = ((t_out - t_in) if rho > 0 else (t_in - t_out)) % (2 * math.pi)
        length += abs(rho) * d
        arcs.append((cx, cz, rho, t_in, d))
    return length, strands, arcs


def belt_loop_n(circles, t, y, w):
    """Solid belt loop for belt_path() circles given as (cx, cz, r_contact, sign): r_contact = pulley tip radius
    (sign +1, belt outside it) or idler radius (sign -1, belt back on it). The belt is t thick."""
    mid = [(cx, cz, sg * (rc + t / 2)) for cx, cz, rc, sg in circles]
    _, strands, arcs = belt_path(mid)
    out = []
    for pa, pb, u, r in strands:
        L = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
        if L > 0.5:
            out.append(obox(((pa[0] + pb[0]) / 2, y, (pa[1] + pb[1]) / 2), L + 0.6, t, w, (u[0], 0.0, u[1]), (r[0], 0.0, r[1])))
    for (cx, cz, rho, t_in, d), (_, _, rc, sg) in zip(arcs, circles):
        if d < math.radians(2):
            continue
        a0, a1 = t_in, t_in + (d if rho > 0 else -d)
        sgn = 1 if rho > 0 else -1
        p1 = (cx + rho * math.cos(a0), cz + rho * math.sin(a0)); p2 = (cx + rho * math.cos(a1), cz + rho * math.sin(a1))
        am = t_in + (d / 2 if rho > 0 else -d / 2)
        pm = (cx + rho * math.cos(am), cz + rho * math.sin(am))
        e = (p2[0] - p1[0], p2[1] - p1[1]); el = math.hypot(*e); e = (e[0] / el, e[1] / el)
        nn = (-e[1], e[0])
        q = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
        sm = 1 if (pm[0] - q[0]) * nn[0] + (pm[1] - q[1]) * nn[1] > 0 else -1
        big = 4 * (rc + t)
        hc = (q[0] - sm * nn[0] * big / 2, y, q[1] - sm * nn[1] * big / 2)
        half = obox(hc, big * 2, big, w + 2, (e[0], 0.0, e[1]), (nn[0], 0.0, nn[1]))
        ring_out = cyl((cx, y - w / 2, cz), (cx, y + w / 2, cz), rc + t)
        ring_in = cyl((cx, y - w / 2 - 1, cz), (cx, y + w / 2 + 1, cz), rc)
        out.append(pdiff(ring_out, [ring_in, half]))
    return out


def soft_slab(x0, x1, y0, y1, z0, z1, R, r, top=True, bottom=False, nseg=5):
    """Plan-rounded slab (corner radius R) whose top (and/or bottom) edges are rounded with radius r."""
    r = min(r, (z1 - z0) / 2 - 0.01, R)
    # clean-mesh rules: straight edges must exist between the corners (no duplicate corner surfaces) and the corner
    # torus must not be a spindle/horn torus
    assert x1 - x0 > 2 * R + 0.5 and y1 - y0 > 2 * R + 0.5, "soft_slab: corner radius %.1f too big for %.1f x %.1f" % (R, x1 - x0, y1 - y0)
    assert R - r >= r or R - r < 0.6, "soft_slab: edge radius %.1f too big for corner radius %.1f (need r <= R/2)" % (r, R)
    zc0 = z0 + (r if bottom else 0.0)
    zc1 = z1 - (r if top else 0.0)
    out = list(rbox(x0, x1, y0, y1, zc0, zc1, R)) if zc1 - zc0 > 0.05 else []
    levels = ([(zc1, z1, 1)] if top else []) + ([(z0, zc0, -1)] if bottom else [])
    for za, zb, sgn in levels:
        ze = za if sgn > 0 else zb          # height of the edge-cylinder axes
        Ri = max(R - r, 0.0)
        if Ri > 0.6:
            out += rbox(x0 + r, x1 - r, y0 + r, y1 - r, min(za, zb), max(za, zb), Ri)
        else:
            out.append(box(x0 + r, x1 - r, y0 + r, y1 - r, min(za, zb), max(za, zb)))
        # straight edges
        if x1 - x0 - 2 * R > 0.1:
            for yy in (y0 + r, y1 - r):
                out.append(cyl_x(x0 + R, x1 - R, yy, ze, r))
        if y1 - y0 - 2 * R > 0.1:
            for xx in (x0 + r, x1 - r):
                out.append(cyl_y(y0 + R, y1 - R, xx, ze, r))
        # corners: spheres along the corner arc (centre radius R - r). (An exact torus was tried on 2026-11-17: Fusion's
        # booleans failed on its tangent joins and left loose donuts, so keep the sphere chain that Fusion handles.)
        for cx, cy, ax, ay in ((x0 + R, y0 + R, -1, -1), (x1 - R, y0 + R, 1, -1), (x0 + R, y1 - R, -1, 1), (x1 - R, y1 - R, 1, 1)):
            n = 1 if Ri < 0.6 else nseg
            for k in range(n):
                t = (math.pi / 2) * (k / (n - 1) if n > 1 else 0.5)
                out.append(sph((cx + ax * Ri * math.cos(t), cy + ay * Ri * math.sin(t), ze), r))
    return out


def cyl_x(x0, x1, y, z, r):
    return cyl((x0, y, z), (x1, y, z), r)


def cyl_y(y0, y1, x, z, r):
    return cyl((x, y0, z), (x, y1, z), r)


def cyl_z(z0, z1, x, y, r):
    return cyl((x, y, z0), (x, y, z1), r)


def rbox(x0, x1, y0, y1, z0, z1, r):
    """Box with rounded vertical edges (plan-view corner radius r)."""
    r = min(r, (x1 - x0) / 2 - 0.01, (y1 - y0) / 2 - 0.01)
    out = [box(x0 + r, x1 - r, y0, y1, z0, z1), box(x0, x1, y0 + r, y1 - r, z0, z1)]
    for cx in (x0 + r, x1 - r):
        for cy in (y0 + r, y1 - r):
            out.append(cyl_z(z0, z1, cx, cy, r))
    return out


def rbox_y(x0, x1, y0, y1, z0, z1, r):
    """Box with rounded edges parallel to y (side-view corner radius r)."""
    r = min(r, (x1 - x0) / 2 - 0.01, (z1 - z0) / 2 - 0.01)
    out = [box(x0 + r, x1 - r, y0, y1, z0, z1), box(x0, x1, y0, y1, z0 + r, z1 - r)]
    for cx in (x0 + r, x1 - r):
        for cz in (z0 + r, z1 - r):
            out.append(cyl_y(y0, y1, cx, cz, r))
    return out


def vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vmul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    n = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
    return (a[0] / n, a[1] / n, a[2] / n)


def slot(p0, p1, width, depth_dir, depth, centre_depth):
    """Rounded slot from p0 to p1 (centres), cut `depth` along depth_dir, centred at centre_depth offset."""
    d = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    L = math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2)
    off = vmul(depth_dir, centre_depth)
    a0 = vadd(vadd(p0, off), vmul(depth_dir, -depth / 2))
    out = [cyl(a0, vadd(a0, vmul(depth_dir, depth)), width / 2)]
    a1 = vadd(vadd(p1, off), vmul(depth_dir, -depth / 2))
    out.append(cyl(a1, vadd(a1, vmul(depth_dir, depth)), width / 2))
    if L > 1e-6:
        u = norm(d)
        c = vadd(vmul(vadd(p0, p1), 0.5), off)
        out.append(obox(c, L, depth, width, u, depth_dir))
    return out


# ----------------------------------------------------------------------------------------------------------
# Model
# ----------------------------------------------------------------------------------------------------------
class Model:
    def __init__(self):
        self.parts = []      # dicts: name, group, color, adds, cuts, mass, material, moving, note
        self.holes = []      # audit records
        self.routes = []     # wiring: name, points, gauge
        self.info = {}       # derived numbers for the checker / docs
        self.nests = []      # (a, b, why): hardware pairs that legitimately interpenetrate in this simplified model
        self.removals = []   # (group, direction, travel_mm, already_removed): assembly / service sweeps

    def part(self, name, group, color, adds, cuts=None, mass=None, material=None, moving=False, note="",
             mates=(), hidden=False, pre=None):
        """pre: optional list of (adds, cuts) stages built first, e.g. an outer shell minus its cavity; the part's own
        adds are then unioned and its cuts (holes, pockets) applied last: body = U(A_i - C_i) U A - C."""
        p = {"name": name, "group": group, "color": color, "adds": list(adds), "cuts": list(cuts or []),
             "mass": mass, "material": material, "moving": moving, "note": note, "mates": set(mates),
             "hidden": hidden, "pre": [(list(a), list(c)) for a, c in (pre or [])]}
        self.parts.append(p)
        return p

    def get(self, name):
        for p in self.parts:
            if p["name"] == name:
                return p
        raise KeyError(name)

    def hole(self, host, hid, a, b, d, kind="clear", for_item="", audit=True):
        """Cylindrical hole along a->b in host part. kind: clear | insert | seat | slot."""
        self.get(host)["cuts"].append(cyl(a, b, d / 2))
        self.holes.append({"host": host, "id": hid, "a": a, "b": b, "d": d, "kind": kind, "for": for_item,
                           "audit": audit})

    def slot_hole(self, host, hid, p0, p1, w, depth_dir, depth, for_item="", centre_depth=0.0, audit=True):
        self.get(host)["cuts"].extend(slot(p0, p1, w, depth_dir, depth, centre_depth))
        mid = vmul(vadd(p0, p1), 0.5)
        a = vadd(vadd(mid, vmul(depth_dir, centre_depth)), vmul(depth_dir, -depth / 2))
        b = vadd(a, vmul(depth_dir, depth))
        self.holes.append({"host": host, "id": hid, "a": a, "b": b, "d": w, "kind": "slot", "for": for_item,
                           "slot": (p0, p1), "audit": audit})


def build_model(P=None):
    P = dict(PARAMS, **(P or {}))
    m = Model()
    co = P["cut_over"]
    hw = P["tub_hw"]; wl = P["wall"]; inner = hw - wl
    x0, x1 = P["tub_x0"], P["tub_x1"]
    zf0 = P["floor_z0"]; zf1 = zf0 + P["floor_t"]; top = P["tub_top"]
    xf_in = x1 - P["front_t"]; xr_in = x0 + P["rear_t"]
    az = P["axle_z"]; wr = P["wheel_d"] / 2
    dz0, dz1 = P["deck_z0"], P["deck_z0"] + P["deck_t"]

    # drive geometry (all derived): belt centre distance from the belt length, belt plane from the motor length
    P["pul_m_pd"] = 3.0 * P["pul_m_t"] / math.pi
    P["pul_w_pd"] = 3.0 * P["pul_w_t"] / math.pi
    P["pul_r_pd"] = 3.0 * P["pul_r_t"] / math.pi
    lo_, hi_ = 20.0, 200.0
    for _ in range(80):
        cc = (lo_ + hi_) / 2
        Lc = 2 * cc + math.pi * (P["pul_m_pd"] + P["pul_w_pd"]) / 2 + (P["pul_w_pd"] - P["pul_m_pd"]) ** 2 / (4 * cc)
        lo_, hi_ = (cc, hi_) if Lc < P["belt_len"] else (lo_, cc)
    P["belt_cc"] = cc
    P["belt_plane_y"] = hw + P["motor_env_l"] + 1.5 + 5.5      # pulley bolted to the bell: flange 1.5 + half tooth width
    P["wheel_inner_y"] = P["belt_plane_y"] + 7.0 + 3.5         # wheel pulley half width + adapter disc
    # 1:1 belt: slack the idler takes up (belt longer than the straight 2C + pi*PD loop)
    # 1:1 belt: wheelbase from the belt length (straight loop = belt - take-up), idler pressing the upper run down at
    # mid-span; slot ends where it takes up 0.2 and 2.0 mm (belt length / print tolerance)
    P["wheelbase"] = (P["belt2_len"] - math.pi * P["pul_r_pd"] - P["belt2_take_up"]) / 2
    xra = -P["wheelbase"]                      # rear axle x
    AX = (("F", 0.0), ("R", xra))              # axles: front (driven by the motor belt), rear (driven by the 1:1 belt)
    P["idler_x"] = xra / 2
    rp2, rip = P["pul_r_pd"] / 2, P["idler_r"] + P["belt_t"] - 0.38      # pitch radii (idler: on the belt back)
    z_touch = az + rp2 + rip

    def idler_z(extra):
        lo, hi = z_touch - 40, z_touch
        for _ in range(60):
            zm = (lo + hi) / 2
            Lm = belt_path([(0.0, az, rp2), (P["idler_x"], zm, -rip), (xra, az, rp2)])[0]
            lo, hi = (zm, hi) if Lm - (P["belt2_len"] - P["belt2_take_up"]) > extra else (lo, zm)
        return (lo + hi) / 2
    izc = idler_z(P["belt2_take_up"])
    iz_hi, iz_lo = idler_z(P["belt2_take_up_range"][0]), idler_z(P["belt2_take_up_range"][1])
    m.info["idler_travel"] = (iz_lo, izc, iz_hi)
    mdx = P["motor_dx"]; mdz = math.sqrt(P["belt_cc"] ** 2 - mdx ** 2)
    mc_x, mc_z = mdx, az + mdz
    bu = norm((mdx, 0.0, mdz))                 # belt direction (axle -> motor)
    bv = norm(cross(bu, Y))                    # in-plane perpendicular; bu x bv = +y
    if cross(bu, bv)[1] < 0:
        bv = vmul(bv, -1)
    m.info.update(motor_centre=(mc_x, mc_z), belt_u=bu, belt_v=bv, rear_axle_x=xra)
    by = P["belt_plane_y"]; wiy = P["wheel_inner_y"]; by2 = P["belt2_plane_y"]
    wpr = P["pul_w_pd"] / 2 - 0.38            # 56T tooth tip radius (pitch line sits 0.38 mm outside the tips)
    rpr = P["pul_r_pd"] / 2 - 0.38            # 28T tooth tip radius
    wfl = wpr + 3.0; rfl = rpr + 3.0          # flange radii
    sd = P["axle_sleeve_d"]

    # ---------------- TUB (one piece: both axles, battery tunnel, mast socket) ----------------
    tub_adds = [
        box(x0, x1, -hw, hw, zf0, zf1),                                  # floor
        box(x0, x1, inner - 0.01, hw, zf0, top), box(x0, x1, -hw, -inner + 0.01, zf0, top),   # side walls
        box(xf_in - 0.01, x1, -hw, hw, zf0, top),                        # front wall
        box(x0, xr_in + 0.01, -hw, hw, zf0, top),                        # rear wall
        box(P["mast_x"] - 17, xf_in + 0.01, -24, 24, dz1, top),          # mast socket block (behind front wall)
    ]
    for _, ax in AX:   # axle tie beam: carries each axle's bending couple across the tub (with the floor); 62 mm bridge
        tub_adds.append(box(ax - 12, ax + 12, -inner, inner, 66, dz0))
    m.info["axle_tie"] = {"w": 24.0, "z0": 66.0, "z1": dz0, "boss_len": 36.0, "floor_t": P["floor_t"], "zf0": zf0}
    # print floor-down without supports: everything hanging off a wall either runs to the floor or has a 45 deg underside.
    # 45 deg wedge under the mast socket block (rotated square, clipped inside the front wall)
    hd = xf_in - (P["mast_x"] - 17) - 0.2
    tub_adds.append(pdiff(obox((xf_in, 0.0, dz1), 2 * hd / math.sqrt(2), 48.0, 2 * hd / math.sqrt(2),
                               norm((1, 0, 1)), Y), [box(x1 - 0.5, x1 + 2 * hd, -30, 30, 0, 250),
                                                     box(P["mast_x"] - 40, x1 + 2 * hd, -30, 30, 0, zf1 + P["bat_h"] + 1.5)]))
    # (clipped 1.5 mm above the battery so the pack still slides out of the front: a 12 mm ledge remains)
    for s in (1, -1):
        for _, ax in AX:
            tub_adds += [box(ax - 18, ax + 18, s * 31, s * (inner + 0.01), zf0, 72),       # axle boss
                         cyl_y(s * (hw - 0.01), s * (hw + 7.8), ax, az, 15),                # boss ring (0.2 below the collar)
                         box(ax - 15, ax + 15, min(s * (hw - 0.01), s * (hw + 7.8)), max(s * (hw - 0.01), s * (hw + 7.8)),
                             zf0, az)]                                                       # ring's flat foot (no bed overhang)
        tub_adds += [
            box(xr_in, xf_in, s * (inner - 4), s * inner, dz0 - 4, dz0),               # deck ledge
            obox(((xr_in + xf_in) / 2, s * inner, dz0 - 4), xf_in - xr_in, 3.8 * math.sqrt(2), 3.8 * math.sqrt(2),
                 X, norm((0, 1, 1))),                                                  # 45 deg underside of the ledge
            box(xf_in - 12, xf_in, s * 46, s * inner, zf1 - 0.01, top),                # front shell post (floor to top)
            box(xr_in, xr_in + 14, s * 50, s * inner, zf1 - 0.01, top),                # rear shell post (floor to top)
            box(xf_in - 8, xf_in + 0.01, s * 34, s * 46, zf1 - 0.01, P["us_z"] + 8),  # door + HC-SR04 insert column
            box(P["bat_x0"], xf_in, s * (P["bat_w"] / 2 + 1.5), s * (P["bat_w"] / 2 + 5), zf1 - 0.01, zf1 + 8),  # rails
            box(P["idler_x"] - 10, P["idler_x"] + 10, s * (inner - 6), s * (inner + 0.01), zf1 - 0.01, dz0 - 0.01),  # idler pad
            box(mc_x - 30, mc_x + 30, s * (inner - 8), s * (inner + 0.01), zf1 - 0.01, mc_z - 10),  # under the motor pad
        ]
        for k in (-1, 1):                                                              # pad behind the motor plate
            cc = vadd((mc_x, 0, mc_z), vmul(bu, k * P["tension_travel"] / 2))
            tub_adds.append(cyl_y(s * (inner - 8), s * (inner + 0.01), cc[0], cc[2], P["motor_plate"] / 2 + 5))
        tub_adds.append(obox((mc_x, s * (inner - 4 + 0.005), mc_z), P["tension_travel"], P["motor_plate"] + 10, 8.01, bu, bv))  # flush with the pad cylinders (no sliver)
    tub = m.part("tub", "printed", COL["frame"], tub_adds, material="PAHT-CF",
                 note="one-piece main body (300 mm): both axles, battery tunnel, mast socket; print floor-down")
    tub_cuts = tub["cuts"]
    for s in (1, -1):                         # softer look: rounded vertical corners, chamfered bed-side edges
        for xe, ia in ((x1, (-1, 0, 0)), (x0, (1, 0, 0))):
            tub_cuts.append(round_edge_cut((xe, s * hw, zf0 - 1), (xe, s * hw, top + 1), ia, (0, -s, 0), 10.0))
        tub_cuts.append(chamfer_cut((x0, s * hw, zf0), (x1, s * hw, zf0), (0, -s, 0), (0, 0, 1), 2.5))
    tub_cuts.append(chamfer_cut((x1, -hw, zf0), (x1, hw, zf0), (-1, 0, 0), (0, 0, 1), 2.5))
    tub_cuts.append(chamfer_cut((x0, -hw, zf0), (x0, hw, zf0), (1, 0, 0), (0, 0, 1), 2.5))
    for s_ in (1, -1):
        for _, ax in AX:
            tub_cuts.append(box(ax - 12 - co, ax + 12 + co, min(s_ * 39, s_ * 51), max(s_ * 39, s_ * 51), 72.5, dz0 + co))
    tub_cuts.append(box(xf_in - co, x1 + co, -28, 28, zf1 - co, 62.7))               # battery door opening ...
    tub_cuts.append(pdiff(obox(((xf_in + x1) / 2, 0.0, 62.5), x1 - xf_in + 2 * co, 28 * math.sqrt(2), 28 * math.sqrt(2),
                               X, norm((0, 1, 1))), [box(xf_in - 5, x1 + 5, -40, 40, 76, 200)]))   # ... with a 45 deg top
    for s_ in (1, -1):                         # notch the battery rails so the front axle nylocs drop into their pockets
        tub_cuts.append(box(-9.5, 9.5, min(s_ * 26.0, s_ * 30.5), max(s_ * 26.0, s_ * 30.5), zf1, zf1 + 8 + co))
    ms = P["mast_sq"] + P["slip"]
    tub_cuts.append(box(P["mast_x"] - ms / 2, P["mast_x"] + ms / 2, -ms / 2, ms / 2, 92, top + co))
    tub_cuts.append(box(P["mast_x"] - 17 - co, P["mast_x"], -7, 7, 92, 104))
    tub_cuts.append(box(P["mast_x"] - 17 - co, P["mast_x"], -5, 5, 92, top + co))   # rear slot: cables lift out with the mast
    for s in (1, -1):
        m.hole("tub", "mast_flange_" + ("L" if s > 0 else "R"), (P["mast_x"], s * 17.65, top + co),
               (P["mast_x"], s * 17.65, top - P["ins_m4_depth"]), P["ins_m4"], kind="insert", for_item="mast flange M4")
    m.hole("tub", "us_wire", (xf_in - co, -30, 131), (x1 + co, -30, 131), 9, for_item="HC-SR04 cable", audit=False)
    tub_cuts.append(box(x0 - co, xr_in + co, -38.2, -21.8, 116.9, 125.1))              # XT60 arming plug (panel)
    m.holes.append({"host": "tub", "id": "xt60_arming", "a": (x0, -30, 121), "b": (xr_in, -30, 121), "d": 16.4,
                    "kind": "slot", "for": "XT60 arming plug (panel)", "audit": False})
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        ys = s * 1.0
        for nm, ax in AX:
            tub_cuts.append(box(ax - 7.7, ax + 7.7, s * (31 - co), s * 40, az - 6.7, az + 6.7))   # captive M8 nyloc pocket
            m.hole("tub", "axle_sleeve_%s%s" % (nm, side), (ax, s * 39.7, az), (ax, s * (hw + 8 + co), az), sd + 0.4,
                   for_item="inner axle sleeve %g x 8.2 x 30.3 (through the boss)" % sd)
        # motor plate pocket (capsule along the belt) + shaft clearance + tension slots + pull-screw stop
        for k in (-1, 1):
            cc = vadd((mc_x, 0, mc_z), vmul(bu, k * P["tension_travel"] / 2))
            tub_cuts.append(cyl_y(s * (hw - P["motor_plate_t"]), s * (hw + co), cc[0], cc[2], P["motor_plate"] / 2 + 0.5))
        tub_cuts.append(obox((mc_x, s * (hw - P["motor_plate_t"] / 2 + co / 2), mc_z), P["tension_travel"],
                             P["motor_plate"] + 1.0, P["motor_plate_t"] + co, bu, bv))
        tub_cuts.append(obox((mc_x, s * (inner - 4), mc_z), 16 + P["tension_travel"], 16, 30, bu, bv))
        for k, (ox, oz) in enumerate(MOTOR_PLATE_BOLTS):
            c = (mc_x + ox * P["motor_bolt_r"], 0, mc_z + oz * P["motor_bolt_r"])
            p0 = vadd(c, vmul(bu, -P["tension_travel"] / 2)); p1 = vadd(c, vmul(bu, P["tension_travel"] / 2))
            p0 = (p0[0], s * (inner - 6), p0[2]); p1 = (p1[0], s * (inner - 6), p1[2])
            m.slot_hole("tub", "tension_%s_%d" % (side, k), p0, p1, P["clr_m3"], (0, ys, 0), 6,
                        for_item="motor plate M3 (tension slot)")
        tab_c = vadd((mc_x, s * (inner - 4), mc_z), vmul(bu, P["motor_plate"] / 2 - 6))
        tub_cuts.append(obox(tab_c, 12 + P["tension_travel"] + 2, 12, 12, bu, bv))
        boss_c = vadd((mc_x, s * (inner - 9), mc_z), vmul(bu, P["motor_plate"] / 2 + P["tension_travel"] + 4))
        tub["adds"].append(obox((boss_c[0], s * (inner - 6.995), boss_c[2]), 8, 14, 14.01, bu, bv))   # boss reaches the wall
        a_ = vadd(boss_c, vmul(bu, 4 + co)); b_ = vadd(boss_c, vmul(bu, -4 - co))
        m.hole("tub", "pull_" + side, a_, b_, P["clr_m3"], for_item="belt pull screw M3 x 25 (head on the boss)")
        m.hole("tub", "motor_wire_" + side, (-80, s * (inner - 10), 62), (-80, s * (hw + co), 62), 12,   # fully clear of the plate pad
               for_item="motor phase wires grommet " + side, audit=False)
        # idler: vertical slot for an M8 bolt; the 608 presses the 1:1 belt's upper run down
        for kx in (-1, 1):                     # hex channel: holds the M8 head (13 AF) so the nyloc tightens from outside
            tub["adds"].append(box(min(P["idler_x"] + kx * 6.8, P["idler_x"] + kx * 10), max(P["idler_x"] + kx * 6.8, P["idler_x"] + kx * 10),
                                   min(s * (inner - 11.5), s * (inner - 5.99)), max(s * (inner - 11.5), s * (inner - 5.99)),
                                   zf1 - 0.01, dz0 - 0.01))
        m.slot_hole("tub", "idler_" + side, (P["idler_x"], s * (inner - 1), iz_lo), (P["idler_x"], s * (inner - 1), iz_hi),
                    P["clr_m8"], (0, ys, 0), 10 + 2 * co, for_item="idler M8 (vertical tension slot)")
        # shell inserts (front + rear posts), deck inserts, door + bracket inserts
        m.hole("tub", "lid_f_" + side, (xf_in - 6, s * 52, top + co), (xf_in - 6, s * 52, top - P["ins_m3_depth"]),
               P["ins_m3"], kind="insert", for_item="shell M3")
        m.hole("tub", "lid_r_" + side, (xr_in + 7, s * 54, top + co), (xr_in + 7, s * 54, top - P["ins_m3_depth"]),
               P["ins_m3"], kind="insert", for_item="shell M3")
        for nm, ax in AX:
            m.hole("tub", "deck_%s%s" % (nm, side), (ax, s * 28, dz0 + co), (ax, s * 28, dz0 - P["ins_m3_depth"]),
                   P["ins_m3"], kind="insert", for_item="deck M3")
        m.hole("tub", "door_" + side, (x1 + co, s * 40, 49), (x1 - P["ins_m4_depth"] - 4, s * 40, 49), P["ins_m4"],
               kind="insert", for_item="bumper-door M4 thumb screw")
        m.hole("tub", "us_" + side, (x1 + co, s * 40, P["us_z"]), (x1 - P["ins_m3_depth"], s * 40, P["us_z"]),
               P["ins_m3"], kind="insert", for_item="HC-SR04 bracket M3")

    # ---------------- MOTOR PLATES ----------------
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        name = "motor_plate_" + side
        ya, yb = s * (hw - P["motor_plate_t"]), s * hw
        m.part(name, "printed", COL["accent"], [cyl_y(ya, yb, mc_x, mc_z, P["motor_plate"] / 2)], material="PAHT-CF",
               note="replaceable motor interface; universal M3 cross for 16/19/25/30 mm patterns", mates={"tub"})
        yc = (ya + yb) / 2
        m.hole(name, "shaft", (mc_x, ya - s * co, mc_z), (mc_x, yb + s * co, mc_z), 9.0, for_item="motor shaft end/circlip", audit=False)
        for k in range(8):
            ang = k * math.pi / 4
            d = (math.cos(ang), 0.0, math.sin(ang))
            p0 = vadd((mc_x, 0, mc_z), vmul(d, 8.0)); p1 = vadd((mc_x, 0, mc_z), vmul(d, 16.0))
            m.slot_hole(name, "mslot_%d" % k, (p0[0], yc, p0[2]), (p1[0], yc, p1[2]), P["clr_m3"], (0, 1, 0),
                        P["motor_plate_t"] + 2 * co, for_item="motor base M3 (any 16-30 mm pattern)", audit=False)
            m.get(name)["cuts"].extend(slot((p0[0], 0, p0[2]), (p1[0], 0, p1[2]), 6.4, (0, s, 0), 3.0 + co,
                                            hw - P["motor_plate_t"] + 1.5 - co / 2))
        # central pocket at counterbore depth: merges the 8 counterbore ends (they overlap below r 8.4 and would leave
        # knife edges / non-manifold cusps against the shaft hole)
        m.get(name)["cuts"].append(cyl_y(s * (hw - P["motor_plate_t"] - co), s * (hw - P["motor_plate_t"] + 3.0), mc_x, mc_z, 11.5))
        tab_c = vadd((mc_x, s * (inner - 4), mc_z), vmul(bu, P["motor_plate"] / 2 - 6))
        tab_c = (tab_c[0], s * (hw - P["motor_plate_t"] - 5), tab_c[2])
        m.get(name)["adds"].append(obox(tab_c, 12, 11, 10.02, bu, bv))
        a_ = vadd(tab_c, vmul(bu, 6 + co)); b_ = vadd(tab_c, vmul(bu, 6 - P["ins_m3_depth"]))
        m.hole(name, "pull_ins", a_, b_, P["ins_m3"], kind="insert", for_item="belt pull screw M3 insert")
        for k, (ox, oz) in enumerate(MOTOR_PLATE_BOLTS):
            c = (mc_x + ox * P["motor_bolt_r"], 0, mc_z + oz * P["motor_bolt_r"])
            m.hole(name, "ins_%d" % k, (c[0], s * (hw - P["motor_plate_t"] - co), c[2]),
                   (c[0], s * (hw - P["motor_plate_t"] + P["ins_m3_depth"]), c[2]), P["ins_m3"], kind="insert",
                   for_item="motor plate <- wall M3")
            hy0 = s * (inner - 8)
            m.part("plate_bolt_%s%d" % (side, k), "hardware", COL["hw_metal"],
                   [cyl_y(hy0 - s * 3.0, hy0 + s * 0.01, c[0], c[2], 2.9)], mass=0.003, mates={"tub"},
                   note="M3 x 16 socket head + washer")
            m.part("tool_access_%s%d" % (side, k), "keepout", COL["keepout"],
                   [cyl_y(hy0 - s * 17.0, hy0 - s * 3.0, c[0], c[2], 5.0)],
                   note="2.5 mm ball-end hex key (shell off)", mates={"tub", "plate_bolt_%s%d" % (side, k)})

    # ---------------- DECK (with the ESP32 cradle printed on it) ----------------
    g = P["slip"] / 2
    ecx, el, ew = -195.0, P["esp_l"], P["esp_w"]
    pcb_z = dz1 + 30                           # ESP32 PCB underside: jumper plugs on the downward pins clear the deck
    deck_adds = [box(xr_in + g, xf_in - g, -inner + g, inner - g, dz0, dz1)]
    for sx in (-1, 1):                         # cradle rails with a PCB groove (slide the DevKit in from -y)
        rx0 = ecx + sx * (ew / 2 + 0.3)
        deck_adds.append(box(min(rx0, rx0 + sx * 4), max(rx0, rx0 + sx * 4), -30, 30, dz1 - 0.01, pcb_z + 5))
    deck = m.part("deck", "printed", COL["light"], deck_adds, material="PETG",
                  note="removable electronics tray: 10 mm M3 grid, Pi holes, ESP32 slide-in cradle printed on",
                  mates={"tub"})
    dc = deck["cuts"]
    for sx in (-1, 1):
        g0 = ecx + sx * (ew / 2 + 0.3)
        dc.append(box(min(g0, g0 + sx * 1.2), max(g0, g0 + sx * 1.2), -30 - co, 30 + co, pcb_z - 0.2, pcb_z + 1.9))
    for s in (1, -1):
        dc.append(box(xf_in - 12.5, xf_in + co, s * 45.5 if s > 0 else -inner - co, s * (inner + co) if s > 0 else -45.5, dz0 - co, dz1 + co))
        dc.append(box(xr_in - co, xr_in + 14.5, s * 49.5 if s > 0 else -inner - co, s * (inner + co) if s > 0 else -49.5, dz0 - co, dz1 + co))
        dc.append(box(mc_x - 44, mc_x + 44, s * 47.5 if s > 0 else -inner - co, s * (inner + co) if s > 0 else -47.5, dz0 - co, dz1 + co))
        dc.append(box(mc_x - 40, mc_x - 3, s * 42.5, s * (inner + co), dz0 - co, dz1 + co))   # belt-tension tab + pull boss
        dc.append(box(-130, -112, s * 30 if s > 0 else -47, s * 47 if s > 0 else -30, dz0 - co, dz1 + co))   # cable pass
        dc.append(box(xf_in - 8.5, xf_in + co, s * 33.5, s * 46.5, dz0 - co, dz1 + co))       # insert column notch
    dc.append(box(P["mast_x"] - 17.5, xf_in + co, -24.5, 24.5, dz0 - co, dz1 + co))     # mast socket notch
    pi_x0 = P["pi_x0"]; pi_x1 = pi_x0 + 85.0
    pi_holes = [(pi_x1 - 3.5, -24.5), (pi_x1 - 3.5, 24.5), (pi_x1 - 61.5, -24.5), (pi_x1 - 61.5, 24.5)]
    for i, (hx, hy) in enumerate(pi_holes):
        m.hole("deck", "pi_%d" % i, (hx, hy, dz0 - co), (hx, hy, dz1 + co), P["clr_m25"], for_item="Raspberry Pi 5 M2.5 standoff")
    m.info["pi"] = (pi_x0, pi_x1)
    gp = P["grid_pitch"]
    xs = [xr_in + 25 + gp * i for i in range(int((xf_in - xr_in - 40) / gp))]
    ys = [gp * j for j in range(-4, 5)]
    for gx in xs:
        for gy in ys:
            if any(math.hypot(gx - kx, gy - ky) < 7.5 for kx, ky in pi_holes):
                continue
            if (abs(gx - mc_x) < 47 and abs(gy) > 44) or (mc_x - 44 < gx < mc_x + 1 and abs(gy) > 38):
                continue
            if gx > P["mast_x"] - 21 and abs(gy) < 28 or gx > xf_in - 16:
                continue
            if any(abs(gx - ax) < 6 and abs(abs(gy) - 28) < 6 for _, ax in AX):
                continue
            if xr_in + 6 < gx < xr_in + 22 and abs(gy) > 44 or (-134 < gx < -108 and 26 < abs(gy) < 51):
                continue
            if 4 < gx < 38 and 24 < abs(gy) < 56 or (gx > xf_in - 12 and abs(gy) > 30):
                continue
            if abs(gx - ecx) < ew / 2 + 7 and abs(gy) < 34:
                continue
            dc.append(cyl_z(dz0 - co, dz1 + co, gx, gy, P["clr_m3"] / 2))
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        for nm, ax in AX:
            m.hole("deck", "screw_%s%s" % (nm, side), (ax, s * 28, dz0 - co), (ax, s * 28, dz1 + co), P["clr_m3"],
                   for_item="deck M3")
        for zx in (12, 30):                     # zip-tie slots for the regulators
            dc.append(box(zx - 1.5, zx + 1.5, s * 29, s * 49, dz0 - co, dz1 + co))
        for zx in (-170, -140):                 # zip-tie slots for the ESCs
            dc.append(box(zx - 1.5, zx + 1.5, s * 32, s * 54, dz0 - co, dz1 + co))

    # ---------------- PULLEYS ----------------
    def teeth(name, ycen, xc, n, rtip):
        for k in range(n):
            ang = 2 * math.pi * k / n
            rc = rtip - 1.14 + 0.9
            m.get(name)["cuts"].append(cyl_y(ycen - 5.5, ycen + 5.5, xc + rc * math.cos(ang), az + rc * math.sin(ang), 0.9))

    for s in (1, -1):
        side = "L" if s > 0 else "R"
        # front: 56T (motor belt) + 28T (rear belt) in one print
        name = "wheel_pulley_F" + side
        adds = [cyl_y(s * (by - 7.0), s * (by + 7.0), 0, az, wpr),
                cyl_y(s * (by - 7.0), s * (by - 5.5), 0, az, wfl), cyl_y(s * (by + 5.5), s * (by + 7.0), 0, az, wfl),
                cyl_y(s * (by + 7.0), s * (wiy - 0.01), 0, az, wfl),
                cyl_y(s * (by2 - 7.0), s * (by2 + 7.0), 0, az, rpr),
                cyl_y(s * (by2 - 7.0), s * (by2 - 5.5), 0, az, rfl), cyl_y(s * (by2 + 5.5), s * (by2 + 7.0), 0, az, rfl),
                cyl_y(s * (by2 + 7.0 - 0.01), s * (by - 7.0 + 0.01), 0, az, rfl)]
        m.part(name, "printed", COL["accent"], adds, material="PAHT-CF", moving=True,
               note="double pulley: %dT (motor belt) + %dT (rear belt); 4 inserts, screwed on through the wheel core"
                    % (P["pul_w_t"], P["pul_r_t"]), mates={"wheel_F" + side, "belt_" + side, "belt2_" + side})
        teeth(name, s * by, 0.0, P["pul_w_t"], wpr)
        teeth(name, s * by2, 0.0, P["pul_r_t"], rpr)
        # rear: 28T + hub + adapter disc
        rname = "wheel_pulley_R" + side
        m.part(rname, "printed", COL["accent"],
               [cyl_y(s * (by2 - 7.0), s * (by2 + 7.0), xra, az, rpr),
                cyl_y(s * (by2 - 7.0), s * (by2 - 5.5), xra, az, rfl), cyl_y(s * (by2 + 5.5), s * (by2 + 7.0), xra, az, rfl),
                cyl_y(s * (by2 + 7.0 - 0.01), s * (wiy - 3.5), xra, az, 22.0),
                cyl_y(s * (wiy - 3.5 - 0.01), s * (wiy - 0.01), xra, az, wfl)],
               material="PAHT-CF", moving=True, note="%dT rear pulley + hub; 4 inserts, screwed on through the wheel core" % P["pul_r_t"],
               mates={"wheel_R" + side, "belt2_" + side})
        teeth(rname, s * by2, xra, P["pul_r_t"], rpr)
        for nm, ax, ystart in (("F", 0.0, by2 - 7), ("R", xra, by2 - 7)):
            pn = "wheel_pulley_" + nm + side
            m.hole(pn, "bore", (ax, s * (ystart - co), az), (ax, s * (wiy + co), az), sd + 6.0,
                   for_item="clears the axle sleeve", audit=False)
            for k in range(4):
                ang = k * math.pi / 2 + math.pi / 4
                hx, hz = ax + 17.5 * math.cos(ang), az + 17.5 * math.sin(ang)
                m.hole(pn, "core_%d" % k, (hx, s * (wiy + co), hz), (hx, s * (wiy - P["ins_m3_depth"] - 1.5), hz),
                       P["ins_m3"], kind="insert", for_item="M3 x 60 from the wheel's outer face through the core "
                       "(holes drilled 3.0 -> 3.15 with the jig: they centre the pulley)")
    m.info["wheel_pulley_flange_r"] = wfl
    for k in (-1, 1):                          # wire grommet must clear the plate pad (no knife edges)
        cc = vadd((mc_x, 0, mc_z), vmul(bu, k * P["tension_travel"] / 2))
        assert math.hypot(cc[0] + 80, cc[2] - 62) >= P["motor_plate"] / 2 + 5 + 6 + 1.5, "motor wire hole touches the pad"
    # printed tooth sections need >= 3 mm of plastic between the tooth roots and the bore (sleeve clearance)
    root_wall = min(wpr, rpr) - 1.14 - (sd + 6.0) / 2
    m.info["pulley_root_wall"] = root_wall
    assert root_wall >= 3.0, "pulley root wall %.1f mm < 3 mm: more teeth or a thinner axle sleeve" % root_wall

    jz = -150.0                                # drill jig (tool, printed once, hidden)
    jig = m.part("pulley_drill_jig", "tools", COL["light"],
                 [cyl_z(jz, jz + 5, 0, 0, 25), cyl_z(jz + 4.99, jz + 11, 0, 0, 10.9)], material="PETG",
                 note="drill the wheel core 3.0 through these guides, then open to 3.2 -> pulley runout < 0.2 mm", hidden=True)
    for k in range(4):
        ang = k * math.pi / 2 + math.pi / 4
        jig["cuts"].append(cyl_z(jz - co, jz + 5 + co, 17.5 * math.cos(ang), 17.5 * math.sin(ang), 1.5))

    # ---------------- PROTOTYPE KIT (hidden): printed axle sleeves + TPU belts, laid out flat for printing ----------------
    pz = -150.0
    for i, (nm, _) in enumerate(AX * 2):
        k = "%s%d" % (nm, i // 2 + 1)
        cx, cy = 120.0 + 40 * i, 0.0
        ins = m.part("proto_sleeve_in_" + k, "prototype", COL["accent"],
                     [cyl_z(pz, pz + 2.0, cx, cy, 15.0), cyl_z(pz + 1.99, pz + 32.3, cx, cy, sd / 2)],
                     [cyl_z(pz - co, pz + 32.3 + co, cx, cy, 4.2)], material="PAHT-CF", hidden=True,
                     note="prototype: inner sleeve + collar in one, collar on the bed; replaces the steel tube + collar")
        lo = wiy - 0.5 - 72.0
        m.part("proto_sleeve_out_" + k, "prototype", COL["accent"],
               [cyl_z(pz, pz + lo, cx, cy + 40, sd / 2), cyl_z(pz + lo - 0.01, pz + lo + 1.5, cx, cy + 40, 6.0)],
               [cyl_z(pz - co, pz + lo + 1.5 + co, cx, cy + 40, 4.2)], material="PAHT-CF", hidden=True,
               note="prototype: outer sleeve + inner-race lip (%.1f mm: set to your wheel's bearing inset)" % (lo + 1.5))
    EA = P["tpu_E"] * P["belt_w"] * 2.0
    for nm, L, cx, cy in (("motor", P["belt_len"], -100.0, 0.0), ("rear", P["belt2_len"], -100.0, 230.0)):
        n_t = int(round(L / 3.0))
        Lp = L / (1 + P["tpu_T0"] / EA)
        R = Lp / (2 * math.pi)
        teeth_ = [cyl_z(pz, pz + P["belt_w"], cx + (R - 0.62) * math.cos(2 * math.pi * j / n_t),
                        cy + (R - 0.62) * math.sin(2 * math.pi * j / n_t), 0.85) for j in range(n_t)]
        m.part("proto_belt_%s" % nm, "prototype", COL["belt"], teeth_,
               pre=[([cyl_z(pz, pz + P["belt_w"], cx, cy, R + 1.62)], [cyl_z(pz - co, pz + P["belt_w"] + co, cx, cy, R - 0.38)])],
               material="TPU (hardest you have)", hidden=True,
               note="prototype HTD-3M %dT belt printed %.1f mm long (stretches to %.0f at %.0f N); print flat, 100 %% infill, x2"
                    % (n_t, Lp, L, P["tpu_T0"]))
        m.info.setdefault("proto_belts", {})[nm] = (n_t, Lp, L)
    m.info["tpu_EA"] = EA

    # ---------------- BUMPER-DOOR ("mouth") ----------------
    bfront = x1 + 28
    bz0 = zf1 + 0.5                            # bumper bottom = prong bottom: both on the bed when printed upright
    bump = rbox(x1, bfront, -50, 50, bz0, 76, 13)   # plan-rounded only: edge rounds made Fusion/OCC booleans fail
    bd = m.part("bumper_door", "printed", COL["accent"],
                bump + [box(x1, x1 + 14, -50, 50, bz0, 76)]      # flat square back, flush on the tub
                + [box(P["bat_x0"] + P["bat_l"] + 0.5, x1 + 0.01, min(sy_ * 12, sy_ * 22), max(sy_ * 12, sy_ * 22), bz0, zf1 + 20)
                   for sy_ in (1, -1)],
                material="PETG (hollow crush zone; optional TPU skin)",
                note="battery door + front bumper; two prongs hold the pack against the rear wall", mates={"tub", "battery"})
    for s in (1, -1):
        m.hole("bumper_door", "thumb_" + ("L" if s > 0 else "R"), (x1 - co, s * 40, 49), (bfront + co, s * 40, 49),
               P["clr_m4"], for_item="M4 thumb screw")
        bd["cuts"].append(cyl_x(x1 + 10, bfront + co, s * 40, 49, 6.0))
    for zz in (60, 67):
        bd["cuts"].append(box(bfront - 1.5, bfront + co, -30, 30, zz - 1, zz + 1))
    bd["cuts"].append(box(x1 + 3, bfront - 3, -32, 32, bz0 + 3, 56))

    # ---------------- HC-SR04 bracket ("eyes") ----------------
    t = math.radians(P["us_tilt_deg"]); n = (math.cos(t), 0.0, math.sin(t)); tv = (-math.sin(t), 0.0, math.cos(t))
    uz = P["us_z"]; plate_c = (x1 + 21, 0.0, uz)
    br_adds = [obox(plate_c, 4.0, 60.0, 30.0, n, Y)]
    for s in (1, -1):
        br_adds.append(box(x1, bfront, s * 29, s * 47, uz - 13, uz + 13))
    br_adds.append(box(plate_c[0] + 2.0, bfront, -47, 47, uz + 13, uz + 17))
    m.part("us_bracket", "printed", COL["light"], br_adds, material="PETG", mates={"tub"},
           note="tilts the sensor %.0f deg up so the ground return lands beyond slow_distance" % P["us_tilt_deg"])
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        m.hole("us_bracket", "screw_" + side, (x1 - co, s * 40, uz), (bfront + co, s * 40, uz), P["clr_m3"],
               for_item="bracket M3")
        a_ = vadd(vadd(plate_c, vmul(Y, s * 13)), vmul(n, -2 - co)); b_ = vadd(a_, vmul(n, 4 + 2 * co))
        m.hole("us_bracket", "eye_" + side, a_, b_, 18.0, for_item="HC-SR04 transducer")
        m.slot_hole("us_bracket", "ztie_" + side, vadd(plate_c, vmul(vadd(vmul(Y, s * 26.5), vmul(tv, -7)), 1)),
                    vadd(plate_c, vmul(vadd(vmul(Y, s * 26.5), vmul(tv, 7)), 1)), 2.5, n, 4 + 2 * co,
                    for_item="zip tie holding the HC-SR04 PCB", audit=False)
    m.info["us"] = {"x": plate_c[0] + 7, "z": uz, "tilt": P["us_tilt_deg"], "half": P["us_beam_half_deg"],
                    "n": n, "c": vadd(plate_c, vmul(n, 6.5))}

    # ---------------- SHELL: lid + both motor pods in one print ----------------
    shell_pre = []
    for s in (1, -1):
        sy = lambda a, b: (s * a, s * b) if s > 0 else (-b, -a)  # noqa: E731
        ya, yb = sy(hw - 0.01, 119)
        ia, ib = sy(hw - 6, 116.5)
        shell_pre.append((rbox(mc_x - 37, mc_x + 47, ya, yb, 98, top + 5, 10),     # sharp top edge = clean bed edge
                          rbox(mc_x - 34.5, mc_x + 44.5, ia, ib, 60, top - 0.5, 7.5)))
    shell_adds = rbox(x0, x1, -hw, hw, top, top + 5, 10)    # flat top: prints lid-down, no supports
    shell = m.part("shell", "printed", COL["shell"], shell_adds, pre=shell_pre, material="PETG",
                   mates={"tub"}, note="one-piece top: lid + both motor pods; 4 screws; remove the mast first, then lift")
    shell["cuts"].append(box(P["mast_x"] - ms / 2, P["mast_x"] + ms / 2, -ms / 2, ms / 2, top - co, top + 7 + co))
    for s in (1, -1):
        m.hole("shell", "mast_flange_" + ("L" if s > 0 else "R"), (P["mast_x"], s * 17.65, top - co),
               (P["mast_x"], s * 17.65, top + 5 + co), P["clr_m4"], for_item="mast flange M4 thumb screw")
    for i in range(6):
        xx = -60 + i * 12
        shell["cuts"] += slot((xx, -16, 0), (xx, 16, 0), 4.0, (0, 0, 1), 7 + 2 * co, top + 3.5)
    for i in range(4):
        xx = -215 + i * 12
        shell["cuts"] += slot((xx, -22, 0), (xx, 22, 0), 4.0, (0, 0, 1), 7 + 2 * co, top + 3.5)
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        m.hole("shell", "f_" + side, (xf_in - 6, s * 52, top - co), (xf_in - 6, s * 52, top + 5 + co), P["clr_m3"], for_item="shell M3")
        m.hole("shell", "r_" + side, (xr_in + 7, s * 54, top - co), (xr_in + 7, s * 54, top + 5 + co), P["clr_m3"], for_item="shell M3")

    # ---------------- MAST + PAN HEAD + UWB plate (one print) ----------------
    mx = P["mast_x"]; sq = P["mast_sq"]; mw = P["mast_wall"]; seat = P["servo_seat_z"]
    sl, sw = P["servo_l"], P["servo_w"]; sbx = mx - P["servo_shaft_off"]
    head_z0 = seat - P["servo_ear_z"] - 4
    head = (sbx - sl / 2 - 10, sbx + sl / 2 + 10, -sw / 2 - 4, sw / 2 + 4, head_z0, seat)
    hx1 = sbx + sl / 2 + 10
    uwz0, uwz1 = top + 11, seat - 3
    upx = max(mx + sq / 2, hx1) + 0.5
    mast_adds = ([box(mx - sq / 2, mx + sq / 2, -sq / 2, sq / 2, 92 + 0.3, head_z0 + 0.01)]
                 + rbox(mx - 14, mx + 14, -24, 24, top + 5, top + 10, 5)
                 + rbox(*head, 6)
                 + [box(hx1 - 0.5, upx + 4, -28, 28, head_z0 + 4, uwz1),                     # UWB plate (front)
                    box(mx + sq / 2 - 0.01, upx + 0.01, -11, 11, uwz0, head_z0 + 0.01)])      # UWB plate web to the mast
    mast = m.part("mast", "printed", COL["mast"], mast_adds, material="PAHT-CF (print lying on its side)",
                  mates={"tub", "shell"}, note="removable: 2 M4 thumb screws through the flange; UWB plate printed on")
    mc = mast["cuts"]
    mc.append(box(mx - sq / 2 + mw, mx + sq / 2 - mw, -sq / 2 + mw, sq / 2 - mw, 92 - co, head_z0 + 6))
    mc.append(box(mx - sq / 2 - co, mx, -6, 6, 92 - co, 104))
    mc.append(box(sbx - sl / 2 - 0.4, sbx + sl / 2 + 0.4, -sw / 2 - 0.4, sw / 2 + 0.4, head_z0 + 4, seat + co))
    mc.append(box(sbx - sl / 2 - 6, sbx - sl / 2 + 6, 4, 11, head_z0 - co, seat + co))   # servo lead notch (corner kept off the rounded edge)
    mc.append(box(mx - sq / 2 - co, mx - sq / 2 + mw + co, -5, 5, 92 - co, head_z0 + 0.5))
    for s in (1, -1):
        m.slot_hole("mast", "servo_ear_" + ("A" if s > 0 else "B"), (sbx + s * 24.0, 0, seat), (sbx + s * 27.0, 0, seat),
                    2.4, (0, 0, 1), 12, centre_depth=-6 + co / 2, for_item="servo ear screw (self-tapping)")
        m.hole("mast", "flange_" + ("L" if s > 0 else "R"), (mx, s * 17.65, top + 5 - co), (mx, s * 17.65, top + 10 + co),
               P["clr_m4"], for_item="mast flange M4 thumb screw")
    for sy_ in (1, -1):
        for zz in (head_z0 + 10, uwz1 - 5):
            m.slot_hole("mast", "uwb_board_%d_%d" % (sy_, int(zz)), (upx + 2, sy_ * 14, zz), (upx + 2, sy_ * 23, zz),
                        P["clr_m3"], (1, 0, 0), 4 + 2 * co, for_item="UWB board M3 [M]", audit=False)
    m.info["mast"] = {"x": mx, "sq": sq, "wall": mw, "z0": top + 5, "z_head": head_z0}

    horn_z0 = seat + 13 + 5
    piv_z = horn_z0 + 4 + 9
    buckle_adds = [cyl_z(horn_z0, horn_z0 + 4, mx, 0, 15)]
    prong_t, gap = 3.0, 3.3
    for k in (-1, 0, 1):
        yc = k * (prong_t + gap)
        buckle_adds.append(box(mx - 7.5, mx + 7.5, yc - prong_t / 2, yc + prong_t / 2, horn_z0 + 3.99, piv_z))
        buckle_adds.append(cyl_y(yc - prong_t / 2, yc + prong_t / 2, mx, piv_z, 7.5))
    m.part("pan_buckle", "printed", COL["accent"], buckle_adds, material="PETG", moving=True, mates={"servo", "gopro"},
           note="horn adapter + GoPro 3-prong buckle; centre hole + slots fit any round/cross horn")
    m.hole("pan_buckle", "pivot", (mx, -12, piv_z), (mx, 12, piv_z), P["clr_m5"], for_item="GoPro M5 thumb screw")
    m.hole("pan_buckle", "horn_centre", (mx, 0, horn_z0 - co), (mx, 0, horn_z0 + 4 + co), 3.2, for_item="horn screw",
           audit=False)
    for k in range(4):
        ang = k * math.pi / 2
        m.slot_hole("pan_buckle", "horn_%d" % k, (mx + 6 * math.cos(ang), 6 * math.sin(ang), horn_z0 + 2),
                    (mx + 13 * math.cos(ang), 13 * math.sin(ang), horn_z0 + 2), 2.2, (0, 0, 1), 4 + 2 * co,
                    for_item="horn arm screws", audit=False)
    sgl, sgw = P["sg90_l"], P["sg90_w"]; sgx = mx - P["sg90_shaft_off"]
    ad = m.part("servo9g_adapter", "printed", COL["light"],
                [box(sbx - sl / 2, sbx + sl / 2, -sw / 2, sw / 2, head_z0 + 4.2, seat),
                 box(sbx - sl / 2 - 3.5, sbx + sl / 2 + 3.5, -sw / 2 - 3.5, sw / 2 + 3.5, seat, seat + 3)],
                material="PETG", mates={"mast", "servo"}, hidden=True,
                note="bench only (hidden): SG90 cannot hold the GoPro on the robot; replaces the standard servo")
    ad["cuts"].append(box(sgx - sgl / 2 - 0.4, sgx + sgl / 2 + 0.4, -sgw / 2 - 0.4, sgw / 2 + 0.4, seat - 20, seat + 3 + co))
    for s in (1, -1):
        m.slot_hole("servo9g_adapter", "ear_" + ("A" if s > 0 else "B"), (sgx + s * 13.0, 0, seat + 1.5), (sgx + s * 15.0, 0, seat + 1.5),
                    2.0, (0, 0, 1), 3 + 2 * co, for_item="SG90 ear screw", audit=False)
        m.slot_hole("servo9g_adapter", "seat_" + ("A" if s > 0 else "B"), (sbx + s * 24.0, 0, seat + 1.5), (sbx + s * 27.0, 0, seat + 1.5),
                    2.4, (0, 0, 1), 3 + 2 * co, for_item="adapter -> head (servo ear slots)", audit=False)

    # ================= REFERENCE HARDWARE (not printed) =================
    H = "hardware"
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        for nm, ax in AX:
            k = nm + side
            ya, yb = (wiy, wiy + P["wheel_w"]) if s > 0 else (-wiy - P["wheel_w"], -wiy)
            m.part("wheel_" + k, H, COL["wheel"], [cyl_y(ya, yb, ax, az, wr)], mass=0.16, moving=True,
                   mates={"wheel_pulley_" + k, "axle_" + k}, note="90 x 52 longboard wheel [M mass]")
            ye = wiy + P["wheel_w"]
            m.part("axle_" + k, H, COL["hw_metal"], [cyl_y(s * 31.5, s * (ye + 1.6), ax, az, 4),
                                                       cyl_y(s * (ye + 0.01), s * (ye + 1.6), ax, az, 8.0),
                                                       cyl_y(s * (ye + 1.59), s * (ye + 7.0), ax, az, 7.5)],
                   mass=0.045, mates={"tub", "wheel_" + k, "spacer_" + k, "axle_nut_" + k},
                   note="M8 x 140 hex bolt 12.9 from outside + washer; turns into the captive nyloc")
            m.part("axle_nut_" + k, H, COL["hw_metal"], [box(ax - 7.4, ax + 7.4, s * 31.6, s * 39.6, az - 6.4, az + 6.4)],
                   mass=0.005, mates={"tub", "axle_" + k, "spacer_" + k}, note="M8 nyloc, hex held by the pocket")
            m.part("spacer_" + k, H, COL["hw_metal"],
                   [cyl_y(s * 39.7, s * 70.0, ax, az, sd / 2), cyl_y(s * 70.0, s * 72.0, ax, az, 15.0),
                    cyl_y(s * 72.0, s * (wiy - 0.5), ax, az, sd / 2), cyl_y(s * (wiy - 0.51), s * (wiy + 1.0), ax, az, 6.0)],
                   mass=0.07, mates={"tub", "wheel_" + k, "axle_" + k, "axle_nut_" + k},
                   note="nested stock tubes 12x2 in 16x2: inner 30.3 + collar 8.4 x 30 x 2 + outer (cut to measure) + speed washer")
        my0, my1 = (hw, hw + P["motor_env_l"]) if s > 0 else (-hw - P["motor_env_l"], -hw)
        m.part("motor_" + side, H, COL["hw_metal"], [cyl_y(my0, my1, mc_x, mc_z, P["motor_env_d"] / 2)], mass=0.11,
               moving=True, mates={"motor_plate_" + side, "tub", "shell", "motor_pulley_" + side},
               note="5010-class outrunner (buy one, measure, set PARAMS) inside a O62 x 33 envelope")
        mpn = "motor_pulley_" + side
        mpr = P["pul_m_pd"] / 2 - 0.38
        y0p = hw + P["motor_env_l"]
        m.part(mpn, "printed", COL["accent"],
               [cyl_y(s * y0p, s * (y0p + 1.5), mc_x, mc_z, mpr + 2.5),
                cyl_y(s * (y0p + 1.49), s * (y0p + 12.51), mc_x, mc_z, mpr),
                cyl_y(s * (y0p + 12.5), s * (y0p + 14.0), mc_x, mc_z, mpr + 2.5)],
               material="PAHT-CF", moving=True, mates={"motor_" + side, "belt_" + side},
               note="%dT HTD-3M, 4 x M3 low-head screws into the bell's prop holes, centred on the shaft" % P["pul_m_t"])
        m.hole(mpn, "bore", (mc_x, s * (y0p - co), mc_z), (mc_x, s * (y0p + 8), mc_z), P["motor_shaft_d"] + 0.1,
               for_item="motor shaft stub (centres the pulley; drill to fit)", audit=False)
        for k in range(4):
            ang = math.pi / 4 + k * math.pi / 2
            hx_, hz_ = mc_x + P["motor_prop_pcd"] / 2 * math.cos(ang), mc_z + P["motor_prop_pcd"] / 2 * math.sin(ang)
            m.hole(mpn, "prop_%d" % k, (hx_, s * (y0p - co), hz_), (hx_, s * (y0p + 14 + co), hz_), P["clr_m3"],
                   for_item="M3 x 12 low head into the bell")
            m.hole(mpn, "cbore_%d" % k, (hx_, s * (y0p + 10), hz_), (hx_, s * (y0p + 14 + co), hz_), 5.8, kind="cbore",
                   for_item="M3 low-head counterbore", audit=False)
        for k in range(P["pul_m_t"]):
            ang = 2 * math.pi * k / P["pul_m_t"]
            rc = mpr - 1.14 + 0.9
            m.get(mpn)["cuts"].append(cyl_y(s * (y0p + 1.5), s * (y0p + 12.5), mc_x + rc * math.cos(ang),
                                            mc_z + rc * math.sin(ang), 0.9))
        m.part("belt_" + side, H, COL["belt"], belt_loop((0.0, az), wpr, (mc_x, mc_z), mpr, P["belt_t"], s * by, P["belt_w"]),
               mass=0.012, moving=True, mates={"wheel_pulley_F" + side, "motor_pulley_" + side},
               note="HTD-3M %d mm (%dT) x 9 mm closed belt" % (P["belt_len"], P["belt_len"] / 3))
        m.part("belt2_" + side, H, COL["belt"], belt_loop_n([(0.0, az, rpr, 1), (P["idler_x"], iz_hi, P["idler_r"], -1),
                                                             (xra, az, rpr, 1)], P["belt_t"], s * by2, P["belt_w"]),
               mass=0.02, moving=True, mates={"wheel_pulley_F" + side, "wheel_pulley_R" + side, "idler_" + side},
               note="HTD-3M %d mm (%dT) x 9 mm closed belt, front -> rear wheel 1:1" % (P["belt2_len"], P["belt2_len"] / 3))
        # idler: 608 on an M8 bolt in the wall slot, rolling on the back of the upper run
        m.part("idler_" + side, H, COL["hw_metal"],
               [cyl_y(s * (by2 - 7.0), s * (by2 + 7.0), P["idler_x"], iz_hi, P["idler_r"]),
                cyl_y(s * (inner - 6), s * (by2 + 11), P["idler_x"], iz_hi, 4.0),
                cyl_y(s * (inner - 11.3), s * (inner - 6 + 0.01), P["idler_x"], iz_hi, 6.5),
                cyl_y(s * hw, s * (by2 - 7.0), P["idler_x"], iz_hi, 6.0)],
               mass=0.045, moving=True, mates={"tub", "belt2_" + side, "idler_travel_" + side},
               note="2 x 608-2RS side by side (14 mm under the 9 mm belt) on an M8 x 55 hex bolt; head held in the "
                    "pad's hex channel, so the outside nyloc tightens it; 8 x 12 shims against the inner races")
        m.part("idler_travel_" + side, "keepout", COL["keepout"],
               [cyl_y(s * (by2 - 7.0), s * (by2 + 7.0), P["idler_x"], iz_lo, P["idler_r"]),
                box(P["idler_x"] - P["idler_r"], P["idler_x"] + P["idler_r"], min(s * (by2 - 7), s * (by2 + 7)),
                    max(s * (by2 - 7), s * (by2 + 7)), iz_lo, iz_hi)],
               note="idler swept over its tension slot (%.1f-%.1f mm take-up)" % P["belt2_take_up_range"],
               mates={"idler_" + side, "belt2_" + side, "tub"})
        m.info.setdefault("idler", {})[side] = (P["idler_x"], iz_hi)
    m.part("battery", H, COL["battery"], [box(P["bat_x0"], P["bat_x0"] + P["bat_l"], -P["bat_w"] / 2, P["bat_w"] / 2, zf1, zf1 + P["bat_h"])],
           mass=0.40, mates={"tub", "bumper_door"}, note="3S 5000 mAh hard case <= 145 x 50 x 30 [M]")
    m.part("pi5", H, COL["pcb_green"], [box(pi_x0, pi_x1, -28, 28, dz1 + 6, dz1 + 7.6),
                                        box(pi_x0 + 20, pi_x0 + 70, -24, 24, dz1 + 7.6, dz1 + 22)], mass=0.07,
           mates={"deck"}, note="Raspberry Pi 5 + Active Cooler on 6 mm M2.5 standoffs")
    for i, (hx, hy) in enumerate(pi_holes):
        m.part("pi_standoff_%d" % i, H, COL["hw_metal"], [cyl_z(dz1, dz1 + 6, hx, hy, 2.5)], mass=0.002,
               mates={"deck", "pi5"})
    m.part("usb_keepout", "keepout", COL["keepout"], [box(pi_x0 - 25, pi_x0, -28, 28, dz1 + 2, dz1 + 24)],
           note="right-angle USB-A plugs (GoPro, ESP32, UWB) need 25 mm", mates={"pi5"})
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        y0_, y1_ = (26.5, 51.9) if s > 0 else (-51.9, -26.5)
        m.part("reg_" + side, H, COL["pcb_blue"], [box(8.3, 33.7, y0_, y1_, dz1 + 0.01, dz1 + 9.5)], mass=0.007,
               mates={"deck"}, note=("Pololu D36V50F5 5 V 5.5 A (Pi)" if s > 0 else "Pololu D36V50F6 6 V (servo)") + " 25.4 sq [V]")
        ey0, ey1 = (29, 56) if s > 0 else (-56, -29)
        m.part("esc_" + side, H, COL["hw_dark"], [box(-179, -131, ey0, ey1, dz1 + 0.01, dz1 + 14)],
               mass=0.03, mates={"deck"}, note="AM32 ESC envelope 48 x 28 x 14 [M], zip-tied to the deck")
    m.part("esp32", H, COL["pcb_blue"],
           [box(ecx - ew / 2, ecx + ew / 2, -el / 2, el / 2, pcb_z, pcb_z + 1.6),
            box(ecx - 9, ecx + 9, -el / 2 + 1, -el / 2 + 26.5, pcb_z + 1.6, pcb_z + 4.8),
            box(ecx - 4, ecx + 4, el / 2 - 5, el / 2 + 1, pcb_z + 1.6, pcb_z + 4.6)]
           + [box(ecx + sx * (ew / 2 - 2.5) - 1.25, ecx + sx * (ew / 2 - 2.5) + 1.25, -24, 24, pcb_z - 8.5, pcb_z)
              for sx in (-1, 1)],
           mass=0.012, mates={"deck", "esp32_plugs"}, note="ESP32 DevKit (WROOM-32, CP2102) [M size]")
    m.part("esp32_plugs", "keepout", COL["keepout"],
           [box(ecx + sx * (ew / 2 - 2.5) - 1.8, ecx + sx * (ew / 2 - 2.5) + 1.8, -24, 24, pcb_z - 8.5 - 18, pcb_z - 2.5)
            for sx in (-1, 1)], note="female jumper plugs on the downward pins + wire bend", mates={"esp32", "deck"})
    m.part("esp32_usb", "keepout", COL["keepout"], [box(ecx - 6, ecx + 6, el / 2 + 1, el / 2 + 22, pcb_z - 3, pcb_z + 8)],
           note="right-angle micro-USB plug to the Pi", mates={"esp32"})
    esp_pin = (ecx + ew / 2 - 2.5, 0.0, pcb_z - 16)
    m.info["esp32"] = {"pcb_z": pcb_z, "x": ecx}
    m.part("fuse_zone", "keepout", (255, 160, 0), [box(-180, -146, -30, 30, zf1 + 1, zf1 + 28)],
           note="20 A blade fuse + XT30/Wago distribution, below the deck behind the battery [M]", mates={"tub"})
    m.part("xt60_plug", H, COL["battery"], [box(x0 - 14, xr_in, -38, -22, 117, 125)], mass=0.01, mates={"tub"},
           note="XT60 arming plug in the rear wall: pull it = robot dead (main switch)")
    uc = vadd(plate_c, vmul(n, -6.0))
    us_adds = [obox(uc, 1.6, 45.0, 20.0, n, Y)]
    for s in (1, -1):
        a_ = vadd(uc, vmul(Y, s * 13)); us_adds.append(cyl(a_, vadd(a_, vmul(n, 12.5)), 8.0))
    hdr = vadd(vadd(uc, vmul(tv, 10.0 + 4.0)), vmul(n, -1.5))
    us_adds.append(obox(hdr, 2.5, 10.2, 8.0, n, Y))
    m.part("hcsr04", H, COL["pcb_blue"], us_adds, mass=0.009, mates={"us_bracket", "us_plug"},
           note="HC-SR04 upside down (header up) [M: confirm the header lies in the board plane]")
    plug = vadd(vadd(uc, vmul(tv, 10.0 + 8.0 + 9.0)), vmul(n, -1.5))
    m.part("us_plug", "keepout", COL["keepout"], [obox(plug, 7.0, 13.0, 18.0, n, Y)],
           note="female Dupont plug (14 mm) + wire bend", mates={"hcsr04"})
    m.part("servo", H, COL["hw_dark"], [box(sbx - sl / 2, sbx + sl / 2, -sw / 2, sw / 2, seat - P["servo_ear_z"], seat + 13),
                                        box(sbx - P["servo_ear_span"] / 2, sbx + P["servo_ear_span"] / 2, -sw / 2, sw / 2, seat, seat + 2.5),
                                        cyl_z(seat + 13, horn_z0 + 0.01, mx, 0, 3)],
           mass=0.055, mates={"mast", "pan_buckle", "servo9g_adapter"}, note="MG996R / DS3218 class")
    cam_bot = piv_z + 7.0
    gw, gh, gd = P["gopro_w"], P["gopro_h"], P["gopro_d"]
    cam_c = (mx, 0.0, cam_bot + gh / 2)
    lens = (mx + gd / 2 + P["gopro_lens_protrude"], P["gopro_lens_dy"], cam_c[2] + P["gopro_lens_dz"])
    m.part("gopro", H, COL["hw_dark"], [box(mx - gd / 2, mx + gd / 2, -gw / 2, gw / 2, cam_bot, cam_bot + gh),
                                         cyl_x(mx + gd / 2 - 0.01, lens[0], lens[1], lens[2], 11),
                                         box(mx - 7.5, mx + 7.5, -4.6, 4.6, piv_z - 7.5, cam_bot + 0.01)],
           mass=P["gopro_mass"], moving=True, mates={"pan_buckle"}, note="HERO9 Black 71 x 55 x 33.6, 158 g [V]")
    m.info["camera"] = {"lens": lens, "pan_axis": (mx, 0.0), "centre": cam_c, "bottom": cam_bot}
    m.part("pan_sweep", "keepout", COL["keepout"],
           [cyl_z(cam_bot - 1, cam_bot + gh + 1, mx, 0, math.hypot(gw / 2, gd / 2 + P["gopro_lens_protrude"]) + 2)],
           note="GoPro swept volume over the pan range", mates={"gopro", "pan_buckle", "servo"})
    m.part("uwb_anchor", H, COL["pcb_green"], [box(upx + 6, upx + 12, -25, 25, uwz0 + 2, uwz1 - 2)],
           mass=0.03, mates={"mast"}, note="UWB AoA anchor envelope 50 x %.0f [M]" % (uwz1 - uwz0 - 4))
    m.info["P"] = P

    # ================= nests, assembly sweeps, wiring =================
    for side in ("L", "R"):
        for nm, _ in AX:
            k = nm + side
            m.nests += [("axle_" + k, "wheel_" + k, "bolt through the wheel bearings"),
                        ("axle_" + k, "axle_nut_" + k, "nut on its bolt"),
                        ("axle_" + k, "spacer_" + k, "bolt through the spacer"),
                        ("spacer_" + k, "wheel_" + k, "spacer reaches the recessed inner race")]
    m.nests += [("gopro", "pan_buckle", "camera fingers interleave the buckle prongs"),
                ("servo", "servo9g_adapter", "alternatives: only one is fitted")]
    mast_grp = ["mast", "servo", "pan_buckle", "gopro", "uwb_anchor", "servo9g_adapter"]
    deck_grp = ["deck", "pi5", "reg_L", "reg_R", "esc_L", "esc_R", "esp32"] + ["pi_standoff_%d" % i for i in range(4)]
    bolts = ["plate_bolt_%s%d" % (sd_, k) for sd_ in "LR" for k in range(3)]
    m.removals = [
        (mast_grp, (0, 0, 1), 170, []),
        (["shell"], (0, 0, 1), 60, mast_grp),
        (deck_grp, (0, 0, 1), 90, mast_grp + ["shell"]),
        (["bumper_door"], (1, 0, 0), 70, []),
        (["battery"], (1, 0, 0), 180, ["bumper_door"]),
    ]
    for side, sgn in (("L", 1), ("R", -1)):
        for nm, _ in AX:
            k = nm + side
            m.removals += [
                (["axle_" + k], (0, sgn, 0), 150, ["axle_nut_" + k, "wheel_" + k, "spacer_" + k]),
                (["axle_nut_" + k], (0, -sgn, 0), 25, ["axle_" + k, "spacer_" + k, "battery", "bumper_door"]),
            ]
        m.removals.append((["wheel_F" + side, "wheel_pulley_F" + side, "wheel_R" + side, "wheel_pulley_R" + side,
                            "belt2_" + side], (0, sgn, 0), 120,
                           ["axle_" + a_ + side for a_ in "FR"] + ["axle_nut_" + a_ + side for a_ in "FR"]
                           + ["spacer_" + a_ + side for a_ in "FR"] + ["belt_" + side, "idler_" + side,
                                                                      "idler_travel_" + side]))
        m.removals.append((["motor_" + side, "motor_plate_" + side, "motor_pulley_" + side], (0, sgn, 0), 120,
                           ["wheel_F" + side, "wheel_pulley_F" + side, "belt_" + side, "belt2_" + side, "shell",
                            "axle_F" + side, "axle_nut_F" + side, "spacer_F" + side] + bolts))
    sx = mx - 20
    ch = (mx - sq / 2 + mw / 2, 0.0)
    cz = head_z0 - 7
    pz = dz1 + 12
    ein = [(ecx + 22, 36, esp_pin[2]), (esp_pin[0], 36, esp_pin[2]), esp_pin]   # into the cradle from its open +y end
    fz = (-163, 0, zf1 + 14)                                                    # fuse / distribution block
    m.routes = [
        ("battery main 12 AWG", [(48, -26, 48), (38, -26, 60), (30, -45, 73), (-150, -45, 73), (-163, -20, zf1 + 20), fz]),
        ("ESC L power + phase", [(-80, inner - 12, 62), (-118, 40, 70), (-121, 40, 90), (-134, 42, dz1 + 7)]),
        ("ESC R power + phase", [(-80, -(inner - 12), 62), (-118, -40, 70), (-121, -40, 90), (-134, -42, dz1 + 7)]),
        ("regulator inputs 18 AWG", [fz, (-163, 20, zf1 + 20), (-150, 35, 73), (-121, 35, 73), (-121, 35, 92), (-110, 30, 112), (20, 30, 112), (20, 39, dz1 + 11)]),
        ("GoPro USB-C (service loop around the pan)", [(mx - 6, 38, cam_bot + 20), (mx - 30, 38, cam_bot + 10), (mx - 30, 30, cz),
                                                      (ch[0], 0, cz), (ch[0], 0, 120), (sx, 0, 120),
                                                      (pi_x0 - 12, -10, 120), (pi_x0 - 12, -10, pz)]),
        ("servo signal (pan)", [(sbx - sl / 2 - 3, 8, head_z0 + 10), (sbx - sl / 2 - 3, 8, cz), (ch[0], 2, cz),
                                (ch[0], 2, 124), (-150, 36, 124)] + ein),
        ("servo 6 V from reg R", [(20, -39, dz1 + 11), (20, -24, 110), (ch[0] - 10, -3, 110), (ch[0], -3, 115)]),
        ("UWB anchor USB -> Pi", [(upx + 8, 22, uwz0 + 6), (upx + 8, 33, uwz0 + 6), (upx - 3, 33, uwz0 + 6), (mx, 26, cz),
                                  (mx - 16, 26, cz), (mx - 16, 0, cz), (ch[0], -2, cz),
                                  (ch[0], -2, 118), (sx, -3, 118), (pi_x0 - 12, 0, 118), (pi_x0 - 12, 0, pz)]),
        ("ESP32 USB", [(ecx, el / 2 + 18, pcb_z + 2), (ecx + 30, 40, pcb_z + 2), (pi_x0 - 12, 10, 120), (pi_x0 - 12, 10, pz)]),
        ("ESC signals -> ESP32", [(-134, 40, dz1 + 15), (-150, 36, 120)] + ein),
        ("Pi 5 V from reg L", [(20, 39, dz1 + 11), (20, 30, 116), (pi_x0 + 40, 26, 116), (pi_x0 + 40, 26, dz1 + 12)]),
        ("HC-SR04", [(plug[0], 0, plug[2] + 8), (x1 + 6, -12, 133), (x1 + 3, -30, 131), (xf_in - 8, -30, 131),
                     (xf_in - 20, -30, 128), (-150, 36, 124)] + ein),
    ]
    return m


# ----------------------------------------------------------------------------------------------------------
# Fusion 360 builder
# ----------------------------------------------------------------------------------------------------------
def run(context):
    import adsk.core
    import adsk.fusion
    app = adsk.core.Application.get()
    ui = app.userInterface
    failures = []
    try:
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        try:
            design.designType = adsk.fusion.DesignTypes.DirectDesignType
        except Exception:
            pass
        direct = design.designType == adsk.fusion.DesignTypes.DirectDesignType
        root = design.rootComponent
        tbm = adsk.fusion.TemporaryBRepManager.get()
        model = build_model()
        S = 0.1  # mm -> cm

        def P3(p):
            return adsk.core.Point3D.create(p[0] * S, p[1] * S, p[2] * S)

        def V3(v):
            return adsk.core.Vector3D.create(v[0], v[1], v[2])

        sphere_fail = []
        bool_fail = []

        def boolop(a, b, kind):
            try:
                tbm.booleanOperation(a, b, kind)
            except Exception:
                bool_fail.append(1)

        def prim(p):
            if p["t"] == "tor":
                try:
                    return tbm.createTorus(P3(p["c"]), V3((0.0, 0.0, 1.0)), p["R"] * S, p["r"] * S)
                except Exception:          # fallback: chain of spheres along the ring
                    sphere_fail.append(1)
                    body = None
                    n_s = min(96, max(12, int(math.ceil(2 * math.pi * p["R"] / (0.8 * p["r"])))))   # overlapping spheres
                    for k in range(n_s):
                        a_ = 2 * math.pi * k / n_s
                        sb = prim(sph((p["c"][0] + p["R"] * math.cos(a_), p["c"][1] + p["R"] * math.sin(a_), p["c"][2]), p["r"]))
                        if sb is None:
                            continue
                        if body is None:
                            body = sb
                        else:
                            boolop(body, sb, adsk.fusion.BooleanTypes.UnionBooleanType)
                    return body
            if p["t"] == "sph":
                try:
                    return tbm.createSphere(P3(p["c"]), p["r"] * S)
                except Exception:          # fallback: skip (edge corners stay slightly squarer)
                    sphere_fail.append(1)
                    return None
            if p["t"] == "diff":
                a = prim(p["a"])
                for q in p["b"]:
                    bq = prim(q)
                    if a is not None and bq is not None:
                        boolop(a, bq, adsk.fusion.BooleanTypes.DifferenceBooleanType)
                return a
            if p["t"] == "box":
                w = norm(cross(p["u"], p["v"]))
                v = cross(w, p["u"])  # re-orthogonalise
                obb = adsk.core.OrientedBoundingBox3D.create(P3(p["c"]), V3(norm(p["u"])), V3(norm(v)),
                                                             p["s"][0] * S, p["s"][1] * S, p["s"][2] * S)
                return tbm.createBox(obb)
            return tbm.createCylinderOrCone(P3(p["a"]), p["r"] * S, P3(p["b"]), p["r"] * S)

        def merge(prims):
            body = None
            for q in prims:
                b = prim(q)
                if b is None:
                    continue
                if body is None:
                    body = b
                else:
                    boolop(body, b, adsk.fusion.BooleanTypes.UnionBooleanType)
            return body

        appearances = {}

        def appearance(name, rgb):
            if name in appearances:
                return appearances[name]
            a = None
            try:
                a = design.appearances.itemByName(name)
                if a is None:
                    base = None
                    for i in range(app.materialLibraries.count):
                        lib = app.materialLibraries.item(i)
                        if "Appearance" not in lib.name:
                            continue
                        for nm in ("Plastic - Matte (Yellow)", "Plastic - Glossy (Yellow)", "Paint - Enamel Glossy (Yellow)"):
                            base = lib.appearances.itemByName(nm)
                            if base:
                                break
                        if base is None:
                            for j in range(min(lib.appearances.count, 400)):
                                cand = lib.appearances.item(j)
                                if cand.name.startswith("Plastic"):
                                    base = cand
                                    break
                        if base:
                            break
                    if base is not None:
                        a = design.appearances.addByCopy(base, name)
                        for j in range(a.appearanceProperties.count):
                            cp = adsk.core.ColorProperty.cast(a.appearanceProperties.item(j))
                            if cp and cp.id in ("opaque_albedo", "generic_diffuse", "plastic_albedo", "color"):
                                cp.value = adsk.core.Color.create(int(rgb[0]), int(rgb[1]), int(rgb[2]), 0)
            except Exception:
                a = None
            appearances[name] = a
            return a

        groups = {}
        for gname, label in (("printed", "Printed parts"), ("hardware", "Reference hardware"), ("keepout", "Keep-out volumes"),
                             ("tools", "Tools (print once)"), ("prototype", "Prototype kit (printed sleeves, TPU belts)")):
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occ.component.name = label
            groups[gname] = occ.component
        for part in model.parts:
            try:
                parent = groups[part["group"]]
                occ = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                comp = occ.component
                comp.name = part["name"]
                body = None
                for st_adds, st_cuts in part.get("pre", []):
                    sb = merge(st_adds)
                    if st_cuts:
                        boolop(sb, merge(st_cuts), adsk.fusion.BooleanTypes.DifferenceBooleanType)
                    if body is None:
                        body = sb
                    else:
                        boolop(body, sb, adsk.fusion.BooleanTypes.UnionBooleanType)
                ab = merge(part["adds"]) if part["adds"] else None
                if ab is not None:
                    if body is None:
                        body = ab
                    else:
                        boolop(body, ab, adsk.fusion.BooleanTypes.UnionBooleanType)
                if part["cuts"]:
                    tool = merge(part["cuts"])
                    boolop(body, tool, adsk.fusion.BooleanTypes.DifferenceBooleanType)
                if direct:
                    b = comp.bRepBodies.add(body)
                else:   # parametric design: bodies must live in a base feature
                    bf = comp.features.baseFeatures.add()
                    bf.startEdit()
                    b = comp.bRepBodies.add(body, bf)
                    bf.finishEdit()
                b.name = part["name"]
                ap = appearance("honebot %d-%d-%d" % part["color"], part["color"])
                if ap is not None:
                    b.appearance = ap
                if part["group"] == "keepout":
                    try:
                        b.opacity = 0.25
                    except Exception:
                        pass
                    occ.isLightBulbOn = False
                if part.get("hidden"):
                    occ.isLightBulbOn = False
            except Exception:
                failures.append("%s: %s" % (part["name"], traceback.format_exc().splitlines()[-1]))
        try:
            app.activeViewport.fit()
        except Exception:
            pass
        msg = "honebot: built %d of %d parts." % (len(model.parts) - len(failures), len(model.parts))
        if failures:
            msg += "\n\nFailed parts:\n" + "\n".join(failures)
        if bool_fail:
            msg += "\n\n%d boolean operations failed and were skipped (a part may miss a hole or a rounded edge)." % len(bool_fail)
        if sphere_fail:
            msg += "\n\n(Spheres unavailable in this Fusion version: %d rounded corners were skipped.)" % len(sphere_fail)
        msg += ("\n\nKeep-out volumes are hidden (turn on 'Keep-out volumes' to see swept envelopes)."
                "\nTo print: right-click a component under 'Printed parts' > Save As Mesh (3MF/STL).")
        ui.messageBox(msg, "honebot chassis")
    except Exception:
        if ui:
            ui.messageBox("honebot script failed:\n%s" % traceback.format_exc(), "honebot chassis")
