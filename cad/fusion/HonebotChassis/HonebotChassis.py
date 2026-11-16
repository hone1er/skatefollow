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
    "clr_m2": 2.4, "clr_m25": 2.9, "clr_m3": 3.4, "clr_m4": 4.5, "clr_m5": 5.4, "clr_m8": 8.5,   # [D] through-holes
    "ins_m3": 4.0, "ins_m3_depth": 6.5, "ins_m4": 5.6, "ins_m4_depth": 9.0,  # [D] heat-set inserts (check insert maker's chart)
    "bearing608_seat": 22.2, "bearing608_w": 7.0,  # [D] 608 = 8x22x7; 22.2 seat + retaining compound
    "slip": 0.6,          # [D] gap for parts that slide into each other (mast in socket, deck between walls)
    "cut_over": 0.6,      # [D] cut tools overshoot faces by this much (robust booleans)
    # drive wheels: 90 x 52 mm longboard wheels with 608 bearings [M: core holes / bearing inset]
    "wheel_d": 90.0, "wheel_w": 52.0, "axle_z": 45.0,
    # tub (main printed body, PAHT-CF)
    "tub_x0": -150.0, "tub_x1": 58.0, "tub_hw": 62.0, "wall": 4.0, "floor_z0": 28.0, "floor_t": 4.0,
    "tub_top": 140.0, "front_t": 6.0, "rear_t": 6.0,
    # battery: 3S 5000 mAh hard case. Tunnel fits <= 145 x 50 x 30 [M]
    "bat_l": 145.0, "bat_w": 50.0, "bat_h": 30.0, "bat_x0": -141.0,
    # electronics deck (removable tray, PETG or PAHT-CF)
    "deck_z0": 79.0, "deck_t": 3.0, "grid_pitch": 10.0,
    # drive: 5008/5010-class outrunner (envelope, [M]) -> HTD-3M 9 mm belt -> printed 48T wheel pulley
    # reference motor: MAD 5010 EEE V2.0 370KV - O56 x 32.7, 5 mm shaft, base M3 on D25/D30, prop M3 on D12/D18
    # [V, retailer listings]; envelope below leaves margin. Any motor with M3 prop holes on D12 fits the pulley.
    "motor_env_d": 62.0, "motor_env_l": 33.0, "motor_shaft_d": 5.0, "motor_prop_pcd": 12.0,
    "motor_dx": -35.0,                              # [D] motor centre 35 mm behind the axle
    "pul_m_t": 24, "pul_w_t": 48, "belt_len": 231.0,   # [D] HTD-3M teeth (PD = 3*T/pi [V]); 231 mm = 77T belt
    "belt_w": 9.0,
    "motor_plate": 66.0, "motor_plate_t": 8.0, "motor_bolt_r": 27.0, "tension_travel": 8.0,
    # caster (rear): 70 mm wheel, 51 wide [M], 25 mm trail
    "caster_d": 70.0, "caster_w": 51.0, "caster_trail": 25.0, "pivot_x": -223.0,
    "tail_x0": -298.0, "tail_hw": 66.0, "tail_deck_z": 112.0,
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
             mates=(), hidden=False):
        p = {"name": name, "group": group, "color": color, "adds": list(adds), "cuts": list(cuts or []),
             "mass": mass, "material": material, "moving": moving, "note": note, "mates": set(mates),
             "hidden": hidden}
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
    lo_, hi_ = 20.0, 200.0
    for _ in range(80):
        cc = (lo_ + hi_) / 2
        Lc = 2 * cc + math.pi * (P["pul_m_pd"] + P["pul_w_pd"]) / 2 + (P["pul_w_pd"] - P["pul_m_pd"]) ** 2 / (4 * cc)
        lo_, hi_ = (cc, hi_) if Lc < P["belt_len"] else (lo_, cc)
    P["belt_cc"] = cc
    P["belt_plane_y"] = hw + P["motor_env_l"] + 1.5 + 5.5      # pulley bolted to the bell: flange 1.5 + half tooth width
    P["wheel_inner_y"] = P["belt_plane_y"] + 7.0 + 3.5         # wheel pulley half width + adapter disc
    mdx = P["motor_dx"]; mdz = math.sqrt(P["belt_cc"] ** 2 - mdx ** 2)
    mc_x, mc_z = mdx, az + mdz
    bu = norm((mdx, 0.0, mdz))                 # belt direction (axle -> motor)
    bv = norm(cross(bu, Y))                    # in-plane perpendicular; bu x bv = +y
    if cross(bu, bv)[1] < 0:
        bv = vmul(bv, -1)
    m.info.update(motor_centre=(mc_x, mc_z), belt_u=bu, belt_v=bv)

    # ---------------- TUB ----------------
    tub_adds = [
        box(x0, x1, -hw, hw, zf0, zf1),                                  # floor
        box(x0, x1, inner - 0.01, hw, zf0, top), box(x0, x1, -hw, -inner + 0.01, zf0, top),   # side walls
        box(xf_in - 0.01, x1, -hw, hw, zf0, top),                        # front wall
        box(x0, xr_in + 0.01, -hw, hw, zf0, top),                        # rear wall
        box(-12, 12, -inner, inner, 66, dz0),                            # axle bridge beam (deck rests on it)
        box(P["mast_x"] - 17, xf_in + 0.01, -24, 24, dz1, top),          # mast socket block (behind front wall)
        box(xr_in - 0.01, xr_in + 8, -24, 24, dz0 - 9, dz0),             # rear deck-screw block
        box(x0 - 14, x0 + 0.01, -50, 50, P["tail_deck_z"] - 12, P["tail_deck_z"]),  # rear ledge for the tail lip
    ]
    for s in (1, -1):
        tub_adds += [
            box(-15, 15, s * 36, s * (inner + 0.01), zf0, 70) if s > 0 else box(-15, 15, -(inner + 0.01), -36, zf0, 70),
            cyl_y(s * (hw - 0.01), s * (hw + 4), 0, az, 11),             # axle boss ring outside the wall
            # deck ledges
            box(xr_in, xf_in, s * (inner - 4), s * inner, dz0 - 4, dz0) if s > 0 else box(xr_in, xf_in, -inner, -(inner - 4), dz0 - 4, dz0),
            # lid / lap posts
            box(xf_in - 12, xf_in, s * 46, s * inner, dz1, top) if s > 0 else box(xf_in - 12, xf_in, -inner, -46, dz1, top),
            box(xr_in, xr_in + 14, s * 50, s * inner, dz1, top),
            # door-insert bosses behind the front wall
            box(xf_in - 8, xf_in + 0.01, s * 34, s * 46, 40, 58) if s > 0 else box(xf_in - 8, xf_in + 0.01, -46, -34, 40, 58),
            # battery guide rails on the floor
            box(P["bat_x0"], xf_in, s * (P["bat_w"] / 2 + 1.5), s * (P["bat_w"] / 2 + 5), zf1 - 0.01, zf1 + 8) if s > 0
            else box(P["bat_x0"], xf_in, -(P["bat_w"] / 2 + 5), -(P["bat_w"] / 2 + 1.5), zf1 - 0.01, zf1 + 8),
        ]
        # inner wall pad behind the motor plate (oriented in the belt frame)
        for k in (-1, 1):
            cc = vadd((mc_x, 0, mc_z), vmul(bu, k * P["tension_travel"] / 2))
            tub_adds.append(cyl_y(s * (inner - 8), s * (inner + 0.01), cc[0], cc[2], P["motor_plate"] / 2 + 5))
        tub_adds.append(obox((mc_x, s * (inner - 4), mc_z), P["tension_travel"], P["motor_plate"] + 10, 8.02, bu, bv))
        # inner bosses: pod-cover screws, HC-SR04 bracket inserts
        for xx in (mc_x - 38, mc_x + 50):
            tub_adds.append(box(xx - 6, xx + 6, s * (inner - 5), s * (inner + 0.01), top - 12, top))
        tub_adds.append(box(xf_in - 6, xf_in + 0.01, s * 34, s * 46, P["us_z"] - 8, P["us_z"] + 8))
    tub = m.part("tub", "printed", COL["frame"], tub_adds, material="PAHT-CF", note="main body, print floor-down")
    tub_cuts = tub["cuts"]
    # battery door opening through the front wall
    tub_cuts.append(box(xf_in - co, x1 + co, -28, 28, zf1 - co, 66))
    # tail window in the rear wall (cables tub <-> tail)
    tub_cuts.append(box(x0 - co, xr_in + co, -40, 40, 90, 126))
    # wire pass-throughs in the bridge beam (side channels under the deck)
    for s in (1, -1):
        m.hole("tub", "beam_pass_%s" % ("L" if s > 0 else "R"), (-12 - co, s * 45, 72), (12 + co, s * 45, 72), 12,
               for_item="harness side channel", audit=False)
    # mast socket bore + bottom/rear cable exit
    ms = P["mast_sq"] + P["slip"]
    tub_cuts.append(box(P["mast_x"] - ms / 2, P["mast_x"] + ms / 2, -ms / 2, ms / 2, 92, top + co))
    tub_cuts.append(box(P["mast_x"] - 17 - co, P["mast_x"], -7, 7, 92, 104))
    tub_cuts.append(box(P["mast_x"] - 17 - co, P["mast_x"], -5, 5, 92, top + co))     # rear slot: cables lift out with the mast
    for s in (1, -1):   # mast flange thumb screws (M4 inserts in the socket top)
        m.hole("tub", "mast_flange_" + ("L" if s > 0 else "R"), (P["mast_x"], s * 17.65, top + co),
               (P["mast_x"], s * 17.65, top - P["ins_m4_depth"]), P["ins_m4"], kind="insert", for_item="mast flange M4")
    # HC-SR04 wire hole through the front wall (offset to clear the mast socket)
    m.hole("tub", "us_wire", (xf_in - co, -29, 100), (x1 + co, -29, 100), 8, for_item="HC-SR04 cable", audit=False)
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        ys = s * 1.0
        # stub axle: M8 through boss and wall
        m.hole("tub", "axle_" + side, (0, s * 36 - s * co, az), (0, s * (hw + 4 + co), az), P["clr_m8"],
               for_item="M8 stub axle " + side)
        tub_cuts.append(box(-7.7, 7.7, s * (36 - co), s * 42, az - 6.7, az + 6.7))   # M8 head pocket (stops it turning)
        # motor plate pocket (outer face, capsule along the belt for tensioning) + shaft clearance + tension slots
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
        m.hole("tub", "motor_wire_" + side, (-66, s * (inner - 10), 72), (-66, s * (hw + co), 72), 12,
               for_item="motor phase wires grommet " + side, audit=False)
        # tail lap: 2 x M4 inserts per side in the rear posts
        for zz in (100, 125):
            m.hole("tub", "lap_%s_%d" % (side, zz), (xr_in + 7, s * (hw + co), zz),
                   (xr_in + 7, s * (hw - P["ins_m4_depth"]), zz), P["ins_m4"], kind="insert", for_item="tail lap M4")
        # lid inserts (front + rear posts)
        m.hole("tub", "lid_f_" + side, (xf_in - 6, s * 52, top + co), (xf_in - 6, s * 52, top - P["ins_m3_depth"]),
               P["ins_m3"], kind="insert", for_item="tub lid M3")
        m.hole("tub", "lid_r_" + side, (xr_in + 7, s * 54, top + co), (xr_in + 7, s * 54, top - P["ins_m3_depth"]),
               P["ins_m3"], kind="insert", for_item="tub lid M3")
        # deck inserts: bridge beam (front pair) and rear blocks
        m.hole("tub", "deck_f_" + side, (0, s * 28, dz0 + co), (0, s * 28, dz0 - P["ins_m3_depth"]), P["ins_m3"],
               kind="insert", for_item="deck M3")
        m.hole("tub", "deck_r_" + side, (xr_in + 4, s * 16, dz0 + co), (xr_in + 4, s * 16, dz0 - P["ins_m3_depth"]),
               P["ins_m3"], kind="insert", for_item="deck M3")
        # door thumb-screw inserts (M4) through front wall into the bosses
        m.hole("tub", "door_" + side, (x1 + co, s * 40, 49), (x1 - P["ins_m4_depth"] - 4, s * 40, 49), P["ins_m4"],
               kind="insert", for_item="bumper-door M4 thumb screw")
        # HC-SR04 bracket inserts (front face)
        m.hole("tub", "us_" + side, (x1 + co, s * 40, P["us_z"]), (x1 - P["ins_m3_depth"], s * 40, P["us_z"]),
               P["ins_m3"], kind="insert", for_item="HC-SR04 bracket M3")
        # tail lip inserts in the rear ledge
        m.hole("tub", "lip_" + side, (x0 - 7.5, s * 35, P["tail_deck_z"] + co), (x0 - 7.5, s * 35, P["tail_deck_z"] - P["ins_m4_depth"]),
               P["ins_m4"] - 0.0, kind="insert", for_item="tail lip M4")

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
            # counterbore for the screw heads on the wall side
            m.get(name)["cuts"].extend(slot((p0[0], 0, p0[2]), (p1[0], 0, p1[2]), 6.4, (0, s, 0), 3.0 + co,
                                            hw - P["motor_plate_t"] + 1.5 - co / 2))
        for k, (ox, oz) in enumerate(MOTOR_PLATE_BOLTS):
            c = (mc_x + ox * P["motor_bolt_r"], 0, mc_z + oz * P["motor_bolt_r"])
            m.hole(name, "ins_%d" % k, (c[0], s * (hw - P["motor_plate_t"] - co), c[2]),
                   (c[0], s * (hw - P["motor_plate_t"] + P["ins_m3_depth"]), c[2]), P["ins_m3"], kind="insert",
                   for_item="motor plate <- wall M3")
            # bolt head (inside the tub) + hex-key access envelope, both checked for clearance
            hy0 = s * (inner - 8)
            m.part("plate_bolt_%s%d" % (side, k), "hardware", COL["hw_metal"],
                   [cyl_y(hy0 - s * 3.0, hy0 + s * 0.01, c[0], c[2], 2.9)], mass=0.003, mates={"tub"},
                   note="M3 x 16 socket head + washer")
            m.part("tool_access_%s%d" % (side, k), "keepout", COL["keepout"],
                   [cyl_y(hy0 - s * 17.0, hy0 - s * 3.0, c[0], c[2], 5.0)],
                   note="2.5 mm ball-end hex key (lid off)", mates={"tub", "plate_bolt_%s%d" % (side, k)})

    # ---------------- DECK ----------------
    g = P["slip"] / 2
    deck = m.part("deck", "printed", COL["light"], [box(xr_in + g, xf_in - g, -inner + g, inner - g, dz0, dz1)],
                  material="PETG", note="removable electronics tray, 10 mm M3 grid", mates={"tub"})
    dc = deck["cuts"]
    for s in (1, -1):
        dc.append(box(xf_in - 12.5, xf_in + co, s * 45.5 if s > 0 else -inner - co, s * (inner + co) if s > 0 else -45.5, dz0 - co, dz1 + co))
        dc.append(box(xr_in - co, xr_in + 14.5, s * 49.5 if s > 0 else -inner - co, s * (inner + co) if s > 0 else -49.5, dz0 - co, dz1 + co))
        # clear the inner motor pads
        dc.append(box(mc_x - 44, mc_x + 44, s * 47.5 if s > 0 else -inner - co, s * (inner + co) if s > 0 else -47.5, dz0 - co, dz1 + co))
        # cable pass slots at the rear corners
        dc.append(box(xr_in + 15, xr_in + 24, s * 30 if s > 0 else -47, s * 47 if s > 0 else -30, dz0 - co, dz1 + co))
    dc.append(box(P["mast_x"] - 17.5, xf_in + co, -24.5, 24.5, dz0 - co, dz1 + co))     # mast socket notch
    for s in (1, -1):   # notches so the deck lifts straight out past the wall bosses
        dc.append(box(mc_x + 50 - 6.5, mc_x + 50 + 6.5, s * 52.5, s * (inner + co), dz0 - co, dz1 + co))
        dc.append(box(xf_in - 6.5, xf_in + co, s * 33.5, s * 46.5, dz0 - co, dz1 + co))
    # Raspberry Pi 5: 85 x 56, USB end facing rear; M2.5 holes 58 x 49, 3.5 from the non-USB end [V]
    pi_x0 = P["pi_x0"]; pi_x1 = pi_x0 + 85.0
    pi_holes = [(pi_x1 - 3.5, -24.5), (pi_x1 - 3.5, 24.5), (pi_x1 - 61.5, -24.5), (pi_x1 - 61.5, 24.5)]
    for i, (hx, hy) in enumerate(pi_holes):
        m.hole("deck", "pi_%d" % i, (hx, hy, dz0 - co), (hx, hy, dz1 + co), P["clr_m25"], for_item="Raspberry Pi 5 M2.5 standoff")
    m.info["pi"] = (pi_x0, pi_x1)
    # 10 mm grid (M3 clearance) for everything with seller-dependent holes (ESP32 ext board etc. live in the tail)
    gp = P["grid_pitch"]
    keep = [(hx, hy) for hx, hy in pi_holes]
    xs = [xr_in + 25 + gp * i for i in range(int((xf_in - xr_in - 40) / gp))]
    ys = [gp * j for j in range(-4, 5)]
    for gx in xs:
        for gy in ys:
            if any(math.hypot(gx - kx, gy - ky) < 7.5 for kx, ky in keep):
                continue
            if abs(gx - mc_x) < 47 and abs(gy) > 44:
                continue
            if gx > P["mast_x"] - 21 and abs(gy) < 28:
                continue
            if gx > xf_in - 16 or (gx > xf_in - 16 and abs(gy) > 42):
                continue
            if (abs(gx) < 6 and abs(abs(gy) - 28) < 6) or (gx < xr_in + 10 and abs(abs(gy) - 16) < 6):
                continue
            if xr_in + 6 < gx < xr_in + 28 and 26 < abs(gy) < 51:
                continue
            if 4 < gx < 38 and 24 < abs(gy) < 56 or (gx > xf_in - 12 and abs(gy) > 30):
                continue
            dc.append(cyl_z(dz0 - co, dz1 + co, gx, gy, P["clr_m3"] / 2))
    m.info["deck_grid_n"] = len(xs) * len(ys)
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        m.hole("deck", "screw_f_" + side, (0, s * 28, dz0 - co), (0, s * 28, dz1 + co), P["clr_m3"], for_item="deck M3")
        m.hole("deck", "screw_r_" + side, (xr_in + 4, s * 16, dz0 - co), (xr_in + 4, s * 16, dz1 + co), P["clr_m3"], for_item="deck M3")
        # zip-tie slot pairs for the two regulators (x 7..35, |y| 30..56)
        for zx in (12, 30):
            dc.append(box(zx - 1.5, zx + 1.5, s * 29, s * 49, dz0 - co, dz1 + co))

    # ---------------- TAIL (caster module) ----------------
    tx0 = P["tail_x0"]; th = P["tail_hw"]; tdz = P["tail_deck_z"]; px = P["pivot_x"]
    tail_adds = [box(tx0, x0 - 0.01, -th, th, tdz, tdz + 3),           # deck plate
                 box(x0 - 14, x0, -50, 50, tdz, tdz + 3),                # front lip (on the tub's rear ledge)
                 box(tx0, tx0 + 4, -th, th, 90, top),                    # rear wall
                 cyl_z(84, tdz + 0.01, px, 0, 18),                        # pivot boss
                 box(px - 3, px + 3, -th + 0.01, th - 0.01, 100, tdz + 0.01),   # cross rib through the boss
                 box(px - 40, px + 40, -3, 3, 100, tdz + 0.01)]          # fore-aft rib
    for s in (1, -1):
        tail_adds += [
            box(tx0, x0 + 24, s * (hw - 0.01), s * th, 90, top) if s > 0 else box(tx0, x0 + 24, -th, -(hw - 0.01), 90, top),
            box(tx0 + 4, tx0 + 12, s * 54, s * hw, tdz, top) if s > 0 else box(tx0 + 4, tx0 + 12, -hw, -54, tdz, top),
            box(x0 - 12, x0 - 4, s * 54, s * hw, tdz, top) if s > 0 else box(x0 - 12, x0 - 4, -hw, -54, tdz, top),
        ]
    tail = m.part("tail", "printed", COL["frame"], tail_adds, material="PAHT-CF",
                  note="caster module, laps over the tub's rear posts", mates={"tub", "deck"})
    tc = tail["cuts"]
    m.hole("tail", "pivot", (px, 0, 80), (px, 0, tdz + 3 + co), P["clr_m8"], for_item="M8 caster pivot bolt")
    tc.append(cyl_z(84 - co, 84 + P["bearing608_w"] + 0.5, px, 0, P["bearing608_seat"] / 2))
    tc.append(cyl_z(tdz + 3 - P["bearing608_w"] - 0.5, tdz + 3 + co, px, 0, P["bearing608_seat"] / 2))
    m.holes.append({"host": "tail", "id": "608_seats", "a": (px, 0, 84), "b": (px, 0, tdz + 3),
                    "d": P["bearing608_seat"], "kind": "seat", "for": "2 x 608 caster swivel bearings", "audit": True})
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        for zz in (100, 125):
            m.hole("tail", "lap_%s_%d" % (side, zz), (xr_in + 7, s * (hw - co), zz), (xr_in + 7, s * (th + co), zz),
                   P["clr_m4"], for_item="tail lap M4")
        m.hole("tail", "lip_" + side, (x0 - 7.5, s * 35, tdz - co), (x0 - 7.5, s * 35, tdz + 3 + co), P["clr_m4"],
               for_item="tail lip M4")
        m.hole("tail", "lid_r_" + side, (tx0 + 8, s * 58, top + co), (tx0 + 8, s * 58, top - P["ins_m3_depth"]),
               P["ins_m3"], kind="insert", for_item="tail lid M3")
        m.hole("tail", "lid_f_" + side, (x0 - 8, s * 58, top + co), (x0 - 8, s * 58, top - P["ins_m3_depth"]),
               P["ins_m3"], kind="insert", for_item="tail lid M3")
    # arming plug (XT60 female, panel) [M] and cable notch to the tail deck
    tc.append(box(tx0 - co, tx0 + 4 + co, -38.2, -21.8, 120.9, 129.1))
    m.holes.append({"host": "tail", "id": "xt60_arming", "a": (tx0, -30, 125), "b": (tx0 + 4, -30, 125), "d": 16.4,
                    "kind": "slot", "for": "XT60 arming plug (panel)", "audit": False})
    # tail deck grid (ESP32 + SunFounder board, ESCs, fuse) - all [M] parts
    for gx in [tx0 + 14 + gp * i for i in range(int((x0 - 14 - tx0 - 14) / gp) + 1)]:
        for gy in [gp * j for j in range(-5, 6)]:
            if math.hypot(gx - px, gy) < 26 or abs(gx - px) < 4 or abs(gy) < 4 and abs(gx - px) < 41:
                continue
            tc.append(cyl_z(tdz - co, tdz + 3 + co, gx, gy, P["clr_m3"] / 2))

    # ---------------- CASTER FORK ----------------
    cw = P["caster_w"]; cr = P["caster_d"] / 2; cxw = px - P["caster_trail"]
    fork_adds = [box(cxw - 12, px + 12, -cw / 2 - 9, cw / 2 + 9, cr * 2 + 6, 83.5)]
    for s in (1, -1):
        fork_adds.append(box(cxw - 10, cxw + 10, s * (cw / 2 + 3), s * (cw / 2 + 9), cr - 12, cr * 2 + 6.01) if s > 0
                         else box(cxw - 10, cxw + 10, -(cw / 2 + 9), -(cw / 2 + 3), cr - 12, cr * 2 + 6.01))
        fork_adds.append(cyl_y(s * (cw / 2 + 3), s * (cw / 2 + 9), cxw, cr - 2, 10))
    m.part("caster_fork", "printed", COL["accent"], fork_adds, material="PAHT-CF", moving=True,
           note="25 mm trail; friction washer under the pivot nut damps shimmy", mates={"tail"})
    m.hole("caster_fork", "pivot", (px, 0, cr * 2 + 6 - co), (px, 0, 83.5 + co), P["clr_m8"], for_item="M8 caster pivot bolt")
    m.hole("caster_fork", "axle", (cxw, -cw / 2 - 9 - co, cr), (cxw, cw / 2 + 9 + co, cr), P["clr_m8"],
           for_item="M8 caster wheel axle")
    m.info["caster"] = {"pivot_x": px, "wheel_x": cxw, "r": cr, "w": cw}
    m.info["caster_sweep_r"] = math.hypot(P["caster_trail"] + cr, cw / 2)

    # ---------------- WHEEL PULLEYS (printed 48T HTD-3M + adapter disc) ----------------
    wpr = P["pul_w_pd"] / 2 - 0.38
    by = P["belt_plane_y"]; wiy = P["wheel_inner_y"]
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        name = "wheel_pulley_" + side
        ys = [by - 5.5, by + 5.5]
        adds = [cyl_y(s * (by - 7.0), s * (by + 7.0), 0, az, wpr),
                cyl_y(s * (by - 7.0), s * (by - 5.5), 0, az, 26.0),
                cyl_y(s * (by + 5.5), s * (by + 7.0), 0, az, 26.0),
                cyl_y(s * (by + 7.0), s * (wiy - 0.01), 0, az, 30.0)]
        m.part(name, "printed", COL["accent"], adds, material="PAHT-CF", moving=True,
               note="48T HTD-3M, print teeth axis vertical with 0.4 nozzle; bolts through the wheel core",
               mates={"wheel_" + side})
        m.hole(name, "bore", (0, s * (by - 7 - co), az), (0, s * (wiy + co), az), 15.0, for_item="spacer clearance",
               audit=False)
        # 48 HTD-3M grooves (approximate round-bottom profile, 1.14 mm deep) - see docs for an exact profile
        for k in range(48):
            ang = 2 * math.pi * k / 48
            rc = wpr - 1.14 + 0.9
            gx_, gz_ = rc * math.cos(ang), az + rc * math.sin(ang)
            m.get(name)["cuts"].append(cyl_y(s * (by - 5.5), s * (by + 5.5), gx_, gz_, 0.9))
        for k in range(4):
            ang = k * math.pi / 2 + math.pi / 4
            hx, hz = 16.5 * math.cos(ang), az + 16.5 * math.sin(ang)
            m.hole(name, "core_%d" % k, (hx, s * (by - 7 - co), hz), (hx, s * (wiy + co), hz), P["clr_m3"],
                   for_item="M3 x 70 through wheel core")

    # ---------------- POD COVERS (belt guards) ----------------
    # Caps over the motor's upper half and the motor pulley. Open below z = 98 so nothing reaches into the
    # wheel pulley / lower belt (moving) zone; the lower motor half sits in the wheel's shadow.
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        name = "pod_" + side
        sy = lambda a, b: (s * a, s * b) if s > 0 else (-b, -a)  # noqa: E731
        ya, yb = sy(hw - 0.01, 119)
        adds = rbox(mc_x - 37, mc_x + 47, ya, yb, 98, top + 2, 10)
        for xx in (mc_x - 38, mc_x + 50):
            ta, tb = sy(hw, hw + 4)
            adds.append(box(xx - 6, xx + 6, ta, tb, top - 12, top + 2))
        ia, ib = sy(hw - co, 116.5)
        cuts = rbox(mc_x - 34.5, mc_x + 44.5, ia, ib, 90, top - 0.5, 8)   # open bottom (cut overshoots)
        p = m.part(name, "printed", COL["shell"], adds, cuts, material="PETG",
                   note="cap over motor + motor pulley; screws into the wall bosses", mates={"tub", "lid"})
        for xx in (mc_x - 38, mc_x + 50):
            # the screw tab is re-added after the cavity cut by keeping it outside the cavity in y
            m.hole(name, "screw_%d" % int(xx), (xx, s * (hw - co), top - 6), (xx, s * (hw + 4 + co), top - 6), P["clr_m3"],
                   for_item="pod M3 into wall", audit=False)
            m.hole("tub", "pod_%s_%d" % (side, int(xx)), (xx, s * (hw + co), top - 6), (xx, s * (hw - P["ins_m3_depth"]), top - 6),
                   P["ins_m3"], kind="insert", for_item="pod M3")

    # ---------------- BUMPER-DOOR ("mouth") ----------------
    bfront = x1 + 26
    bd = m.part("bumper_door", "printed", COL["accent"],
                rbox(x1, bfront, -50, 50, 30, 76, 14)
                + [box(P["bat_x0"] + P["bat_l"] + 0.5, x1 + 0.01, sy_ * 12, sy_ * 22, zf1 + 4, zf1 + 20) for sy_ in (1, -1)]
                + [box(x1 + 8, bfront, sy_ * 28, sy_ * 46, 16, 30.01) for sy_ in (1, -1)],
                material="PETG (hollow crush zone; optional TPU skin)",
                note="battery door + front bumper; two prongs hold the pack against the rear wall; "
                     "skids stop a nose-over at ~10 deg pitch", mates={"tub", "battery"})
    for s in (1, -1):
        m.hole("bumper_door", "thumb_" + ("L" if s > 0 else "R"), (x1 - co, s * 40, 49), (bfront + co, s * 40, 49),
               P["clr_m4"], for_item="M4 thumb screw")
        bd["cuts"].append(cyl_x(x1 + 10, bfront + co, s * 40, 49, 6.0))      # thumb-screw head pocket
    for zz in (60, 67):
        bd["cuts"].append(box(bfront - 1.5, bfront + co, -30, 30, zz - 1, zz + 1))   # grille grooves
    bd["cuts"].append(box(x1 + 3, bfront - 3, -32, 32, 33, 56))            # hollow crush zone (TPU)

    # ---------------- HC-SR04 bracket ("eyes") ----------------
    t = math.radians(P["us_tilt_deg"]); n = (math.cos(t), 0.0, math.sin(t)); tv = (-math.sin(t), 0.0, math.cos(t))
    uz = P["us_z"]; plate_c = (x1 + 18, 0.0, uz)
    br_adds = [obox(plate_c, 4.0, 60.0, 30.0, n, Y)]
    for s in (1, -1):
        br_adds.append(box(x1, bfront, s * 29, s * 47, uz - 13, uz + 13))
    br_adds.append(box(x1, bfront, -47, 47, uz + 13, uz + 17))             # brow (protects the transducers)
    m.part("us_bracket", "printed", COL["light"], br_adds, material="PETG", mates={"tub"},
           note="tilts the sensor %.0f deg up so the ground return lands beyond slow_distance" % P["us_tilt_deg"])
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        m.hole("us_bracket", "screw_" + side, (x1 - co, s * 40, uz), (bfront + co, s * 40, uz), P["clr_m3"],
               for_item="bracket M3")
        # transducer windows (18 mm round, centres 26 mm apart [M]) and zip-tie slots for the PCB
        a_ = vadd(vadd(plate_c, vmul(Y, s * 13)), vmul(n, -2 - co)); b_ = vadd(a_, vmul(n, 4 + 2 * co))
        m.hole("us_bracket", "eye_" + side, a_, b_, 18.0, for_item="HC-SR04 transducer")
        m.slot_hole("us_bracket", "ztie_" + side, vadd(plate_c, vmul(vadd(vmul(Y, s * 26.5), vmul(tv, -7)), 1)),
                    vadd(plate_c, vmul(vadd(vmul(Y, s * 26.5), vmul(tv, 7)), 1)), 2.5, n, 4 + 2 * co,
                    for_item="zip tie holding the HC-SR04 PCB", audit=False)
    m.info["us"] = {"x": plate_c[0] + 7, "z": uz, "tilt": P["us_tilt_deg"], "half": P["us_beam_half_deg"]}

    # ---------------- LIDS ----------------
    lid_adds = rbox(x0, x1, -hw, hw, top, top + 5, 14) + rbox(x0 + 30, P["mast_x"] - 24, -22, 22, top + 4.99, top + 7, 8)
    lid = m.part("lid", "printed", COL["shell"], lid_adds, material="PETG", mates={"tub", "pod_L", "pod_R"},
                 note="remove the mast first (2 front thumb screws), then lift")
    lid["cuts"].append(box(P["mast_x"] - ms / 2, P["mast_x"] + ms / 2, -ms / 2, ms / 2, top - co, top + 7 + co))
    for s in (1, -1):
        m.hole("lid", "mast_flange_" + ("L" if s > 0 else "R"), (P["mast_x"], s * 17.65, top - co),
               (P["mast_x"], s * 17.65, top + 5 + co), P["clr_m4"], for_item="mast flange M4 thumb screw")
    for i in range(6):
        xx = -60 + i * 12
        lid["cuts"] += slot((xx, -16, 0), (xx, 16, 0), 4.0, (0, 0, 1), 7 + 2 * co, top + 3.5)
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        m.hole("lid", "f_" + side, (xf_in - 6, s * 52, top - co), (xf_in - 6, s * 52, top + 5 + co), P["clr_m3"], for_item="tub lid M3")
        m.hole("lid", "r_" + side, (xr_in + 7, s * 54, top - co), (xr_in + 7, s * 54, top + 5 + co), P["clr_m3"], for_item="tub lid M3")
    tlid = m.part("tail_lid", "printed", COL["shell"],
                  rbox(tx0, x0 - 0.5, -th, th, top, top + 5, 14) + [box(tx0 + 20, x0 - 20, -th + 0.01, -th + 3, top + 4.99, top + 7)],
                  material="PETG", mates={"tail"})
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        m.hole("tail_lid", "r_" + side, (tx0 + 8, s * 58, top - co), (tx0 + 8, s * 58, top + 5 + co), P["clr_m3"], for_item="tail lid M3")
        m.hole("tail_lid", "f_" + side, (x0 - 8, s * 58, top - co), (x0 - 8, s * 58, top + 5 + co), P["clr_m3"], for_item="tail lid M3")
    for i in range(5):
        tlid["cuts"] += slot((tx0 + 40 + i * 14, -30, 0), (tx0 + 40 + i * 14, 30, 0), 4.0, (0, 0, 1), 7 + 2 * co, top + 3.5)

    # ---------------- MAST + PAN HEAD ----------------
    mx = P["mast_x"]; sq = P["mast_sq"]; mw = P["mast_wall"]; seat = P["servo_seat_z"]
    sl, sw = P["servo_l"], P["servo_w"]; sbx = mx - P["servo_shaft_off"]       # servo body centre x
    head_z0 = seat - P["servo_ear_z"] - 4
    head = (sbx - sl / 2 - 10, sbx + sl / 2 + 10, -sw / 2 - 4, sw / 2 + 4, head_z0, seat)
    mast_adds = ([box(mx - sq / 2, mx + sq / 2, -sq / 2, sq / 2, 92 + 0.3, head_z0 + 0.01)]
                 + rbox(mx - 14, mx + 14, -24, 24, top + 5, top + 10, 5)          # flange, sits on the lid
                 + rbox(*head, r=4))
    mast = m.part("mast", "printed", COL["mast"], mast_adds, material="PAHT-CF (print lying on its side)",
                  mates={"tub", "lid", "uwb_plate"}, note="removable: 2 M4 thumb screws through the flange")
    mc = mast["cuts"]
    mc.append(box(mx - sq / 2 + mw, mx + sq / 2 - mw, -sq / 2 + mw, sq / 2 - mw, 92 - co, head_z0 + 6))  # bore
    mc.append(box(mx - sq / 2 - co, mx, -6, 6, 92 - co, 104))                    # bottom cable exit (rear)
    mc.append(box(sbx - sl / 2 - 0.4, sbx + sl / 2 + 0.4, -sw / 2 - 0.4, sw / 2 + 0.4, head_z0 + 4, seat + co))   # servo pocket
    mc.append(box(sbx - sl / 2 - 6, sbx - sl / 2 + 6, 4, 12, head_z0 - co, seat + co))    # servo cable notch (beside the ear screw), through the floor
    mc.append(box(mx - sq / 2 - co, mx - sq / 2 + mw + co, -5, 5, 92 - co, head_z0 + 0.5))  # open rear channel: cables lay in
    for s in (1, -1):
        m.slot_hole("mast", "servo_ear_" + ("A" if s > 0 else "B"), (sbx + s * 24.0, 0, seat), (sbx + s * 27.0, 0, seat),
                    2.4, (0, 0, 1), 12, centre_depth=-6 + co / 2, for_item="servo ear screw (self-tapping)")
        m.hole("mast", "flange_" + ("L" if s > 0 else "R"), (mx, s * 17.65, top + 5 - co), (mx, s * 17.65, top + 10 + co),
               P["clr_m4"], for_item="mast flange M4 thumb screw")
    m.info["mast"] = {"x": mx, "sq": sq, "wall": mw, "z0": top + 5, "z_head": head_z0}

    # UWB anchor plate in front of the mast + pan head [M board size]: through-bolted across the mast
    uwz0, uwz1 = top + 11, seat - 3
    hx1 = sbx + sl / 2 + 10                    # front face of the pan head
    upx = max(mx + sq / 2, hx1) + 0.5
    uwb = m.part("uwb_plate", "printed", COL["light"], [box(upx, upx + 4, -28, 28, uwz0, uwz1)],
                 material="PETG", mates={"mast"}, note="slots fit the UWB anchor board (measure on arrival)")
    for sy_ in (1, -1):
        for hz in (head_z0 + 6, head_z0 + 14):
            m.hole("uwb_plate", "head_%d_%d" % (sy_, int(hz)), (upx - co, sy_ * 5, hz), (upx + 4 + co, sy_ * 5, hz),
                   P["clr_m3"], for_item="UWB plate -> pan head M3")
            m.hole("mast", "uwb_ins_%d_%d" % (sy_, int(hz)), (hx1 + co, sy_ * 5, hz), (hx1 - P["ins_m3_depth"], sy_ * 5, hz),
                   P["ins_m3"], kind="insert", for_item="UWB plate M3")
    for sy_ in (1, -1):
        for zz in (uwz0 + 5, uwz1 - 5):
            m.slot_hole("uwb_plate", "board_%d_%d" % (sy_, int(zz)), (upx + 2, sy_ * 14, zz), (upx + 2, sy_ * 23, zz),
                        P["clr_m3"], (1, 0, 0), 4 + 2 * co, for_item="UWB board M3 [M]", audit=False)

    # pan horn + GoPro buckle (3 prongs, M5 thumb screw)
    horn_z0 = seat + 13 + 5          # servo body top + spline
    disc = (mx, 0, horn_z0)
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
    # 9 g servo adapter (drops into the standard pocket, puts the SG90 shaft on the same pan axis)
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
        ya, yb = (wiy, wiy + P["wheel_w"]) if s > 0 else (-wiy - P["wheel_w"], -wiy)
        m.part("wheel_" + side, H, COL["wheel"], [cyl_y(ya, yb, 0, az, wr)], mass=0.16, moving=True,
               mates={"wheel_pulley_" + side, "axle_" + side}, note="90 x 52 longboard wheel [M mass]")
        m.part("axle_" + side, H, COL["hw_metal"], [cyl_y(s * 36, s * (wiy + P["wheel_w"] + 9), 0, az, 4),
                                                     box(-6.5, 6.5, s * 36.5, s * 41.8, az - 6.5, az + 6.5),
                                                     cyl_y(s * (wiy + P["wheel_w"] + 1.5), s * (wiy + P["wheel_w"] + 9.5), 0, az, 7.3)],
               mass=0.04, mates={"tub", "wheel_" + side, "wheel_pulley_" + side, "spacer_" + side},
               note="M8 x 140 hex bolt, 12.9 (stub axle): head in the boss pocket, washer + nyloc outside the wheel")
        m.part("spacer_" + side, H, COL["hw_metal"], [cyl_y(s * (hw + 4), s * (wiy + 1), 0, az, 6)],
               mass=0.01, mates={"tub", "wheel_" + side, "wheel_pulley_" + side, "axle_" + side},
               note="Al tube 12/8.2 (clears the 608 shield), cut to measured length: boss ring -> inner 608 race")
        my0, my1 = (hw, hw + P["motor_env_l"]) if s > 0 else (-hw - P["motor_env_l"], -hw)
        m.part("motor_" + side, H, COL["hw_metal"], [cyl_y(my0, my1, mc_x, mc_z, P["motor_env_d"] / 2)], mass=0.15,
               moving=True, mates={"motor_plate_" + side, "tub", "pod_" + side, "motor_pulley_" + side},
               note="MAD 5010 EEE V2.0 370KV (O56 x 32.7) inside a O62 x 33 envelope [mass M]")
        # motor pulley: printed 24T HTD-3M, bolted to the bell's prop holes (no long shaft needed)
        mpn = "motor_pulley_" + side
        mpr = P["pul_m_pd"] / 2 - 0.38
        y0p = hw + P["motor_env_l"]
        m.part(mpn, "printed", COL["accent"],
               [cyl_y(s * y0p, s * (y0p + 1.5), mc_x, mc_z, mpr + 2.5),
                cyl_y(s * (y0p + 1.49), s * (y0p + 12.51), mc_x, mc_z, mpr),
                cyl_y(s * (y0p + 12.5), s * (y0p + 14.0), mc_x, mc_z, mpr + 2.5)],
               material="PAHT-CF", moving=True, mates={"motor_" + side},
               note="24T HTD-3M, 4 x M3 low-head screws into the bell's prop holes on D12, centred on the shaft")
        m.hole(mpn, "bore", (mc_x, s * (y0p - co), mc_z), (mc_x, s * (y0p + 10), mc_z), P["motor_shaft_d"] + 0.1,
               for_item="motor shaft stub (centres the pulley; drill to fit)", audit=False)
        for k in range(4):
            ang = math.pi / 4 + k * math.pi / 2
            hx_, hz_ = mc_x + P["motor_prop_pcd"] / 2 * math.cos(ang), mc_z + P["motor_prop_pcd"] / 2 * math.sin(ang)
            m.hole(mpn, "prop_%d" % k, (hx_, s * (y0p - co), hz_), (hx_, s * (y0p + 14 + co), hz_), P["clr_m3"],
                   for_item="M3 x 12 low head into the bell")
            m.get(mpn)["cuts"].append(cyl_y(s * (y0p + 10), s * (y0p + 14 + co), hx_, hz_, 2.9))   # head counterbore
        for k in range(P["pul_m_t"]):
            ang = 2 * math.pi * k / P["pul_m_t"]
            rc = mpr - 1.14 + 0.9
            m.get(mpn)["cuts"].append(cyl_y(s * (y0p + 1.5), s * (y0p + 12.5), mc_x + rc * math.cos(ang),
                                            mc_z + rc * math.sin(ang), 0.9))
        # belt envelope (two pulleys + straight runs)
        bc = (mc_x / 2, s * by, (az + mc_z) / 2)
        m.part("belt_" + side, H, COL["belt"], [obox(bc, P["belt_cc"], 2 * (wpr + 1.5), P["belt_w"], bu, bv),
                                               cyl_y(s * (by - 4.5) if s > 0 else -(by + 4.5), (by + 4.5) if s > 0 else -(by - 4.5), 0, az, wpr + 1.5)],
               mass=0.01, moving=True, mates={"wheel_pulley_" + side, "motor_pulley_" + side, "pod_" + side, "motor_" + side, "axle_" + side, "spacer_" + side},
               note="HTD-3M %d mm (%dT) x 9 mm closed belt" % (P["belt_len"], P["belt_len"] / 3))
    m.part("battery", H, COL["battery"], [box(P["bat_x0"], P["bat_x0"] + P["bat_l"], -P["bat_w"] / 2, P["bat_w"] / 2, zf1, zf1 + P["bat_h"])],
           mass=0.40, mates={"tub", "bumper_door"}, note="3S 5000 mAh hard case <= 145 x 50 x 30 [M]")
    m.part("pi5", H, COL["pcb_green"], [box(pi_x0, pi_x1, -28, 28, dz1 + 6, dz1 + 7.6),
                                        box(pi_x0 + 20, pi_x0 + 70, -24, 24, dz1 + 7.6, dz1 + 22)], mass=0.07,
           mates={"deck"}, note="Raspberry Pi 5 + Active Cooler on 6 mm M2.5 standoffs")
    for i, (hx, hy) in enumerate(pi_holes):
        m.part("pi_standoff_%d" % i, H, COL["hw_metal"], [cyl_z(dz1, dz1 + 6, hx, hy, 2.5)], mass=0.002,
               mates={"deck", "pi5"})
    m.part("usb_keepout", "keepout", COL["keepout"], [box(pi_x0 - 25, pi_x0, -28, 28, dz1 + 2, dz1 + 24)],
           note="right-angle USB-A plugs (GoPro, ESP32) need 25 mm", mates={"pi5"})
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        y0_, y1_ = (26.5, 51.9) if s > 0 else (-51.9, -26.5)
        m.part("reg_" + side, H, COL["pcb_blue"], [box(8.3, 33.7, y0_, y1_, dz1 + 0.01, dz1 + 9.5)], mass=0.007,
               mates={"deck"}, note=("Pololu D36V50F5 5 V 5.5 A (Pi)" if s > 0 else "Pololu D36V50F6 6 V (servo)") + " 25.4 sq [V]")
    # tail electronics (all [M] envelopes, on the 10 mm grid)
    tz = tdz + 3
    m.part("esp32_zone", "keepout", (120, 120, 255), [box(tx0 + 12, px + 55, -30, 30, tz + 10, tz + 22)],
           note="ESP32 DevKit + SunFounder extension board on 10 mm standoffs (clears the pivot nut) [M]",
           mates={"tail"})
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        y0_, y1_ = (33, 61) if s > 0 else (-61, -33)
        m.part("esc_" + side, H, COL["hw_dark"], [box(tx0 + 14, tx0 + 62, y0_, y1_, tz + 0.01, tz + 14)], mass=0.03,
               mates={"tail"}, note="AM32 ESC envelope 48 x 28 x 14 [M], zip-tied to the grid")
    m.part("fuse_zone", "keepout", (255, 160, 0), [box(px + 30, x0 - 14, 33, 61, tz + 0.01, tz + 16)],
           note="20 A blade fuse holder + Wago/XT30 distribution [M]", mates={"tail"})
    m.part("xt60_plug", H, COL["battery"], [box(tx0 - 14, tx0 + 4, -38, -22, 121, 129)], mass=0.01, mates={"tail"},
           note="XT60 arming plug: pull it = robot dead (main switch)")
    m.part("caster_wheel", H, COL["wheel"], [cyl_y(-cw / 2, cw / 2, cxw, cr, cr)], mass=0.10, moving=True,
           mates={"caster_fork", "caster_axle"}, note="70 mm wheel with 608 bearings [M]")
    m.part("caster_axle", H, COL["hw_metal"], [cyl_y(-cw / 2 - 12, cw / 2 + 12, cxw, cr, 4)], mass=0.02,
           mates={"caster_fork", "caster_wheel"})
    m.part("caster_pivot_bolt", H, COL["hw_metal"], [cyl_z(cr * 2 + 3, tdz + 12, px, 0, 4)], mass=0.03,
           mates={"tail", "caster_fork", "esp32_zone"})
    # HC-SR04 on the bracket (PCB behind the plate, transducers through it)
    uc = vadd(plate_c, vmul(n, -6.0))
    us_adds = [obox(uc, 1.6, 45.0, 20.0, n, Y)]
    for s in (1, -1):
        a_ = vadd(uc, vmul(Y, s * 13)); us_adds.append(cyl(a_, vadd(a_, vmul(n, 12.5)), 8.0))
    m.part("hcsr04", H, COL["pcb_blue"], us_adds, mass=0.009, mates={"us_bracket"}, note="HC-SR04 [M] hole pattern")
    # servo + GoPro
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
           mass=0.03, mates={"uwb_plate"}, note="UWB AoA anchor envelope 50 x %.0f [M]" % (uwz1 - uwz0 - 4))
    m.part("caster_crown_sweep", "keepout", COL["keepout"], [cyl_z(cr * 2 + 5.9, 83.5, px, 0, 52.0)],
           note="fork crown swept disc; its top runs on the thrust washer under the pivot boss",
           mates={"caster_fork", "caster_pivot_bolt", "tail"})
    m.part("caster_sweep", "keepout", COL["keepout"], [cyl_z(0.5, cr * 2 + 5.9, px, 0, m.info["caster_sweep_r"] + 1.0)],
           note="caster swivel envelope (360 deg)", mates={"caster_fork", "caster_wheel", "caster_axle", "caster_pivot_bolt"})
    m.info["P"] = P

    # ================= WIRING ROUTES (centre lines, checked against solids) =================
    # intended interpenetration of simplified hardware (solid wheels with bolts through them, belt envelopes ...)
    for sd in ("L", "R"):
        m.nests += [("belt_" + sd, "wheel_pulley_" + sd, "belt envelope wraps the pulley"),
                    ("belt_" + sd, "motor_pulley_" + sd, "belt envelope wraps the pulley"),
                    ("belt_" + sd, "axle_" + sd, "envelope drawn solid"), ("belt_" + sd, "spacer_" + sd, "envelope drawn solid"),
                    ("axle_" + sd, "wheel_" + sd, "bolt through the wheel bearings"),
                    ("axle_" + sd, "spacer_" + sd, "bolt through the spacer"),
                    ("spacer_" + sd, "wheel_" + sd, "spacer reaches the recessed inner race")]
    m.nests += [("gopro", "pan_buckle", "camera fingers interleave the buckle prongs"),
                ("servo", "servo9g_adapter", "alternatives: only one is fitted"),
                ("caster_wheel", "caster_axle", "axle through the bearings")]
    mast_grp = ["mast", "servo", "pan_buckle", "gopro", "uwb_plate", "uwb_anchor", "servo9g_adapter"]
    deck_grp = ["deck", "pi5", "reg_L", "reg_R"] + ["pi_standoff_%d" % i for i in range(4)]
    bolts = ["plate_bolt_%s%d" % (sd, k) for sd in "LR" for k in range(3)]
    m.removals = [
        (mast_grp, (0, 0, 1), 170, []),
        (["lid"], (0, 0, 1), 40, mast_grp),
        (["tail_lid"], (0, 0, 1), 40, []),
        (deck_grp, (0, 0, 1), 90, mast_grp + ["lid"]),
        (["bumper_door"], (1, 0, 0), 70, []),
        (["battery"], (1, 0, 0), 180, ["bumper_door"]),
        (["pod_L"], (0, 1, 0), 70, []), (["pod_R"], (0, -1, 0), 70, []),
        (["wheel_L", "wheel_pulley_L"], (0, 1, 0), 90, ["axle_L", "spacer_L", "belt_L"]),
        (["wheel_R", "wheel_pulley_R"], (0, -1, 0), 90, ["axle_R", "spacer_R", "belt_R"]),
        (["motor_L", "motor_plate_L", "motor_pulley_L"], (0, 1, 0), 120,
         ["wheel_L", "wheel_pulley_L", "belt_L", "pod_L", "axle_L", "spacer_L"] + bolts),
        (["motor_R", "motor_plate_R", "motor_pulley_R"], (0, -1, 0), 120,
         ["wheel_R", "wheel_pulley_R", "belt_R", "pod_R", "axle_R", "spacer_R"] + bolts),
    ]
    sx = mx - 20   # cable drop point behind the socket
    ch = (mx - sq / 2 + mw / 2, 0.0)     # mast rear cable channel (x, y)
    cz = head_z0 - 7                      # level where cables turn into the channel, under the pan head
    pz = dz1 + 12                         # Pi USB plug height
    m.routes = [
        ("battery main 12 AWG", [(48, -20, 64), (42, -45, 72), (xr_in + 30, -45, 72), (xr_in + 19, -38, 72),
                                 (xr_in + 19, -38, 118), (x0 - 10, -36, 118), (px + 40, -47, 120), (px + 40, -47, tz + 8)]),
        ("ESC L phase wires", [(-66, inner - 12, 72), (-80, 45, 72), (xr_in + 19, 38, 72), (xr_in + 19, 38, 118),
                               (x0 - 10, 30, 118), (tx0 + 60, 47, tz + 8)]),
        ("ESC R phase wires", [(-66, -(inner - 12), 72), (-80, -45, 75), (xr_in + 30, -45, 75), (xr_in + 19, -34, 75),
                               (xr_in + 19, -34, 121), (x0 - 10, -26, 121), (tx0 + 60, -47, tz + 8)]),
        ("regulator inputs 18 AWG (fuse -> regs)", [(px + 60, 47, tz + 8), (x0 - 10, 20, 122), (xr_in + 30, 20, 122),
                                                     (20, 24, 100), (20, 24, dz1 + 11)]),
        ("GoPro USB-C (service loop around the pan)", [(mx - 6, 38, cam_bot + 20), (mx - 30, 38, cam_bot + 10), (mx - 30, 30, cz),
                                                      (ch[0], 0, cz), (ch[0], 0, 120), (sx, 0, 120),
                                                      (pi_x0 - 12, -10, 120), (pi_x0 - 12, -10, pz)]),
        ("servo signal + 6 V (pan)", [(sbx - sl / 2 - 3, 8, head_z0 + 10), (sbx - sl / 2 - 3, 8, cz), (ch[0], 2, cz),
                                      (ch[0], 2, 124), (x0 - 10, 0, 124), (px - 30, 0, tz + 16)]),
        ("servo 6 V from reg R", [(20, -39, dz1 + 11), (20, -24, 110), (ch[0] - 10, -3, 110), (ch[0], -3, 115)]),
        ("UWB anchor USB -> Pi", [(upx + 8, 22, uwz0 + 6), (upx + 8, 33, uwz0 + 6), (upx - 3, 33, uwz0 + 6), (mx, 26, cz),
                                  (mx - 16, 26, cz), (mx - 16, 0, cz), (ch[0], -2, cz),
                                  (ch[0], -2, 118), (sx, -3, 118), (pi_x0 - 12, 0, 118), (pi_x0 - 12, 0, pz)]),
        ("ESP32 USB", [(px - 30, 0, tz + 16), (x0 - 10, 10, 120), (pi_x0 - 12, 10, 120), (pi_x0 - 12, 10, pz)]),
        ("Pi 5 V from reg L", [(20, 39, dz1 + 11), (20, 30, 116), (pi_x0 + 40, 26, 116), (pi_x0 + 40, 26, dz1 + 12)]),
        ("HC-SR04", [(x1 + 8, -20, 100), (x1 + 4, -27, 100), (xf_in - 8, -28, 100), (xf_in - 20, -29, 128),
                     (x0 - 10, -20, 124), (px - 30, -10, tz + 16)]),
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

        def prim(p):
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
                if body is None:
                    body = b
                else:
                    tbm.booleanOperation(body, b, adsk.fusion.BooleanTypes.UnionBooleanType)
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
        for gname, label in (("printed", "Printed parts"), ("hardware", "Reference hardware"), ("keepout", "Keep-out volumes")):
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occ.component.name = label
            groups[gname] = occ.component
        for part in model.parts:
            try:
                parent = groups[part["group"]]
                occ = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                comp = occ.component
                comp.name = part["name"]
                body = merge(part["adds"])
                if part["cuts"]:
                    tool = merge(part["cuts"])
                    tbm.booleanOperation(body, tool, adsk.fusion.BooleanTypes.DifferenceBooleanType)
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
        msg += ("\n\nKeep-out volumes are hidden (turn on 'Keep-out volumes' to see swept envelopes)."
                "\nTo print: right-click a component under 'Printed parts' > Save As Mesh (3MF/STL).")
        ui.messageBox(msg, "honebot chassis")
    except Exception:
        if ui:
            ui.messageBox("honebot script failed:\n%s" % traceback.format_exc(), "honebot chassis")
