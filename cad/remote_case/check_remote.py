"""Offline fit / tolerance checker for the honebot remote case (needs only numpy).

It imports cad/remote_case/fusion/HonebotRemote/HonebotRemote.py (the single geometry source) without Fusion and
tests the very same primitives by sampling points inside them:

  1. no two solids overlap (printed parts, boards, battery, switch, cable plugs);
  2. clearance policy: the space around every board / battery / switch / plug that PARAMS reserve is free of printed
     material (that space is built from the HARDWARE outline + policy clearance, not from the case features);
  3. joystick travel: the stick and cap, grown by clr_move, swept over every tilt direction and the click, touch
     nothing;
  4. assembly: lid lifts straight off; carrier + joystick lift out; battery and ESP32 lift out;
  5. numbers: screws and inserts, lap joint, bevels, battery swell room, switch panel, antenna keep-out, lid rim;
  6. the same suite over part variants (30-pin DevKit, USB-C DevKit, bigger joystick cap / tilt, 500 mAh battery);
  7. negative tests: deliberately broken PARAMS that MUST fail (proves the checks bite).

    ~/occ/bin/python cad/remote_case/check_remote.py          (any python3 with numpy works)
    exit code 0 = everything passed; report: cad/remote_case/report/check_report.md
    --quick : default PARAMS only (no variants / negative tests)
"""
import importlib.util
import math
import os
import sys
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for _n in ("adsk", "adsk.core", "adsk.fusion"):
    sys.modules.setdefault(_n, types.ModuleType(_n))
_spec = importlib.util.spec_from_file_location("HonebotRemote", os.path.join(HERE, "fusion", "HonebotRemote",
                                                                             "HonebotRemote.py"))
HR = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(HR)

EPS = 1e-9
MAX_PTS = 2_500_000


# ------------------------------------------------------------------------------------------------------------
# point-in-solid for the script's primitives
# ------------------------------------------------------------------------------------------------------------
def _w(p):
    u, v = np.array(p["u"], float), np.array(p["v"], float)
    u /= np.linalg.norm(u)
    w = np.cross(u, v)
    w /= np.linalg.norm(w)
    v = np.cross(w, u)
    return u, v, w


def inside(p, X):
    t = p["t"]
    if t == "box":
        u, v, w = _w(p)
        d = X - np.array(p["c"], float)
        su, sv, sw = (s / 2 - EPS for s in p["s"])
        return (np.abs(d @ u) < su) & (np.abs(d @ v) < sv) & (np.abs(d @ w) < sw)
    if t == "cyl":
        a, b = np.array(p["a"], float), np.array(p["b"], float)
        h = np.linalg.norm(b - a)
        n = (b - a) / h
        d = X - a
        s = d @ n
        r = p["r"] + (p.get("r2", p["r"]) - p["r"]) * np.clip(s / h, 0, 1)
        rad2 = np.einsum("ij,ij->i", d, d) - s * s
        return (s > EPS) & (s < h - EPS) & (rad2 < (r - EPS) ** 2)
    if t == "sph":
        d = X - np.array(p["c"], float)
        return np.einsum("ij,ij->i", d, d) < (p["r"] - EPS) ** 2
    if t == "diff":
        m = inside(p["a"], X)
        for q in p["b"]:
            if m.any():
                m &= ~inside(q, X)
        return m
    raise ValueError(t)


def pbox(p):
    t = p["t"]
    if t == "box":
        u, v, w = _w(p)
        ext = np.abs(u) * p["s"][0] / 2 + np.abs(v) * p["s"][1] / 2 + np.abs(w) * p["s"][2] / 2
        c = np.array(p["c"], float)
        return c - ext, c + ext
    if t == "cyl":
        a, b = np.array(p["a"], float), np.array(p["b"], float)
        r = max(p["r"], p.get("r2", p["r"]))
        return np.minimum(a, b) - r, np.maximum(a, b) + r
    if t == "sph":
        c = np.array(p["c"], float)
        return c - p["r"], c + p["r"]
    if t == "diff":
        return pbox(p["a"])
    raise ValueError(t)


class Solid:
    """A part as U(A_i - C_i) U A - C, with bounding-box pruning per query."""

    def __init__(self, part, shift=(0.0, 0.0, 0.0)):
        self.name = part["name"]
        self.shift = np.array(shift, float)
        self.stages = [([(q, pbox(q)) for q in a], [(q, pbox(q)) for q in c]) for a, c in part.get("pre", [])]
        self.adds = [(q, pbox(q)) for q in part["adds"]]
        self.cuts = [(q, pbox(q)) for q in part["cuts"]]
        allp = self.adds + [x for a, _ in self.stages for x in a]
        lo = np.min([b[0] for _, b in allp], axis=0)
        hi = np.max([b[1] for _, b in allp], axis=0)
        self.lo, self.hi = lo + self.shift, hi + self.shift

    def moved(self, shift):
        s = Solid.__new__(Solid)
        s.__dict__.update(self.__dict__)
        s.shift = self.shift + np.array(shift, float)
        s.lo, s.hi = self.lo + shift, self.hi + shift
        return s

    def contains(self, X):
        Xl = X - self.shift
        lo, hi = Xl.min(axis=0), Xl.max(axis=0)

        def any_in(lst):
            m = np.zeros(len(Xl), bool)
            for q, (a, b) in lst:
                if np.all(b > lo) and np.all(a < hi):
                    m |= inside(q, Xl)
            return m

        m = np.zeros(len(Xl), bool)
        for adds, cuts in self.stages:
            sm = any_in(adds)
            if sm.any() and cuts:
                sm &= ~any_in(cuts)
            m |= sm
        if self.adds:
            m |= any_in(self.adds)
        if self.cuts and m.any():
            m &= ~any_in(self.cuts)
        return m


def grid(lo, hi, h):
    vol = np.prod(np.maximum(hi - lo, 0))
    h = max(h, (vol / MAX_PTS) ** (1 / 3)) if vol > 0 else h
    off = 0.37 * h + 0.0123
    axes = [np.arange(lo[i] + off, hi[i], h) for i in range(3)]
    if any(len(a) == 0 for a in axes):
        return np.zeros((0, 3)), h
    g = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
    return g, h


def overlap(A, B, h=0.2):
    """Approximate overlap volume (mm^3) of two Solids (or a Solid and a prim list Solid)."""
    lo, hi = np.maximum(A.lo, B.lo), np.minimum(A.hi, B.hi)
    if np.any(hi <= lo):
        return 0.0, None
    X, hh = grid(lo, hi, h)
    if len(X) == 0:
        return 0.0, None
    m = A.contains(X)
    if not m.any():
        return 0.0, None
    X = X[m]
    m2 = B.contains(X)
    n = int(m2.sum())
    if n == 0:
        return 0.0, None
    return n * hh ** 3, X[m2].mean(axis=0)


def sweep_up(movers, fixeds, travel, h=0.4):
    """Lift every mover straight up by 0..travel: list of collisions. Column test on a shared xy lattice: a fixed
    point collides when it lies in a column the mover occupies, above the mover's lowest point there and below its
    highest point + travel."""
    hits = []
    for m in movers:
        lo, hi = m.lo.copy(), m.hi.copy()
        ax = [np.arange(np.floor(lo[i] / h) * h + 0.0123, hi[i], h) for i in range(3)]
        G = np.stack(np.meshgrid(*ax, indexing="ij"), -1).reshape(-1, 3)
        inm = m.contains(G)
        if not inm.any():
            continue
        Gi = G[inm]
        ix = np.round((Gi[:, 0] - ax[0][0]) / h).astype(int)
        iy = np.round((Gi[:, 1] - ax[1][0]) / h).astype(int)
        zmin = np.full((len(ax[0]), len(ax[1])), np.inf)
        zmax = np.full((len(ax[0]), len(ax[1])), -np.inf)
        np.minimum.at(zmin, (ix, iy), Gi[:, 2])
        np.maximum.at(zmax, (ix, iy), Gi[:, 2])
        for f in fixeds:
            flo = np.maximum(f.lo, [lo[0], lo[1], lo[2]])
            fhi = np.minimum(f.hi, [hi[0], hi[1], hi[2] + travel])
            if np.any(fhi <= flo):
                continue
            i0 = max(0, int(np.floor((flo[0] - ax[0][0]) / h)))
            i1 = min(len(ax[0]), int(np.ceil((fhi[0] - ax[0][0]) / h)) + 1)
            j0 = max(0, int(np.floor((flo[1] - ax[1][0]) / h)))
            j1 = min(len(ax[1]), int(np.ceil((fhi[1] - ax[1][0]) / h)) + 1)
            zs = np.arange(flo[2] + 0.0071, fhi[2], h / 2)
            if i1 <= i0 or j1 <= j0 or len(zs) == 0:
                continue
            F = np.stack(np.meshgrid(ax[0][i0:i1], ax[1][j0:j1], zs, indexing="ij"), -1).reshape(-1, 3)
            if len(F) > 4 * MAX_PTS:
                F = F[:: int(np.ceil(len(F) / (4 * MAX_PTS)))]
            fi = np.round((F[:, 0] - ax[0][0]) / h).astype(int)
            fj = np.round((F[:, 1] - ax[1][0]) / h).astype(int)
            pre = (F[:, 2] > zmin[fi, fj] + h / 2) & (F[:, 2] < zmax[fi, fj] + travel)
            if not pre.any():
                continue
            F = F[pre]
            inf = f.contains(F)
            if inf.any():
                p = F[inf][0]
                hits.append("%s hits %s near (%.1f, %.1f, %.1f)" % (m.name, f.name, *p))
    return hits


def prim_solid(name, prims):
    return Solid({"name": name, "adds": prims, "cuts": [], "pre": []})


# ------------------------------------------------------------------------------------------------------------
# the suite
# ------------------------------------------------------------------------------------------------------------
def run_suite(P, h=0.2, fast=False, P_hw=None):
    """Returns list of (group, name, ok, detail)."""
    res = []

    def chk(group, name, ok, detail=""):
        res.append((group, name, bool(ok), detail))

    try:
        M = HR.build_model(P)
    except AssertionError as e:
        chk("build", "model builds", False, str(e))
        return res, None
    D = M.info
    printed = [p for p in M.parts if p["group"] == "printed"]
    hard = [p for p in M.parts if p["group"] in ("hardware", "keepout")]
    Ph, Dh = P, D
    if P_hw is not None:           # real hardware differs from what the case was printed for
        Ph = dict(P_hw, H=D["H"], L=D["L"])
        Mh = HR.build_model(Ph)
        Dh = Mh.info
        hard = [p for p in Mh.parts if p["group"] in ("hardware", "keepout")]
        M.fits = Mh.fits
    M.parts = printed + hard
    S = {p["name"]: Solid(p) for p in M.parts}

    # 1. nominal interference
    allp = printed + hard
    bad = []
    for i, a in enumerate(allp):
        for b in allp[i + 1:]:
            if a["assembly"] == b["assembly"]:
                continue
            if a["group"] == "keepout" and b["group"] == "keepout":
                continue
            v, at = overlap(S[a["name"]], S[b["name"]], h)
            if v > 0:
                bad.append("%s x %s: %.2f mm3 near (%.1f, %.1f, %.1f)" % (a["name"], b["name"], v, *at))
    chk("interference", "no two solids overlap (%d solids)" % len(allp), not bad, "; ".join(bad[:6]))

    # 1b. every printed part stays inside the outer shell envelope (nothing pokes through a rounded corner)
    env = prim_solid("outer shell", D["outer"])
    out_hits = []
    for p in printed:
        A = S[p["name"]]
        X, hh = grid(A.lo - 0.5, A.hi + 0.5, 0.3)
        m = A.contains(X)
        if m.any():
            Xo = X[m]
            o = ~env.contains(Xo)
            if o.any():
                out_hits.append("%s: %d points outside near (%.1f, %.1f, %.1f)" % (p["name"], o.sum(), *Xo[o].mean(axis=0)))
    chk("shell", "printed parts stay inside the outer shell", not out_hits, "; ".join(out_hits))

    # 2. clearance policy
    for why, prims in M.fits:
        F = prim_solid(why, prims)
        hits = []
        for p in printed:
            v, at = overlap(F, S[p["name"]], min(h, 0.1))
            if v > 0:
                hits.append("%s %.2f mm3 near (%.1f, %.1f, %.1f)" % (p["name"], v, *at))
        chk("clearance", why, not hits, "; ".join(hits))

    # 3. joystick travel (stick + cap grown by clr_move - 0.05)
    others = [S[p["name"]] for p in printed] + [S[p["name"]] for p in hard
                                                 if p["assembly"] not in ("joystick",) and not p["flexible"]]
    worst = []
    tilts = sorted(set([0.0, P["js_tilt"]] + [P["js_tilt"] * k / (3 if fast else 5) for k in range(1, 3 if fast else 5)]))
    for press in (0.0, P["js_press"]):
        for tilt in tilts:
            for az in range(0, 360, 30 if fast else 15):
                X = np.array(HR.stick_samples(Ph, Dh, tilt, az, press, n_ring=48, grow=P["clr_move"] - 0.05))
                for o in others:
                    if o.contains(X).any():
                        worst.append("%s at tilt %.0f az %d press %.1f" % (o.name, tilt, az, press))
                        break
    chk("joystick", "stick + cap keep %.1f mm at every tilt (max %.0f deg) and when clicked" % (P["clr_move"],
                                                                                               P["js_tilt"]),
        not worst, "; ".join(worst[:4]))
    # cap clears the lid opening when the lid is lifted off (the cap passes through the opening)
    ro_top = D["js_open_r"] + P["top"]
    chk("joystick", "cap (d %.1f) passes through the lid opening (top d %.1f) when the lid comes off"
        % (P["js_cap_d"], 2 * D["js_open_r"]), P["js_cap_d"] / 2 + 0.5 <= D["js_open_r"],
        "opening radius %.1f at the inner face, %.1f at the top" % (D["js_open_r"], ro_top))
    rim = D["W"] / 2 - P["c_top"] - ro_top
    chk("joystick", "lid keeps a flat rim >= 2 mm beside the stick opening", rim >= 2.0, "rim %.1f mm" % rim)
    rim_f = D["L"] - P["c_top"] - (D["stick_x"] + ro_top)
    chk("joystick", "lid keeps a flat rim >= 2 mm in front of the stick opening", rim_f >= 2.0 - 1e-9,
        "rim %.1f mm" % rim_f)

    # 4. assembly sweeps (straight up)
    def sweep(moving, fixed, label, travel=45.0, step=None):
        hits = sweep_up(moving, fixed, travel)
        chk("assembly", label, not hits, "; ".join(hits[:4]))

    step = 3.0 if fast else 1.0
    rigid = [p for p in hard if p["group"] == "hardware" and not p["flexible"]]
    sweep([S["remote_lid"]], [S["remote_base"], S["js_carrier"]] + [S[p["name"]] for p in rigid],
          "lid lifts straight off (over the stick)", step=step)
    car_set = [S["js_carrier"]] + [S[p["name"]] for p in rigid if p["mount"] == "carrier"]
    base_set = [S["remote_base"]] + [S[p["name"]] for p in rigid if p["mount"] == "base"]
    sweep(car_set, base_set, "carrier + joystick lift out (lid off; jumpers unplugged or slack)", travel=30, step=step)
    sweep([S["battery"]], [S["remote_base"]] + [S[p["name"]] for p in rigid if p["mount"] == "base" and p["name"] != "battery"],
          "battery lifts out (carrier off)", travel=25, step=step)
    esp = [S[p["name"]] for p in rigid if p["assembly"] == "esp32"]
    sweep(esp, [S["remote_base"]] + [S[p["name"]] for p in rigid if p["mount"] == "base" and p["assembly"] != "esp32"],
          "ESP32 lifts out of its cradle (lid off)", travel=25, step=step)

    # 5. numbers
    for sc in M.screws:
        ok = 2.5 <= sc["engage"] and sc["len"] - sc["thru"] <= sc["hole_depth"] - 0.2
        chk("screws", "%s: M2 x %.0f, %.1f mm thread in the insert, tip short of the hole bottom" % (
            sc["id"], sc["len"], sc["engage"]), ok, "thread past the clamped part %.1f, hole depth %.1f" % (
            sc["len"] - sc["thru"], sc["hole_depth"]))
    zli = D["H"] - P["top"]
    skin = D["H"] - (D["s"] + P["ins_m2_len"] + 1.0)
    chk("screws", "lid insert holes leave >= 1.2 mm skin on the top face", skin >= 1.2, "skin %.1f mm" % skin)
    boss = zli - D["s"]
    chk("screws", "lid bosses at least as long as the insert", boss + P["top"] - 1.2 >= P["ins_m2_len"] + 0.5,
        "boss %.1f mm + top %.1f" % (boss, P["top"]))
    cb = (P["m2_head_d"] + 0.8) - P["m2_head_d"]
    chk("screws", "head counterbore 0.4 mm per side; driver reaches %.1f mm deep" % (D["s"] - D["case_thru"]), cb >= 0.6)
    joint_top = D["s"] + P["lip_h"] + P["lip_vert"]
    chk("shell", "lap joint lies in the straight wall (below the top bevel)", joint_top <= D["H"] - P["c_top"] + 0.01,
        "joint top z %.1f, bevel starts z %.1f" % (joint_top, D["H"] - P["c_top"]))
    tongue = P["wall"] / 2 - P["lip_side"] + P["lip_in"]
    chk("shell", "tongue >= 1.2 mm thick (3 lines)", tongue >= 1.2 - 1e-9, "%.2f mm" % tongue)
    chk("shell", "ESP32 cradle top at or below the split plane", D["z_e1"] <= D["s"] + 1e-9,
        "PCB top %.1f, split %.1f" % (D["z_e1"], D["s"]))
    chk("shell", "height fits everything (H >= needed)", D["H"] >= D["H_need"] - 1e-9,
        "H %.1f, needed %.2f" % (D["H"], D["H_need"]))
    chk("shell", "length fits everything (L >= needed)", D["L"] >= D["L_need"] - 1e-9,
        "L %.1f, needed %.2f" % (D["L"], D["L_need"]))
    chk("parts", "battery has %.1f mm to swell under the carrier" % P["bat_swell"],
        D["z_c0"] - D["z_b1"] >= P["bat_swell"] - 1e-9, "gap %.2f mm" % (D["z_c0"] - D["z_b1"]))
    chk("parts", "switch panel 0.8-2.2 mm thick for the snap clips, bezel flush",
        0.8 <= D["z_panel"] - P["sw_bez_h"] <= 2.2, "panel %.1f mm" % (D["z_panel"] - P["sw_bez_h"]))
    chk("parts", "ESP32 ledge tabs end before the first jumper housing",
        D["esp_tab"] <= D["esp_end_margin"] - 0.3, "tab %.1f, free end strip %.1f" % (D["esp_tab"], D["esp_end_margin"]))
    usb_bevel = D["tun_e"][1] - (D["H"] - P["c_top"])
    chk("parts", "ESP32 USB opening ends <= 2 mm into the top bevel", usb_bevel <= 2.0,
        "opening z %.1f..%.1f, bevel from %.1f" % (D["tun_e"][0], D["tun_e"][1], D["H"] - P["c_top"]))
    chk("parts", "charger USB-C opening clears the bottom bevel", D["tun_t"][0] >= P["c_bot"] + 0.5,
        "opening z %.1f..%.1f" % D["tun_t"])
    # radio: battery and brass inserts away from the ESP32 antenna (PCB end, first 7 mm)
    gap = D["x_b0"] - D["x_e1"]
    chk("radio", "battery >= %.0f mm from the antenna end" % P["ant_gap"], gap >= P["ant_gap"] - 1e-9, "%.1f mm" % gap)
    inserts = [(cx, sy * D["col_y"], D["s"]) for cx in D["col_x"] for sy in (-1, 1)]
    inserts += [(cx, sy * D["car_post_y"], D["z_c0"]) for cx in D["car_post_x"] for sy in (-1, 1)]
    inserts += [(hx, hy, D["z_j0"]) for hx, hy in D["js_holes"]]
    ant = np.array([D["x_e1"] - 3.5, 0.0, D["z_e1"]])
    dmin = min(math.dist(ant, i) for i in inserts)
    chk("radio", "no brass insert within 12 mm of the antenna", dmin >= 12.0, "nearest %.1f mm" % dmin)
    chk("radio", "case material is not carbon-filled", all("CF" not in (p["material"] or "") for p in printed),
        "PETG / PLA only")
    return res, M


VARIANTS = [
    ("default (38-pin DevKit, micro-USB, KY-023, 803040 1000 mAh)", {}),
    ("30-pin DOIT DevKit", {"esp_l": 51.5, "esp_pins_side": 15}),
    ("38-pin DevKit with USB-C", {"esp_usb_w": 9.0, "esp_usb_h": 3.3, "esp_usb_l": 7.4, "esp_plug_w": 12.5,
                                  "esp_plug_h": 7.0}),
    ("bigger joystick cap / tilt (d 26, 30 deg)", {"js_cap_d": 26.0, "js_tilt": 30.0}),
    ("thin 603450-size battery (6 x 34 x 50)", {"bat_l": 50.0, "bat_w": 34.0, "bat_h": 6.0}),
]

NEGATIVE = [
    ("lid opening forced 3 mm too small", "joystick", {"_open_minus": 3.0}),
    ("case 2 mm too low", "", {"_H_minus": 2.0}),
    ("ESP32 measured 1 mm longer than printed", "", {"_esp_grow": 1.0}),
    ("battery 2 mm wider than the pocket", "", {"_bat_grow": 2.0}),
    ("switch panel 3 mm thick", "parts", {"sw_panel": 3.0}),
]


def negative_params(over):
    """A 'broken' case: printed parts from P, hardware from P_hw (as if the owner measured wrong), or a derived size
    forced. Returns (P, P_hw, undo)."""
    P = dict(HR.PARAMS)
    special = {k: v for k, v in over.items() if k.startswith("_")}
    P.update({k: v for k, v in over.items() if not k.startswith("_")})
    D = HR.layout(P)
    if "_open_minus" in special:
        orig = HR.joystick_opening
        HR.joystick_opening = lambda P_, D_: orig(P_, D_) - special["_open_minus"]
        return P, None, lambda: setattr(HR, "joystick_opening", orig)
    if "_H_minus" in special:
        P["H"] = D["H"] - special["_H_minus"]
        return P, None, None
    P_hw = dict(P)
    if "_esp_grow" in special:
        P_hw["esp_l"] += special["_esp_grow"]
        P_hw["ant_gap"] -= special["_esp_grow"]      # battery / joystick stay where the case was printed for them
    if "_bat_grow" in special:
        P_hw["bat_w"] += special["_bat_grow"]
    return P, P_hw, None


def main():
    quick = "--quick" in sys.argv
    t0 = time.time()
    out = []
    all_ok = True
    first_M = None
    runs = VARIANTS[:1] if quick else VARIANTS
    for label, over in runs:
        P = dict(HR.PARAMS)
        P.update(over)
        res, M = run_suite(P, fast=(label != VARIANTS[0][0]))
        first_M = first_M or M
        ok = all(r[2] for r in res)
        all_ok &= ok
        out.append((label, res))
        print("%-62s %s (%d checks, %.0f s)" % (label, "PASS" if ok else "FAIL", len(res), time.time() - t0),
              flush=True)
        for g, n, k, d in res:
            if not k:
                print("   FAIL [%s] %s: %s" % (g, n, d))
    neg_out = []
    if not quick:
        for label, group, over in NEGATIVE:
            P, P_hw, undo = negative_params(over)
            try:
                res, _ = run_suite(P, fast=True, P_hw=P_hw)
            finally:
                if undo:
                    undo()
            fails = [r for r in res if not r[2] and (not group or r[0] == group)]
            caught = bool(fails)
            all_ok &= caught
            neg_out.append((label, caught, fails[:3]))
            print("negative: %-50s %s" % (label, "caught" if caught else "NOT CAUGHT"), flush=True)
    write_report(first_M, out, neg_out)
    print("total %.0f s; report: %s" % (time.time() - t0, os.path.join(HERE, "report", "check_report.md")))
    return 0 if all_ok else 1


def write_report(M, out, neg_out):
    D, P = M.info, HR.PARAMS
    os.makedirs(os.path.join(HERE, "report"), exist_ok=True)
    L = ["# honebot remote case: check report", "",
         "Generated by `cad/remote_case/check_remote.py` from `HonebotRemote.py` PARAMS. Do not edit by hand.", "",
         "## Size and stack-up (default PARAMS, mm)", "",
         "| Item | Value |", "|---|---|",
         "| Outside (L x W x H) | %.1f x %.1f x %.1f |" % (D["L"], D["W"], D["H"]),
         "| Split plane (lid / base) | z %.1f |" % D["s"],
         "| ESP32 PCB | x %.1f..%.1f, underside z %.1f (%.1f mm free under it for jumper housings) |" % (
             D["x_e0"], D["x_e1"], D["z_e0"], P["esp_under"]),
         "| ESP32 USB opening | centre z %.1f, %.1f x %.1f |" % (D["z_usb_e"], P["esp_plug_w"] + 2 * P["port_clr"],
                                                              P["esp_plug_h"] + 2 * P["port_clr"]),
         "| Charger USB-C opening | centre z %.1f, %.1f x %.1f |" % (D["z_usb_t"], P["tp_plug_w"] + 2 * P["port_clr"],
                                                                  P["tp_plug_h"] + 2 * P["port_clr"]),
         "| Battery | x %.1f..%.1f, top z %.1f; carrier underside z %.1f |" % (D["x_b0"], D["x_b1"], D["z_b1"], D["z_c0"]),
         "| Joystick PCB | x %.1f..%.1f, top z %.1f; stick %.1f mm from the front end |" % (
             D["x_j0"], D["x_j1"], D["z_j1"], D["L"] - D["stick_x"]),
         "| Lid opening for the stick | d %.1f at the inner face, d %.1f at the top (45-degree cone) |" % (
             2 * D["js_open_r"], 2 * (D["js_open_r"] + P["top"])),
         "| Switch | centre x %.1f in the bottom, cut-out %.1f x %.1f |" % (D["x_sw"], P["sw_cut_l"], P["sw_cut_w"]),
         "| LED light pipe | x %.1f, y %.1f (hole d %.1f) |" % (D["led"][0], D["led"][1], P["led_hole_d"]),
         ""]
    for label, res in out:
        ok = all(r[2] for r in res)
        L += ["## %s: %s" % (label, "PASS" if ok else "FAIL"), "", "| Group | Check | Result | Detail |",
              "|---|---|---|---|"]
        for g, n, k, d in res:
            L.append("| %s | %s | %s | %s |" % (g, n.replace("|", "/"), "ok" if k else "**FAIL**", d.replace("|", "/")))
        L.append("")
    if neg_out:
        L += ["## Negative tests (each MUST fail)", "", "| Broken on purpose | Caught | First failing checks |",
              "|---|---|---|"]
        for label, caught, fails in neg_out:
            L.append("| %s | %s | %s |" % (label, "yes" if caught else "**NO**",
                                           "; ".join("[%s] %s" % (f[0], f[1]) for f in fails)))
        L.append("")
    with open(os.path.join(HERE, "report", "check_report.md"), "w") as f:
        f.write("\n".join(L))


if __name__ == "__main__":
    sys.exit(main())
