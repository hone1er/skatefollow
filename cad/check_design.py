#!/usr/bin/env python3
"""Offline design checks for the honebot chassis (no Fusion needed).

Imports cad/fusion/HonebotChassis/HonebotChassis.py (the single geometry source) and checks:
  clearances (static + moving parts, keep-outs), mounting-hole audit, GoPro field of view over the pan range,
  mass / CG, tip-over and braking, stub axle / idler / mast / belt loads, HC-SR04 ground return, wiring routes,
  printability (bed size), overall envelope.
Writes cad/report/check_report.md and PNG views. Needs numpy + matplotlib.

    python3 cad/check_design.py            # full run (~1 min)
"""
import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "fusion", "HonebotChassis", "HonebotChassis.py")
OUT = os.path.join(HERE, "report")


def load():
    spec = importlib.util.spec_from_file_location("honebot_chassis", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


HC = load()

# ------------------------------------------------------------------------------------------------ geometry


def _frame(p):
    u = np.array(p["u"], float); u /= np.linalg.norm(u)
    w = np.cross(u, np.array(p["v"], float)); w /= np.linalg.norm(w)
    v = np.cross(w, u)
    return u, v, w


_BB = {}


def _prim_bb(p):
    k = id(p)
    if k not in _BB:
        c = prim_corners(p)
        _BB[k] = (c.min(0), c.max(0), p)     # keep p alive so id() stays unique
    return _BB[k][0], _BB[k][1]


def inside_prim(p, pts, grow=0.0):
    """Point-in-primitive with bounding-box culling (most primitives are small holes)."""
    lo, hi = _prim_bb(p)
    g = abs(grow) + 1e-9
    sel = np.all((pts >= lo - g) & (pts <= hi + g), axis=1)
    out = np.zeros(len(pts), bool)
    if sel.any():
        out[sel] = _inside_prim(p, pts[sel], grow)
    return out


def _inside_prim(p, pts, grow=0.0):
    if p["t"] == "sph":
        d = pts - np.array(p["c"])
        return np.einsum("ij,ij->i", d, d) <= (p["r"] + grow) ** 2
    if p["t"] == "diff":
        m = inside_prim(p["a"], pts, grow)
        for q in p["b"]:
            m &= ~inside_prim(q, pts, -grow)
        return m
    if p["t"] == "box":
        u, v, w = _frame(p)
        d = pts - np.array(p["c"])
        s = np.array(p["s"]) / 2 + grow
        return (np.abs(d @ u) <= s[0]) & (np.abs(d @ v) <= s[1]) & (np.abs(d @ w) <= s[2])
    a = np.array(p["a"], float); b = np.array(p["b"], float)
    ax = b - a; L = np.linalg.norm(ax); ax /= L
    d = pts - a
    t = d @ ax
    r2 = np.einsum("ij,ij->i", d, d) - t * t
    return (t >= -grow) & (t <= L + grow) & (r2 <= (p["r"] + grow) ** 2)


def inside_part(part, pts, grow=0.0, use_cuts=True):
    m = np.zeros(len(pts), bool)
    for st_adds, st_cuts in part.get("pre", []):
        sm = np.zeros(len(pts), bool)
        for p in st_adds:
            sm |= inside_prim(p, pts, grow)
        if use_cuts:
            for p in st_cuts:
                sm &= ~inside_prim(p, pts, -grow)
        m |= sm
    for p in part["adds"]:
        m |= inside_prim(p, pts, grow)
    if use_cuts and part["cuts"]:
        c = np.zeros(len(pts), bool)
        for p in part["cuts"]:
            c |= inside_prim(p, pts, -grow)
        m &= ~c
    return m


def all_adds(part):
    out = list(part["adds"])
    for a, _ in part.get("pre", []):
        out += a
    return out


def prim_corners(p):
    if p["t"] == "diff":
        return prim_corners(p["a"])
    if p["t"] == "sph":
        c = np.array(p["c"]); r = p["r"]
        return np.array([c + r * np.array([i, j, k]) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    if p["t"] == "box":
        u, v, w = _frame(p)
        c = np.array(p["c"]); s = np.array(p["s"]) / 2
        return np.array([c + i * s[0] * u + j * s[1] * v + k * s[2] * w for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    a = np.array(p["a"], float); b = np.array(p["b"], float)
    ax = (b - a) / np.linalg.norm(b - a)
    t1 = np.cross(ax, [1, 0, 0]) if abs(ax[0]) < 0.9 else np.cross(ax, [0, 1, 0])
    t1 /= np.linalg.norm(t1); t2 = np.cross(ax, t1)
    ang = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    ring = np.outer(np.cos(ang), t1) + np.outer(np.sin(ang), t2)
    return np.vstack([a + p["r"] * ring, b + p["r"] * ring])


_D = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]
              + [[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float)
DIRS = _D / np.linalg.norm(_D, axis=1)[:, None]


def near(part, pts, g):
    """True for points of `pts` that come within g of `part` (dilate the points, not the part: exact for holes)."""
    if g <= 0:
        return inside_part(part, pts)
    m = inside_part(part, pts)
    for dvec in DIRS:
        m |= inside_part(part, pts + g * dvec)
    return m


def deep_inside(part, pts, depth):
    """True for points that are inside `part` by more than `depth` in every sampled direction."""
    m = inside_part(part, pts)
    for dvec in DIRS:
        m &= inside_part(part, pts + depth * dvec)
    return m


def aabb(part):
    pts = np.vstack([prim_corners(p) for p in all_adds(part)])
    return pts.min(0), pts.max(0)


def sample(part, h=2.0, use_cuts=True):
    lo, hi = aabb(part)
    axes = [np.arange(lo[i] + h / 2, hi[i], h) for i in range(3)]
    if any(len(a) == 0 for a in axes):
        axes = [a if len(a) else np.array([(lo[i] + hi[i]) / 2]) for i, a in enumerate(axes)]
    g = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
    return g[inside_part(part, g, use_cuts=use_cuts)]


# ------------------------------------------------------------------------------------------------ checks

def check_clearance(M, report):
    parts = M.parts
    boxes = {p["name"]: aabb(p) for p in parts}
    samples = {}
    rows = []
    fails = 0

    def mates(a, b):
        return b["name"] in a["mates"] or a["name"] in b["mates"]

    nests = {frozenset((x, y)) for x, y, _ in M.nests}
    mate_rows = []
    for i, a in enumerate(parts):
        for b in parts[i + 1:]:
            if mates(a, b):
                # mates may touch but must not interpenetrate (unless declared as a nest)
                if "keepout" in (a["group"], b["group"]) or frozenset((a["name"], b["name"])) in nests:
                    continue
                la, ha = boxes[a["name"]]; lb, hb = boxes[b["name"]]
                if np.any(la > hb) or np.any(lb > ha):
                    continue
                small, big = (a, b) if np.prod(ha - la) < np.prod(hb - lb) else (b, a)
                if small["name"] not in samples:
                    samples[small["name"]] = sample(small, 1.5, use_cuts=True)
                pts = samples[small["name"]]
                deep = deep_inside(big, pts, 0.3) if len(pts) else np.zeros(0, bool)
                if deep.any():
                    n = int(deep.sum())
                    mate_rows.append((a["name"], b["name"], n))
                    fails += 1
                continue
            if a["group"] == "keepout" and b["group"] == "keepout":
                continue
            moving = a["moving"] or b["moving"] or a["group"] == "keepout" or b["group"] == "keepout"
            need = 3.0 if moving else 0.5
            la, ha = boxes[a["name"]]; lb, hb = boxes[b["name"]]
            if np.any(la - 8 > hb) or np.any(lb - 8 > ha):
                continue
            # sample the smaller part, test against the other inflated
            small, big = (a, b) if np.prod(ha - la) < np.prod(hb - lb) else (b, a)
            key = small["name"]
            if key not in samples:
                samples[key] = sample(small, 1.5, use_cuts=small["group"] != "keepout")
            pts = samples[key]
            if len(pts) == 0:
                continue
            # only points that can be near `big` at all
            lb2, hb2 = boxes[big["name"]]
            pts = pts[np.all((pts > lb2 - 9) & (pts < hb2 + 9), axis=1)]
            if len(pts) == 0:
                continue
            gap = None
            if not near(big, pts, 8.0).any():
                continue
            for g in (0.0, 0.5, 1.0, 2.0, 3.0, 5.0, 8.0):
                if near(big, pts, g).any():
                    gap = g
                    break
            ok = gap >= need
            # gap = first inflation that touches -> real clearance is between previous step and gap
            prev = {0.0: 0.0, 0.5: 0.0, 1.0: 0.5, 2.0: 1.0, 3.0: 2.0, 5.0: 3.0, 8.0: 5.0}[gap]
            if not ok:
                fails += 1
            rows.append((a["name"], b["name"], "moving" if moving else "static", need,
                         "overlap" if gap == 0.0 else "%.1f-%.1f" % (prev, gap), "OK" if ok else "FAIL"))
    rows.sort(key=lambda r: (r[5] != "FAIL", r[0]))
    report.append("## 1. Clearances\n")
    report.append("Every pair of parts that are not declared mates (bolted/sitting on each other) is checked. Required gap: "
                  "0.5 mm static, 3 mm when either part moves or is a swept keep-out volume (GoPro pan, "
                  "wheels, belts, pulleys, USB plugs). Only pairs within 8 mm are listed.\n")
    report.append("| Part A | Part B | type | need (mm) | measured gap (mm) | result |\n|---|---|---|---|---|---|")
    for r in rows:
        report.append("| %s | %s | %s | %.1f | %s | **%s** |" % r)
    report.append("\nMated pairs (bolted / resting on each other) are checked for interpenetration deeper than 0.3 mm; "
                  "%d declared nests are exempt (%s).\n" % (len(M.nests), "; ".join(sorted({w for _, _, w in M.nests}))))
    if mate_rows:
        report.append("| mated part A | mated part B | overlapping samples |\n|---|---|---|")
        for r in mate_rows:
            report.append("| %s | %s | **%d (FAIL)** |" % r)
    else:
        report.append("Mated pairs: no interpenetration.\n")
    report.append("\nClearance failures: **%d**\n" % fails)
    return fails


def check_holes(M, report):
    report.append("## 2. Mounting-hole audit\n")
    report.append("For every hardware fastener hole: the hole axis must run through its host part for at least the "
                  "required engagement (inserts: insert depth; through-holes: 2 mm) and a ring of material at "
                  "r + max(1.5 mm, d/2) must surround it there (edge distance). Slots are checked at both ends.\n")
    report.append("| host | hole | for | type | d | engaged length (mm) | wall ring | result |\n|---|---|---|---|---|---|---|---|")
    fails = 0
    P = M.info["P"]
    for h in M.holes:
        if not h.get("audit", True):
            continue
        host = M.get(h["host"])
        a = np.array(h["a"], float); b = np.array(h["b"], float)
        ax = b - a; L = np.linalg.norm(ax); ax /= L
        t1 = np.cross(ax, [1, 0, 0]) if abs(ax[0]) < 0.9 else np.cross(ax, [0, 1, 0]); t1 /= np.linalg.norm(t1)
        t2 = np.cross(ax, t1)
        wall = 3.0 if h["kind"] == "seat" else max(1.5, min(h["d"] / 2, 3.0))
        rr = h["d"] / 2 + wall
        ang = np.linspace(0, 2 * np.pi, 24, endpoint=False)
        ring = rr * (np.outer(np.cos(ang), t1) + np.outer(np.sin(ang), t2))
        centres = [np.zeros(3)]
        if h["kind"] == "slot":
            p0, p1 = np.array(h["slot"][0], float), np.array(h["slot"][1], float)
            mid = (p0 + p1) / 2
            centres = [p0 - mid, p1 - mid]
        stations = np.linspace(0, L, max(4, int(L / 0.5)))
        good = 0.0; worst = 1.0
        for t in stations:
            frac_all = []
            for c in centres:
                pts = a + t * ax + c + ring
                inn = inside_part(host, pts)
                if h["kind"] == "slot":   # ignore ring points that lie along the slot itself
                    q0 = a + t * ax + centres[0]; q1 = a + t * ax + centres[-1]
                    seg = q1 - q0; sl2 = max(float(seg @ seg), 1e-9)
                    tt = np.clip(((pts - q0) @ seg) / sl2, 0, 1)
                    dist = np.linalg.norm(pts - (q0 + tt[:, None] * seg), axis=1)
                    keep = dist > rr * 0.999
                    inn = inn[keep] if keep.any() else inn
                frac_all.append(inn.mean())
            f = min(frac_all)
            if f > 0.0:
                worst = min(worst, f)
            if f >= 0.999:
                good += L / len(stations)
        need = {"insert": P["ins_m3_depth"] - 0.5 if h["d"] < 5 else P["ins_m4_depth"] - 1.0,
                "seat": 7.0, "clear": 2.0, "slot": 2.0}[h["kind"]]
        ok = good >= need - 1e-6
        if not ok:
            fails += 1
        report.append("| %s | %s | %s | %s | %.1f | %.1f (need %.1f) | %d %% | **%s** |" %
                      (h["host"], h["id"], h["for"], h["kind"], h["d"], good, need, round(100 * worst), "OK" if ok else "FAIL"))
    report.append("\nHole failures: **%d** (holes for seller-dependent parts are slots or the 10 mm grid and are listed in "
                  "docs/chassis_cad.md)\n" % fails)
    return fails


def _hole_core(h, step=0.5):
    a = np.array(h["a"], float); b = np.array(h["b"], float)
    n = max(2, int(np.linalg.norm(b - a) / step))
    axis = a + np.linspace(0, 1, n)[:, None] * (b - a)
    if h["kind"] != "slot" or "slot" not in h:
        return axis
    p0, p1 = np.array(h["slot"][0], float), np.array(h["slot"][1], float)
    mid = (p0 + p1) / 2
    k = max(2, int(np.linalg.norm(p1 - p0) / step))
    offs = (p0 - mid) + np.linspace(0, 1, k)[:, None] * (p1 - p0)
    return (axis[:, None, :] + offs[None, :, :]).reshape(-1, 3)


def _hole_surface(h, step=0.5, nang=16):
    a = np.array(h["a"], float); b = np.array(h["b"], float)
    ax = b - a; L = np.linalg.norm(ax); ax /= L
    t1 = np.cross(ax, [1, 0, 0]) if abs(ax[0]) < 0.9 else np.cross(ax, [0, 1, 0]); t1 /= np.linalg.norm(t1)
    t2 = np.cross(ax, t1)
    ang = np.linspace(0, 2 * np.pi, nang, endpoint=False)
    ring = (h["d"] / 2) * (np.outer(np.cos(ang), t1) + np.outer(np.sin(ang), t2))
    core = _hole_core(h, step)
    return (core[:, None, :] + ring[None, :, :]).reshape(-1, 3)


def check_hole_spacing(M, report):
    report.append("### Hole-to-hole spacing\n")
    report.append("Every pair of fastener holes in the same part must keep >= 1.5 mm of material between them.\n")
    fails = 0; rows = []
    by_host = {}
    for h in M.holes:
        by_host.setdefault(h["host"], []).append(h)
    for host, hs in by_host.items():
        cores = [_hole_core(h) for h in hs]
        for i in range(len(hs)):
            for j in range(i + 1, len(hs)):
                if "seat" in (hs[i]["kind"], hs[j]["kind"]):
                    continue   # bearing seats are coaxial with their bolt by design
                ai, bi = np.array(hs[i]["a"], float), np.array(hs[i]["b"], float)
                aj, bj = np.array(hs[j]["a"], float), np.array(hs[j]["b"], float)
                ui = (bi - ai) / np.linalg.norm(bi - ai); uj = (bj - aj) / np.linalg.norm(bj - aj)
                if abs(abs(ui @ uj) - 1) < 1e-6 and np.linalg.norm(np.cross(aj - ai, ui)) < 1e-6:
                    continue   # coaxial (counterbore on its own hole, insert under a clearance hole ...)
                A, B = cores[i], cores[j]
                if np.any(A.min(0) - 25 > B.max(0)) or np.any(B.min(0) - 25 > A.max(0)):
                    continue
                d = np.min(np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2)) - hs[i]["d"] / 2 - hs[j]["d"] / 2
                if d < 1.5:     # axis estimate is conservative: confirm with the true surface-to-surface distance
                    SA, SB = _hole_surface(hs[i]), _hole_surface(hs[j])
                    near_b = SB[np.linalg.norm(SB - A.mean(0), axis=1) < np.linalg.norm(A.max(0) - A.min(0)) + hs[i]["d"] + 4]
                    if len(near_b):
                        d = min(np.min(np.linalg.norm(SA[k:k + 400, None, :] - near_b[None, :, :], axis=2))
                                for k in range(0, len(SA), 400))
                if d < 1.5:
                    fails += 1
                    rows.append((host, hs[i]["id"], hs[j]["id"], d))
    if rows:
        report.append("| part | hole | hole | web (mm) |\n|---|---|---|---|")
        for r in rows:
            report.append("| %s | %s | %s | **%.1f FAIL** |" % r)
    else:
        report.append("All hole pairs keep >= 1.5 mm web.\n")
    return fails


def check_removals(M, report):
    report.append("## 2b. Assembly and service: can each part come out?\n")
    report.append("Each group is slid along its exit direction (2 mm steps) after its fasteners and the listed parts "
                  "are removed; it must not hit anything that stays (fasteners and keep-outs ignored).\n")
    report.append("| group | direction | travel (mm) | removed first | result |\n|---|---|---|---|---|")
    fails = 0
    fast = {p["name"] for p in M.parts if p["name"].startswith(("plate_bolt", "pi_standoff"))}
    for grp, d, travel, before in M.removals:
        d = np.array(d, float)
        pts = np.vstack([sample(M.get(n), 2.0) for n in grp if len(sample(M.get(n), 2.0))])
        stay = [p for p in M.parts if p["group"] != "keepout" and p["name"] not in grp and p["name"] not in before
                and p["name"] not in fast]
        boxes = {p["name"]: aabb(p) for p in stay}
        hit = None
        for t in np.arange(3.0, travel + 0.1, 3.0):
            q = pts + t * d
            qlo, qhi = q.min(0), q.max(0)
            for p in stay:
                lo, hi = boxes[p["name"]]
                if np.any(qlo > hi) or np.any(lo > qhi):
                    continue
                sel = np.all((q >= lo) & (q <= hi), axis=1)
                if sel.any() and deep_inside(p, q[sel], 0.2).any():
                    hit = (p["name"], t)
                    break
            if hit:
                break
        ok = hit is None
        fails += 0 if ok else 1
        report.append("| %s | %s | %d | %s | %s |" % (", ".join(grp[:3]) + (" ..." if len(grp) > 3 else ""),
                                                     tuple(int(v) for v in d), travel, ", ".join(before[:4]) + (" ..." if len(before) > 4 else "") or "-",
                                                     "**OK**" if ok else "**FAIL: hits %s after %.0f mm**" % hit))
    report.append("")
    return fails


FOV_MODES = {   # degrees (H, V). GoPro published figures for HERO9; not re-verified this session -> 5 deg margin
    "Wide 4:3": (122.6, 94.4),
    "Wide 16:9": (118.2, 69.5),
    "SuperView 16:9": (121.7, 70.4),
    "Linear 4:3": (86.0, 70.0),
    "Linear 16:9": (86.0, 55.0),
}
MARGIN = 5.0


def robot_points(M, h=3.0):
    pts = []
    for p in M.parts:
        if p["group"] == "keepout" or p["name"] in ("gopro", "pan_buckle"):
            continue
        s = sample(p, h, use_cuts=False)
        if len(s):
            pts.append(s)
    return np.vstack(pts)


def cam_project(M, pts, pan_deg, mode):
    cam = M.info["camera"]; P = M.info["P"]
    pax = np.array(cam["pan_axis"])
    lens0 = np.array(cam["lens"])
    pan = math.radians(pan_deg); tilt = math.radians(P["cam_tilt_deg"])
    R = np.array([[math.cos(pan), -math.sin(pan)], [math.sin(pan), math.cos(pan)]])
    lxy = R @ (lens0[:2] - pax) + pax
    lens = np.array([lxy[0], lxy[1], lens0[2]])
    f = np.array([math.cos(pan) * math.cos(tilt), math.sin(pan) * math.cos(tilt), math.sin(tilt)])
    left = np.array([-math.sin(pan), math.cos(pan), 0.0])
    up = np.cross(f, left)
    d = pts - lens
    zc = d @ f; xc = d @ left; yc = d @ up
    H, V = FOV_MODES[mode]
    h2, v2 = math.radians(H / 2 - MARGIN), math.radians(V / 2 - MARGIN)
    rect = (zc > 0) & (np.abs(np.arctan2(xc, zc)) < h2) & (np.abs(np.arctan2(yc, zc)) < v2)
    theta = np.arccos(np.clip(zc / np.linalg.norm(d, axis=1), -1, 1))
    phi = np.arctan2(yc, xc)
    fish = (theta * np.abs(np.cos(phi)) < h2) & (theta * np.abs(np.sin(phi)) < v2)
    # image-plane coords for plotting (equidistant), normalised to the half-FOV (incl. margin)
    hx = theta * np.cos(phi) / math.radians(H / 2); hy = theta * np.sin(phi) / math.radians(V / 2)
    return rect | fish, hx, hy, theta


def check_camera(M, report, pts=None, quiet=False):
    P = M.info["P"]
    if pts is None:
        pts = robot_points(M)
    res = {}
    for mode in FOV_MODES:
        worst = 0
        for pan in range(-int(P["pan_half_range_deg"]), int(P["pan_half_range_deg"]) + 1, 5):
            vis, *_ = cam_project(M, pts, pan, mode)
            worst = max(worst, int(vis.sum()))
        res[mode] = worst
    if not quiet:
        cam = M.info["camera"]
        report.append("## 3. GoPro view: robot must not appear in frame\n")
        report.append("Lens at x = %.0f, z = %.0f mm (camera centre z = %.0f), lens %.0f mm left of the pan axis; pan "
                      "swept -%d..+%d deg in 5 deg steps; robot sampled every 3 mm (wheels, pods, bumper, sensor bracket, "
                      "UWB plate, mast head, lid ...). A point counts as visible if inside the frame under EITHER a "
                      "rectilinear or an equidistant-fisheye model, with %.0f deg margin on every edge (mount tolerance, "
                      "pitch under braking, FOV figures not re-verified).\n" %
                      (cam["lens"][0], cam["lens"][2], cam["centre"][2], cam["lens"][1], P["pan_half_range_deg"],
                       P["pan_half_range_deg"], MARGIN))
        report.append("| lens mode | H x V (deg) | robot points in frame (worst pan) | result |\n|---|---|---|---|")
        for mode, n in res.items():
            report.append("| %s | %.1f x %.1f | %d | **%s** |" % (mode, *FOV_MODES[mode], n, "OK" if n == 0 else "FAIL"))
        report.append("")
    return res, pts


def camera_sweeps(report):
    """How low / far back can the camera go and still pass Wide 4:3 (the widest mode)?"""
    report.append("### Camera position sensitivity (Wide 4:3, worst pan)\n")
    report.append("| servo seat z (mm) | lens z | mast x | robot points in frame | result |\n|---|---|---|---|---|")
    base = HC.PARAMS["servo_seat_z"]
    rows = []
    for seat in (base - 60, base - 40, base - 20, base):
        for mx in (HC.PARAMS["mast_x"] - 10, HC.PARAMS["mast_x"]):
            M = HC.build_model({"servo_seat_z": seat, "mast_x": mx})
            M.parts = [p for p in M.parts if p["group"] != "tools"]
            res, _ = check_camera(M, [], quiet=True)
            n = res["Wide 4:3"]
            rows.append((seat, M.info["camera"]["lens"][2], mx, n))
            report.append("| %d | %.0f | %d | %d | %s |" % (seat, M.info["camera"]["lens"][2], mx, n, "OK" if n == 0 else "in view"))
    report.append("")
    return rows


def tilt_sweep(report, pts):
    """Largest downward camera tilt (deg) that keeps the robot out of frame, per lens mode."""
    report.append("### Camera tilt allowance\n")
    report.append("| lens mode | max down-tilt with the robot out of frame (deg, incl. 5 deg margin) |\n|---|---|")
    for mode in FOV_MODES:
        best = None
        for tilt in range(0, -31, -1):
            M = HC.build_model({"cam_tilt_deg": float(tilt)})
            M.parts = [p for p in M.parts if p["group"] != "tools"]
            worst = 0
            for pan in range(-45, 46, 5):
                vis, *_ = cam_project(M, pts, pan, mode)
                worst = max(worst, int(vis.sum()))
            if worst:
                break
            best = tilt
        report.append("| %s | %s |" % (mode, ("%d" % -best + (" or more" if best == -30 else "")) if best is not None else "none (in view level)"))
    report.append("\nTilting **up** never brings the robot into view. Set the GoPro's tilt (M5 thumb screw) within these limits.\n")


def plot_camera(M, pts, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.4))
    for ax, pan in zip(axs, (-45, 0, 45)):
        vis, hx, hy, theta = cam_project(M, pts, pan, "Wide 4:3")
        near = theta < math.radians(95)
        ax.scatter(-hx[near], hy[near], s=1, c="#bbbbbb", label="robot (outside frame)")
        if vis.any():
            ax.scatter(-hx[vis], hy[vis], s=2, c="red", label="robot IN FRAME")
        ax.add_patch(plt.Rectangle((-1, -1), 2, 2, fill=False, lw=2, ec="k"))
        m = 1 - MARGIN / (FOV_MODES["Wide 4:3"][0] / 2)
        mv = 1 - MARGIN / (FOV_MODES["Wide 4:3"][1] / 2)
        ax.add_patch(plt.Rectangle((-m, -mv), 2 * m, 2 * mv, fill=False, lw=1, ls="--", ec="tab:blue"))
        ax.set_xlim(-1.8, 1.8); ax.set_ylim(-1.8, 1.2); ax.set_aspect("equal")
        ax.set_title("GoPro Wide 4:3, pan %+d deg" % pan)
        ax.set_xlabel("image x (1 = frame edge)")
    axs[0].legend(loc="lower left", fontsize=7)
    fig.suptitle("Where the robot lands relative to the GoPro frame (black = frame, dashed = 5 deg margin)")
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


DENS = {"PAHT-CF": 1.20, "PETG": 1.27, "TPU": 1.21}
FILL = 0.65    # effective density factor for 4-6 walls + 30-50 % infill


def mass_props(M, report):
    rows = []
    tot = 0.0; mom = np.zeros(3)
    for p in M.parts:
        if p["group"] == "keepout":
            continue
        pts = sample(p, 1.5 if p["group"] == "printed" else 2.0, use_cuts=True)
        if len(pts) == 0:
            continue
        vol = len(pts) * (1.5 ** 3 if p["group"] == "printed" else 8.0) / 1000.0  # cm3
        if p["group"] == "printed":
            mat = p["material"].split()[0]
            rho = DENS.get(mat, 1.25)
            mass = vol * rho * FILL / 1000.0
        else:
            mass = p["mass"] or 0.0
        cg = pts.mean(0)
        rows.append((p["name"], p["group"], mass, cg, vol))
        tot += mass; mom += mass * cg
    misc = [("wiring + connectors", 0.08, np.array([-60.0, 0, 90])), ("fasteners, inserts, bearings", 0.10, np.array([-60.0, 0, 70]))]
    for n, mm, c in misc:
        rows.append((n, "misc", mm, c, 0)); tot += mm; mom += mm * c
    cg = mom / tot
    report.append("## 4. Mass and centre of gravity\n")
    report.append("Printed masses from the modelled volume x density x %.2f fill factor; hardware masses from datasheets or "
                  "estimates ([M] = weigh yours).\n" % FILL)
    report.append("| part | group | mass (g) | CG x, y, z (mm) |\n|---|---|---|---|")
    for n, g, mm, c, v in sorted(rows, key=lambda r: -r[2]):
        report.append("| %s | %s | %.0f | %.0f, %.0f, %.0f |" % (n, g, 1000 * mm, c[0], c[1], c[2]))
    report.append("\n**Total %.2f kg, CG at x = %.0f, y = %.1f, z = %.0f mm** (x from the drive axle, + forward).\n" %
                  (tot, cg[0], cg[1], cg[2]))
    return tot, cg


def dynamics(M, report, mass, cg):
    P = M.info["P"]
    tw = P["wheel_inner_y"] + P["wheel_w"] / 2
    xr = M.info["rear_axle_x"]
    L = -xr
    h = cg[2]
    report.append("## 5. Stability, traction, braking\n")
    # rectangle of 4 wheel contacts: x in [xr, 0], y = +/-tw
    lat_g = (tw - abs(cg[1])) / h
    fx = (cg[0] - xr) / L             # static fraction on the front axle
    pitch_g = -cg[0] / h if cg[0] < 0 else 0.0   # tip forward over the front axle
    back_g = (cg[0] - xr) / h         # tip backward over the rear axle
    report.append("- Support rectangle: 4 wheel contacts at x = 0 / %.0f mm, y = +/-%.0f mm (all four driven, belt-coupled "
                  "per side).\n" % (xr, tw))
    report.append("- Static load: front axle **%.0f %%**, rear %.0f %%. All four wheels brake and drive.\n" % (100 * fx, 100 * (1 - fx)))
    report.append("- Sideways tip-over needs **%.2f g** (urethane on asphalt slides at ~0.7-0.9 g, so it slides first%s).\n" %
                  (lat_g, "" if lat_g > 0.9 else " ONLY JUST - keep heavy parts low"))
    nose = sample(M.get("bumper_door"), 2.0)
    nose = nose[nose[:, 0] > 0]
    touch = math.degrees(np.min(np.arctan2(nose[:, 2], nose[:, 0])))
    tip_deg = math.degrees(math.atan2(-cg[0], h))
    report.append("- Forward pitch-over about the front axle needs **%.2f g** of braking, backward (hard launch) **%.2f g**. "
                  "If it does pitch forward, the bumper touches down after %.1f deg (the CG passes over the axle at "
                  "%.0f deg).\n" % (pitch_g, back_g, touch, tip_deg))
    # pivot turn (skid steer): wheels scrub sideways; thrust per side F = mu W (wheelbase/2) / track
    wb = -xr; trk = 2 * tw
    mu_lat = 0.8
    F_turn = mu_lat * mass * 9.81 * (wb / 2) / trk + 0.02 * mass * 9.81 / 2
    T_motor = F_turn * (P["wheel_d"] / 2000) / (P["pul_w_t"] / P["pul_m_t"])
    I_turn = T_motor / (9.55 / 370)
    report.append("- **Turning in place** (skid steer): the wheels must scrub sideways. With grip mu 0.8 the robot needs "
                  "**%.0f N per side** (thrust / traction = wheelbase / track = %.2f; under 1 means it can pivot), i.e. "
                  "~%.2f N m and **~%.0f A per motor at near-zero rpm** (30 A ESC). The 52 mm wide contact patches add "
                  "scrub torque on top. Yaw rate will be lower than (vR - vL) / track: tune `track_width` up (typically "
                  "1.2-1.6 x the geometric %.0f mm) and expect a small turn dead-band.\n" % (F_turn, wb / trk, T_motor, I_turn, trk))
    rows = []
    for mu in (0.8, 0.6):
        a_fd = min(mu, pitch_g) * 9.81                             # 4WD: all wheels brake (traction or pitch limit)
        a_rd = mu * 9.81 * (cg[0] - xr) / (L - mu * h)             # comparison: only the front axle braking (2WD + caster)
        a_rd = min(a_rd, pitch_g * 9.81)
        for v_mph in (8, 12, 15):
            v = v_mph * 0.44704
            d_fd = v * 0.25 + v * v / (2 * a_fd)
            d_rd = v * 0.25 + v * v / (2 * a_rd)
            rows.append((mu, v_mph, a_fd / 9.81, d_fd, a_rd / 9.81, d_rd))
    report.append("\nStopping distance = 0.25 s reaction (gate + ESC ramp) + v^2 / 2a, a limited by drive-wheel traction "
                  "(weight transfer included) and by pitch-over:\n")
    report.append("| mu | speed (mph) | THIS layout (4WD): decel (g) | stop (m) | front axle braking only: decel (g) | stop (m) |\n|---|---|---|---|---|---|")
    for r in rows:
        report.append("| %.1f | %d | %.2f | %.1f | %.2f | %.1f |" % r)
    report.append("\nThe follow distance must exceed the stopping distance at the current speed if the skater can stop "
                  "instantly (a fall). Recommended `follow_distance` >= the 'stop' column, e.g. speed-dependent "
                  "d = 1.5 m + 0.35 s x v + v^2 / (2 x 0.45 g).\n")
    return {"lat_g": lat_g, "fx": fx, "pitch_g": pitch_g, "back_g": back_g, "rows": rows, "I_turn": I_turn, "wb_tr": wb / trk}


def drivetrain(report, mass):
    report.append("## 6. Drivetrain: speed and thrust (3S, 11.1 V nominal)\n")
    report.append("Drag at 15 mph: rolling (Crr 0.02) + air (CdA 0.055 m^2). Requirement: thrust at 15 mph >= drag + "
                  "1.5 m/s^2 x mass.\n")
    v = 6.7; crr = 0.02; cda = 0.055
    drag = crr * mass * 9.81 + 0.5 * 1.2 * cda * v * v
    need = drag + mass * 1.5
    report.append("Drag %.1f N, required %.1f N.\n" % (drag, need))
    report.append("| option | no-load speed (mph) | thrust at 15 mph (N) | thrust limit (N) | motors (kg) | verdict |\n|---|---|---|---|---|---|")

    def bldc(name, kv, R, imax, ratio, wd, mm):
        wheel_rpm = v / (math.pi * wd / 1000) * 60
        emf = wheel_rpm * ratio / kv
        i = max(0.0, min(imax, (11.1 - emf) / R))
        kt = 9.55 / kv
        F = 2 * kt * i * ratio / (wd / 2000)
        F0 = 2 * kt * min(imax, 11.1 / R) * ratio / (wd / 2000)
        vmax = kv * 11.1 / ratio * math.pi * wd / 1000 / 60
        report.append("| %s | %.1f | %.1f | %.0f (traction-limited) | %.2f | %s |" % (name, vmax * 2.237, F, F0, 2 * mm, "OK" if F >= need else "FAIL"))

    def brushed(name, rpm12, T12, wd, mm):
        w0 = rpm12 * 11.1 / 12; T0 = T12 * 11.1 / 12
        vmax = w0 * math.pi * wd / 1000 / 60
        F = 2 * T0 * max(0.0, 1 - v / vmax) / (wd / 2000)
        report.append("| %s | %.1f | %.1f | %.0f | %.2f | %s |" % (name, vmax * 2.237, F, 2 * T0 / (wd / 2000), 2 * mm, "OK" if F >= need else "FAIL"))

    P = HC.build_model().info["P"]
    r = P["pul_w_t"] / P["pul_m_t"]; tag = "%dT:%dT" % (P["pul_m_t"], P["pul_w_t"])
    bldc("**5010 370KV + %s HTD-3M, 90 mm (chosen)**" % tag, 370, 0.15, 30, r, 90, 0.15)
    bldc("same, 83 mm wheels", 370, 0.15, 30, r, 83, 0.15)
    bldc("same, pack sagged to 3.5 V/cell", 370 * 10.5 / 11.1, 0.15, 30, r, 90, 0.15)
    bldc("5010 370KV + 20T:48T (shaft-mounted pulley variant)", 370, 0.15, 30, 2.4, 90, 0.15)
    brushed("goBILDA 5203 3.7:1 planetary, direct, 90 mm", 1620, 0.53, 90, 0.40)
    brushed("goBILDA 5203 3.7:1 planetary, direct, 107 mm", 1620, 0.53, 107, 0.40)
    report.append("\nMotor model: Kv 370 (MAD 5010 EEE 370KV), phase resistance 0.15 ohm (typical 5010 class, [M]), 30 A ESC limit. The "
                  "sag row scales Kv to mimic 10.5 V. goBILDA figures: 1620 rpm and 5.4 kg-cm stall at 12 V [V].\n")


K_LIP = 50e3      # N/m, urethane wheel contact stiffness on a sharp lip edge [assumption - measure: load a wheel on a 10 mm bar]


def lip_impact(m_eff, r_mm, h_mm, v):
    """Peak contact force when a wheel of radius r meets a square lip of height h at speed v (no suspension):
    the wheel must gain v*sin(theta) upward, cushioned only by the urethane (spring k)."""
    th = math.acos((r_mm - h_mm) / r_mm)
    vv = v * math.sin(th)
    return vv * math.sqrt(K_LIP * m_eff), th


def structural(M, report, mass, cg, dyn):
    P = M.info["P"]; g = 9.81; fx = dyn["fx"]
    report.append("## 7. Structural checks (hand calculations) at the worst road event\n")
    cases = [("6 mm crack at 15 mph", 6.0, 6.7), ("10 mm lip at 3 m/s (driveway, slowed down)", 10.0, 3.0)]
    m_w = mass * max(fx, 1 - fx) / 2 + 0.25          # the more loaded axle, one wheel + its unsprung share
    rw = P["wheel_d"] / 2
    report.append("Design event: a wheel hits a square lip. Peak force = v sin(theta) x sqrt(k m), with the urethane "
                  "contact stiffness k = %.0f N/mm [assumption] and the mass riding on that wheel (%.2f kg, heavier "
                  "axle). The skater will not cross a 10 mm lip at 15 mph either; the follower must slow for driveways.\n"
                  % (K_LIP / 1000, m_w))
    report.append("| event | wheel force (N) |\n|---|---|")
    Fw = 0.0; thw = 0.0
    for name, h, v in cases:
        a, ta = lip_impact(m_w, rw, h, v)
        report.append("| %s | %.0f |" % (name, a))
        if a > Fw:
            Fw, thw = a, ta
    rows = []
    # ---- stub axle: M8 12.9 bolt inside a steel sleeve (wall -> wheel) ----
    lever = (P["wheel_inner_y"] + P["wheel_w"] / 2) - P["tub_hw"]
    sd = P["axle_sleeve_d"]
    E_sl, E_st, Y_sl = P["axle_sleeve_E"], 210e3, P["axle_sleeve_yield"]
    I_sl = math.pi / 64 * (sd ** 4 - 8.2 ** 4); I_b = math.pi / 64 * 6.47 ** 4
    M_root = Fw * lever
    kappa = M_root / (E_sl * I_sl + E_st * I_b)
    s_root = E_sl * kappa * sd / 2
    rows.append(("stub axle root: nested steel tubes %gx8.2 over M8 (yield %.0f MPa)" % (sd, Y_sl),
                 "%.0f N at %.0f mm" % (Fw, lever), "%.0f MPa" % s_root, Y_sl / s_root))
    s_bolt_root = E_st * kappa * 6.47 / 2
    rows.append(("M8 12.9 at the root (shares the sleeve's curvature)", "", "%.0f MPa" % s_bolt_root, 1080 / s_bolt_root))
    span = P["wheel_w"] - 8
    I_sp = math.pi / 64 * (12 ** 4 - 8.2 ** 4)
    s_bolt = (Fw / 2 * span) / ((I_sp + I_b) / 6.0)
    rows.append(("M8 + the wheel's 12 x 8 bearing spacer, between the 608s", "F/2 x %.0f mm bearing span" % span,
                 "%.0f MPa" % s_bolt, 1080 / s_bolt))
    Lb = (P["tub_hw"] + 8) - 37.0
    s_boss = 6 * M_root / ((sd + 0.4) * Lb ** 2) + Fw / ((sd + 0.4) * Lb)
    rows.append(("axle boss bearing on the sleeve, PAHT-CF (allow 40 MPa)", "moment over a %.0f mm boss" % Lb,
                 "%.0f MPa" % s_boss, 40 / s_boss))
    s_blk = M_root / (36.0 * 44.0 ** 2 / 6)
    rows.append(("boss block root into floor + bridge beam (allow 40 MPa)", "36 x 44 mm section", "%.1f MPa" % s_blk, 40 / s_blk))
    # ---- 1:1 belt idler (608 on an M8 in a wall slot) ----
    T2 = P["belt_pretension_n"] + (mu_tr := 0.9) * (mass * g / 4 * 1.3) * rw / 1000 / (P["pul_r_pd"] / 2000)
    F_id = 2 * T2 * math.sin(math.radians(15))          # ~15 deg wrap deflection per side of the idler
    rows.append(("idler M8 clamped in a 6 mm wall slot (bearing, allow 40 MPa)", "rear belt tight side %.0f N" % T2,
                 "%.1f MPa" % (F_id / (8 * 6)), 40 / (F_id / (8 * 6))))
    rows.append(("608-2RS idler, static rating C0 1370 N", "", "%.0f N" % F_id, 1370 / F_id))
    # ---- motor shaft (pulley bolted to the bell) ----
    mu = 0.9
    Tw = mu * (mass * g / 2 * 1.3) * rw / 1000   # one motor drives both wheels on its side
    Te = Tw / (P["pul_w_pd"] / 2000)
    T0 = P["belt_pretension_n"]
    F_sh = (T0 + Te / 2) + max(0.0, T0 - Te / 2)
    a_oh = 14.0
    s_sh = F_sh * a_oh / (math.pi * P["motor_shaft_d"] ** 3 / 32)
    rows.append(("motor shaft bending (5 mm, %d mm overhang [M]); endurance ~250 MPa" % a_oh,
                 "belt %.0f N set + %.0f N pull" % (T0, Te), "%.0f MPa" % s_sh, 250 / s_sh))
    rows.append(("turn in place: motor current vs 30 A ESC (wheelbase/track %.2f)" % dyn["wb_tr"], "mu 0.8 scrub",
                 "%.0f A" % dyn["I_turn"], 30 / dyn["I_turn"]))
    rows.append(("pulley tooth root to bore wall (need 3 mm)", "", "%.1f mm" % M.info["pulley_root_wall"],
                 M.info["pulley_root_wall"] / 3.0 * 1.5))
    k_sh = 3 * 210e3 * (math.pi * P["motor_shaft_d"] ** 4 / 64) / a_oh ** 3
    report.append("")
    report.append("| item | load case | result | safety factor |\n|---|---|---|---|")
    for r in rows:
        sf = ("%.1f" % r[3]) if r[3] else "-"
        bad = r[3] is not None and r[3] < 1.5
        report.append("| %s | %s | %s | %s |" % (r[0], r[1], r[2], ("**%s (LOW)**" % sf) if bad else sf))
    report.append("\nMotor shaft: the 5 mm shaft over ~%d mm is ~%.0f N/mm stiff, so a printed outboard bearing (well under "
                  "1000 N/mm through the bracket) would carry <10 %% of the belt load and add misalignment stress. The "
                  "shaft is protected instead by: pulley bolted straight to the bell (shortest overhang), belt set to "
                  "%.0f N per strand (deflection spec in docs), and the pull-screw stop so tension cannot creep up.\n"
                  % (a_oh, k_sh, T0))
    low = sum(1 for r in rows if r[3] is not None and r[3] < 1.5)
    # mast (unchanged model)
    mi = M.info["mast"]; cam = M.info["camera"]
    E = 3.5e9
    so, si = mi["sq"] / 1000, (mi["sq"] - 2 * mi["wall"]) / 1000
    I = (so ** 4 - si ** 4) / 12
    masses = [(P["gopro_mass"] + 0.015, cam["centre"][2]), (0.055 + 0.03, mi["z_head"] + 20), (0.03 + 0.02, mi["z0"] + 40)]
    z0 = mi["z0"]

    def defl(zq, zl, F):
        a = (zl - z0) / 1000; x = (zq - z0) / 1000
        if x <= a:
            return F * x * x * (3 * a - x) / (6 * E * I)
        return F * a * a * (3 * x - a) / (6 * E * I)
    ys = [sum(defl(zq, zl, mm * g) for mm, zl in masses) for _, zq in masses]
    num = sum(mm * g * y for (mm, _), y in zip(masses, ys)); den = sum(mm * y * y for (mm, _), y in zip(masses, ys))
    f1 = math.sqrt(num / den) / (2 * math.pi) if den > 0 else float("nan")
    report.append("Mast first bending mode (E 3.5 GPa, ideal clamp): **%.0f Hz** (target > 30 Hz).\n" % f1)
    return low


def terrain(M, report):
    """Approach, departure, breakover (crest) and gutter (valley) angles; climbable lip."""
    P = M.info["P"]
    skip = tuple("wheel_" + a + b for a in "FR" for b in "LR")
    pts = []
    for p in M.parts:
        if p["group"] == "keepout" or p["name"] in skip:
            continue
        sp = sample(p, 3.0, use_cuts=False)
        if len(sp):
            pts.append(sp[sp[:, 2] < 90])
    pts = np.vstack(pts)
    x, z = pts[:, 0], pts[:, 2]
    xc = M.info["rear_axle_x"]

    def ok(gfun):
        g0, gc = gfun(np.array([0.0]))[0], gfun(np.array([xc]))[0]
        slope = (g0 - gc) / (0 - xc)
        zw = z + gc + (x - xc) * slope
        return np.all(zw - gfun(x) >= 0.0)

    res = {}
    def search(make, positions):
        best = 0
        for d in np.arange(1, 46, 1.0):
            t = math.tan(math.radians(d))
            if all(ok(make(t, xa)) for xa in positions):
                best = d
            else:
                break
        return best
    res["approach (ramp ahead of the front wheels)"] = search(lambda t, xa: (lambda xx: t * np.maximum(0, xx - xa)),
                                                              np.arange(0, 121, 10))
    res["departure (ramp behind the rear wheels)"] = search(lambda t, xa: (lambda xx: t * np.maximum(0, xa - xx)),
                                                       np.arange(xc - 120, xc + 1, 10))
    res["breakover (crest between the wheels)"] = search(lambda t, xa: (lambda xx: -np.tan(math.atan(t) / 2) * np.abs(xx - xa)),
                                                         np.arange(xc, 1, 10))
    res["gutter / valley"] = search(lambda t, xa: (lambda xx: np.tan(math.atan(t) / 2) * np.abs(xx - xa)),
                                    np.arange(xc - 60, 101, 10))
    mu = 0.8
    lip_w = P["wheel_d"] / 2 * (1 - 1 / math.sqrt(1 + mu * mu))
    report.append("## 5b. Street -> driveway\n")
    report.append("Assumed driveway profile: a gutter/apron transition with up to **20 deg** of slope change (15 % apron "
                  "plus the gutter dip) and lips up to **10 mm**. Each angle below is the largest slope change the "
                  "chassis clears with nothing but the wheels touching (small-angle model, every body point checked).\n")
    report.append("| case | clears up to | need | result |\n|---|---|---|---|")
    fails = 0
    for k, v in res.items():
        okk = v >= 20
        fails += 0 if okk else 1
        report.append("| %s | %.0f deg | 20 deg | **%s** |" % (k, v, "OK" if okk else "FAIL"))
    report.append("\nLowest points: under the floor %.0f mm, wheel-pulley flange %.0f mm (inside the wheel's shadow). A driven 90 mm wheel climbs a square lip of about **%.0f mm** from standstill (mu 0.8); "
                  "more with momentum. Treat 10 mm lips as a slow-down event (section 7).\n" %
                  (P["floor_z0"], P["axle_z"] - M.info["wheel_pulley_flange_r"], lip_w))
    return fails


def beam_obstruction(M, report):
    u = M.info["us"]
    c = np.array(u["c"]); n = np.array(u["n"])
    t1 = np.array([0.0, 1.0, 0.0]); t2 = np.cross(n, t1)
    pts = []
    half = math.radians(u["half"])
    for dist in np.arange(3, 400, 3):
        rr = dist * math.tan(half)
        for ang in np.linspace(0, 2 * np.pi, 24, endpoint=False):
            for f in (0.33, 0.66, 1.0):
                pts.append(c + dist * n + f * rr * (math.cos(ang) * t1 + math.sin(ang) * t2))
    pts = np.array(pts)
    hits = set()
    for p in M.parts:
        if p["group"] == "keepout" or p["name"] in ("hcsr04",):
            continue
        if inside_part(p, pts).any():
            hits.add(p["name"])
    report.append("Beam obstruction (cone sampled to 0.4 m): %s\n" % ("**none**" if not hits else "**HITS " + ", ".join(sorted(hits)) + "**"))
    return len(hits)


FASTENERS = {
    "insert": "heat-set insert (M3 x 5.7 in a 4.0 hole / M4 x 8.1 in a 5.6 hole), then a machine screw",
    "clear": "clearance hole, screw passes through (nut, insert or threaded part on the far side)",
    "slot": "slotted clearance, adjust then tighten",
    "seat": "bearing seat, press in with a drop of retaining compound",
    "cbore": "counterbore for a low-head screw",
}


def fastener_schedule(M, report):
    report.append("## 11. Fastener schedule (generated from the model)\n")
    report.append("Threads in printed parts are always heat-set inserts (never tapped plastic); servo ear screws are the "
                  "servo's own self-tappers in 2.4 mm slots. Screw length = the parts it clamps + 1.5 x d engagement.\n")
    report.append("| for | hole | Ø (mm) | count | method |\n|---|---|---|---|---|")
    groups = {}
    for h in M.holes:
        key = (h["for"], h["kind"], round(h["d"], 1))
        groups[key] = groups.get(key, 0) + 1
    for (f, k, d), cnt in sorted(groups.items()):
        report.append("| %s | %s | %.1f | %d | %s |" % (f, k, d, cnt, FASTENERS.get(k, "")))
    report.append("")


def ultrasonic(M, report):
    u = M.info["us"]
    lo = math.radians(u["tilt"] - u["half"])
    report.append("## 8. HC-SR04 beam\n")
    if lo >= 0:
        hit = float("inf")
    else:
        hit = u["z"] / math.tan(-lo) / 1000
    report.append("Sensor centre z = %.0f mm, tilted up %.0f deg; with a +/-%.0f deg beam the lower edge reaches the ground "
                  "at **%.2f m** (must be beyond `slow_distance` = 1.0 m, otherwise the gate slows for the road). "
                  "Lower beam edge height at 0.4 m (`stop_distance`): %.0f mm, at 1.0 m: %.0f mm -> kerbs taller than "
                  "that are seen. The beam half-angle is a datasheet-style figure: confirm with the test in "
                  "docs/chassis_cad.md.\n" % (u["z"], u["tilt"], u["half"], hit,
                                              u["z"] + 400 * math.tan(lo), u["z"] + 1000 * math.tan(lo)))
    return hit


def routes(M, report):
    report.append("## 9. Wiring routes\n")
    report.append("Centre lines sampled every 1 mm; a route fails if it passes through printed material or hardware "
                  "(holes, slots and windows count as free space). First/last 4 mm are the connectors.\n")
    report.append("| harness | length (mm) | blocked samples | result |\n|---|---|---|---|")
    fails = 0
    solids = [p for p in M.parts if p["group"] != "keepout"]
    for name, pts in M.routes:
        pts = np.array(pts, float)
        seg = []
        for a, b in zip(pts[:-1], pts[1:]):
            n = max(2, int(np.linalg.norm(b - a)))
            seg.append(a + np.linspace(0, 1, n, endpoint=False)[:, None] * (b - a))
        seg.append(pts[-1:])
        s = np.vstack(seg)
        L = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
        s = s[4:-4]
        hit = np.zeros(len(s), bool); who = set()
        for p in solids:
            m = inside_part(p, s)
            if m.any():
                who.add(p["name"])
            hit |= m
        ok = not hit.any()
        if not ok:
            fails += 1
        report.append("| %s | %.0f | %d %s | **%s** |" % (name, L, int(hit.sum()), ("(" + ", ".join(sorted(who)) + ")") if who else "",
                                                       "OK" if ok else "FAIL"))
    report.append("\nAdd ~30 %% for slack and service loops when cutting wires. Power runs under the deck in the side "
                  "channels, signal cables over the deck.\n")
    return fails


def printability(M, report):
    report.append("## 10. Printability and envelope\n")
    report.append("| printed part | material | size x, y, z (mm) | fits 300 x 300 x 300 |\n|---|---|---|---|")
    fails = 0
    for p in M.parts:
        if p["group"] != "printed":
            continue
        lo, hi = aabb(p)
        d = hi - lo
        ok = all(sorted(d)[i] <= 300 for i in range(3))
        fails += 0 if ok else 1
        report.append("| %s | %s | %.0f x %.0f x %.0f | %s |" % (p["name"], p["material"], d[0], d[1], d[2], "yes" if ok else "**NO**"))
    lo = np.min([aabb(p)[0] for p in M.parts if p["group"] != "keepout"], axis=0)
    hi = np.max([aabb(p)[1] for p in M.parts if p["group"] != "keepout"], axis=0)
    no_mast = [p for p in M.parts if p["group"] != "keepout" and p["name"] not in
               ("mast", "uwb_plate", "uwb_anchor", "servo", "pan_buckle", "gopro", "servo9g_adapter")]
    hi2 = np.max([aabb(p)[1] for p in no_mast], axis=0)
    lo2 = np.min([aabb(p)[0] for p in no_mast], axis=0)
    nw = [p for p in no_mast if not p["name"].startswith(("wheel", "axle", "spacer"))]
    wy = np.max([aabb(p)[1][1] for p in nw]) - np.min([aabb(p)[0][1] for p in nw])
    report.append("\nOverall: **%.0f long x %.0f wide x %.0f tall mm** with the camera mast; %.0f x %.0f x %.0f mm with "
                  "the mast removed (backpack). Taking both wheels off (one nyloc each, pulleys stay on the wheels) "
                  "brings the width to %.0f mm.\n" %
                  (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], hi2[0] - lo2[0], hi2[1] - lo2[1], hi2[2] - lo2[2], wy))
    return fails, (hi - lo), (hi2 - lo2)


def plot_views(M, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    try:
        from scipy.spatial import ConvexHull  # noqa
    except Exception:
        ConvexHull = None

    def hull2(pts):
        pts = np.unique(np.round(pts, 3), axis=0)
        if len(pts) < 3:
            return pts
        # monotone chain
        pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]
        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
        lower, upper = [], []
        for p in pts:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(tuple(p))
        for p in pts[::-1]:
            while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(tuple(p))
        return np.array(lower[:-1] + upper[:-1])

    views = [("Side view (from the left)", 0, 2, 1, False), ("Top view", 0, 1, 2, False), ("Front view", 1, 2, 0, True)]
    fig, axs = plt.subplots(1, 3, figsize=(20, 7), gridspec_kw={"width_ratios": [1.3, 1.3, 1.0]})
    for ax, (title, i, j, k, flip) in zip(axs, views):
        items = []
        for p in M.parts:
            col = np.array(p["color"]) / 255.0
            for q in all_adds(p):
                if q["t"] == "sph":
                    continue
                c = prim_corners(q)
                items.append((c[:, k].mean(), c, col, p["group"]))
        items.sort(key=lambda t: t[0] if k != 1 else -t[0])
        for depth, c, col, grp in items:
            h = hull2(c[:, [i, j]] * (np.array([-1, 1]) if flip else 1))
            if len(h) < 3:
                continue
            if grp == "keepout":
                ax.add_patch(Polygon(h, closed=True, fill=False, ec=col, lw=0.6, ls="--"))
            else:
                ax.add_patch(Polygon(h, closed=True, fc=col, ec="k", lw=0.25, alpha=0.92))
        ax.autoscale_view(); ax.set_aspect("equal"); ax.grid(alpha=0.3)
        ax.set_title(title); ax.set_xlabel("mm")
        if k == 1:
            # GoPro lens + lower FOV edge at pan 0 (Wide 4:3)
            L = M.info["camera"]["lens"]
            vh = math.radians(FOV_MODES["Wide 4:3"][1] / 2)
            ax.plot([L[0], L[0] + 400], [L[2], L[2] - 400 * math.tan(vh)], "r--", lw=1)
            ax.plot([L[0], L[0] + 400], [L[2], L[2] + 400 * math.tan(vh)], "r--", lw=1)
            u = M.info["us"]
            for s in (1, -1):
                a = math.radians(u["tilt"] + s * u["half"])
                ax.plot([u["x"], u["x"] + 600], [u["z"], u["z"] + 600 * math.tan(a)], color="tab:blue", lw=0.8)
            ax.axhline(0, color="k", lw=1)
            ax.set_xlim(-300, 420); ax.set_ylim(-10, 360)
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    M = HC.build_model()
    M.parts = [p for p in M.parts if p["group"] != "tools"]      # printed jigs are not part of the robot
    rep = ["# honebot chassis - automated design check\n",
           "Generated by `cad/check_design.py` from `cad/fusion/HonebotChassis/HonebotChassis.py` (same primitives "
           "Fusion builds). Re-run after any parameter change.\n",
           "![views](views.png)\n"]
    summary = {}
    summary["clearance"] = check_clearance(M, rep)
    summary["holes"] = check_holes(M, rep)
    summary["hole spacing"] = check_hole_spacing(M, rep)
    summary["assembly"] = check_removals(M, rep)
    res, pts = check_camera(M, rep)
    summary["camera"] = sum(1 for v in res.values() if v)
    camera_sweeps(rep)
    tilt_sweep(rep, pts)
    rep.append("![camera](camera_views.png)\n")
    plot_camera(M, pts, os.path.join(OUT, "camera_views.png"))
    mass, cg = mass_props(M, rep)
    dyn = dynamics(M, rep, mass, cg)
    drivetrain(rep, mass)
    summary["terrain"] = terrain(M, rep)
    summary["structure (SF < 1.5)"] = structural(M, rep, mass, cg, dyn)
    ultrasonic(M, rep)
    summary["sensor beam"] = beam_obstruction(M, rep)
    summary["routes"] = routes(M, rep)
    summary["print"], env, env2 = printability(M, rep)
    fastener_schedule(M, rep)
    plot_views(M, os.path.join(OUT, "views.png"))
    head = ["## Summary\n", "| check | failures |\n|---|---|"] + ["| %s | %d |" % kv for kv in summary.items()] + [
        "\nMass %.2f kg, CG (%.0f, %.0f, %.0f) mm, %.0f %% on the front axle, sideways tip at %.2f g.\n" %
        (mass, cg[0], cg[1], cg[2], 100 * dyn["fx"], dyn["lat_g"])]
    rep = rep[:3] + head + rep[3:]
    with open(os.path.join(OUT, "check_report.md"), "w") as f:
        f.write("\n".join(rep) + "\n")
    print("\n".join(head))
    print("report:", os.path.join(OUT, "check_report.md"))
    return 0 if not any(summary.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
