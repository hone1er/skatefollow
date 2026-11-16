#!/usr/bin/env python3
"""Offline design checks for the honebot chassis (no Fusion needed).

Imports cad/fusion/HonebotChassis/HonebotChassis.py (the single geometry source) and checks:
  clearances (static + moving parts, keep-outs), mounting-hole audit, GoPro field of view over the pan range,
  mass / CG, tip-over and braking, stub axle / caster / mast / belt loads, HC-SR04 ground return, wiring routes,
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


def inside_prim(p, pts, grow=0.0):
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
    for p in part["adds"]:
        m |= inside_prim(p, pts, grow)
    if use_cuts and part["cuts"]:
        c = np.zeros(len(pts), bool)
        for p in part["cuts"]:
            c |= inside_prim(p, pts, -grow)
        m &= ~c
    return m


def prim_corners(p):
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


def aabb(part):
    pts = np.vstack([prim_corners(p) for p in part["adds"]])
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
                if len(pts) and inside_part(big, pts, grow=-0.3).any():
                    n = int(inside_part(big, pts, grow=-0.3).sum())
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
            gap = None
            for g in (0.0, 0.5, 1.0, 2.0, 3.0, 5.0, 8.0):
                if inside_part(big, pts, grow=g, use_cuts=big["group"] != "keepout").any():
                    gap = g
                    break
            if gap is None:
                continue
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
                  "0.5 mm static, 3 mm when either part moves or is a swept keep-out volume (caster swivel, GoPro pan, "
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
                A, B = cores[i], cores[j]
                if np.any(A.min(0) - 25 > B.max(0)) or np.any(B.min(0) - 25 > A.max(0)):
                    continue
                d = np.min(np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2)) - hs[i]["d"] / 2 - hs[j]["d"] / 2
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
        pts = np.vstack([sample(M.get(n), 1.5) for n in grp if len(sample(M.get(n), 1.5))])
        stay = [p for p in M.parts if p["group"] != "keepout" and p["name"] not in grp and p["name"] not in before
                and p["name"] not in fast]
        hit = None
        for t in np.arange(2.0, travel + 0.1, 2.0):
            q = pts + t * d
            for p in stay:
                if inside_part(p, q, grow=-0.2).any():
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
    P = M.info["P"]; c = M.info["caster"]
    tw = P["wheel_inner_y"] + P["wheel_w"] / 2
    xc_tr = c["pivot_x"] - P["caster_trail"]
    L = -xc_tr
    h = cg[2]
    report.append("## 5. Stability, traction, braking\n")
    # lateral tip: triangle (0, +tw) (0, -tw) caster contact on the trail circle (worst position)
    worst = 1e9
    for ang in np.linspace(0, 2 * np.pi, 72, endpoint=False):
        cc = np.array([c["pivot_x"] + P["caster_trail"] * math.cos(ang), P["caster_trail"] * math.sin(ang)])
        for wpt in (np.array([0.0, tw]), np.array([0.0, -tw])):
            e = cc - wpt; q = cg[:2] - wpt
            dist = abs(e[0] * q[1] - e[1] * q[0]) / np.linalg.norm(e)
            worst = min(worst, dist)
    lat_g = worst / h
    fx = (cg[0] - xc_tr) / L          # fraction of weight on the drive wheels (static)
    pitch_g = -cg[0] / h              # tip forward over the drive axle (CG behind it by -cg_x)
    report.append("- Support triangle: drive wheel contacts at y = +/-%.0f mm, caster contact %.0f mm behind the axle "
                  "(trail %.0f mm, worst swivel angle used for tipping).\n" % (tw, L, P["caster_trail"]))
    report.append("- Static load on the drive wheels: **%.0f %%** (traction and braking come only from them).\n" % (100 * fx))
    report.append("- Sideways tip-over needs **%.2f g** (urethane on asphalt slides at ~0.7-0.9 g, so it slides first%s).\n" %
                  (lat_g, "" if lat_g > 0.9 else " ONLY JUST - keep heavy parts low"))
    report.append("- Forward pitch-over about the drive axle under braking starts at **%.2f g**. The bumper skids "
                  "(bottom z = 16 mm at x = 84 mm) touch down after %.1f deg of pitch, so the robot then rides on the "
                  "skids instead of flipping. Software should still cap deceleration at ~0.5 g (see docs).\n" %
                  (pitch_g, math.degrees(math.atan2(16.0, 84.0))))
    rows = []
    for mu in (0.8, 0.6):
        a_fd = mu * 9.81 * (cg[0] - xc_tr) / (L - mu * h)          # drive front (this design)
        a_fd = min(a_fd, pitch_g * 9.81)
        a_rd = mu * 9.81 * (cg[0] - xc_tr) / (L + mu * h)          # same robot with the drive axle at the rear
        for v_mph in (8, 12, 15):
            v = v_mph * 0.44704
            d_fd = v * 0.25 + v * v / (2 * a_fd)
            d_rd = v * 0.25 + v * v / (2 * a_rd)
            rows.append((mu, v_mph, a_fd / 9.81, d_fd, a_rd / 9.81, d_rd))
    report.append("\nStopping distance = 0.25 s reaction (gate + ESC ramp) + v^2 / 2a, a limited by drive-wheel traction "
                  "(weight transfer included) and by pitch-over:\n")
    report.append("| mu | speed (mph) | THIS layout: decel (g) | stop (m) | if drive wheels were at the rear: decel (g) | stop (m) |\n|---|---|---|---|---|---|")
    for r in rows:
        report.append("| %.1f | %d | %.2f | %.1f | %.2f | %.1f |" % r)
    report.append("\nThe follow distance must exceed the stopping distance at the current speed if the skater can stop "
                  "instantly (a fall). Recommended `follow_distance` >= the 'stop' column, e.g. speed-dependent "
                  "d = 1.5 m + 0.35 s x v + v^2 / (2 x 0.45 g).\n")
    return {"lat_g": lat_g, "fx": fx, "pitch_g": pitch_g, "rows": rows}


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


def structural(M, report, mass, cg, dyn):
    P = M.info["P"]
    report.append("## 7. Structural checks (hand calculations)\n")
    g = 9.81
    fx = dyn["fx"]
    Fw = mass * g * fx / 2                     # static per drive wheel
    k_imp = 4.0                                # impact factor: curb drop / pothole on urethane
    lever = (P["wheel_inner_y"] + P["wheel_w"] / 2) - P["tub_hw"]   # wall outer face -> wheel centre
    lat = 0.9 * Fw * 2                          # worst lateral (sliding) force on one wheel ~ 0.9 x its share x 2
    M_b = (k_imp * Fw) * lever + lat * P["wheel_d"] / 2
    d_minor = 6.47
    Zb = math.pi * d_minor ** 3 / 32
    sig = M_b / Zb
    rows = [("M8 stub axle, 4 g bump (class 8.8: 640 MPa / 12.9: 1080 MPa)", "%.0f N x4 at %.0f mm + %.0f N side at r" % (Fw, lever, lat),
             "%.0f MPa" % sig, "%.1f / %.1f" % (640 / sig, 1080 / sig))]
    M10 = (10 * Fw) * lever + lat * P["wheel_d"] / 2
    s10 = M10 / Zb
    rows.append(("M8 stub axle, 10 g kerb strike", "%.0f N x10 at %.0f mm" % (Fw, lever), "%.0f MPa" % s10,
                 "%.1f / %.1f" % (640 / s10, 1080 / s10)))
    # boss bearing (PAHT-CF, across layers ~ compressive 40 MPa conservative)
    Lb = 26.0 + 4.0
    sig_b = 6 * M_b / (8.0 * Lb ** 2) + (k_imp * Fw) / (8.0 * Lb)
    rows.append(("axle boss bearing in PAHT-CF (allow 40 MPa)", "moment couple over %.0f mm boss" % Lb, "%.0f MPa" % sig_b, 40 / sig_b))
    # caster
    Fc = mass * g * (1 - fx)
    Mc = k_imp * Fc * P["caster_trail"]
    Fbr = Mc / 24.0 + k_imp * Fc / 2
    rows.append(("608 swivel bearings (C0 ~1370 N)", "trail moment over 24 mm spacing", "%.0f N" % Fbr, 1370 / Fbr))
    t = 7.5; b = P["caster_w"] + 18
    sig_c = 6 * Mc / (b * t ** 2)
    rows.append(("caster fork crown, PAHT-CF (allow 25 MPa in Z)", "%.0f N x4 x %.0f mm trail" % (Fc, P["caster_trail"]), "%.1f MPa" % sig_c, 25 / sig_c))
    # belt: traction-limited wheel torque
    mu = 0.9
    Tw = mu * (Fw * 1.3) * P["wheel_d"] / 2000     # N m, with braking weight transfer ~ +30 %
    Fbelt = Tw / (P["pul_w_pd"] / 2000)
    rows.append(("HTD-3M 9 mm belt effective pull", "traction-limited wheel torque %.2f N m" % Tw, "%.0f N" % Fbelt,
                 None))
    # mast: cantilever from the lid top
    mi = M.info["mast"]; cam = M.info["camera"]
    E = 3.5e9
    so, si = mi["sq"] / 1000, (mi["sq"] - 2 * mi["wall"]) / 1000
    I = (so ** 4 - si ** 4) / 12
    masses = [(P["gopro_mass"] + 0.015, cam["centre"][2]), (0.055 + 0.03, mi["z_head"] + 20), (0.03 + 0.02, mi["z0"] + 40)]
    z0 = mi["z0"]
    # Rayleigh with the static deflection shape of point loads
    def defl(zq, zl, F):
        a = (zl - z0) / 1000; x = (zq - z0) / 1000
        if x <= a:
            return F * x * x * (3 * a - x) / (6 * E * I)
        return F * a * a * (3 * x - a) / (6 * E * I)
    ys = [sum(defl(zq, zl, mm * g) for mm, zl in masses) for _, zq in masses]
    num = sum(mm * g * y for (mm, _), y in zip(masses, ys)); den = sum(mm * y * y for (mm, _), y in zip(masses, ys))
    f1 = math.sqrt(num / den) / (2 * math.pi) if den > 0 else float("nan")   # Rayleigh: w^2 = sum(F y) / sum(m y^2)
    Mroot = sum(mm * g * 3.0 * (zl - z0) / 1000 for mm, zl in masses)
    sig_m = Mroot / (I / (so / 2)) / 1e6
    rows.append(("mast first bending mode (E 3.5 GPa)", "GoPro + servo + head + UWB on a %.0f mm tube" % mi["sq"], "%.0f Hz" % f1, None))
    rows.append(("mast root stress at 3 g sideways (allow 40 MPa)", "", "%.1f MPa" % sig_m, 40 / sig_m))
    report.append("| item | load case | result | safety factor |\n|---|---|---|---|")
    for r in rows:
        sf = r[3] if isinstance(r[3], str) else (("%.1f" % r[3]) if r[3] else "-")
        report.append("| %s | %s | %s | %s |" % (r[0], r[1], r[2], sf))
    report.append("\nNotes: the axle boss figure assumes the wheel load reaches the bolt through the 14 mm spacer and the boss "
                  "and wall pad act together; the bridge beam ties both bosses so they do not rotate the side walls. "
                  "Belt: HTD-3M 9 mm belts are sold for loads well above this pull (check your supplier's rating); the "
                  "48T printed pulley carries ~%d teeth in mesh. Mast mode should stay above ~30 Hz so road vibration "
                  "does not excite it; HyperSmooth handles the rest.\n" % int(round(48 * 0.45)))
    return f1


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
            for q in p["adds"]:
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
    structural(M, rep, mass, cg, dyn)
    ultrasonic(M, rep)
    summary["routes"] = routes(M, rep)
    summary["print"], env, env2 = printability(M, rep)
    plot_views(M, os.path.join(OUT, "views.png"))
    head = ["## Summary\n", "| check | failures |\n|---|---|"] + ["| %s | %d |" % kv for kv in summary.items()] + [
        "\nMass %.2f kg, CG (%.0f, %.0f, %.0f) mm, %.0f %% on the drive wheels, sideways tip at %.2f g.\n" %
        (mass, cg[0], cg[1], cg[2], 100 * dyn["fx"], dyn["lat_g"])]
    rep = rep[:3] + head + rep[3:]
    with open(os.path.join(OUT, "check_report.md"), "w") as f:
        f.write("\n".join(rep) + "\n")
    print("\n".join(head))
    print("report:", os.path.join(OUT, "check_report.md"))
    return 0 if not any(summary.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
