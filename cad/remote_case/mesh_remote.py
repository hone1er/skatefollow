"""Mesh export + preview images for the honebot remote case, with a real CAD kernel (OpenCascade).

Rebuilds every part from the primitives in fusion/HonebotRemote/HonebotRemote.py (same boolean order as the Fusion
script), then
  - writes the printed parts as STL in their print orientation to cad/remote_case/stl/ (ready for Bambu Studio),
  - reports per printed part: valid B-rep, non-manifold and open mesh edges (must be 0), area that would need support
    (faces flatter than 40 degrees from horizontal facing down, more than 0.3 mm above the bed; bridges and hole roofs
    count too, so it is an upper bound) and a filament estimate,
  - renders cad/remote_case/report/{assembled,exploded,section,print_layout}.png.

Setup once (any machine with Python 3.10+; same venv as cad/check_mesh.py):
    python3 -m venv ~/occ && ~/occ/bin/pip install cadquery-ocp==8.0.1.1.0 trimesh==5.2.1 numpy matplotlib
    ~/occ/bin/python cad/remote_case/mesh_remote.py
"""
import importlib.util
import math
import os
import sys
import tempfile
import types

import numpy as np
import trimesh
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import (BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCone, BRepPrimAPI_MakeCylinder,
                             BRepPrimAPI_MakeSphere)
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.OCP.collections import List_TopoDS_Shape
from OCP.StlAPI import StlAPI_Writer

HERE = os.path.dirname(os.path.abspath(__file__))
for _n in ("adsk", "adsk.core", "adsk.fusion"):
    sys.modules.setdefault(_n, types.ModuleType(_n))
_spec = importlib.util.spec_from_file_location("HonebotRemote", os.path.join(HERE, "fusion", "HonebotRemote",
                                                                             "HonebotRemote.py"))
HR = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(HR)

FUZZ = float(os.environ.get("CHECK_MESH_FUZZ", "1e-4"))
OVERHANG_DEG = 50.0
DENS = {"PETG": 1.27, "PLA": 1.24}
PROFILES = (("3 walls, 15 % gyroid", 3.0, 0.15), ("4 walls, 25 % gyroid", 4.0, 0.25))


def _op(cls, a, b):
    op = cls()
    la, lb = List_TopoDS_Shape(), List_TopoDS_Shape()
    la.Append(a)
    lb.Append(b)
    op.SetArguments(la)
    op.SetTools(lb)
    op.SetFuzzyValue(FUZZ)
    op.Build()
    return op.Shape()


def _pnt(v):
    return gp_Pnt(*map(float, v))


def prim(p):
    t = p["t"]
    if t == "box":
        u, v = np.array(p["u"], float), np.array(p["v"], float)
        u /= np.linalg.norm(u)
        w = np.cross(u, v)
        w /= np.linalg.norm(w)
        lu, lv, lw = p["s"]
        v = np.cross(w, u)
        corner = np.array(p["c"], float) - u * lu / 2 - v * lv / 2 - w * lw / 2
        return BRepPrimAPI_MakeBox(gp_Ax2(_pnt(corner), gp_Dir(*w), gp_Dir(*u)), lu, lv, lw).Shape()
    if t == "cyl":
        a, b = np.array(p["a"], float), np.array(p["b"], float)
        h = np.linalg.norm(b - a)
        ax = gp_Ax2(_pnt(a), gp_Dir(*((b - a) / h)))
        if "r2" in p:
            return BRepPrimAPI_MakeCone(ax, p["r"], p["r2"], h).Shape()
        return BRepPrimAPI_MakeCylinder(ax, p["r"], h).Shape()
    if t == "sph":
        return BRepPrimAPI_MakeSphere(_pnt(p["c"]), p["r"]).Shape()
    if t == "diff":
        s = prim(p["a"])
        for q in p["b"]:
            s = _op(BRepAlgoAPI_Cut, s, prim(q))
        return s
    raise ValueError(t)


def _merge(lst):
    s = None
    for q in lst:
        b = prim(q)
        s = b if s is None else _op(BRepAlgoAPI_Fuse, s, b)
    return s


def build(part):
    """Same order as the Fusion builder: body = U(A_i - C_i) U A - C."""
    body = None
    for adds, cuts in part.get("pre", []):
        sb = _merge(adds)
        if cuts:
            sb = _op(BRepAlgoAPI_Cut, sb, _merge(cuts))
        body = sb if body is None else _op(BRepAlgoAPI_Fuse, body, sb)
    if part["adds"]:
        ab = _merge(part["adds"])
        body = ab if body is None else _op(BRepAlgoAPI_Fuse, body, ab)
    if part["cuts"]:
        body = _op(BRepAlgoAPI_Cut, body, _merge(part["cuts"]))
    return body


def _rot(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def to_mesh(shape, defl=0.05):
    BRepMesh_IncrementalMesh(shape, defl, False, 0.3, True)
    fd, path = tempfile.mkstemp(suffix=".stl")
    os.close(fd)
    StlAPI_Writer().Write(shape, path)
    m = trimesh.load(path, process=True, force="mesh")
    os.remove(path)
    return m


def analyse(part, shape, out_dir):
    valid = BRepCheck_Analyzer(shape).IsValid()
    m = to_mesh(shape)
    _, cnt = np.unique(np.sort(m.edges, axis=1), axis=0, return_counts=True)
    nm, open_ = int(np.sum(cnt > 2)), int(np.sum(cnt == 1))
    R = _rot(*part["orient"])
    pm = m.copy()
    pm.vertices = pm.vertices @ R.T
    pm.vertices -= [pm.vertices[:, 0].min(), pm.vertices[:, 1].min(), pm.vertices[:, 2].min()]
    pm.export(os.path.join(out_dir, part["name"] + ".stl"))
    n = pm.face_normals
    z = pm.triangles_center[:, 2]
    over = (n[:, 2] < -math.sin(math.radians(OVERHANG_DEG))) & (z > 0.3)
    over_area = float(pm.area_faces[over].sum())
    mat = part.get("material") or "PETG"
    grams = []
    for _, walls, infill in PROFILES:
        t = walls * 0.42
        shell_v = min(m.area * t, abs(m.volume))
        grams.append(DENS[mat] * (shell_v + (abs(m.volume) - shell_v) * infill) / 1000.0)
    ext = pm.vertices.max(axis=0)
    return valid, nm, open_, m.is_watertight, over_area, abs(m.volume) / 1000.0, grams, ext, pm


# ------------------------------------------------------------------------------------------------------------
# preview rendering (matplotlib, one depth-sorted collection per image)
# ------------------------------------------------------------------------------------------------------------
def fine(m, edge=2.0):
    """Split long triangles so matplotlib's per-triangle depth sort draws the surfaces in the right order."""
    v, f = trimesh.remesh.subdivide_to_size(m.vertices, m.faces, max_edge=edge, max_iter=12)
    return trimesh.Trimesh(v, f, process=False)


def render(meshes, path, elev, azim, title, size=(11, 7), xlim=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    tris, cols = [], []
    light = np.array([0.4, -0.5, 0.75])
    light /= np.linalg.norm(light)
    for m, rgb, alpha in meshes:
        n = m.face_normals
        shade = 0.45 + 0.55 * np.clip(n @ light, 0, 1)
        c = np.clip(np.outer(shade, np.array(rgb) / 255.0), 0, 1)
        tris.append(m.triangles)
        cols.append(np.hstack([c, np.full((len(c), 1), alpha)]))
    T = np.concatenate(tris)
    C = np.concatenate(cols)
    fig = plt.figure(figsize=size, dpi=110)
    ax = fig.add_subplot(111, projection="3d")
    pc = Poly3DCollection(T, facecolors=C, edgecolors="none", linewidths=0)
    ax.add_collection3d(pc)
    allv = T.reshape(-1, 3)
    lo, hi = allv.min(axis=0), allv.max(axis=0)
    c = (lo + hi) / 2
    r = (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ax.set_title(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def main():
    M = HR.build_model()
    D = M.info
    stl_dir = os.path.join(HERE, "stl")
    rep_dir = os.path.join(HERE, "report")
    os.makedirs(stl_dir, exist_ok=True)
    os.makedirs(rep_dir, exist_ok=True)
    shapes = {p["name"]: build(p) for p in M.parts if p["group"] in ("printed", "hardware")}
    bad = 0
    lines = ["| Part | B-rep | Non-manifold edges | Open edges | Support area (mm2) | Solid (cm3) | Size on the bed (mm) | "
             + " / ".join(p[0] for p in PROFILES) + " (g) |", "|---|---|---|---|---|---|---|---|"]
    print("%-12s %-5s %3s %4s %8s %7s %-20s %s" % ("part", "brep", "nm", "open", "support", "cm3", "bed size", "grams"))
    printed_meshes = {}
    for p in [q for q in M.parts if q["group"] == "printed"]:
        valid, nm, op, wt, ov, vol, grams, ext, pm = analyse(p, shapes[p["name"]], stl_dir)
        printed_meshes[p["name"]] = pm
        bad += (nm > 0) + (op > 0) + (not valid)
        size = "%.0f x %.0f x %.0f" % tuple(ext)
        print("%-12s %-5s %3d %4d %8.0f %7.1f %-20s %s" % (p["name"], "ok" if valid else "BAD", nm, op, ov, vol, size,
                                                          " / ".join("%.0f" % g for g in grams)), flush=True)
        lines.append("| %s | %s | %d | %d | %.0f | %.1f | %s | %s |" % (p["name"], "ok" if valid else "BAD", nm, op, ov,
                                                                       vol, size, " / ".join("%.0f" % g for g in grams)))
    with open(os.path.join(rep_dir, "mesh_report.md"), "w") as f:
        f.write("# honebot remote case: mesh report\n\nGenerated by `cad/remote_case/mesh_remote.py`. STL files are in "
                "`cad/remote_case/stl/`, already in print orientation.\n\n" + "\n".join(lines) + "\n")

    # previews
    meshes = {n: fine(to_mesh(s, 0.08)) for n, s in shapes.items()}
    col = {p["name"]: p["color"] for p in M.parts}
    mount = {p["name"]: p["mount"] for p in M.parts}

    def mset(names, lift=None, alpha=None):
        out = []
        for n in names:
            m = meshes[n].copy()
            if lift:
                m.vertices = m.vertices + [0, 0, lift.get(mount[n], 0.0)]
            out.append((m, col[n], (alpha or {}).get(n, 1.0)))
        return out

    names = list(meshes)
    render(mset(names), os.path.join(rep_dir, "assembled.png"), 24, -128,
           "honebot remote: %.1f x %.0f x %.1f mm (rear end with the USB ports at the left)" % (D["L"], D["W"], D["H"]))
    render(mset(names, lift={"lid": 45.0, "carrier": 18.0}), os.path.join(rep_dir, "exploded.png"), 20, -128,
           "exploded: lid (orange) up, joystick carrier (teal) + joystick up; base with ESP32, charger, battery")
    # section at y = 0 (keep the -y half), looking from +y
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut as Cut
    half = HR.box(-50, 250, -100, 0.0, -50, 100)
    cut_tool = prim(half)
    sec = []
    for n in names:
        try:
            s = _op(Cut, shapes[n], cut_tool)
            m = fine(to_mesh(s, 0.08))
            if len(m.faces):
                sec.append((m, col[n], 1.0))
        except Exception:
            pass
    render(sec, os.path.join(rep_dir, "section.png"), 4, -90, "section on the centre line: USB end at the left, joystick at the right",
           size=(12, 6))
    # bottom view
    render(mset(names), os.path.join(rep_dir, "bottom.png"), -60, -60, "underside: flush rocker switch, 4 lid screws")
    # print layout
    pl = []
    x_off = 0.0
    for n, pm in printed_meshes.items():
        m = fine(pm)
        m.vertices = m.vertices + [x_off, 0, 0]
        x_off += m.vertices[:, 0].max() - x_off + 10
        pl.append((m, col[n], 1.0))
    render(pl, os.path.join(rep_dir, "print_layout.png"), 35, -60, "print orientation (no supports): base, lid (top face "
                                                                   "down), carrier", size=(12, 6))
    print("STL: %s   previews + mesh_report.md: %s" % (stl_dir, rep_dir))
    return bad


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
