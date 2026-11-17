"""Mesh-level check of every printed part with a real CAD kernel (OpenCascade), the way a slicer sees it.

For each part: rebuild the solid from the same primitives and boolean order as the Fusion script, export an STL
(cad/stl/, git-ignored, ready for Bambu Studio), then report
  - non-manifold edges (shared by more than 2 triangles) and open edges: must both be 0,
  - overhang area that needs support in the recommended print orientation (faces more than 50 deg from vertical,
    i.e. flatter than 40 deg, more than 0.3 mm above the bed; Bambu Studio's default threshold is 60 deg from
    vertical. Bridges and the roofs of horizontal holes are counted too, so treat it as an upper bound),
  - filament estimate for two slicer profiles.

Booleans run with a small fuzzy tolerance by default; CHECK_MESH_FUZZ=0 gives strict exact booleans (closest to
Fusion: a torus-cornered version that passed the fuzzy run left loose donuts in Fusion and also failed strict).
The tub exported on 2026-11-16 shows the same 4 non-manifold edges Bambu Studio reported; the fixed tub shows 0.

Setup once (any machine with Python 3.10+):
    python3 -m venv ~/occ && ~/occ/bin/pip install cadquery-ocp==8.0.1.1.0 trimesh==5.2.1 numpy
    ~/occ/bin/python cad/check_mesh.py            # all printed parts, ~1 min
    ~/occ/bin/python cad/check_mesh.py tub shell  # selected parts
"""
import importlib.util
import math
import os
import sys
import types

import numpy as np
import trimesh
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import (BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeSphere,
                             BRepPrimAPI_MakeTorus)
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCP.OCP.collections import List_TopoDS_Shape
from OCP.StlAPI import StlAPI_Writer

HERE = os.path.dirname(os.path.abspath(__file__))
for _n in ("adsk", "adsk.core", "adsk.fusion"):
    sys.modules.setdefault(_n, types.ModuleType(_n))
_spec = importlib.util.spec_from_file_location("HC", os.path.join(HERE, "fusion", "HonebotChassis", "HonebotChassis.py"))
HC = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(HC)

FUZZ = float(os.environ.get("CHECK_MESH_FUZZ", "1e-4"))   # 0 = strict exact booleans
OVERHANG_DEG = 50.0      # faces more than this from vertical need support (Bambu default threshold: 60 deg)
DENS = {"PAHT-CF": 1.20, "PETG": 1.27, "TPU": 1.21}
PROFILES = (("docs v1: 5-6 walls, 40 % infill", 5.5, 0.40), ("recommended: 3 walls, 15 % gyroid", 3.0, 0.15))
# print orientation: rotation (axis, degrees) that puts the recommended face on the bed
ORIENT = {
    "shell": ("x", 180), "bumper_door": ("z", 0), "motor_plate_L": ("x", -90), "motor_plate_R": ("x", 90),
    "wheel_pulley_FL": ("x", -90), "wheel_pulley_RL": ("x", -90), "wheel_pulley_FR": ("x", 90),
    "wheel_pulley_RR": ("x", 90), "motor_pulley_L": ("x", 90), "motor_pulley_R": ("x", -90), "mast": ("x", 90), "us_bracket": ("x", 90),
    "servo9g_adapter": ("x", 180), "esp_ext_plate": ("x", 180),
}


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
        w = np.cross(u, v)
        lu, lv, lw = p["s"]
        corner = np.array(p["c"], float) - u * lu / 2 - v * lv / 2 - w * lw / 2
        return BRepPrimAPI_MakeBox(gp_Ax2(_pnt(corner), gp_Dir(*w), gp_Dir(*u)), lu, lv, lw).Shape()
    if t == "cyl":
        a, b = np.array(p["a"], float), np.array(p["b"], float)
        h = np.linalg.norm(b - a)
        return BRepPrimAPI_MakeCylinder(gp_Ax2(_pnt(a), gp_Dir(*((b - a) / h))), p["r"], h).Shape()
    if t == "sph":
        return BRepPrimAPI_MakeSphere(_pnt(p["c"]), p["r"]).Shape()
    if t == "tor":
        return BRepPrimAPI_MakeTorus(gp_Ax2(_pnt(p["c"]), gp_Dir(0, 0, 1)), p["R"], p["r"]).Shape()
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


def analyse(part, out_dir):
    shape = build(part)
    valid = BRepCheck_Analyzer(shape).IsValid()
    BRepMesh_IncrementalMesh(shape, 0.05, False, 0.3, True)
    path = os.path.join(out_dir, part["name"] + ".stl")
    StlAPI_Writer().Write(shape, path)
    m = trimesh.load(path, process=True, force="mesh")
    _, cnt = np.unique(np.sort(m.edges, axis=1), axis=0, return_counts=True)
    nm, open_ = int(np.sum(cnt > 2)), int(np.sum(cnt == 1))
    R = _rot(*ORIENT.get(part["name"], ("z", 0)))
    n = m.face_normals @ R.T
    z = (m.triangles_center @ R.T)[:, 2]
    zmin = (m.vertices @ R.T)[:, 2].min()
    over = (n[:, 2] < -math.sin(math.radians(OVERHANG_DEG))) & (z > zmin + 0.3)
    over_area = float(m.area_faces[over].sum())
    mat = next((k for k in DENS if k in (part.get("material") or "")), "PETG")
    grams = []
    for _, walls, infill in PROFILES:
        t = walls * 0.42
        shell_v = min(m.area * t, abs(m.volume))
        grams.append(DENS[mat] * (shell_v + (abs(m.volume) - shell_v) * infill) / 1000.0)
    return valid, nm, open_, m.is_watertight, over_area, abs(m.volume) / 1000.0, mat, grams


def main(names):
    M = HC.build_model()
    parts = [M.get(n) for n in names] if names else [p for p in M.parts if p["group"] in ("printed", "prototype")
                                                     and not p.get("hidden")]
    out_dir = os.path.join(HERE, "stl")
    os.makedirs(out_dir, exist_ok=True)
    bad = 0
    tot = [0.0, 0.0]
    print("%-18s %-6s %4s %5s %-5s %10s %9s %-8s %s" % ("part", "brep", "nm", "open", "tight", "support mm2", "solid cm3",
                                                     "material", " / ".join(p[0] for p in PROFILES) + " (g)"))
    for p in parts:
        valid, nm, op, wt, ov, vol, mat, grams = analyse(p, out_dir)
        bad += (nm > 0) + (op > 0) + (not valid)
        tot = [a + b for a, b in zip(tot, grams)]
        print("%-18s %-6s %4d %5d %-5s %10.0f %9.1f %-8s %s" % (p["name"], "ok" if valid else "BAD", nm, op, wt, ov, vol, mat,
                                                           " / ".join("%.0f" % g for g in grams)), flush=True)
    print("total filament: " + " / ".join("%.0f g" % g for g in tot) + "   STL files: " + out_dir)
    return bad


if __name__ == "__main__":
    sys.exit(1 if main(sys.argv[1:]) else 0)
