"""Find non-manifold edges in the printed parts before Fusion / the slicer does.

A non-manifold edge appears where two pieces of material (or two cut-outs) meet only along a line, e.g. a hole
that grazes the edge of a boss and leaves a knife edge. Method: build a grid from every face coordinate of the
part's primitives (plus 1 mm splits), classify each cell centre as inside/outside with the same geometry code the
checker uses, and report every grid edge whose four surrounding cells form a checkerboard (2 diagonal in, 2 out).

Curved or slanted surfaces that cross a grid edge exactly can give a rare false hit (one isolated cell); a real
defect shows up as a run of cells along the edge or as a symmetric pair on both sides of the robot.

    python3 cad/check_manifold.py            # all printed parts (~3 min)
    python3 cad/check_manifold.py tub deck   # selected parts
"""
import os
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_design as C  # noqa: E402


def _coords(part):
    coords = [set(), set(), set()]

    def walk(p):
        if p["t"] == "box":
            c, s, u, v = p["c"], p["s"], p["u"], p["v"]
            w = np.cross(u, v)
            for k in range(3):
                h = (abs(u[k]) * s[0] + abs(v[k]) * s[1] + abs(w[k]) * s[2]) / 2
                coords[k].update([round(c[k] - h, 6), round(c[k] + h, 6)])
        elif p["t"] == "cyl":
            a, b, r = np.array(p["a"]), np.array(p["b"]), p["r"]
            d = b - a
            axial = np.count_nonzero(np.abs(d) > 1e-9) == 1
            for k in range(3):
                if axial and abs(d[k]) > 1e-9:
                    coords[k].update([round(a[k], 6), round(b[k], 6)])
                else:
                    coords[k].update([round(min(a[k], b[k]) - r, 6), round(max(a[k], b[k]) + r, 6)])
        elif p["t"] == "diff":
            walk(p["a"])
            for q in p["b"]:
                walk(q)
        elif p["t"] == "sph":
            for k in range(3):
                coords[k].update([round(p["c"][k] - p["r"], 6), round(p["c"][k] + p["r"], 6)])

    for p in part["adds"] + part["cuts"] + [q for a, c in part.get("pre", []) for q in a + c]:
        walk(p)
    return coords


def scan(part):
    lo, hi = C.aabb(part)
    edges = []
    for k, cs in enumerate(_coords(part)):
        arr = sorted(x for x in cs if lo[k] - 1 <= x <= hi[k] + 1)
        out = [arr[0]]
        for a, b in zip(arr[:-1], arr[1:]):
            if b - a > 1e-9:
                out += list(np.linspace(a, b, int(np.ceil((b - a) / 1.0)) + 1)[1:])
        edges.append(np.array(out))
    cen = [(e[:-1] + e[1:]) / 2 for e in edges]
    X, Y, Z = np.meshgrid(*cen, indexing="ij")
    pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
    S = np.zeros(len(pts), bool)
    for i in range(0, len(pts), 400000):
        S[i:i + 400000] = C.inside_part(part, pts[i:i + 400000])
    S = S.reshape(X.shape)
    hits = defaultdict(list)
    for k in range(3):
        i, j = [a for a in range(3) if a != k]
        Sm = np.moveaxis(S, (i, j, k), (0, 1, 2))
        a, b, c, d = Sm[:-1, :-1], Sm[1:, :-1], Sm[:-1, 1:], Sm[1:, 1:]
        for p, q, r in np.argwhere((a & d & ~b & ~c) | (b & c & ~a & ~d)):
            hits[(("x", "y", "z")[k], round(edges[i][p + 1], 2), round(edges[j][q + 1], 2))].append(round(cen[k][r], 2))
    return hits


def main(names):
    M = C.HC.build_model()
    parts = [M.get(n) for n in names] if names else [p for p in M.parts if p["group"] in ("printed", "prototype")
                                                     and not p["name"].startswith("proto_belt")]
    bad = 0
    for p in parts:
        hits = scan(p)
        print("%-22s %s" % (p["name"], "ok" if not hits else "%d edge(s)" % len(hits)))
        for (ax, u, v), along in sorted(hits.items()):
            print("    edge along %s at (%s, %s), %s cell(s) near %s" % (ax, u, v, len(along), along[0]))
        bad += len(hits)
    return bad


if __name__ == "__main__":
    sys.exit(1 if main(sys.argv[1:]) else 0)
