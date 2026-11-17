"""Flags a join that lands on a region an earlier cut on the same body removed (Fusion would refill the cut)."""
import sys, math
src = open(sys.argv[2]).read(); S = sys.argv[1]
top = src[:src.index("ns = {}; exec")]
g = {'S': S, '__name__': 'mock'}; exec(top, g)
ns = {}; exec(compile(open(S).read(), S, 'exec'), ns)
B = ns['Builder']; log = []
def pts(o, acc):
    if isinstance(o, (tuple, list)):
        if len(o) == 2 and all(isinstance(v, (int, float)) for v in o): acc.append(o); return
        for x in o: pts(x, acc)
def shapes(paths, circles, rects):
    out = []
    for pth in list(paths) + [ns['rrect_segs'](r, 0.0) for r in rects]:
        a = []; pts(pth, a)
        if a: out.append((min(q[0] for q in a), min(q[1] for q in a), max(q[0] for q in a), max(q[1] for q in a)))
    for c in circles: out.append((c[0] - c[2], c[1] - c[2], c[0] + c[2], c[1] + c[2]))
    return out
def wrap(kind, orig):
    def f(self, op, name, *a, paths=(), circles=(), body=None, rects=()):
        r = orig(self, op, name, *a, paths=paths, circles=circles, body=body, rects=rects)
        tgt = r if op == 'new' else body
        if kind == 'xy': ax, a0, a1 = 'z', a[0], a[1]
        else: ax, a0, a1 = a[0], a[1], a[2]
        log.append((op, name, ax, min(a0, a1), max(a0, a1), (paths, circles, rects), id(tgt)))
        return r
    return f
B.xy = wrap('xy', B.xy); B.side = wrap('side', B.side)
def inside(q, kind, geo):
    if kind == 'c': return math.dist(q, geo[:2]) < geo[2] - 1e-6
    poly = geo; n = len(poly); c = False
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > q[1]) != (y2 > q[1]) and q[0] < (x2 - x1) * (q[1] - y1) / (y2 - y1) + x1: c = not c
    return c
def geos(paths, circles, rects):
    out = []
    for pth in list(paths) + [ns['rrect_segs'](r, 0.0) for r in rects]:
        a = []; pts(pth, a)
        if len(a) >= 3: out.append(('p', a))
    for c in circles: out.append(('c', c))
    return out
def samples(kind, geo):
    if kind == 'c': return [geo[:2]]
    xs = [q[0] for q in geo]; ys = [q[1] for q in geo]
    return [((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)]
hits = set()
for combo in [dict()]:
    p = dict(ns['P']); p.update(combo); log.clear()
    ns['build'](g['root'], p)
    L = [(op, nm, ax, a0, a1, geos(*sh), bid) for op, nm, ax, a0, a1, sh, bid in log]
    for i, (op, nm, ax, a0, a1, gs, bid) in enumerate(L):
        if op != 'cut': continue
        for kind, geo in gs:
            for q in samples(kind, geo):
                if not inside(q, kind, geo): continue
                for zf in (0.25, 0.5, 0.75):
                    z = a0 + (a1 - a0) * zf
                    state = False
                    for op2, nm2, ax2, b0, b1, gs2, bid2 in L[i + 1:]:
                        if bid2 != bid or ax2 != ax or not (b0 < z < b1): continue
                        if any(inside(q, k2, g2) for k2, g2 in gs2):
                            state = (op2 == 'join') if op2 in ('join', 'cut') else state
                            last = nm2
                    if state: hits.add((nm, last, round(q[0], 1), round(q[1], 1), round(z, 1)))
for h in sorted(hits): print(f'  cut "{h[0]}" refilled by "{h[1]}" at ({h[2]}, {h[3]}) depth {h[4]}')
print(f'{len(hits)} refilled cut points')


# ---------------------------------------------------------------------------
# 2) every join must touch solid that already exists (a floating join prints as a loose part)
# 3) hardware and bought parts must not overlap printed parts (bolts in holes, nuts in pockets, boards on standoffs)
# ---------------------------------------------------------------------------
def to3(ax, u, v, n):
    return {'z': (u, v, n), 'y': (u, n, v), 'x': (n, u, v)}[ax]


def from3(ax, q):
    x, y, z = q
    return {'z': ((x, y), z), 'y': ((x, z), y), 'x': ((y, z), x)}[ax]


def solid_at(ops, q):
    state = False
    for op, nm, ax, a0, a1, gs, bid in ops:
        (u, v), n = from3(ax, q)
        if not (a0 < n < a1):
            continue
        if any(inside((u, v), k, g_) for k, g_ in gs):
            state = op in ('new', 'join')
    return state


def probe_out(kind, geo, ax, a0, a1, d=0.3):
    """Points just inside and just outside a shape's outline (every ~3 mm, at 3 depths), plus grids on both end faces
    just beyond them: if none of them is already solid, the join touches nothing."""
    if kind == 'c':
        cx, cy, r = geo
        n_ = max(16, int(2 * math.pi * r / 3))
        edge = [(cx + rr * math.cos(2 * math.pi * i / n_), cy + rr * math.sin(2 * math.pi * i / n_)) for i in range(n_) for rr in (r - d, r + d)]
        face = [(cx + f_ * r * math.cos(2 * math.pi * i / 12), cy + f_ * r * math.sin(2 * math.pi * i / 12)) for i in range(12) for f_ in (0.3, 0.7, 0.95)] + [(cx, cy)]
    else:
        xs_ = [q[0] for q in geo]; ys_ = [q[1] for q in geo]
        c = (sum(xs_) / len(xs_), sum(ys_) / len(ys_))
        edge = []
        m_ = len(geo)
        for i in range(m_):
            q0, q1 = geo[i], geo[(i + 1) % m_]
            Lq = math.dist(q0, q1)
            if Lq < 1e-6:
                continue
            nx, ny = (q1[1] - q0[1]) / Lq, -(q1[0] - q0[0]) / Lq
            k = max(1, int(Lq / 3))
            for j in range(k):
                t = (j + 0.5) / k
                px_, py_ = q0[0] + t * (q1[0] - q0[0]), q0[1] + t * (q1[1] - q0[1])
                edge += [(px_ + nx * d, py_ + ny * d), (px_ - nx * d, py_ - ny * d)]
        st_ = max(2.0, max(max(xs_) - min(xs_), max(ys_) - min(ys_)) / 40)
        grid = [(min(xs_) + st_ * (i + 0.5), min(ys_) + st_ * (j + 0.5)) for i in range(int((max(xs_) - min(xs_)) / st_))
                for j in range(int((max(ys_) - min(ys_)) / st_))]
        face = [q for q in edge + grid if inside(q, 'p', geo)] + [c]
    pts3 = []
    for n in (a0 + d, (a0 + a1) / 2, a1 - d):
        pts3 += [to3(ax, u, v, n) for u, v in edge]
    pts3 += [to3(ax, u, v, a0 - d) for u, v in face] + [to3(ax, u, v, a1 + d) for u, v in face]
    return pts3


def probe_in(kind, geo, ax, a0, a1):
    out = []
    if kind == 'c':
        cx, cy, r = geo
        pts2 = [(cx, cy)] + [(cx + 0.8 * r * math.cos(t), cy + 0.8 * r * math.sin(t)) for t in [i * math.pi / 4 for i in range(8)]]
    else:
        xs_ = [q[0] for q in geo]; ys_ = [q[1] for q in geo]
        c = (sum(xs_) / len(xs_), sum(ys_) / len(ys_))
        pts2 = [c] + [(c[0] + 0.8 * (q[0] - c[0]), c[1] + 0.8 * (q[1] - c[1])) for q in geo]
        pts2 = [q for q in pts2 if inside(q, 'p', geo)]
    k = max(3, int((a1 - a0) / 2.0))
    for n in [a0 + (a1 - a0) * (i + 0.5) / k for i in range(k)]:
        out += [to3(ax, u, v, n) for u, v in pts2]
    return out


floating, overlaps = set(), set()
for combo in [dict(), dict(mast_extension=30.0)]:
    p = dict(ns['P']); p.update(combo); log.clear()
    ns['build'](g['root'], p)
    L = [(op, nm, ax, a0, a1, geos(*sh), bid) for op, nm, ax, a0, a1, sh, bid in log]
    names = {bid: nm for op, nm, ax, a0, a1, gs, bid in L if op == 'new'}
    by_body = {}
    for i, rec in enumerate(L):
        by_body.setdefault(rec[6], []).append(rec)
        op, nm, ax, a0, a1, gs, bid = rec
        if op != 'join':
            continue
        prior = by_body[bid][:-1]
        for k, g_ in gs:
            if not any(solid_at(prior, q) for q in probe_out(k, g_, ax, a0, a1)):
                floating.add((names.get(bid, '?'), nm))
    printed = [b_ for b_ in by_body if '(print' in names.get(b_, '')]
    parts = [b_ for b_ in by_body if any(t in names.get(b_, '') for t in ('hardware', 'reference')) and
             not any(t in names.get(b_, '') for t in ('belt', 'GoPro', 'wheel ('))]
    for hb in parts:
        new_op = by_body[hb][0]
        for k, g_ in new_op[5]:
            for q in probe_in(k, g_, new_op[2], new_op[3], new_op[4]):
                for pb in printed:
                    if solid_at(by_body[pb], q):
                        overlaps.add((names[hb], names[pb]))
for a, b_ in sorted(floating):
    print(f'  FLOATING join "{b_}" on {a}')
for a, b_ in sorted(overlaps):
    print(f'  OVERLAP {a}  <->  {b_}')
print(f'{len(floating)} floating joins, {len(overlaps)} part/print overlaps')

# ---------------------------------------------------------------------------
# 4) every printed body fits the print bed (Bambu Lab H2C single nozzle ~325 x 320 x 320; flat orientation)
# ---------------------------------------------------------------------------
BED = (325.0, 320.0, 320.0)
p = dict(ns['P']); log.clear(); ns['build'](g['root'], p)
L = [(op, nm, ax, a0, a1, geos(*sh), bid) for op, nm, ax, a0, a1, sh, bid in log]
names = {bid: nm for op, nm, ax, a0, a1, gs, bid in L if op == 'new'}
box = {}
for op, nm, ax, a0, a1, gs, bid in L:
    if op not in ('new', 'join') or '(print' not in names.get(bid, ''):
        continue
    for k, g_ in gs:
        if k == 'c':
            us, vs = [g_[0] - g_[2], g_[0] + g_[2]], [g_[1] - g_[2], g_[1] + g_[2]]
        else:
            us, vs = [q[0] for q in g_], [q[1] for q in g_]
        for u in us:
            for v in vs:
                for n in (a0, a1):
                    q = to3(ax, u, v, n)
                    b_ = box.setdefault(bid, [list(q), list(q)])
                    b_[0] = [min(b_[0][i], q[i]) for i in range(3)]
                    b_[1] = [max(b_[1][i], q[i]) for i in range(3)]
bad_fit = 0
for bid, (lo, hi) in box.items():
    dims = sorted([hi[i] - lo[i] for i in range(3)], reverse=True)
    fits = dims[0] <= BED[0] and dims[1] <= BED[1] and dims[2] <= BED[2]
    bad_fit += not fits
    print(f'  {"OK  " if fits else "FAIL"} {names[bid]}: {dims[0]:.0f} x {dims[1]:.0f} x {dims[2]:.0f} mm')
print(f'{bad_fit} printed parts too big for the bed')
