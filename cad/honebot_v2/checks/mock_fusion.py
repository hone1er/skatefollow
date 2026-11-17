"""Mock of the Fusion 360 API: runs honebot_v2.build() outside Fusion and checks every sketch outline closes,
every join/cut touches its body, and every joint finds its edge.  Usage: python3 mock_fusion.py ../honebot_v2.py"""
import sys, types, math
S = sys.argv[1]
stats = dict(sketch=0, extrude=0, plane=0, arcs=0, lines=0, circles=0, badclose=0)
class P3:
    def __init__(s, x, y, z): s.x, s.y, s.z = x, y, z
    @staticmethod
    def create(x, y, z): return P3(x, y, z)
    def distanceTo(s, o): return math.dist((s.x, s.y, s.z), (o.x, o.y, o.z))
class SP:
    def __init__(s, g): s.geometry = g
def pt(p): return p.geometry if isinstance(p, SP) else p
class Ent:
    def __init__(s, a, b): s.startSketchPoint = a if isinstance(a, SP) else SP(a); s.endSketchPoint = b if isinstance(b, SP) else SP(b)
class Coll(list):
    def add(s, x): s.append(x); return x
    @staticmethod
    def create(): return Coll()
    def item(s, i): return s[i]
class Plane:
    def __init__(s, axis, d): s.axis, s.d = axis, d
    @property
    def geometry(s):
        o = {'x': (s.d, 0, 0), 'y': (0, s.d, 0), 'z': (0, 0, s.d)}[s.axis]
        return types.SimpleNamespace(origin=P3(*o))
    def deleteMe(s): pass
class Sketch:
    def __init__(s, plane):
        s.plane = plane; s.ents = []; s.circles = 0; s.pts = []
        lines = types.SimpleNamespace(addByTwoPoints=s._line, addTwoPointRectangle=lambda a, b: None)
        arcs = types.SimpleNamespace(addByThreePoints=s._arc)
        circ = types.SimpleNamespace(addByCenterRadius=s._circ)
        s.sketchCurves = types.SimpleNamespace(sketchLines=lines, sketchArcs=arcs, sketchCircles=circ)
    def modelToSketchSpace(s, p):
        a = s.plane.axis
        if a == 'z': return P3(p.x, p.y, p.z - s.plane.d)
        if a == 'y': return P3(p.x, p.z, p.y - s.plane.d)
        return P3(p.y, p.z, p.x - s.plane.d)
    def _line(s, a, b):
        assert pt(a).distanceTo(pt(b)) > 1e-6, 'zero-length line'
        s.pts += [pt(a), pt(b)]; e = Ent(a, b); s.ents.append(e); stats['lines'] += 1; return e
    def _arc(s, a, m, b):
        assert isinstance(m, P3)
        s.pts += [pt(a), m, pt(b)]; e = Ent(a, b); s.ents.append(e); stats['arcs'] += 1; return e
    def _circ(s, c, r):
        assert r > 0; s.circles += 1; stats['circles'] += 1; s.circ_list = getattr(s, 'circ_list', []) + [(c, r)]; s.pts += [P3(c.x - r, c.y - r, 0), P3(c.x + r, c.y + r, 0)]
    @property
    def profiles(s):
        Ext.last_sk = s
        # closed-loop check: every sketch point used exactly twice
        from collections import Counter
        cnt = Counter()
        for e in s.ents: cnt[id(e.startSketchPoint)] += 1; cnt[id(e.endSketchPoint)] += 1
        if s.ents and any(v != 2 for v in cnt.values()): stats['badclose'] += 1; raise AssertionError(f'open outline in sketch {s.name}')
        n = s.circles + (1 if s.ents else 0)
        assert n > 0, f'empty sketch {s.name}'
        return [object()] * n
class Circ:
    def __init__(s, c, r): s.center = P3(c[0] / 10, c[1] / 10, c[2] / 10); s.radius = r / 10; s.objectType = 'Circle3D'
class Edge:
    def __init__(s, c, r): s.geometry = Circ(c, r)
    def createForAssemblyContext(s, occ): return s
class Body:
    def __init__(s): s.name = ''; s.edges = []
class Ext:
    def createInput(s, prof, op):
        assert len(prof) > 0
        prof.sk = Ext.last_sk
        ns_ = types.SimpleNamespace(startExtent=None, participantBodies=None, op=op, sk=prof.sk, sym=None, dist=None)
        def sde(flag, v): ns_.dist = v
        def sse(v, full): ns_.sym = v
        ns_.setDistanceExtent = sde; ns_.setSymmetricExtent = sse
        return ns_
    def add(s, inp):
        stats['extrude'] += 1
        sk = inp.sk; pl = sk.plane
        us = [q.x * 10 for q in sk.pts]; vs = [q.y * 10 for q in sk.pts]
        if inp.sym is not None: n0, n1 = pl.d * 10 - inp.sym * 5, pl.d * 10 + inp.sym * 5
        else: n0 = (inp.startExtent or 0) * 10; n1 = n0 + inp.dist * 10
        u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
        box = {'z': (u0, v0, n0, u1, v1, n1), 'y': (u0, n0, v0, u1, n1, v1), 'x': (n0, u0, v0, n1, u1, v1)}[pl.axis]
        circs = []
        for (cc, r) in getattr(sk, 'circ_list', []):
            for n in (n0, n1):
                u, v = cc.x * 10, cc.y * 10
                ctr = {'z': (u, v, n), 'y': (u, n, v), 'x': (n, u, v)}[pl.axis]
                circs.append(Edge(ctr, r * 10))
        if inp.op == 'new':
            b = Body(); b.boxes = [box]; b.edges = circs; return types.SimpleNamespace(bodies=Coll([b]))
        assert inp.participantBodies and inp.participantBodies[0] is not None, 'join/cut without a body'
        body = inp.participantBodies[0]
        def rel(a, c, tol):  # overlap (+) / touch (0) / apart (-)
            return min(min(a[i + 3], c[i + 3]) - max(a[i], c[i]) for i in range(3))
        best = max(rel(box, c, 0) for c in body.boxes)
        if inp.op == 'join':
            if best < -1e-6: stats.setdefault('floating_joins', []).append((body.name, sk.name, round(best, 2)))
            body.boxes.append(box); body.edges += circs
        else:
            if best <= 1e-6: stats.setdefault('cuts_missing', []).append((body.name, sk.name))
            body.edges += circs
        return types.SimpleNamespace(bodies=Coll())
class Planes:
    def createInput(s):
        ns = types.SimpleNamespace()
        def setByOffset(base, v): ns.base, ns.v = base, v
        ns.setByOffset = setByOffset; return ns
    def add(s, inp):
        stats['plane'] += 1; return Plane(inp.base.axis, inp.base.d + inp.v)
class Sketches:
    def add(s, plane):
        stats['sketch'] += 1; return Sketch(plane)
def mkcomp():
    return types.SimpleNamespace(xYConstructionPlane=Plane('z', 0), xZConstructionPlane=Plane('y', 0), yZConstructionPlane=Plane('x', 0),
                             features=types.SimpleNamespace(extrudeFeatures=Ext()), constructionPlanes=Planes(), sketches=Sketches(), name='')
class Occs:
    def addNewComponent(s, m):
        stats['components'] = stats.get('components', 0) + 1
        return types.SimpleNamespace(component=mkcomp(), isGrounded=False)
class Joints:
    def createInput(s, a, b, geo):
        ns_ = types.SimpleNamespace(geo=geo, kind=None)
        ns_.setAsRevoluteJointMotion = lambda d: setattr(ns_, 'kind', 'rev')
        ns_.setAsRigidJointMotion = lambda: setattr(ns_, 'kind', 'rigid')
        ns_.setAsSliderJointMotion = lambda d: setattr(ns_, 'kind', 'slider')
        return ns_
    def add(s, inp):
        assert inp.kind; stats[inp.kind] = stats.get(inp.kind, 0) + 1; return types.SimpleNamespace(name='')
class Links:
    def createInput(s, a, b): return types.SimpleNamespace()
    def add(s, i): stats['links'] = stats.get('links', 0) + 1; return types.SimpleNamespace(name='')
root = mkcomp(); root.occurrences = Occs(); root.asBuiltJoints = Joints(); root.motionLinks = Links()
core = types.SimpleNamespace(Point3D=P3, ObjectCollection=Coll, ValueInput=types.SimpleNamespace(createByReal=lambda v: v, createByString=lambda v: v), Matrix3D=types.SimpleNamespace(create=lambda: None), Circle3D=types.SimpleNamespace(classType=lambda: 'Circle3D'))
fusion = types.SimpleNamespace(JointGeometry=types.SimpleNamespace(createByCurve=lambda e, k: e), JointKeyPointTypes=types.SimpleNamespace(CenterKeyPoint=1), JointDirections=types.SimpleNamespace(ZAxisJointDirection=3), FeatureOperations=types.SimpleNamespace(NewBodyFeatureOperation='new', JoinFeatureOperation='join', CutFeatureOperation='cut'),
                               OffsetStartDefinition=types.SimpleNamespace(create=lambda v: v))
adsk = types.ModuleType('adsk'); adsk.core = core; adsk.fusion = fusion
sys.modules['adsk'] = adsk; sys.modules['adsk.core'] = core; sys.modules['adsk.fusion'] = fusion
ns = {}; exec(compile(open(S).read(), S, 'exec'), ns)
bad = False
for combo in [dict(), dict(hardware=False, belts=False), dict(joints=False, build_lid=False), dict(mast_extension=30.0), dict(mast_extension=60.0)]:
    p = dict(ns['P']); p.update(combo)
    out = ns['build'](root, p)
    print(combo or 'defaults', '->', len(out['bodies']), 'bodies,', len(out['asm'].occ), 'components, joints', len(out['asm'].joints), 'notes', out['asm'].notes)
    bad |= bool(out['asm'].notes)
print(stats)
bad |= bool(stats.get('floating_joins') or stats.get('cuts_missing') or stats['badclose'])
print('MOCK FAILED' if bad else 'MOCK PASSED')
