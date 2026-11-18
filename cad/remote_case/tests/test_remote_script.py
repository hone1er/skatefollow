"""Runs HonebotRemote.run() (the remote-case Fusion script) against a minimal stand-in for Fusion's `adsk` API.
Adapted from cad/tests/test_fusion_script.py (kept separate so the two scripts can change independently).

This catches Python errors (typos, wrong argument counts, bad geometry such as zero-length cylinders or
non-orthogonal boxes) in the Fusion code path. It does NOT prove the real Fusion API accepts every call;
the script was written against the documented API (TemporaryBRepManager, OrientedBoundingBox3D,
bRepBodies.add in a direct-modelling design) and wraps each part in try/except so one failure only
skips that part and is reported in the final message box.

    python3 cad/remote_case/tests/test_remote_script.py      (no numpy needed)
"""
import importlib.util
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "fusion", "HonebotRemote", "HonebotRemote.py")

LOG = {"boxes": 0, "cyls": 0, "bool": 0, "bodies": [], "messages": [], "components": []}


def _fake_adsk():
    adsk = types.ModuleType("adsk")
    core = types.ModuleType("adsk.core")
    fusion = types.ModuleType("adsk.fusion")

    class Point3D:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

        @staticmethod
        def create(x, y, z):
            for v in (x, y, z):
                assert isinstance(v, (int, float)) and math.isfinite(v), v
            return Point3D(x, y, z)

    class Vector3D(Point3D):
        @staticmethod
        def create(x, y, z):
            assert abs(math.sqrt(x * x + y * y + z * z) - 1) < 1e-6, "direction not normalised"
            return Vector3D(x, y, z)

    class OBB:
        @staticmethod
        def create(c, u, v, lu, lv, lw):
            dot = u.x * v.x + u.y * v.y + u.z * v.z
            assert abs(dot) < 1e-6, "box directions not orthogonal (%g)" % dot
            assert lu > 0 and lv > 0 and lw > 0, (lu, lv, lw)
            return ("obb", c, u, v, lu, lv, lw)

    class Body:
        def __init__(self, kind):
            self.kind = kind

    class TBM:
        @staticmethod
        def get():
            return TBM()

        def createBox(self, obb):
            LOG["boxes"] += 1
            return Body("box")

        def createCylinderOrCone(self, p1, r1, p2, r2):
            L = math.dist((p1.x, p1.y, p1.z), (p2.x, p2.y, p2.z))
            assert L > 1e-6 and r1 > 0 and r2 > 0, (L, r1, r2)
            LOG["cones"] = LOG.get("cones", 0) + (abs(r1 - r2) > 1e-9)
            LOG["cyls"] += 1
            return Body("cyl")

        def createSphere(self, c, r):
            assert r > 0
            LOG["spheres"] = LOG.get("spheres", 0) + 1
            return Body("sph")

        def createTorus(self, c, axis, R, r):
            assert R >= r > 0, (R, r)
            assert abs(math.sqrt(axis.x ** 2 + axis.y ** 2 + axis.z ** 2) - 1) < 1e-6
            LOG["tori"] = LOG.get("tori", 0) + 1
            return Body("tor")

        def booleanOperation(self, target, tool, op):
            assert isinstance(target, Body) and isinstance(tool, Body)
            LOG["bool"] += 1
            return True

    class Matrix3D:
        @staticmethod
        def create():
            return Matrix3D()

    class Collection(list):
        @property
        def count(self):
            return len(self)

        def item(self, i):
            return self[i]

    class BRepBody:
        def __init__(self, b):
            self.name = ""
            self.appearance = None
            self.opacity = 1.0

    class Bodies:
        def add(self, b, *args):
            assert isinstance(b, Body)
            rb = BRepBody(b)
            LOG["bodies"].append(rb)
            return rb

    class Component:
        def __init__(self):
            self.name = ""
            self.occurrences = Occurrences()
            self.bRepBodies = Bodies()

    class Occurrence:
        def __init__(self):
            self.component = Component()
            self.isLightBulbOn = True

    class Occurrences:
        def addNewComponent(self, m):
            o = Occurrence()
            LOG["components"].append(o)
            return o

    class Appearance:
        def __init__(self, name):
            self.name = name
            self.appearanceProperties = Collection([ColorProperty("opaque_albedo")])

    class ColorProperty:
        def __init__(self, pid):
            self.id = pid
            self.value = None

        @staticmethod
        def cast(x):
            return x if isinstance(x, ColorProperty) else None

    class Appearances(Collection):
        def itemByName(self, n):
            for a in self:
                if a.name == n:
                    return a
            return None

        def addByCopy(self, base, name):
            a = Appearance(name)
            self.append(a)
            return a

    class Library:
        name = "Fusion Appearance Library"
        appearances = Appearances([Appearance("Plastic - Matte (Yellow)")])

    class Design:
        def __init__(self):
            self.rootComponent = Component()
            self.appearances = Appearances()
            self.designType = None

        @staticmethod
        def cast(x):
            return x

    class UI:
        def messageBox(self, msg, title=""):
            LOG["messages"].append(msg)

    class Viewport:
        def fit(self):
            pass

    class App:
        def __init__(self):
            self.design = Design()
            self.userInterface = UI()
            self.materialLibraries = Collection([Library()])
            self.activeViewport = Viewport()
            self.documents = types.SimpleNamespace(add=lambda t: object())

        @property
        def activeProduct(self):
            return self.design

    _app = App()

    class Application:
        @staticmethod
        def get():
            return _app

    class Color:
        @staticmethod
        def create(r, g, b, a):
            return (r, g, b, a)

    core.Application = Application
    core.Point3D = Point3D
    core.Vector3D = Vector3D
    core.OrientedBoundingBox3D = OBB
    core.Matrix3D = Matrix3D
    core.ColorProperty = ColorProperty
    core.Color = Color
    core.DocumentTypes = types.SimpleNamespace(FusionDesignDocumentType=0)
    fusion.Design = Design
    fusion.DesignTypes = types.SimpleNamespace(DirectDesignType=0, ParametricDesignType=1)
    fusion.TemporaryBRepManager = TBM
    fusion.BooleanTypes = types.SimpleNamespace(UnionBooleanType=0, DifferenceBooleanType=1, IntersectionBooleanType=2)
    adsk.core = core
    adsk.fusion = fusion
    return adsk, core, fusion


def test_run_builds_every_part():
    adsk, core, fusion = _fake_adsk()
    sys.modules["adsk"] = adsk
    sys.modules["adsk.core"] = core
    sys.modules["adsk.fusion"] = fusion
    spec = importlib.util.spec_from_file_location("HonebotRemote", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    n_parts = len(mod.build_model().parts)
    mod.run(None)
    msg = LOG["messages"][-1]
    assert "Failed parts" not in msg and "failed" not in msg, msg
    assert "built %d of %d parts" % (n_parts, n_parts) in msg, msg
    assert len(LOG["bodies"]) == n_parts
    assert LOG.get("cones", 0) > 0, "bevels and the stick opening use cones"
    names = [p["name"] for p in mod.build_model().parts if p["group"] == "printed"]
    assert names == ["remote_base", "remote_lid", "js_carrier"], names
    print("ok: %d parts, %d boxes, %d cylinders, %d booleans" % (n_parts, LOG["boxes"], LOG["cyls"], LOG["bool"]))


if __name__ == "__main__":
    test_run_builds_every_part()
