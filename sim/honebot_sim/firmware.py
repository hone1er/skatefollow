"""The robot ESP32, emulated: the real drive_logic.h (compiled, via ctypes) plus the sketch's loop glue.

What is shared with the real firmware, byte for byte: drive::decide (who may drive), slewChannel, rampPairToZero,
rampToward, deadbandComp, arcadeMix, shapeStick, yawLoop (the 2.3.0 gyro yaw-rate loop), expo, and every numeric constant (parsed from the .ino).
What is mirrored here by hand: updateOutputs() / handleLine() / applyOutputs() / updateGyro() / yawLoopUsable()
from esp32_ultrasonic.ino, about 60 lines of glue. Keep them in step if the sketch changes (tests/test_firmware.py checks the behaviour).
"""

import ctypes
import hashlib
import os
import shutil
import subprocess
import sys
from pathlib import Path

from .config import FIRMWARE_DIR, REPO

HERE = Path(__file__).resolve().parent
CACHE = REPO / "sim" / ".cache"

SRC_NONE, SRC_PI, SRC_MANUAL = 0, 1, 2
REMOTE_NEVER, REMOTE_LOST, REMOTE_STOP, REMOTE_RUN = 0, 1, 2, 3


def _compiler():
    for c in (os.environ.get("CXX"), "c++", "clang++", "g++"):
        if c and shutil.which(c):
            return c
    raise RuntimeError("no C++ compiler found: on the Mac run `xcode-select --install`; on Linux install g++")


def build_library():
    """Compile drive_logic.h + fw_shim.cpp into a shared library (cached by source hash)."""
    srcs = [HERE / "fw_shim.cpp", FIRMWARE_DIR / "drive_logic.h"]
    digest = hashlib.sha1(b"".join(p.read_bytes() for p in srcs) + sys.platform.encode()).hexdigest()[:12]
    CACHE.mkdir(parents=True, exist_ok=True)
    lib = CACHE / ("libhonebot_fw_%s%s" % (digest, ".dylib" if sys.platform == "darwin" else ".so"))
    if not lib.exists():
        tmp = lib.with_suffix(".tmp")
        cmd = [_compiler(), "-std=c++17", "-O2", "-shared", "-fPIC", "-I", str(FIRMWARE_DIR), "-o", str(tmp),
               str(HERE / "fw_shim.cpp")]
        subprocess.run(cmd, check=True)
        tmp.replace(lib)
    return lib


_LIB = None


def lib():
    global _LIB
    if _LIB is None:
        L = ctypes.CDLL(str(build_library()))
        f, u, i, pf, pi = ctypes.c_float, ctypes.c_uint, ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_int)
        L.hb_state_new.restype = ctypes.c_void_p
        L.hb_state_free.argtypes = [ctypes.c_void_p]
        L.hb_decide.argtypes = [ctypes.c_void_p, i, u, u, u, u, u, f, u, i, u, i, u, i, u, i, f, f, pi, pi]
        L.hb_decide.restype = i
        for name in ("hb_ramp_toward", "hb_deadband_comp", "hb_shape_stick"):
            getattr(L, name).restype = f
        L.hb_ramp_toward.argtypes = [f, f, f]
        L.hb_slew_channel.argtypes = [f, f, f, f]
        L.hb_slew_channel.restype = f
        L.hb_ramp_pair_to_zero.argtypes = [pf, pf, f]
        L.hb_deadband_comp.argtypes = [f, f]
        L.hb_shape_stick.argtypes = [f, f]
        L.hb_arcade_mix.argtypes = [f, f, f, f, pf, pf]
        L.hb_yaw_state_new.restype = ctypes.c_void_p
        for name in ("hb_yaw_state_free", "hb_yaw_state_reset", "hb_yaw_state_tripped"):
            getattr(L, name).argtypes = [ctypes.c_void_p]
        L.hb_yaw_state_tripped.restype = i
        L.hb_yaw_loop.argtypes = [ctypes.c_void_p, f, f, f, f, f, u, u, i, f, u, f, pf, pf]
        L.hb_expo.argtypes = [f, f]
        L.hb_expo.restype = f
        _LIB = L
    return _LIB


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class Esp32:
    """One robot ESP32. Feed it serial lines with line(), call update() every millisecond-ish step, read
    out_a / out_b (wheel channels, -1..1, after the deadband compensation the PWM sees) and out_pan."""

    def __init__(self, fw, remote="run"):
        self.k = fw
        self.L = lib()
        self.state = self.L.hb_state_new()
        g = 9.81 / fw["FULL_SCALE_MPS"]
        self.accel_rate = fw["ACCEL_G"] * g
        self.decel_rate = fw["DECEL_G"] * g
        self.failsafe_rate = fw["FAILSAFE_DECEL_G"] * g
        self.remote = remote            # run | stop | lost | none (never heard): the handheld kill switch
        self.last_remote = 0
        self.remote_run_flag = False    # RUN flag of the last packet received (the sketch's rxFlags)
        self.heartbeat_seen = False
        self.last_hb = 0
        self.cmd_seen = False
        self.last_cmd = 0
        self.cmd_a = self.cmd_b = self.cmd_pan = 0.0
        self.out_a = self.out_b = self.out_pan = 0.0
        self.failsafe = True
        self.failsafe_since = 0
        self.last_loop = 0
        self.source = SRC_NONE
        self.link_ok = False
        self.remote_state = REMOTE_NEVER
        self.stick = (0.0, 0.0)         # (forward, turn) -1..1 from the remote, as in its radio packet
        # yaw-rate loop (2.3.0): gyro state as the sketch keeps it; the sim feeds gyro_read() every GYRO_PERIOD_MS
        self.yaw_state = self.L.hb_yaw_state_new()
        self.gyro_present = bool(fw["YAW_LOOP_ENABLED"])   # setupGyro(): the sim always has a gyro fitted
        self.gyro_bias_sum = 0.0
        self.gyro_bias_n = 0
        self.gyro_bias = 0.0
        self.gyro_bias_done = False
        self.gyro_z = 0.0
        self.gyro_last_ok = 0
        self.drive_a = self.drive_b = 0.0
        self.manual_turn_scale_gyro = (fw["MANUAL_MAX_YAW"] * fw["YAW_TRACK_WIDTH"]
                                       / (2.0 * fw["MANUAL_LIMIT"] * fw["FULL_SCALE_MPS"]))

    def __del__(self):
        try:
            self.L.hb_state_free(self.state)
            self.L.hb_yaw_state_free(self.yaw_state)
        except Exception:
            pass

    # --- handleLine() ---------------------------------------------------------------------------------
    def line(self, text, now_ms):
        if text == "H":
            self.last_hb, self.heartbeat_seen = now_ms, True
            return
        parts = text.split(",")
        try:
            if parts[0] == "C" and len(parts) == 3:
                a, b = float(parts[1]), float(parts[2])
                if a == a and b == b and abs(a) != float("inf") and abs(b) != float("inf"):
                    self.cmd_a, self.cmd_b = _clamp(a, -1, 1), _clamp(b, -1, 1)
                    self.last_cmd, self.cmd_seen = now_ms, True
            elif parts[0] == "P" and len(parts) == 2:
                v = float(parts[1])
                if v == v and abs(v) != float("inf"):
                    self.cmd_pan = _clamp(v, -1, 1)
        except ValueError:
            pass

    # --- updateGyro() / yawLoopUsable() ------------------------------------------------------------------
    def gyro_read(self, w, now_ms):
        """One MPU6050 sample w (rad/s, + = left, already through GYRO_SIGN), or None for a failed read."""
        if not self.gyro_present or w is None:
            return
        if now_ms < self.k["ARM_MS"]:
            self.gyro_bias_sum += w
            self.gyro_bias_n += 1
            return
        if not self.gyro_bias_done:
            self.gyro_bias = self.gyro_bias_sum / self.gyro_bias_n if self.gyro_bias_n else 0.0
            self.gyro_bias_done = True
        self.gyro_z = w - self.gyro_bias
        self.gyro_last_ok = now_ms

    def yaw_loop_usable(self, now_ms):
        return (bool(self.k["YAW_LOOP_ENABLED"]) and self.gyro_present and self.gyro_bias_done
                and now_ms - self.gyro_last_ok < self.k["GYRO_STALE_MS"]
                and not self.L.hb_yaw_state_tripped(self.yaw_state))

    # --- updateOutputs() ------------------------------------------------------------------------------
    def update(self, now_ms):
        k, L = self.k, self.L
        dt = (now_ms - self.last_loop) * 0.001
        self.last_loop = now_ms
        if self.remote in ("run", "stop"):
            self.last_remote = now_ms           # a fresh packet this step ("lost" = packets stopped arriving)
            self.remote_run_flag = self.remote == "run"
        r_seen = self.remote != "none"
        rem, link = ctypes.c_int(), ctypes.c_int()
        self.source = L.hb_decide(self.state, int(k["REMOTE_REQUIRED"]), int(k["ARM_MS"]), int(k["WATCHDOG_MS"]),
                                  int(k["CMD_TIMEOUT_MS"]), int(k["REMOTE_TIMEOUT_MS"]), int(k["MANUAL_HOLD_MS"]),
                                  k["STICK_DEADBAND"], now_ms, int(self.heartbeat_seen), self.last_hb,
                                  int(self.cmd_seen), self.last_cmd, int(r_seen), self.last_remote,
                                  int(self.remote_run_flag), self.stick[0], self.stick[1],
                                  ctypes.byref(rem), ctypes.byref(link))
        self.remote_state, self.link_ok = rem.value, bool(link.value)
        was = self.failsafe
        self.failsafe = self.source == SRC_NONE
        if self.failsafe and not was:
            self.failsafe_since = now_ms
        if self.failsafe:
            if now_ms < k["ARM_MS"] or now_ms - self.failsafe_since >= k["FAILSAFE_MAX_MS"]:
                self.out_a = self.out_b = 0.0
            else:
                a, b = ctypes.c_float(self.out_a), ctypes.c_float(self.out_b)
                L.hb_ramp_pair_to_zero(ctypes.byref(a), ctypes.byref(b), self.failsafe_rate * dt)
                self.out_a, self.out_b = a.value, b.value
        else:
            if self.source == SRC_MANUAL:
                ta, tb = ctypes.c_float(), ctypes.c_float()
                turn = L.hb_shape_stick(self.stick[1], k["STICK_DEADBAND"])
                scale = k["MANUAL_TURN_SCALE"]
                if self.yaw_loop_usable(now_ms):
                    turn = L.hb_expo(turn, k["MANUAL_TURN_EXPO"])
                    scale = self.manual_turn_scale_gyro
                L.hb_arcade_mix(L.hb_shape_stick(self.stick[0], k["STICK_DEADBAND"]), turn,
                                k["MANUAL_LIMIT"], scale, ctypes.byref(ta), ctypes.byref(tb))
                ta, tb = ta.value, tb.value
            else:
                ta, tb = self.cmd_a, self.cmd_b
            lim = k["THROTTLE_LIMIT"]
            up, down = self.accel_rate * dt, self.decel_rate * dt
            self.out_a = L.hb_slew_channel(self.out_a, _clamp(ta, -lim, lim), up, down)
            self.out_b = L.hb_slew_channel(self.out_b, _clamp(tb, -lim, lim), up, down)
        self.out_pan = L.hb_ramp_toward(self.out_pan, self.cmd_pan, k["PAN_RATE_PER_S"] * dt)
        if self.failsafe:
            L.hb_yaw_state_reset(self.yaw_state)
        a, b = ctypes.c_float(self.out_a), ctypes.c_float(self.out_b)
        active = k["DIFF_DRIVE"] and not self.failsafe and self.yaw_loop_usable(now_ms)
        if k["YAW_LOOP_ENABLED"]:
            L.hb_yaw_loop(self.yaw_state, k["YAW_KP"], k["YAW_KI"], k["YAW_MAX_CORR"], k["FULL_SCALE_MPS"],
                          k["YAW_TRACK_WIDTH"], int(k["YAW_SAT_MS"]), int(k["YAW_WRONG_WAY_MS"]), int(active),
                          self.gyro_z, now_ms, dt,
                          ctypes.byref(a), ctypes.byref(b))
        self.drive_a, self.drive_b = a.value, b.value

    # --- applyOutputs(): what the ESCs receive (1000-2000 us -> -1..1) ---------------------------------
    def wheel_outputs(self):
        k = self.k
        a = self.L.hb_deadband_comp(self.drive_a, k["ESC_DEADBAND"])
        b = self.L.hb_deadband_comp(self.drive_b, k["ESC_DEADBAND"])
        span = k["PULSE_MAX_US"] - k["PULSE_NEUTRAL_US"]           # pulseFor(): whole microseconds of pulse width
        q = lambda v: round(_clamp(v, -1, 1) * span) / span
        a, b = q(a), q(b)
        return (-a if k["THROTTLE_REVERSED"] else a), (-b if k["STEER_REVERSED"] else b)

    def status_lines(self, range_m):
        """The R / S / K lines the sketch prints (R every 60 ms, S and K every 100 ms; callers pick)."""
        r = "R,%.3f,%d" % (range_m if range_m is not None else -1.0, 1 if self.link_ok else 0)
        s = "S,%d,%.3f,%.3f" % (1 if self.failsafe else 0, self.drive_a, self.drive_b)
        kk = "K,%d,%d,%d" % (1 if self.k["REMOTE_REQUIRED"] else 0, self.remote_state, self.source)
        return r, s, kk
