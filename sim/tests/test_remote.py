"""Driving the simulated robot with the handheld remote (its USB status lines) and with the keyboard."""

import math
import os
import time

import pytest

from honebot_sim.firmware import SRC_MANUAL, SRC_NONE
from honebot_sim.remote_link import KeyboardRemote
from honebot_sim.run import make


def _drive(sim, seconds, remote):
    sim.remote_input = remote
    end = sim.t + seconds
    while sim.t < end:
        sim.step()


def test_keyboard_remote_drives_through_the_firmware_manual_path():
    sim, cfg, sc = make("playground")
    kb = KeyboardRemote()
    _drive(sim, 3.0, kb)                       # STOP: nobody may drive, even though the follower wants to
    assert sim.esp.source == SRC_NONE and sim.esp.failsafe
    kb.key(kb.R)                               # RUN
    _drive(sim, 0.5, kb)                       # stick centred in RUN -> the Pi (follower) drives
    for _ in range(3):
        kb.key(kb.UP)                          # stick 0.6 forward -> manual, capped at MANUAL_LIMIT
    _drive(sim, 3.0, kb)
    assert sim.esp.source == SRC_MANUAL
    v = math.hypot(sim.d.qvel[0], sim.d.qvel[1])
    assert 0.5 < v < cfg.firmware["MANUAL_LIMIT"] * cfg.firmware["FULL_SCALE_MPS"] * 1.1
    kb.key(kb.R)                               # STOP while driving -> fail-safe ramp to a standstill
    _drive(sim, 4.0, kb)
    assert sim.esp.failsafe and math.hypot(sim.d.qvel[0], sim.d.qvel[1]) < 0.05


def test_real_remote_lines_over_a_serial_port():
    """The remote's USB line, through a pseudo-terminal. A fake clock (= sim time) keeps it independent of how
    fast the machine running the test is."""
    pytest.importorskip("serial")
    from honebot_sim.remote_link import RemoteSerial
    master, slave = os.openpty()
    sim, cfg, sc = make("playground")
    rs = RemoteSerial(os.ttyname(slave), clock=lambda: sim.t)
    sim.remote_input = rs

    def send(x):
        n = rs.lines
        os.write(master, b"state=RUN x=%+.2f y=+0.00 btn=0 robot=0 pi=0 failsafe=1 manual=0\n" % x)
        deadline = time.monotonic() + 5.0
        while rs.lines == n and time.monotonic() < deadline:
            time.sleep(0.01)
        assert rs.lines > n, "line not read"

    def run_for(seconds, x):                  # keep the remote "talking" every 50 ms of sim time
        end = sim.t + seconds
        while sim.t < end:
            send(x)
            for _ in range(25):
                sim.step()
    try:
        assert rs.poll()[0] == "none"
        run_for(2.5, 0.0)                      # arming window with the stick centred (RUN seen centred)
        assert rs.poll() == ("run", 0.0, 0.0)
        run_for(1.0, 0.5)                      # stick forward -> manual
        assert sim.esp.source == SRC_MANUAL
        assert math.hypot(sim.d.qvel[0], sim.d.qvel[1]) > 0.3
        end = sim.t + 0.7                      # the remote goes quiet -> lost (0.12 s link gap + the firmware's
        #                                        REMOTE_TIMEOUT_MS 0.5 s) -> nobody drives
        while sim.t < end:
            sim.step()
        assert rs.poll()[0] == "lost"
        assert sim.esp.source == SRC_NONE
    finally:
        rs.close()
        os.close(master)
        os.close(slave)


class _Fixed:
    def __init__(self):
        self.v = ("run", 0.0, 0.0)

    def poll(self):
        return self.v


def _stick_yaw(angle_deg, yaw_loop):
    sim, cfg, sc = make("playground", yaw_loop=yaw_loop)
    sim.path.at = lambda t: (80.0, 80.0, 0.0, 0.0)
    r = _Fixed()
    sim.remote_input = r
    while sim.t < 2.5:
        sim.step()
    a = math.radians(angle_deg)
    r.v = ("run", math.cos(a), math.sin(a))
    yaws = []
    while sim.t < 7.5:
        sim.step()
        if sim.t > 5.5:
            yaws.append(sim.d.qvel[5])
    return sum(yaws) / len(yaws)


def test_stick_turns_gently_with_the_yaw_loop():
    """With the gyro loop the side stick asks for a yaw rate: a diagonal turns gently, full side = MANUAL_MAX_YAW.
    Without it (2.2.1 behaviour) a diagonal goes straight (scrub threshold, a model prediction)."""
    assert abs(_stick_yaw(45, yaw_loop=False)) < 0.1
    y20, y45, y90 = (_stick_yaw(a, yaw_loop=True) for a in (20, 45, 90))
    assert 0.1 < y20 < y45 < y90
    assert y90 == pytest.approx(3.0, abs=0.4)

