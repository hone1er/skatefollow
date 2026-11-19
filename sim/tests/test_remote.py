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
    serial = pytest.importorskip("serial")
    from honebot_sim.remote_link import RemoteSerial
    master, slave = os.openpty()
    rs = RemoteSerial(os.ttyname(slave))
    try:
        sim, cfg, sc = make("playground")
        sim.remote_input = rs
        assert rs.poll()[0] == "none"
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:            # the remote's 5 Hz status line, stick 0.5 forward
            os.write(master, b"state=RUN x=+0.50 y=+0.00 btn=0 robot=0 pi=0 failsafe=1 manual=0\n")
            time.sleep(0.05)
            if rs.lines:
                break
        assert rs.poll() == ("run", 0.5, 0.0)
        os.write(master, b"state=RUN x=+0.00 y=+0.00 btn=0 robot=0 pi=0 failsafe=1 manual=0\n")
        time.sleep(0.2)
        start = sim.t
        while sim.t < start + 2.5:                 # arming window passes with the stick centred
            sim.step()
        os.write(master, b"state=RUN x=+0.50 y=+0.00 btn=0 robot=0 pi=0 failsafe=1 manual=0\n")
        time.sleep(0.2)
        start = sim.t
        while sim.t < start + 0.3:
            sim.step()
        assert sim.esp.source == SRC_MANUAL
        time.sleep(0.6)                            # remote goes quiet -> lost -> nobody drives
        assert rs.poll()[0] == "lost"
        sim.step()
        assert sim.esp.source == SRC_NONE
    finally:
        rs.close()
        os.close(master)
        os.close(slave)
