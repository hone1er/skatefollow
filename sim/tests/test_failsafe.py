"""Every fail-safe layer from CLAUDE.md, tripped in the middle of a run at full (today's) speed.

The robot follows a 6 mph skater; at t = 8 s, at ~2.6 m/s, one thing fails. It must come to a standstill within
(detection delay + braking at the fail-safe 0.5 g + 1 s for the motors and ramps), touch nobody and nothing,
and stay stopped. The detection delays are the real timeouts, read from the firmware and the node parameters."""

import math

import pytest

from honebot_sim.firmware import REMOTE_LOST, REMOTE_STOP
from honebot_sim.run import make

T_FAULT = 8.0
WINDOW = 5.0


def _speed(s):
    return math.hypot(s.d.qvel[0], s.d.qvel[1])


def _trip(name, fault, t_fault=T_FAULT, window=WINDOW, **kw):
    """Run `name`, inject `fault` at t_fault; returns (sim, metrics, speed at the fault, s until stopped or None,
    still stopped at the end)."""
    sim, cfg, sc = make(name, **kw)
    st = {"v0": None, "stop": None, "moved_again": False}

    def cb(s):
        v = _speed(s)
        if st["v0"] is None and s.t >= t_fault:
            st["v0"] = v
            if fault.startswith("remote_"):
                s.esp.remote = fault[len("remote_"):]
            else:
                s.faults.add(fault)
        if st["v0"] is not None:
            if st["stop"] is None and v < 0.05:
                st["stop"] = s.t - t_fault
            elif st["stop"] is not None and v > 0.15:
                st["moved_again"] = True
        if s.t >= t_fault + window:
            return False

    mt = sim.run(until=t_fault + window, callback=cb)
    return sim, mt, st["v0"], st["stop"], not st["moved_again"]


def _delay(sim, fault):
    k, p = sim.esp.k, sim.pi.params
    return {"uwb_dead": p["follower"]["target_timeout"],
            "pi_dead": k["WATCHDOG_MS"] / 1000.0,
            "rx_dead": p["safety_gate"]["range_timeout"],
            "follower_dead": p["safety_gate"]["cmd_timeout"],
            "gate_dead": p["esp32_bridge"]["cmd_timeout"],
            "us_dead": p["safety_gate"]["blind_timeout"],
            "remote_stop": 0.0,
            "remote_lost": k["REMOTE_TIMEOUT_MS"] / 1000.0}[fault]


@pytest.mark.parametrize("fault", ["uwb_dead", "follower_dead", "gate_dead", "pi_dead", "rx_dead", "us_dead",
                                   "remote_stop", "remote_lost"])
def test_each_failsafe_layer_stops_the_robot_at_speed(fault):
    sim, mt, v0, stop, stayed = _trip("cruise_stop_6", fault)
    assert v0 > 2.0, "robot was not at speed when the fault hit (%.2f m/s)" % v0
    fs_decel = sim.esp.k["FAILSAFE_DECEL_G"] * 9.81
    limit = _delay(sim, fault) + v0 / fs_decel + 1.0
    assert stop is not None and stop < limit, "%s: stopped after %s s, allowed %.2f s" % (fault, stop, limit)
    assert stayed, "%s: moved again after stopping" % fault
    assert not mt.skater_contact and not mt.obstacle_contact and not mt.tipped


def test_the_right_layer_caught_each_fault():
    sim, *_ = _trip("cruise_stop_6", "pi_dead", window=2.0)
    assert sim.esp.failsafe and not sim.esp.link_ok                      # ESP32 watchdog, no Pi needed
    sim, *_ = _trip("cruise_stop_6", "rx_dead", window=2.0)
    assert sim.pi.gate_state in ("LINK_DOWN", "RANGE_STALE")              # safety gate on stale ESP32 data
    assert not sim.esp.failsafe                                           # ...while the ESP32 still hears the Pi
    sim, *_ = _trip("cruise_stop_6", "uwb_dead", window=2.0)
    assert sim.pi.cmd_raw == (0.0, 0.0)                                   # follower's target timeout
    sim, *_ = _trip("cruise_stop_6", "follower_dead", window=2.0)
    assert sim.pi.gate_state == "CMD_STALE" and sim.pi.cmd_vel == (0.0, 0.0)
    sim, *_ = _trip("cruise_stop_6", "gate_dead", window=2.0)
    assert sim.pi.bridge_cmd_line(sim.t) == "C,0.000,0.000" and not sim.esp.failsafe   # bridge's cmd_timeout
    sim, *_ = _trip("cruise_stop_6", "us_dead", window=2.0)
    assert sim.pi.gate_state == "BLIND"                                   # gate's blind check
    sim, *_ = _trip("cruise_stop_6", "remote_stop", window=1.0)
    assert sim.esp.failsafe and sim.esp.remote_state == REMOTE_STOP
    sim, *_ = _trip("cruise_stop_6", "remote_lost", window=1.0)
    assert sim.esp.failsafe and sim.esp.remote_state == REMOTE_LOST


def test_lost_remote_keeps_its_last_run_packet_until_the_timeout():
    """The firmware remembers the last packet: a remote that goes silent while in RUN still counts as RUN for
    REMOTE_TIMEOUT_MS, then LOST. (The sim used to report STOP at once, i.e. 0.5 s optimistic.)"""
    sim, mt, v0, stop_lost, _ = _trip("cruise_stop_6", "remote_lost")
    sim2, mt2, v02, stop_stop, _ = _trip("cruise_stop_6", "remote_stop")
    assert stop_lost - stop_stop == pytest.approx(sim.esp.k["REMOTE_TIMEOUT_MS"] / 1000.0, abs=0.05)


DEAD_US_FAST = dict(ros={"follower": {"max_linear": 7.5}}, fw={"THROTTLE_LIMIT": 1.0})


def _dead_us_top_speed(**ros_gate):
    kw = dict(DEAD_US_FAST)
    kw["ros"] = dict(kw["ros"], safety_gate=dict(blind_timeout=0.0, **ros_gate))
    sim, mt, v0, stop, _ = _trip("cruise_stop_12", "us_dead", t_fault=4.0, window=12.0, **kw)
    return sim, mt


def test_without_the_blind_check_a_dead_ultrasonic_is_still_capped_by_the_sight_limit():
    """blind_timeout 0 (the check off): a broken or unplugged HC-SR04 never echoes, R,-1 -> +inf, which REP 117
    calls 'nothing in range'. The gate's sight limit then still holds speed to what the robot can stop within
    sight_distance. The follower is allowed 7.5 m/s and the skater does 12 mph, so the cap is what binds (raising
    sight_distance lifts it)."""
    from honebot_safety.gate import sight_speed
    sim, mt = _dead_us_top_speed()
    cap = sight_speed(math.inf, sim.pi.gate)
    assert 3.0 < mt.max_speed < cap + 0.2
    assert not mt.skater_contact
    sim, mt = _dead_us_top_speed(sight_distance=8.0)
    assert mt.max_speed > cap + 0.5


def test_dead_ultrasonic_does_not_hit_the_box():
    """The gap the blind check closes: a dead HC-SR04 reads as clear road. With the check off the robot drives
    into the box; with robot.yaml's blind_timeout it stops well short."""
    sim, mt, v0, stop, _ = _trip("obstacle", "us_dead", t_fault=3.0, window=10.0,
                                 ros={"safety_gate": {"blind_timeout": 0.0}})
    assert mt.obstacle_contact
    sim, mt, v0, stop, _ = _trip("obstacle", "us_dead", t_fault=3.0, window=10.0)
    assert not mt.obstacle_contact and sim.pi.gate_state == "BLIND"


def test_healthy_sensor_never_trips_the_blind_check():
    """The simulated HC-SR04 misses 2 % of pings; the ground return must still keep BLIND away in every
    scenario the robot drives today."""
    from honebot_sim.run import SCENARIOS
    for name in SCENARIOS:
        if name == "playground":
            continue
        sim, cfg, sc = make(name)
        mt = sim.run()
        assert mt.gate_time.get("BLIND", 0.0) == 0.0, name
