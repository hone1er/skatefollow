"""Whole-robot runs: the robot software + emulated ESP32 + physics against scripted skaters.

These pin down what the simulator currently predicts. If a change to the robot's software or parameters moves
one of them, look at the run (python -m honebot_sim.run <name> --plot run.png) before updating the test."""

import pytest

from honebot_sim.run import evaluate, make


def _run(name, **kw):
    sim, cfg, sc = make(name, **kw)
    mt = sim.run()
    return sc, mt, cfg


@pytest.mark.parametrize("name", ["walk_stop", "cruise_stop_6", "stop_and_go"])
def test_straight_line_following_today(name):
    sc, mt, cfg = _run(name)
    ok, why = evaluate(sc, mt, cfg)
    assert ok, why
    assert mt.min_gap > 0.5


@pytest.mark.parametrize("name", ["cruise_stop_8", "bail_8", "obstacle"])
def test_safe_even_when_it_cannot_keep_up(name):
    sc, mt, cfg = _run(name)
    assert not mt.skater_contact and not mt.obstacle_contact and not mt.tipped
    assert mt.final_speed < 0.2


def test_sight_limit_caps_speed_with_the_hcsr04():
    sc, mt, cfg = _run("cruise_stop_12")
    assert mt.max_speed < 3.0                       # the gate's sight limit, ~2.7 m/s with the ground return
    assert not mt.skater_contact


def test_turns_need_the_yaw_loop():
    """Model prediction (rigid Coulomb friction; real tyres creep, so read "heavy understeer"): open-loop wheel
    voltages cannot break the tyres loose for small turn commands (scrub threshold);
    a gyro yaw-rate loop on the ESP32 (prototype here) fixes it."""
    sc, mt, cfg = _run("carve_6")
    assert mt.lost_time > 1.0
    sc, mt, cfg = _run("carve_6", yaw_loop=True)
    ok, why = evaluate(sc, mt, cfg)
    assert ok, why


def test_target_profile_reaches_12_mph():
    sc, mt, cfg = _run("cruise_stop_12", profile="target", yaw_loop=True)
    ok, why = evaluate(sc, mt, cfg)
    assert ok, why
    assert mt.max_speed > 5.0 and mt.lost_time == 0.0
