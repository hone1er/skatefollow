"""Side view (film.launch.py side_shot:=true, config/side_shot.yaml): 2 m from the skater, 45 deg round from
directly behind, on their right. Pass marks are fixed in honebot_sim/run.py (SIDE_DIST_TOL, SIDE_ANGLE_TOL,
SIDE_MIN_GAP) and judged by evaluate(): median distance/angle while the skater moves, never driving within 0.5 m.

Side mode needs the gyro (G line), so every run has the yaw loop on. With today's HC-SR04 the gate's sight limit
holds the robot to ~2.7 m/s, which only keeps up with a walking / slow-rolling skater; the `target` profile (12 m
forward lidar) is what the 6-15 mph side view needs."""

import math

import pytest

from honebot_sim.run import evaluate, make


def _run(name, profile="target", **kw):
    sim, cfg, sc = make(name, profile=profile, yaw_loop=True, side=True, **kw)
    mt = sim.run()
    return sim, sc, mt, cfg


@pytest.mark.parametrize("name", ["walk_stop", "cruise_stop_6", "cruise_stop_8", "cruise_stop_12", "bail_8",
                                  "u_turn_5", "u_turn_right_5", "stop_and_go", "obstacle"])
def test_side_view_with_the_long_range_sensor(name):
    sim, sc, mt, cfg = _run(name)
    ok, why = evaluate(sc, mt, cfg)
    assert ok, why
    assert not mt.skater_contact and not mt.obstacle_contact and not mt.tipped


@pytest.mark.parametrize("name", ["walk_stop", "stop_and_go", "u_turn_right_5"])
def test_side_view_today_at_walking_and_rolling_pace(name):
    sim, sc, mt, cfg = _run(name, profile="today")
    ok, why = evaluate(sc, mt, cfg)
    assert ok, why


def test_today_6_mph_is_beyond_the_hcsr04_sight_limit_but_safe():
    """The gate caps the robot near the skater's own speed, so it never reaches the side position."""
    sim, sc, mt, cfg = _run("cruise_stop_6", profile="today")
    dist, _ = mt.side()
    assert dist > 4.0 and mt.max_speed < 3.0
    assert not mt.skater_contact and mt.min_gap_moving > 0.5


def test_slalom_is_a_known_weak_spot():
    """S-curves swing the skater's heading +-43 deg every 2.5 s and the aim point with it: the robot holds a looser
    ~2.9 m / ~32 deg. Safe, but not the 2 m / 45 deg shot."""
    sim, sc, mt, cfg = _run("carve_6")
    dist, ang = mt.side()
    assert dist < 3.6 and ang > 15.0
    assert not mt.skater_contact and mt.min_gap_moving > 0.5


@pytest.mark.parametrize("seed", [0, 1, 2, 3])
def test_15_mph_is_marginal(seed):
    """Validated to 12 mph. At 15 mph the held point depends on the noise (1.8-2.5 m, 25-42 deg) and the robot can
    pass about 0.4 m from the skater's centre while moving (~15 cm from their body): 0.49 m on macOS, 0.40 m on
    Linux CI for seed 0, the run is that close to the edge. Owner's decision (2026-11-19): keep 15 mph in side mode;
    this checks it never touches them and stays above 0.3 m."""
    sim, sc, mt, cfg = _run("cruise_stop_15", seed=seed)
    assert not mt.skater_contact and not mt.tipped and mt.min_gap_moving > 0.3


def test_without_the_gyro_side_mode_never_engages():
    sim, cfg, sc = make("cruise_stop_6", profile="target", yaw_loop=False, side=True)
    seen = []
    sim.run(callback=lambda s: seen.append(s.pi.side.active) and None)
    assert not any(seen)
    assert sim.metrics.min_gap_moving > 0.5 and not sim.metrics.skater_contact


def test_losing_the_status_lines_at_speed_falls_back_to_the_normal_law():
    """S and G lines stop reaching the Pi at 12 mph, in the side position: side mode switches off within
    side_input_timeout and the normal law's stop-behind gap pulls the robot back. Nobody is touched."""
    sim, cfg, sc = make("cruise_stop_12", profile="target", yaw_loop=True, side=True)
    st = {"off": None}

    def cb(s):
        if s.t >= 12.0 and "status_dead" not in s.faults:
            assert s.pi.side.active
            s.faults.add("status_dead")
        if "status_dead" in s.faults and st["off"] is None and not s.pi.side.active:
            st["off"] = s.t - 12.0
    mt = sim.run(callback=cb)
    timeout = sim.pi.params["follower"]["side_input_timeout"]
    assert st["off"] is not None and st["off"] <= timeout + 0.1
    assert not mt.skater_contact and mt.min_gap_moving > 0.5


@pytest.mark.parametrize("side", [False, True])
@pytest.mark.parametrize("name", ["cruise_stop_12", "carve_6", "u_turn_right_5"])
def test_a_stuck_gyro_at_speed_does_not_tip_the_robot(name, side):
    """Firmware 2.4.0: a gyro returning the same raw value 100 times (0.5 s) counts as absent. Before, the loop kept
    adding difference just under its cap, the tyres broke loose and the robot rolled over (5 of 6 seeds here)."""
    for seed in range(3):
        sim, cfg, sc = make(name, seed=seed, profile="target", yaw_loop=True, side=side, gyro_fault="stuck")
        mt = sim.run()
        assert not mt.tipped and not mt.skater_contact and mt.min_gap_moving > 0.5, seed


@pytest.mark.xfail(strict=True, reason="KNOWN GAP: a gyro mounted upside down can still roll the robot over at "
                                       "speed (10 seeds, 4.03 kg: side mode slalom 1/10, bail 3/10, 12 mph 1/10; "
                                       "normal mode 12 mph 4/10). Firmware 2.4.0's runaway check and 75 ms trip "
                                       "reduced it (side-mode slalom was 3/6) but the skid starts before any check "
                                       "can see it. The bench sign check (first_drive.md Stage 1 step 7) is the "
                                       "protection. Sim tyres have a hard breakaway; real ones creep.")
def test_an_upside_down_gyro_at_speed_does_not_tip_the_robot():
    for name, side in (("carve_6", True), ("bail_8", True), ("cruise_stop_12", True), ("cruise_stop_12", False)):
        for seed in range(10):
            sim, cfg, sc = make(name, seed=seed, profile="target", yaw_loop=True, side=side, gyro_fault="sign")
            mt = sim.run()
            assert not mt.tipped, (name, side, seed)


@pytest.mark.parametrize("scale", [0.85, 1.15])
def test_wrong_wheel_speed_scale(scale):
    """max_wheel_speed (no-load guess) 15 % off: the side view still passes."""
    sim, sc, mt, cfg = _run("cruise_stop_8", ros={"esp32_bridge": {"max_wheel_speed": 9.6 * scale}})
    ok, why = evaluate(sc, mt, cfg)
    assert ok, why


@pytest.mark.parametrize("seed", [1, 2])
def test_other_noise_seeds(seed):
    sim, sc, mt, cfg = _run("cruise_stop_12", seed=seed)
    ok, why = evaluate(sc, mt, cfg)
    assert ok, why
