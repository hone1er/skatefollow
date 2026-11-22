"""The emulated ESP32: compiled drive_logic.h + the mirrored loop glue behave like the sketch."""

import hashlib

import pytest

from honebot_sim.config import FIRMWARE_DIR, load_firmware_constants
from honebot_sim.firmware import SRC_NONE, SRC_PI, Esp32

FW = load_firmware_constants()

# The sketch functions mirrored by hand in honebot_sim/firmware.py. If one of these hashes changes, the sketch
# changed: update Esp32.line / update / wheel_outputs to match, then put the new hash here.
MIRRORED = {
    "void updateOutputs(": "6675b365fc98",
    "void handleLine(": "b7effa7893ee",
    "void applyOutputs(": "a3d34909dd2f",
    "bool parseCommand(": "1eac34ef75b5",
    "bool parsePan(": "595be73ee563",
    "void updateGyro(": "dbdc168b4b1a",
    "bool yawLoopUsable(": "61ff5e1bce66",
}


def _function_text(src, head):
    i = src.index(head)
    j = src.index("{", i)
    depth = 0
    for k in range(j, len(src)):
        depth += {"{": 1, "}": -1}.get(src[k], 0)
        if depth == 0:
            return src[i:k + 1]
    raise ValueError(head)


@pytest.mark.parametrize("head", sorted(MIRRORED))
def test_mirrored_sketch_code_unchanged(head):
    src = (FIRMWARE_DIR / "esp32_ultrasonic.ino").read_text()
    h = hashlib.sha1(_function_text(src, head).encode()).hexdigest()[:12]
    assert h == MIRRORED[head], ("%s changed in esp32_ultrasonic.ino: update sim/honebot_sim/firmware.py to match "
                                 "and set its hash to %s in this test" % (head, h))


def test_constants_parsed():
    for k in ("THROTTLE_LIMIT", "ACCEL_G", "DECEL_G", "FAILSAFE_DECEL_G", "FULL_SCALE_MPS", "ARM_MS",
              "WATCHDOG_MS", "CMD_TIMEOUT_MS", "FAILSAFE_MAX_MS", "REMOTE_REQUIRED", "ESC_DEADBAND"):
        assert k in FW, k


def _drive(esp, t0, t1, cmd="C,0.400,0.400", heartbeat=True):
    for t in range(t0, t1):
        if heartbeat and t % 100 == 0:
            esp.line("H", t)
        if cmd and t % 20 == 0:
            esp.line(cmd, t)
        esp.update(t)


def test_arming_window_holds_neutral():
    e = Esp32(FW)
    _drive(e, 0, int(FW["ARM_MS"]) - 1)
    assert e.failsafe and e.out_a == 0.0 and e.source == SRC_NONE


def test_drives_after_arming_with_slew_and_limit():
    e = Esp32(FW)
    _drive(e, 0, int(FW["ARM_MS"]) + 50)
    rate = FW["ACCEL_G"] * 9.81 / FW["FULL_SCALE_MPS"]
    assert e.source == SRC_PI
    assert 0.0 < e.out_a <= rate * 0.051 + 1e-6            # slewing up at the 0.8 g rate
    _drive(e, int(FW["ARM_MS"]) + 50, 4000, cmd="C,1.000,1.000")
    assert e.out_a == pytest.approx(FW["THROTTLE_LIMIT"])   # clamped to THROTTLE_LIMIT


def test_heartbeat_loss_ramps_down_at_failsafe_rate():
    e = Esp32(FW)
    _drive(e, 0, 4000)
    v0 = e.out_a
    _drive(e, 4000, 4000 + int(FW["WATCHDOG_MS"]) + 200, heartbeat=False)
    assert e.failsafe
    rate = FW["FAILSAFE_DECEL_G"] * 9.81 / FW["FULL_SCALE_MPS"]
    _drive(e, 4700, 4900, heartbeat=False)
    assert e.out_a < v0
    # last heartbeat at t = 3.9 s (sent every 100 ms); fail-safe starts WATCHDOG_MS later
    expected_drop = rate * (4.9 - (3.9 + FW["WATCHDOG_MS"] / 1000.0))
    assert v0 - e.out_a == pytest.approx(expected_drop, abs=0.02)


def test_remote_stop_and_never_heard_mean_no_drive():
    for remote in ("stop", "none"):
        e = Esp32(FW, remote=remote)
        _drive(e, 0, 4000)
        assert e.failsafe and e.out_a == 0.0, remote


def test_garbage_lines_ignored_and_values_clamped():
    e = Esp32(FW)
    e.line("C,nan,0.1", 0)
    e.line("C,abc", 0)
    assert not e.cmd_seen
    e.line("C,5,-5", 1)
    assert (e.cmd_a, e.cmd_b) == (1.0, -1.0)
    e.line("P,2", 1)
    assert e.cmd_pan == 1.0


def test_yaw_loop_track_width_matches_robot_yaml():
    """The firmware turns the two wheel commands back into a yaw rate with YAW_TRACK_WIDTH; the Pi built them
    with the bridge's track_width. They must be the same number."""
    from honebot_sim.config import load_robot_yaml
    assert FW["YAW_TRACK_WIDTH"] == pytest.approx(load_robot_yaml()["esp32_bridge"]["track_width"])


def test_yaw_loop_off_by_default_and_passthrough():
    assert FW["YAW_LOOP_ENABLED"] is False
    e = Esp32(FW)
    _drive(e, 0, 4000, cmd="C,0.200,0.300")
    assert (e.drive_a, e.drive_b) == (e.out_a, e.out_b)


def test_yaw_loop_needs_gyro_bias_window_then_corrects():
    fw = dict(FW, YAW_LOOP_ENABLED=True)
    e = Esp32(fw)
    for t in range(0, 2600):                      # stop before the saturation check (1 s pinned) trips the loop
        if t % 5 == 0:
            e.gyro_read(0.02 + (0.002 if t % 10 else -0.002), t)   # still, bias 0.02 rad/s, +-4 counts of noise
        if t % 100 == 0:
            e.line("H", t)
        if t % 20 == 0:
            e.line("C,0.200,0.250", t)
        e.update(t)
    assert e.gyro_bias == pytest.approx(0.02, abs=1e-4) and e.yaw_loop_usable(2599)
    assert e.drive_b - e.drive_a > e.out_b - e.out_a + 0.05     # tyres not turning -> the loop adds difference
    assert (e.drive_a + e.drive_b) / 2 == pytest.approx((e.out_a + e.out_b) / 2, abs=1e-6)



def test_a_frozen_gyro_reading_switches_the_loop_off():
    """2.4.0: GYRO_STUCK_SAMPLES identical raw readings in a row = stuck, treated as absent until restart."""
    fw = dict(FW, YAW_LOOP_ENABLED=True)
    e = Esp32(fw)
    freeze = 2100                                 # after the arming window, well before the 1 s saturation trip
    off_by = freeze + FW["GYRO_STUCK_SAMPLES"] * FW["GYRO_PERIOD_MS"] + FW["GYRO_STALE_MS"] + 10
    for t in range(0, int(off_by) + 1):
        if t % 5 == 0:
            e.gyro_read(0.02 + (0.002 if t % 10 else -0.002) if t < freeze else 0.0, t)
        if t % 100 == 0:
            e.line("H", t)
        if t % 20 == 0:
            e.line("C,0.200,0.250", t)
        e.update(t)
        if t == freeze - 1:
            assert e.yaw_loop_usable(t)
    assert not e.yaw_loop_usable(int(off_by))
    assert not e.L.hb_yaw_state_tripped(e.yaw_state)            # it was the stuck check, not the saturation trip
    assert (e.drive_a, e.drive_b) == (e.out_a, e.out_b)        # open loop, exactly as without a gyro


def test_yaw_loop_respects_throttle_limit():
    fw = dict(FW, YAW_LOOP_ENABLED=True, THROTTLE_LIMIT=0.5)
    e = Esp32(fw)
    for t in range(0, 2600):
        if t % 5 == 0:
            e.gyro_read(0.02 + (0.002 if t % 10 else -0.002), t)
        if t % 100 == 0:
            e.line("H", t)
        if t % 20 == 0:
            e.line("C,0.450,0.500", t)
        e.update(t)
        assert abs(e.drive_a) <= 0.5 + 1e-6 and abs(e.drive_b) <= 0.5 + 1e-6


def test_gyro_requires_min_samples_during_arming():
    fw = dict(FW, YAW_LOOP_ENABLED=True)
    e = Esp32(fw)
    # Only 10 samples during arming (below GYRO_BIAS_MIN_SAMPLES): bias must not lock in.
    for t in range(0, 50, 5):
        e.gyro_read(0.02 + (0.002 if t % 10 else -0.002), t)
    for t in range(int(FW["ARM_MS"]), int(FW["ARM_MS"]) + 200, 5):
        e.gyro_read(0.02 + (0.002 if t % 10 else -0.002), t)
    assert not e.gyro_bias_done
    assert not e.yaw_loop_usable(int(FW["ARM_MS"]) + 200)
