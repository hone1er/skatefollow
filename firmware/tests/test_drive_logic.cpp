// Host tests for the firmware's pure logic. Run: firmware/tests/run_tests.sh
#include <cassert>
#include <cmath>
#include <cstdio>

#include "../esp32_ultrasonic/drive_logic.h"
#include "../esp32_remote/remote_logic.h"

using namespace drive;

static const Config REQ = {true, 2000, 500, 200, 500, 800, 0.15f};
static const Config BENCH = {false, 2000, 500, 200, 500, 800, 0.15f};
static bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) < eps; }

static Inputs piOk(uint32_t now) {
  Inputs in{};
  in.nowMs = now;
  in.heartbeatSeen = true; in.lastHeartbeatMs = now - 50;
  in.cmdSeen = true; in.lastCmdMs = now - 10;
  return in;
}
static Inputs withRemote(Inputs in, bool run, float x = 0, float y = 0, uint32_t age = 10) {
  in.remoteSeen = true; in.lastRemoteMs = in.nowMs - age; in.remoteRun = run; in.stickX = x; in.stickY = y;
  return in;
}

static void test_ramps() {
  assert(near(rampToward(0, 1, 0.1f), 0.1f) && near(rampToward(0.05f, 0, 0.1f), 0));
  // speeding up uses accel, slowing uses decel, a reversal stops at zero first
  assert(near(slewChannel(0.2f, 0.5f, 0.01f, 0.05f), 0.21f));
  assert(near(slewChannel(0.2f, 0.0f, 0.01f, 0.05f), 0.15f));
  assert(near(slewChannel(0.02f, -0.5f, 0.01f, 0.05f), 0.0f));
  assert(near(slewChannel(0.0f, -0.5f, 0.01f, 0.05f), -0.01f));
  assert(near(slewChannel(-0.2f, -0.1f, 0.01f, 0.05f), -0.15f));
  // fail-safe pair ramp keeps the left/right ratio and ends exactly at zero
  float a = 0.4f, b = 0.2f;
  rampPairToZero(&a, &b, 0.1f);
  assert(near(a, 0.3f) && near(b, 0.15f) && near(a / b, 2.0f));
  for (int i = 0; i < 10; i++) rampPairToZero(&a, &b, 0.1f);
  assert(a == 0.0f && b == 0.0f);
  a = -0.3f; b = 0.3f;                       // pivoting: both shrink together
  rampPairToZero(&a, &b, 0.1f);
  assert(near(a, -0.2f) && near(b, 0.2f));
}

static void test_failsafe_ramp_time() {
  // 0.5 g from full scale 9.6 m/s: command 0.7 (15 mph) must reach zero in ~1.4 s, not instantly
  const float rate = 0.5f * 9.81f / 9.6f;
  float a = 0.7f, b = 0.7f, t = 0;
  while (a > 0) { rampPairToZero(&a, &b, rate * 0.01f); t += 0.01f; }
  assert(t > 1.2f && t < 1.5f);
}

static void test_mix_and_stick() {
  float l, r;
  arcadeMix(1, 0, 0.25f, 0.6f, &l, &r); assert(near(l, 0.25f) && near(r, 0.25f));
  arcadeMix(0, 1, 0.25f, 0.6f, &l, &r); assert(near(l, -0.15f) && near(r, 0.15f));   // + turn = left
  arcadeMix(1, 1, 0.25f, 0.6f, &l, &r); assert(std::fabs(r) <= 0.25f + 1e-6f && r > l && l > 0);
  // ESC deadband compensation: zero stays neutral, tiny commands land just above the dead zone, full stays full
  assert(deadbandComp(0.0f, 0.06f) == 0.0f && deadbandComp(0.001f, 0.06f) == 0.0f);
  assert(near(deadbandComp(0.02f, 0.06f), 0.06f + 0.94f * 0.02f) && near(deadbandComp(-0.02f, 0.06f), -(0.06f + 0.94f * 0.02f)));
  assert(near(deadbandComp(1.0f, 0.06f), 1.0f) && near(deadbandComp(0.3f, 0.0f), 0.3f));
  assert(shapeStick(0.1f, 0.15f) == 0.0f && near(shapeStick(1.0f, 0.15f), 1.0f) && shapeStick(-0.5f, 0.15f) < 0);
}

static void test_decide_basic() {
  State st;
  // arming window: nobody, even with everything healthy
  assert(decide(BENCH, piOk(1000), &st).source == SRC_NONE);
  // bench mode, remote never heard: Pi drives
  assert(decide(BENCH, piOk(5000), &st).source == SRC_PI);
  // remote required, never heard: nobody
  st = State();
  Decision d = decide(REQ, piOk(5000), &st);
  assert(d.source == SRC_NONE && d.remote == REMOTE_NEVER && d.piLink);
  // required + RUN + centred: Pi drives
  d = decide(REQ, withRemote(piOk(5000), true), &st);
  assert(d.source == SRC_PI && d.remote == REMOTE_RUN);
  // STOP beats the Pi, in both modes
  assert(decide(REQ, withRemote(piOk(5010), false), &st).source == SRC_NONE);
  st = State();
  assert(decide(BENCH, withRemote(piOk(5010), false), &st).source == SRC_NONE);
  // heard once, then silent: nobody (also in bench mode)
  st = State();
  d = decide(BENCH, withRemote(piOk(6000), true, 0, 0, 600), &st);
  assert(d.source == SRC_NONE && d.remote == REMOTE_LOST);
  // Pi heartbeat lost / command stale: nobody
  st = State();
  Inputs in = withRemote(piOk(7000), true);
  in.lastHeartbeatMs = 7000 - 600;
  assert(decide(REQ, in, &st).source == SRC_NONE);
  in = withRemote(piOk(7000), true);
  in.lastCmdMs = 7000 - 300;
  assert(decide(REQ, in, &st).source == SRC_NONE);
}

static void test_decide_manual() {
  State st;
  // stick already deflected when RUN begins (loose wire / held stick): no manual until seen centred
  Decision d = decide(REQ, withRemote(piOk(5000), true, 0.8f, 0), &st);
  assert(d.source == SRC_PI);
  d = decide(REQ, withRemote(piOk(5020), true, 0.8f, 0), &st);
  assert(d.source == SRC_PI);
  d = decide(REQ, withRemote(piOk(5040), true, 0.0f, 0), &st);      // centred once
  assert(d.source == SRC_PI);
  d = decide(REQ, withRemote(piOk(5060), true, 0.5f, 0), &st);      // now a deflection counts
  assert(d.source == SRC_MANUAL);
  // manual works with the Pi dead
  Inputs dead{}; dead.nowMs = 5080;
  d = decide(REQ, withRemote(dead, true, 0.5f, 0), &st);
  assert(d.source == SRC_MANUAL && !d.piLink);
  // released: stays manual (commanding zero) for the hold time, then the Pi may drive again
  d = decide(REQ, withRemote(piOk(5100), true, 0, 0), &st);
  assert(d.source == SRC_MANUAL);
  d = decide(REQ, withRemote(piOk(5100 + 900), true, 0, 0), &st);
  assert(d.source == SRC_PI);
  // STOP during manual: nobody, and RUN again needs a centred stick again
  decide(REQ, withRemote(piOk(6100), true, 0.5f, 0), &st);
  assert(decide(REQ, withRemote(piOk(6120), false, 0.5f, 0), &st).source == SRC_NONE);
  assert(decide(REQ, withRemote(piOk(6140), true, 0.5f, 0), &st).source == SRC_PI);
  // remote lost during manual: nobody
  st = State();
  decide(REQ, withRemote(piOk(7000), true, 0, 0), &st);
  assert(decide(REQ, withRemote(piOk(7020), true, 0.6f, 0), &st).source == SRC_MANUAL);
  assert(decide(REQ, withRemote(piOk(7700), true, 0.6f, 0, 680), &st).source == SRC_NONE);
  // a packet stamped just after loop() read the clock (timestamp 1 ms in the future) is fresh, and manual holds
  st = State();
  decide(REQ, withRemote(piOk(8000), true, 0, 0), &st);
  assert(decide(REQ, withRemote(piOk(8020), true, 0.6f, 0), &st).source == SRC_MANUAL);
  Inputs ahead = withRemote(piOk(8040), true, 0.6f, 0);
  ahead.lastRemoteMs = ahead.nowMs + 1;
  Decision da = decide(REQ, ahead, &st);
  assert(da.remote == REMOTE_RUN && da.source == SRC_MANUAL);
  // near the 32-bit millis() wrap, unsigned subtraction still gives the right ages
  st = State();
  Inputs w{};
  w.nowMs = 0x00000010u;                     // just wrapped (arming is ignored here: bench config with armMs 0)
  w.heartbeatSeen = true; w.lastHeartbeatMs = 0xFFFFFFF0u;      // 32 ms ago
  w.cmdSeen = true; w.lastCmdMs = 0xFFFFFFFAu;                  // 22 ms ago
  const Config noArm = {false, 0, 500, 200, 500, 800, 0.15f};
  assert(decide(noArm, w, &st).source == SRC_PI);
}

static void test_remote_button() {
  using namespace remote;
  ButtonState b;
  uint32_t t = 0;
  assert(!updateRun(&b, false, true, t, 1500));                 // boots in STOP
  // short click in STOP: still STOP
  updateRun(&b, true, true, t += 10, 1500);
  updateRun(&b, true, true, t += 500, 1500);
  assert(!updateRun(&b, false, true, t += 10, 1500));
  // hold 1.5 s with the stick centred: RUN
  updateRun(&b, true, true, t += 10, 1500);
  assert(!updateRun(&b, true, true, t += 1400, 1500));
  assert(updateRun(&b, true, true, t += 200, 1500));
  assert(updateRun(&b, false, true, t += 10, 1500));            // releasing keeps RUN
  // any click in RUN: STOP at once, and holding that same press does not re-arm
  assert(!updateRun(&b, true, true, t += 10, 1500));
  assert(!updateRun(&b, true, true, t += 3000, 1500));
  assert(!updateRun(&b, false, true, t += 10, 1500));
  // hold with the stick deflected at any moment: refused
  updateRun(&b, true, true, t += 10, 1500);
  updateRun(&b, true, false, t += 500, 1500);
  assert(!updateRun(&b, true, true, t += 1500, 1500));
  updateRun(&b, false, true, t += 10, 1500);
  updateRun(&b, true, false, t += 10, 1500);                    // press begins deflected
  assert(!updateRun(&b, true, true, t += 2000, 1500));
  // axis scaling
  assert(near(scaleAxis(2000, 2000, 4095, false), 0) && near(scaleAxis(4095, 2000, 4095, false), 1));
  assert(near(scaleAxis(0, 2000, 4095, false), -1) && near(scaleAxis(0, 2000, 4095, true), 1));
  assert(centreLooksValid(1900, 4095) && !centreLooksValid(0, 4095) && !centreLooksValid(4095, 4095));
  assert(toWire(1.0f) == 100 && toWire(-2.0f) == -100 && toWire(0.004f) == 0);
}

static const YawConfig YAW = {0.12f, 0.8f, 0.2f, 9.6f, 0.39f, 1000, 150, 1.0f};

static void test_yaw_loop_inactive_is_passthrough() {
  // Off, fail-safe or no gyro: outputs untouched and the integrator cleared (behaviour of 2.2.1).
  YawState st;
  st.integ = 0.1f;
  float a = 0.20f, b = 0.25f;
  yawLoop(YAW, &st, false, 3.0f, 1000, 0.005f, &a, &b);
  assert(a == 0.20f && b == 0.25f && st.integ == 0.0f);
  // standing still: no correction, no wind-up
  a = 0.0f; b = 0.0f;
  yawLoop(YAW, &st, true, 0.5f, 1000, 0.005f, &a, &b);
  assert(a == 0.0f && b == 0.0f && st.integ == 0.0f);
}

static void test_yaw_loop_adds_difference_only() {
  // A gentle left turn the tyres ignore (gyro reads 0): the loop widens the difference, keeps the mean, caps it.
  YawState st;
  float a0 = 0.20f, b0 = 0.25f;               // asks for (0.05 * 9.6 / 0.39) = 1.23 rad/s
  float a = a0, b = b0;
  for (int i = 0; i < 100; i++) {
    a = a0; b = b0;
    yawLoop(YAW, &st, true, 0.0f, 1000 + 5 * i, 0.005f, &a, &b);
    assert(near(0.5f * (a + b), 0.5f * (a0 + b0)));          // forward speed command unchanged
    assert(0.5f * (b - a) - 0.5f * (b0 - a0) <= YAW.maxCorr + 1e-5f);
  }
  assert(b - a > b0 - a0 + 0.1f);                             // grew toward the cap
  // when the robot really turns at the asked rate, the proportional part vanishes
  YawState st2;
  a = a0; b = b0;
  yawLoop(YAW, &st2, true, 1.2308f, 1000, 0.005f, &a, &b);
  assert(near(a, a0, 1e-3f) && near(b, b0, 1e-3f));
}

static void test_yaw_loop_trips_on_wrong_sign() {
  // Gyro mounted upside down: it reports a fast turn opposite to the one asked for. After wrongWayMs the loop
  // gives up (open loop) until reset() at the next fail-safe.
  YawState st;
  float a = 0.20f, b = 0.25f;
  uint32_t t = 1000;
  for (; t < 1000 + 140; t += 5) {
    a = 0.20f; b = 0.25f;
    yawLoop(YAW, &st, true, -1.2f, t, 0.005f, &a, &b);
  }
  assert(!st.tripped);
  for (; t < 1000 + 200; t += 5) {
    a = 0.20f; b = 0.25f;
    yawLoop(YAW, &st, true, -1.2f, t, 0.005f, &a, &b);
  }
  assert(st.tripped);
  a = 0.20f; b = 0.25f;
  yawLoop(YAW, &st, true, -1.2f, t, 0.005f, &a, &b);
  assert(a == 0.20f && b == 0.25f);
  st.reset();
  assert(!st.tripped && st.integ == 0.0f);
}

static void test_yaw_loop_trips_on_stuck_gyro() {
  // Gyro stuck at zero: the correction pins at the cap; after satTimeoutMs the loop gives up.
  YawState st;
  uint32_t t = 1000;
  for (; t < 1000 + 1100; t += 5) {
    float a = 0.20f, b = 0.25f;
    yawLoop(YAW, &st, true, 0.0f, t, 0.005f, &a, &b);
  }
  assert(st.tripped);
}

static void test_yaw_loop_keeps_within_full_scale() {
  YawState st;
  for (int i = 0; i < 400; i++) {
    float a = 0.9f, b = 1.0f;                 // near full speed, asking to turn
    yawLoop(YAW, &st, true, 0.0f, 1000 + 5 * i, 0.005f, &a, &b);
    assert(std::fabs(a) <= 1.0f + 1e-6f && std::fabs(b) <= 1.0f + 1e-6f);
    assert(near(0.5f * (a + b), 0.95f));
  }
  // Bench THROTTLE_LIMIT (0.5): yaw correction must never push either wheel past outLimit.
  YawConfig benchYaw = {0.12f, 0.8f, 0.2f, 9.6f, 0.39f, 1000, 150, 0.5f};
  YawState st2;
  for (int i = 0; i < 200; i++) {
    float a = 0.45f, b = 0.50f;
    yawLoop(benchYaw, &st2, true, 0.0f, 1000 + 5 * i, 0.005f, &a, &b);
    assert(std::fabs(a) <= 0.5f + 1e-6f && std::fabs(b) <= 0.5f + 1e-6f);
    assert(near(0.5f * (a + b), 0.475f));
  }
  assert(near(expo(0.5f, 0.5f), 0.3125f) && near(expo(1.0f, 0.5f), 1.0f) && near(expo(-1.0f, 0.5f), -1.0f));
}

static void test_yaw_loop_trips_on_runaway_even_when_the_request_follows_the_gyro() {
  // Upside-down gyro while the Pi steers by that same gyro: the request agrees with the gyro's (wrong) sign, so the
  // wrong-way check stays quiet. The correction pushes one way, the gyro reports a growing turn the other way.
  YawState st;
  uint32_t t = 1000;
  float g = 1.1f;
  for (; t < 1000 + 70; t += 5) {
    float a = 0.30f, b = 0.40f;                 // asks to turn left (+2.5 rad/s): gyro agrees in sign, so e < 0 ...
    st.integ = -0.2f;                           // ... and the correction is pinned to the right (negative)
    yawLoop(YAW, &st, true, g, t, 0.005f, &a, &b);
    g += 0.2f;                                  // ... while the reported turn keeps growing: positive feedback
  }
  assert(!st.tripped);
  for (; t < 1000 + 200; t += 5) {
    float a = 0.30f, b = 0.40f;
    st.integ = -0.2f;
    yawLoop(YAW, &st, true, g, t, 0.005f, &a, &b);
    g += 0.2f;
  }
  assert(st.tripped);
  // A correct gyro braking an overshoot: correction against the turn, but the turn eases -> no trip.
  YawState ok;
  g = 3.0f;
  for (t = 1000; t < 1000 + 400; t += 5) {
    float a = 0.30f, b = 0.35f;                 // asks 1.2 rad/s, robot overshot to 3 rad/s and is slowing
    yawLoop(YAW, &ok, true, g, t, 0.005f, &a, &b);
    g = std::fmax(1.2f, g - 0.05f);
  }
  assert(!ok.tripped);
}

static void test_stuck_gyro_reading() {
  // Noise (a count or so either way) never trips it; a frozen value does after `limit` repeats, and stays tripped.
  StuckWatch w;
  int16_t noisy[] = {3, 4, 3, 2, 3, 3, 4};
  for (int i = 0; i < 1000; i++) assert(!stuckReading(&w, noisy[i % 7], 100));
  StuckWatch f;
  for (int i = 0; i < 100; i++) assert(!stuckReading(&f, 17, 100));
  assert(stuckReading(&f, 17, 100));                 // 100 repeats after the first reading
  assert(stuckReading(&f, 18, 100));                 // latched: a later change does not clear it
  StuckWatch off;
  for (int i = 0; i < 500; i++) assert(!stuckReading(&off, 5, 0));   // limit 0 = check off
}

int main() {
  test_ramps();
  test_failsafe_ramp_time();
  test_mix_and_stick();
  test_decide_basic();
  test_decide_manual();
  test_remote_button();
  test_yaw_loop_inactive_is_passthrough();
  test_yaw_loop_adds_difference_only();
  test_yaw_loop_trips_on_wrong_sign();
  test_yaw_loop_trips_on_stuck_gyro();
  test_yaw_loop_keeps_within_full_scale();
  test_yaw_loop_trips_on_runaway_even_when_the_request_follows_the_gyro();
  test_stuck_gyro_reading();
  std::printf("ok: firmware logic tests passed\n");
  return 0;
}
