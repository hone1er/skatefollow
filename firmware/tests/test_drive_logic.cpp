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

int main() {
  test_ramps();
  test_failsafe_ramp_time();
  test_mix_and_stick();
  test_decide_basic();
  test_decide_manual();
  test_remote_button();
  std::printf("ok: firmware logic tests passed\n");
  return 0;
}
