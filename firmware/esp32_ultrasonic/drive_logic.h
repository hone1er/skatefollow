// Pure drive logic for the honebot robot firmware: no Arduino calls, so it compiles and is unit-tested on a
// PC (firmware/tests/test_drive_logic.cpp). Everything that decides whether and how fast the motors turn
// lives here; the .ino only reads inputs, calls these, and writes pulses.
#pragma once
#include <math.h>
#include <stdint.h>

namespace drive {

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline float rampToward(float current, float target, float maxStep) {
  float delta = target - current;
  if (delta > maxStep) delta = maxStep;
  if (delta < -maxStep) delta = -maxStep;
  return current + delta;
}

// One channel moving toward its target: speeding up is limited by accelStep, slowing down (toward zero,
// including the first half of a direction change) by decelStep.
inline float slewChannel(float current, float target, float accelStep, float decelStep) {
  bool slowing = (current > 0.0f && target < current) || (current < 0.0f && target > current);
  if (!slowing) return rampToward(current, target, accelStep);
  // Slow toward zero first; never cross zero with the decel step.
  float stop = (current > 0.0f) ? fmaxf(target, 0.0f) : fminf(target, 0.0f);
  float next = rampToward(current, stop, decelStep);
  return next;
}

// Fail-safe ramp for two wheel channels: scale BOTH by the same factor so the larger one falls by `step`.
// Keeping the left/right ratio keeps the robot on the arc it was driving instead of veering as one side
// reaches zero first.
inline void rampPairToZero(float *a, float *b, float step) {
  float big = fmaxf(fabsf(*a), fabsf(*b));
  if (big <= step || big <= 1e-6f) {
    *a = 0.0f;
    *b = 0.0f;
    return;
  }
  float k = (big - step) / big;
  *a *= k;
  *b *= k;
}

// Arcade mix for manual driving: forward (-1..1), turn (-1..1, + = left) -> left/right wheel commands,
// scaled so neither exceeds `limit`.
inline void arcadeMix(float forward, float turn, float limit, float turnScale, float *left, float *right) {
  forward = clampf(forward, -1.0f, 1.0f);
  turn = clampf(turn, -1.0f, 1.0f) * turnScale;
  float l = forward - turn;
  float r = forward + turn;
  float big = fmaxf(fabsf(l), fabsf(r));
  if (big > 1.0f) {
    l /= big;
    r /= big;
  }
  *left = l * limit;
  *right = r * limit;
}

// ---------------------------------------------------------------------------------------------------
// Who may drive the motors.
enum Source : uint8_t { SRC_NONE = 0, SRC_PI = 1, SRC_MANUAL = 2 };
enum RemoteState : uint8_t { REMOTE_NEVER = 0, REMOTE_LOST = 1, REMOTE_STOP = 2, REMOTE_RUN = 3 };

struct Config {
  bool remoteRequired;        // true: no motion at all unless a fresh remote says RUN
  uint32_t armMs;             // hold neutral this long after boot (ESC arming)
  uint32_t watchdogMs;        // Pi heartbeat timeout
  uint32_t cmdTimeoutMs;      // Pi command timeout
  uint32_t remoteTimeoutMs;   // remote packet timeout
  uint32_t manualHoldMs;      // stay in manual this long after the stick returns to centre
  float stickDeadband;        // |stick| below this = centred
};

struct Inputs {
  uint32_t nowMs;
  bool heartbeatSeen;
  uint32_t lastHeartbeatMs;
  bool cmdSeen;
  uint32_t lastCmdMs;
  bool remoteSeen;            // a valid remote packet has arrived since boot
  uint32_t lastRemoteMs;      // robot's own clock at the last valid packet
  bool remoteRun;             // RUN flag in the last packet
  float stickX;               // forward, -1..1, from the last packet
  float stickY;               // turn, -1..1
};

struct State {
  bool centredSeenInRun = false;   // stick seen centred since RUN began (guards a loose wire / held stick)
  bool manualActive = false;
  uint32_t manualUntilMs = 0;
  bool prevRun = false;
};

struct Decision {
  Source source;
  RemoteState remote;
  bool piLink;
};

// `last` may be a millisecond AHEAD of `now`: the radio callback stamps packets on another task after loop()
// has already read the clock. Treat that as age 0 instead of letting the subtraction wrap to "49 days old".
inline bool fresh(bool seen, uint32_t last, uint32_t now, uint32_t timeout) {
  if (!seen) return false;
  int32_t age = (int32_t)(now - last);
  if (age < 0) age = 0;
  return (uint32_t)age < timeout;
}

// Decide who drives. Order matters and every "no" ends in SRC_NONE (motors ramp to neutral):
//   1. inside the arming window                          -> nobody
//   2. remote says STOP, or it was heard and is now lost -> nobody
//   3. remote required but never heard                   -> nobody
//   4. remote in RUN and stick deflected (after it was seen centred in RUN) -> manual
//   5. Pi heartbeat and command fresh                    -> Pi
inline Decision decide(const Config &c, const Inputs &in, State *st) {
  Decision d;
  d.piLink = fresh(in.heartbeatSeen, in.lastHeartbeatMs, in.nowMs, c.watchdogMs);
  bool remoteFresh = fresh(in.remoteSeen, in.lastRemoteMs, in.nowMs, c.remoteTimeoutMs);
  d.remote = !in.remoteSeen ? REMOTE_NEVER : (!remoteFresh ? REMOTE_LOST : (in.remoteRun ? REMOTE_RUN : REMOTE_STOP));

  bool run = d.remote == REMOTE_RUN;
  if (!run || !st->prevRun) {              // leaving RUN, or just entered it: the stick must be re-centred
    st->centredSeenInRun = false;
    st->manualActive = false;
  }
  st->prevRun = run;

  d.source = SRC_NONE;
  if (in.nowMs < c.armMs) return d;
  if (d.remote == REMOTE_STOP || d.remote == REMOTE_LOST) return d;
  if (d.remote == REMOTE_NEVER && c.remoteRequired) return d;

  if (run) {
    bool centred = fabsf(in.stickX) < c.stickDeadband && fabsf(in.stickY) < c.stickDeadband;
    if (centred) {
      st->centredSeenInRun = true;
    } else if (st->centredSeenInRun) {
      st->manualActive = true;
      st->manualUntilMs = in.nowMs + c.manualHoldMs;
    }
    if (st->manualActive && centred && (int32_t)(in.nowMs - st->manualUntilMs) >= 0) st->manualActive = false;
    if (st->manualActive) {
      d.source = SRC_MANUAL;
      return d;
    }
  }
  if (d.piLink && fresh(in.cmdSeen, in.lastCmdMs, in.nowMs, c.cmdTimeoutMs)) d.source = SRC_PI;
  return d;
}

// ESCs ignore a band of pulse widths around neutral and need a minimum duty before the motor starts, so small
// commands (a slow pivot is only ~2 % per side) would do nothing. Map any non-zero command onto
// [minCommand, 1]: zero stays exactly zero (neutral), everything else starts just above the dead zone.
inline float deadbandComp(float v, float minCommand) {
  float a = fabsf(v);
  if (a < 0.002f || minCommand <= 0.0f) return a < 0.002f ? 0.0f : v;
  float out = minCommand + (1.0f - minCommand) * a;
  if (out > 1.0f) out = 1.0f;
  return v < 0.0f ? -out : out;
}

// ---------------------------------------------------------------------------------------------------
// Yaw-rate loop (firmware >= 2.3.0, OFF unless YAW_LOOP_ENABLED and a gyro answers).
//
// Why: the ESCs set motor VOLTAGE, and a skid-steer robot's tyres only start sliding sideways (turning) once the
// left/right difference is big enough (~10 A per motor in the simulator). Small turn requests therefore do nothing
// and big ones spin it fast. The two wheel commands already encode the yaw rate the Pi or the stick asked for:
//     w_cmd = (b - a) * fullScaleMps / trackWidth          (the inverse of the Pi's diffdrive conversion)
// A gyro measures the real yaw rate and a PI term adds the missing difference. Only the DIFFERENCE is changed:
// the mean of the two channels (forward speed) is kept, and the correction is capped at maxCorr.
struct YawConfig {
  float kp;              // throttle per rad/s of yaw-rate error
  float ki;              // throttle per rad of accumulated error (per second)
  float maxCorr;         // cap on the half-difference added to each side (just above the scrub breakaway)
  float fullScaleMps;    // FULL_SCALE_MPS: m/s per unit command
  float trackWidth;      // must equal the bridge's track_width (robot.yaml)
  uint32_t satTimeoutMs; // correction pinned at maxCorr this long -> give up (stuck gyro, stuck wheel)
  uint32_t wrongWayMs;   // gyro says "turning hard the OTHER way" this long -> give up (gyro mounted upside down)
  float outLimit = 1.0f; // cap on each wheel output (THROTTLE_LIMIT), so yaw correction cannot bypass the bench cap
};

struct YawState {
  float integ = 0.0f;
  bool saturated = false;
  uint32_t satSinceMs = 0;
  bool wrongWay = false;
  uint32_t wrongSinceMs = 0;
  bool runaway = false;  // 2.4.0: correction and gyro disagree while the turn grows (see yawLoop)
  uint32_t runawaySinceMs = 0;
  float runawayStartW = 0.0f;
  bool tripped = false;  // latched after a fault; cleared by reset() (next fail-safe)
  void reset() {
    integ = 0.0f;
    saturated = false;
    wrongWay = false;
    runaway = false;
    tripped = false;
  }
};

// active: someone may drive and the gyro reading is valid. When inactive the outputs pass through unchanged and
// the integrator is cleared, so with the loop off or the gyro missing the behaviour is exactly that of 2.2.1.
inline void yawLoop(const YawConfig &c, YawState *s, bool active, float gyroZ, uint32_t nowMs, float dt,
                    float *a, float *b) {
  if (!active || s->tripped || c.trackWidth <= 0.0f) {
    s->integ = 0.0f;
    s->saturated = false;
    return;
  }
  if (fabsf(*a) < 1e-3f && fabsf(*b) < 1e-3f) {       // standing still: nothing to correct, don't wind up
    s->integ = 0.0f;
    s->saturated = false;
    return;
  }
  float wCmd = (*b - *a) * c.fullScaleMps / c.trackWidth;
  // A gyro with the wrong sign turns the loop into positive feedback: the robot spins up within a second. It shows
  // as the gyro reporting a fast turn opposite to the one asked for; real skids do not do that for long.
  bool wrong = wCmd * gyroZ < 0.0f && fabsf(gyroZ) > 1.0f && fabsf(gyroZ) > 0.5f * fabsf(wCmd);
  if (wrong && !s->wrongWay) s->wrongSinceMs = nowMs;
  s->wrongWay = wrong;
  if (wrong && (uint32_t)(nowMs - s->wrongSinceMs) >= c.wrongWayMs) {
    s->tripped = true;
    s->integ = 0.0f;
    return;
  }
  float e = wCmd - gyroZ;
  s->integ = clampf(s->integ + c.ki * e * dt, -c.maxCorr, c.maxCorr);
  float corr = clampf(c.kp * e + s->integ, -c.maxCorr, c.maxCorr);
  // Runaway (2.4.0): the check above needs the gyro to disagree with the turn ASKED for. When the Pi steers by the
  // same wrong gyro (the side-view follower dead-reckons with it), the request follows the gyro's sign and that
  // check never fires. Independent of the request: a correction pushing one way while the gyro reports a hard turn
  // the other way, and that turn not easing for wrongWayMs, means the correction feeds the spin (positive
  // feedback). With a correct gyro a correction against an overshoot makes the turn ease, so it does not trip.
  bool against = corr * gyroZ < 0.0f && fabsf(gyroZ) > 1.0f && fabsf(corr) > 0.5f * c.maxCorr;
  if (against && !s->runaway) {
    s->runawaySinceMs = nowMs;
    s->runawayStartW = fabsf(gyroZ);
  }
  s->runaway = against;
  if (against && (uint32_t)(nowMs - s->runawaySinceMs) >= c.wrongWayMs && fabsf(gyroZ) >= s->runawayStartW) {
    s->tripped = true;
    s->integ = 0.0f;
    return;
  }
  bool pinned = fabsf(corr) >= c.maxCorr * 0.999f;
  if (pinned && !s->saturated) s->satSinceMs = nowMs;
  s->saturated = pinned;
  if (pinned && (uint32_t)(nowMs - s->satSinceMs) >= c.satTimeoutMs) {
    s->tripped = true;                                   // back to open loop until the next fail-safe
    s->integ = 0.0f;
    return;
  }
  float lim = (c.outLimit > 0.0f && c.outLimit <= 1.0f) ? c.outLimit : 1.0f;
  float m = 0.5f * (*a + *b);
  float h = 0.5f * (*b - *a) + corr;
  float big = fabsf(m) + fabsf(h);
  if (big > lim) h = (h > 0.0f ? 1.0f : -1.0f) * fmaxf(0.0f, lim - fabsf(m));   // keep the mean, shrink the turn
  *a = clampf(m - h, -lim, lim);
  *b = clampf(m + h, -lim, lim);
}

// Stuck gyro (2.4.0). A working MPU6050 always shows a little noise: its raw reading changes by a count or so
// from sample to sample. One that hangs and keeps returning the same value reads "not turning" while the loop keeps
// adding difference just under maxCorr (so the saturation trip never fires): in the simulator the tyres then break
// loose and the robot spins over, at 12 mph and even at 6 mph. `limit` identical raw readings in a row -> stuck,
// latched until restart; the sketch then treats the gyro as absent (loop off, G line not usable).
struct StuckWatch {
  int16_t last = 0;
  uint16_t same = 0;
  bool stuck = false;
};

inline bool stuckReading(StuckWatch *w, int16_t raw, uint16_t limit) {
  if (raw == w->last) {
    if (w->same < 65535) w->same++;
  } else {
    w->same = 0;
    w->last = raw;
  }
  if (limit > 0 && w->same >= limit) w->stuck = true;
  return w->stuck;
}

// Stick expo: fine control near the centre, full value at the end. e = 0 linear, e = 1 cubic.
inline float expo(float v, float e) {
  v = clampf(v, -1.0f, 1.0f);
  return (1.0f - e) * v + e * v * v * v;
}

// Remove the deadband and rescale so the output starts from 0 at its edge.
inline float shapeStick(float v, float deadband) {
  float a = fabsf(v);
  if (a < deadband) return 0.0f;
  float out = (a - deadband) / (1.0f - deadband);
  if (out > 1.0f) out = 1.0f;
  return v < 0.0f ? -out : out;
}

}  // namespace drive
