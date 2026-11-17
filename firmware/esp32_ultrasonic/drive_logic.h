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

inline bool fresh(bool seen, uint32_t last, uint32_t now, uint32_t timeout) {
  return seen && (uint32_t)(now - last) < timeout;
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

// Remove the deadband and rescale so the output starts from 0 at its edge.
inline float shapeStick(float v, float deadband) {
  float a = fabsf(v);
  if (a < deadband) return 0.0f;
  float out = (a - deadband) / (1.0f - deadband);
  if (out > 1.0f) out = 1.0f;
  return v < 0.0f ? -out : out;
}

}  // namespace drive
