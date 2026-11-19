// C entry points around the robot firmware's pure logic (firmware/esp32_ultrasonic/drive_logic.h), so the
// simulator runs the SAME code the ESP32 runs instead of a Python copy of it. Built on first use by
// honebot_sim/firmware.py with the system C++ compiler (Xcode's clang on the Mac, g++ on Linux).
#include "drive_logic.h"

extern "C" {

void *hb_state_new() { return new drive::State(); }
void hb_state_free(void *s) { delete static_cast<drive::State *>(s); }

int hb_decide(void *state, int remoteRequired, unsigned armMs, unsigned watchdogMs, unsigned cmdTimeoutMs,
              unsigned remoteTimeoutMs, unsigned manualHoldMs, float stickDeadband, unsigned nowMs,
              int heartbeatSeen, unsigned lastHeartbeatMs, int cmdSeen, unsigned lastCmdMs, int remoteSeen,
              unsigned lastRemoteMs, int remoteRun, float stickX, float stickY, int *remoteOut, int *piLinkOut) {
  drive::Config c = {remoteRequired != 0, armMs, watchdogMs, cmdTimeoutMs, remoteTimeoutMs, manualHoldMs,
                     stickDeadband};
  drive::Inputs in;
  in.nowMs = nowMs;
  in.heartbeatSeen = heartbeatSeen != 0;
  in.lastHeartbeatMs = lastHeartbeatMs;
  in.cmdSeen = cmdSeen != 0;
  in.lastCmdMs = lastCmdMs;
  in.remoteSeen = remoteSeen != 0;
  in.lastRemoteMs = lastRemoteMs;
  in.remoteRun = remoteRun != 0;
  in.stickX = stickX;
  in.stickY = stickY;
  drive::Decision d = drive::decide(c, in, static_cast<drive::State *>(state));
  *remoteOut = (int)d.remote;
  *piLinkOut = d.piLink ? 1 : 0;
  return (int)d.source;
}

float hb_ramp_toward(float current, float target, float maxStep) { return drive::rampToward(current, target, maxStep); }
float hb_slew_channel(float c, float t, float up, float down) { return drive::slewChannel(c, t, up, down); }
void hb_ramp_pair_to_zero(float *a, float *b, float step) { drive::rampPairToZero(a, b, step); }
float hb_deadband_comp(float v, float minCommand) { return drive::deadbandComp(v, minCommand); }
float hb_shape_stick(float v, float deadband) { return drive::shapeStick(v, deadband); }
void hb_arcade_mix(float f, float t, float limit, float turnScale, float *l, float *r) {
  drive::arcadeMix(f, t, limit, turnScale, l, r);
}

void *hb_yaw_state_new() { return new drive::YawState(); }
void hb_yaw_state_free(void *s) { delete static_cast<drive::YawState *>(s); }
void hb_yaw_state_reset(void *s) { static_cast<drive::YawState *>(s)->reset(); }
int hb_yaw_state_tripped(void *s) { return static_cast<drive::YawState *>(s)->tripped ? 1 : 0; }

void hb_yaw_loop(void *state, float kp, float ki, float maxCorr, float fullScale, float track, unsigned satMs,
                 unsigned wrongWayMs, int active, float gyroZ, unsigned nowMs, float dt, float *a, float *b) {
  drive::YawConfig c = {kp, ki, maxCorr, fullScale, track, satMs, wrongWayMs};
  drive::yawLoop(c, static_cast<drive::YawState *>(state), active != 0, gyroZ, nowMs, dt, a, b);
}

float hb_expo(float v, float e) { return drive::expo(v, e); }

}  // extern "C"
