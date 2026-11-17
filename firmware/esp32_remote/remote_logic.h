// Pure logic for the handheld remote: STOP/RUN button state machine and stick scaling. No Arduino calls,
// unit-tested on a PC (firmware/tests/test_drive_logic.cpp).
#pragma once
#include <math.h>
#include <stdint.h>

namespace remote {

// STOP is easy, RUN is deliberate:
//   - boots in STOP
//   - ANY press while in RUN -> STOP at once (the button is the stick's own click: easy to hit by accident,
//     and an accidental STOP is the safe mistake)
//   - RUN only by holding the button for holdMs, starting from STOP, with the stick centred the whole time,
//     after the button has been released once since the last change (so one long press cannot stop and re-arm)
struct ButtonState {
  bool run = false;
  bool lastPressed = false;
  bool holdValid = false;       // the current press started in STOP with the stick centred
  bool changedThisPress = false;
  uint32_t pressStartMs = 0;
};

inline bool updateRun(ButtonState *s, bool pressed, bool stickCentred, uint32_t nowMs, uint32_t holdMs) {
  if (pressed && !s->lastPressed) {            // press edge
    s->pressStartMs = nowMs;
    s->changedThisPress = false;
    if (s->run) {
      s->run = false;
      s->changedThisPress = true;
      s->holdValid = false;
    } else {
      s->holdValid = stickCentred;
    }
  }
  if (pressed && !s->run) {
    if (!stickCentred) s->holdValid = false;
    if (s->holdValid && !s->changedThisPress && (uint32_t)(nowMs - s->pressStartMs) >= holdMs) {
      s->run = true;
      s->changedThisPress = true;
      s->holdValid = false;
    }
  }
  s->lastPressed = pressed;
  return s->run;
}

// Raw ADC reading -> -1..1 around the centre measured at power-up. Each side is scaled by its own travel so
// an off-centre stick still reaches +/-1.
inline float scaleAxis(int raw, int centre, int adcMax, bool reversed) {
  float v;
  if (raw >= centre) v = (adcMax - centre) > 0 ? (float)(raw - centre) / (float)(adcMax - centre) : 0.0f;
  else v = centre > 0 ? (float)(raw - centre) / (float)centre : 0.0f;
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return reversed ? -v : v;
}

// A resting stick reads near mid-scale. Far from it at power-up means a loose wire or a held stick.
inline bool centreLooksValid(int centre, int adcMax) {
  return centre > adcMax * 3 / 10 && centre < adcMax * 7 / 10;
}

inline int8_t toWire(float v) {
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return (int8_t)lroundf(v * 100.0f);
}

}  // namespace remote
