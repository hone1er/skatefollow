// honebot ESP32 firmware: ultrasonic sensor, Pi heartbeat watchdog, throttle/steering
// servo outputs with fail-safe, and a camera pan servo channel.
//
// Serial protocol (115200 baud, one line per message, '\n' terminated):
//   ESP32 -> Pi :  V,<version>                  once at boot
//                  R,<distance_m>,<link_ok>     e.g. "R,0.532,1", every MEASURE_PERIOD_MS
//                     distance_m = -1 when no echo (nothing in range)
//                     link_ok    = 1 if a heartbeat arrived in the last WATCHDOG_MS
//                  S,<failsafe>,<throttle>,<steer>   every STATUS_PERIOD_MS
//                     failsafe = 1 while outputs are forced to neutral
//                     throttle/steer = what is actually being output (-1..1)
//   Pi -> ESP32 :  H                             heartbeat, ~10x per second
//                  C,<a>,<b>                     normalised -1..1, ~50x per second.
//                     Two output channels. Differential drive (the custom robot):
//                     a = left wheel ESC, b = right wheel ESC. Ackermann car:
//                     a = throttle ESC, b = steering servo. The Pi decides (drive_mode).
//                  P,<pan>                       camera pan servo, normalised -1..1 (+1 = full left),
//                     ~20x per second from camera_aim. Not safety-relevant: the servo holds the last
//                     value in fail-safe and is centred at boot. Slew-limited (PAN_RATE_PER_S).
//
// Fail-safe (throttle neutral, steering centred) whenever ANY of:
//   - no heartbeat for WATCHDOG_MS
//   - no C command for CMD_TIMEOUT_MS
//   - still inside the ARM_MS boot window (ESC arming: neutral for ~2 s)
// The onboard LED is ON while the Pi link is healthy.
//
// Wiring (ESP32 DevKit / WROOM-32):
//   HC-SR04 VCC  -> ESP32 VIN/5V
//   HC-SR04 GND  -> ESP32 GND
//   HC-SR04 Trig -> GPIO 5
//   HC-SR04 Echo -> 1k resistor -> GPIO 18, and GPIO 18 -> 2k resistor -> GND
//                  (divider drops the 5 V echo to ~3.3 V)
//   Channel A (left ESC / throttle)  -> GPIO 25 (THROTTLE_PIN); ESC GND -> ESP32 GND
//   Channel B (right ESC / steering) -> GPIO 26 (STEER_PIN)
//   Camera pan servo signal            -> GPIO 27 (PAN_PIN); servo + from a 5-6 V supply, GND common
//   VESC-based ESCs accept this 50 Hz 1000-2000 us signal on their PPM input.
//   Both channels are forced to 1500 us (neutral) in fail-safe.
//
// Requires the esp32 Arduino core 3.x (ledcAttach / ledcWrite by pin).

#include <math.h>

const char *FW_VERSION = "2.1.0";

const int TRIG_PIN = 5;
const int ECHO_PIN = 18;
const int LED_PIN = 2;                     // onboard LED on most DevKits
const int THROTTLE_PIN = 25;
const int STEER_PIN = 26;
const int PAN_PIN = 27;                    // camera pan servo

const unsigned long MEASURE_PERIOD_MS = 60;  // ~16 Hz; HC-SR04 datasheet asks for >= 60 ms between pings
const unsigned long STATUS_PERIOD_MS = 100;
const unsigned long WATCHDOG_MS = 500;       // link lost after this long without "H"
const unsigned long CMD_TIMEOUT_MS = 200;    // neutral after this long without "C"
const unsigned long ARM_MS = 2000;           // hold neutral this long after boot (ESC arming)
const unsigned long ECHO_TIMEOUT_US = 25000; // ~4.3 m max range

// Servo pulse settings (standard RC: 50 Hz, 1000-2000 us, 1500 neutral)
const uint32_t SERVO_FREQ_HZ = 50;
const uint8_t SERVO_RES_BITS = 16;
const int PULSE_MIN_US = 1000;
const int PULSE_NEUTRAL_US = 1500;
const int PULSE_MAX_US = 2000;
const int STEER_TRIM_US = 0;               // adjust so the wheels are straight at steer = 0
const bool STEER_REVERSED = false;
const bool THROTTLE_REVERSED = false;
const int PAN_TRIM_US = 0;                 // adjust so the camera looks straight ahead at pan = 0
const bool PAN_REVERSED = false;           // or set pan_sign:=-1 on the Pi side
const float PAN_RATE_PER_S = 3.0f;         // slew limit on the pan command (full travel in ~0.7 s)

// Bench safety, independent of the Pi: cap and rate-limit the drive channels.
// Differential drive: BOTH channels are wheel throttles -> both capped/ramped.
// Ackermann: only channel A is a throttle; channel B (steering) is passed through.
const bool DIFF_DRIVE = true;              // false for an RC car (A = throttle, B = steering)
const float THROTTLE_LIMIT = 0.5f;         // raise toward 1.0 once the robot behaves
const float THROTTLE_RATE_PER_S = 2.0f;    // max change in normalised throttle per second

unsigned long lastHeartbeatMs = 0;
unsigned long lastCmdMs = 0;
unsigned long lastMeasureMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastLoopMs = 0;
bool linkOk = false;
bool failsafe = true;

float cmdThrottle = 0.0f;                  // last command from the Pi
float cmdSteer = 0.0f;
float outThrottle = 0.0f;                  // what is actually being output
float outSteer = 0.0f;
float cmdPan = 0.0f;                       // camera pan: last command and what is output
float outPan = 0.0f;

char rxBuf[48];
size_t rxLen = 0;

// --- servo output ---------------------------------------------------------
void writePulseUs(int pin, int us) {
  us = constrain(us, PULSE_MIN_US, PULSE_MAX_US);
  const uint32_t periodUs = 1000000UL / SERVO_FREQ_HZ;                 // 20000
  const uint32_t maxDuty = (1UL << SERVO_RES_BITS) - 1;
  uint32_t duty = (uint32_t)(((uint64_t)us * maxDuty) / periodUs);
  ledcWrite(pin, duty);
}

int normToUs(float norm, int trimUs, bool reversed) {
  norm = constrain(norm, -1.0f, 1.0f);
  if (reversed) norm = -norm;
  return PULSE_NEUTRAL_US + trimUs + (int)(norm * (PULSE_MAX_US - PULSE_NEUTRAL_US));
}

void applyOutputs() {
  writePulseUs(THROTTLE_PIN, normToUs(outThrottle, 0, THROTTLE_REVERSED));
  writePulseUs(STEER_PIN, normToUs(outSteer, STEER_TRIM_US, STEER_REVERSED));
  writePulseUs(PAN_PIN, normToUs(outPan, PAN_TRIM_US, PAN_REVERSED));
}

// --- sensor -----------------------------------------------------------------
float readDistanceMeters() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long echoUs = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (echoUs == 0) return -1.0f;           // no echo
  return echoUs * 0.000343f / 2.0f;        // speed of sound 343 m/s, there and back
}

// --- serial input -----------------------------------------------------------
bool parseCommand(const char *line, float *throttle, float *steer) {
  // Expect "C,<float>,<float>"
  if (line[0] != 'C' || line[1] != ',') return false;
  char *end = nullptr;
  float t = strtof(line + 2, &end);
  if (end == line + 2 || *end != ',') return false;
  const char *p = end + 1;
  float s = strtof(p, &end);
  if (end == p || *end != '\0') return false;
  if (isnan(t) || isinf(t) || isnan(s) || isinf(s)) return false;
  *throttle = constrain(t, -1.0f, 1.0f);
  *steer = constrain(s, -1.0f, 1.0f);
  return true;
}

bool parsePan(const char *line, float *pan) {
  // Expect "P,<float>"
  if (line[0] != 'P' || line[1] != ',') return false;
  char *end = nullptr;
  float v = strtof(line + 2, &end);
  if (end == line + 2 || *end != '\0') return false;
  if (isnan(v) || isinf(v)) return false;
  *pan = constrain(v, -1.0f, 1.0f);
  return true;
}

void handleLine(const char *line) {
  if (strcmp(line, "H") == 0) {
    lastHeartbeatMs = millis();
    return;
  }
  float t, s;
  if (parseCommand(line, &t, &s)) {
    cmdThrottle = t;
    cmdSteer = s;
    lastCmdMs = millis();
    return;
  }
  float pan;
  if (parsePan(line, &pan)) {
    cmdPan = pan;
    return;
  }
  // Unknown lines are ignored.
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      handleLine(rxBuf);
      rxLen = 0;
    } else if (c != '\r') {
      if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
      else rxLen = 0;                      // overlong garbage: drop it
    }
  }
}

// --- control --------------------------------------------------------------
float rampToward(float current, float target, float maxStep) {
  float delta = target - current;
  if (delta > maxStep) delta = maxStep;
  if (delta < -maxStep) delta = -maxStep;
  return current + delta;
}

void updateOutputs(unsigned long now) {
  float dt = (now - lastLoopMs) * 0.001f;
  lastLoopMs = now;

  bool armed = now >= ARM_MS;
  bool cmdFresh = lastCmdMs != 0 && (now - lastCmdMs) < CMD_TIMEOUT_MS;
  failsafe = !(linkOk && cmdFresh && armed);

  if (failsafe) {
    outThrottle = 0.0f;                    // drop to neutral immediately, no ramp
    outSteer = 0.0f;
  } else {
    float maxStep = THROTTLE_RATE_PER_S * dt;
    outThrottle = rampToward(outThrottle, constrain(cmdThrottle, -THROTTLE_LIMIT, THROTTLE_LIMIT), maxStep);
    if (DIFF_DRIVE) {
      outSteer = rampToward(outSteer, constrain(cmdSteer, -THROTTLE_LIMIT, THROTTLE_LIMIT), maxStep);
    } else {
      outSteer = cmdSteer;                 // steering is not rate-limited
    }
  }
  // Camera pan is independent of the drive fail-safe: it just follows the last command, slew-limited.
  outPan = rampToward(outPan, cmdPan, PAN_RATE_PER_S * dt);
  applyOutputs();
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  ledcAttach(THROTTLE_PIN, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcAttach(STEER_PIN, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcAttach(PAN_PIN, SERVO_FREQ_HZ, SERVO_RES_BITS);
  outThrottle = 0.0f;
  outSteer = 0.0f;
  outPan = 0.0f;                           // camera straight ahead at boot
  applyOutputs();                          // neutral from the first moment

  lastLoopMs = millis();
  Serial.printf("V,%s\n", FW_VERSION);
}

void loop() {
  readSerial();

  unsigned long now = millis();
  linkOk = lastHeartbeatMs != 0 && (now - lastHeartbeatMs) < WATCHDOG_MS;
  digitalWrite(LED_PIN, linkOk ? HIGH : LOW);

  updateOutputs(now);

  if (now - lastMeasureMs >= MEASURE_PERIOD_MS) {
    lastMeasureMs = now;
    float d = readDistanceMeters();
    Serial.printf("R,%.3f,%d\n", d, linkOk ? 1 : 0);
  }
  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    Serial.printf("S,%d,%.3f,%.3f\n", failsafe ? 1 : 0, outThrottle, outSteer);
  }
}
