// honebot ESP32 firmware (robot side): ultrasonic sensor, Pi heartbeat watchdog, two drive channels with a
// ramped fail-safe, a camera pan servo channel, and the radio link to the handheld remote (kill switch +
// manual driving). The decisions live in drive_logic.h (pure C++, unit-tested on a PC).
//
// Serial protocol (115200 baud, one line per message, '\n' terminated):
//   ESP32 -> Pi :  V,<version>                  once at boot
//                  R,<distance_m>,<link_ok>     e.g. "R,0.532,1", every MEASURE_PERIOD_MS
//                     distance_m = -1 when no echo (nothing in range)
//                     link_ok    = 1 if a heartbeat arrived in the last WATCHDOG_MS
//                  S,<failsafe>,<a>,<b>         every STATUS_PERIOD_MS
//                     failsafe = 1 while nobody is allowed to drive (outputs ramping to / held at neutral)
//                     a/b = what is actually being output (-1..1)
//                  K,<required>,<remote>,<source>   every STATUS_PERIOD_MS (firmware >= 2.2.0)
//                     required = 1 if REMOTE_REQUIRED is compiled in
//                     remote   = 0 never heard, 1 lost, 2 STOP, 3 RUN
//                     source   = 0 nobody (fail-safe), 1 Pi, 2 manual (remote stick)
//   Pi -> ESP32 :  H                             heartbeat, ~10x per second
//                  C,<a>,<b>                     normalised -1..1, ~50x per second.
//                     Differential drive: a = left wheel ESC, b = right wheel ESC.
//                     Ackermann car: a = throttle ESC, b = steering servo. The Pi decides (drive_mode).
//                  P,<pan>                       camera pan servo, normalised -1..1 (+1 = full left)
//
// Who may drive (drive::decide, first match wins):
//   1. inside the ARM_MS boot window                            -> nobody
//   2. remote says STOP, or was heard and is now silent         -> nobody
//   3. REMOTE_REQUIRED and the remote was never heard           -> nobody
//   4. remote in RUN with the stick deflected                   -> manual (works without the Pi)
//   5. Pi heartbeat within WATCHDOG_MS and C within CMD_TIMEOUT -> Pi
// "Nobody" = fail-safe: the wheel channels RAMP to neutral at FAILSAFE_DECEL_G (an instant cut is a
// near-maximum stop that can pitch the robot over), and are forced to neutral after FAILSAFE_MAX_MS.
// At boot the outputs are neutral from the first instruction.
//
// Wiring (ESP32 DevKit / WROOM-32):
//   HC-SR04 VCC  -> ESP32 VIN/5V
//   HC-SR04 GND  -> ESP32 GND
//   HC-SR04 Trig -> GPIO 5
//   HC-SR04 Echo -> 1k resistor -> GPIO 18, and GPIO 18 -> 2k resistor -> GND
//   Channel A (left ESC / throttle)  -> GPIO 25; ESC GND -> ESP32 GND
//   Channel B (right ESC / steering) -> GPIO 26
//   Camera pan servo signal          -> GPIO 27; servo + from a 5-6 V supply, GND common
//   Onboard LED (GPIO 2): ON = someone may drive; slow blink = fail-safe with the Pi link up;
//                         fast blink = remote STOP / lost; OFF = no Pi link and no remote.
//
// Requires the esp32 Arduino core 3.x (ledcAttach / ledcWrite by pin; ESP-NOW receive callback with
// esp_now_recv_info_t).

#include <math.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "drive_logic.h"
#include "honebot_link.h"

const char *FW_VERSION = "2.2.1";

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
// Motor direction: set these (not the Pi's left_reversed/right_reversed) so that the Pi AND the remote's
// stick drive the same way. Channel A = left wheel, channel B = right wheel in diff drive.
const bool THROTTLE_REVERSED = false;      // channel A (left wheel)
const bool STEER_REVERSED = false;         // channel B (right wheel)
const int PAN_TRIM_US = 0;                 // adjust so the camera looks straight ahead at pan = 0
const bool PAN_REVERSED = false;           // or set pan_sign:=-1 on the Pi side
const float PAN_RATE_PER_S = 3.0f;         // slew limit on the pan command (full travel in ~0.7 s)

// Bench safety, independent of the Pi: cap and rate-limit the drive channels.
// Differential drive: BOTH channels are wheel throttles -> both capped/ramped.
// Ackermann: only channel A is a throttle; channel B (steering) is passed through.
const bool DIFF_DRIVE = true;              // false for an RC car (A = throttle, B = steering)
const float THROTTLE_LIMIT = 0.5f;         // raise toward 1.0 once the robot behaves
// Smallest command at which the wheels actually turn (ESC neutral dead zone + start-up duty). 0 = off.
// Measure it on the bench (docs/first_drive.md, Stage 1) and set it a little below that value; without it
// slow pivots and creeping commands (1-3 %) do nothing. Applies to the Pi's commands and to the stick alike.
const float ESC_DEADBAND = 0.0f;

// Ramps, written as accelerations of the robot. FULL_SCALE_MPS is the speed at command 1.0
// (no-load top speed of the drivetrain: 370 KV x 11.1 V / 2:1 on 90 mm wheels = 9.6 m/s).
const float FULL_SCALE_MPS = 9.6f;
const float ACCEL_G = 0.8f;                // speeding up
const float DECEL_G = 0.7f;                // slowing down on command (gate stop, follower, stick)
const float FAILSAFE_DECEL_G = 0.5f;       // nobody may drive: ramp to neutral (pitch-over starts at 0.88 g)
const float ACCEL_RATE_PER_S = ACCEL_G * 9.81f / FULL_SCALE_MPS;
const float DECEL_RATE_PER_S = DECEL_G * 9.81f / FULL_SCALE_MPS;
const float FAILSAFE_RATE_PER_S = FAILSAFE_DECEL_G * 9.81f / FULL_SCALE_MPS;
const unsigned long FAILSAFE_MAX_MS = 2500;  // hard neutral this long after fail-safe began, whatever the ramp says

// Handheld remote (second ESP32, firmware/esp32_remote). true = the robot will not move at all unless the
// remote is on, in range and in RUN. Set false ONLY for bench work with the wheels off the ground: the
// robot then also obeys the Pi alone (a remote that is on is still obeyed, and losing it still stops).
const bool REMOTE_REQUIRED = true;
const unsigned long REMOTE_TIMEOUT_MS = 500;
const unsigned long MANUAL_HOLD_MS = 800;    // after the stick is released, wait this long before the Pi drives again
const float STICK_DEADBAND = 0.15f;
const float MANUAL_LIMIT = 0.25f;            // stick full forward = this command (0.25 ~ 2.4 m/s)
const float MANUAL_TURN_SCALE = 0.6f;

const drive::Config DRIVE_CFG = {REMOTE_REQUIRED, ARM_MS, WATCHDOG_MS, CMD_TIMEOUT_MS,
                                 REMOTE_TIMEOUT_MS, MANUAL_HOLD_MS, STICK_DEADBAND};

unsigned long lastHeartbeatMs = 0;
unsigned long lastCmdMs = 0;
bool heartbeatSeen = false;
bool cmdSeen = false;
unsigned long lastMeasureMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastLoopMs = 0;
unsigned long failsafeSinceMs = 0;
bool linkOk = false;
bool failsafe = true;
drive::State driveState;
drive::Decision decision = {drive::SRC_NONE, drive::REMOTE_NEVER, false};

float cmdThrottle = 0.0f;                  // last command from the Pi
float cmdSteer = 0.0f;
float outThrottle = 0.0f;                  // what is actually being output
float outSteer = 0.0f;
float cmdPan = 0.0f;                       // camera pan: last command and what is output
float outPan = 0.0f;

// Written by the radio callback (WiFi task), read by loop(): guarded by remoteMux.
portMUX_TYPE remoteMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool rxSeen = false;
volatile unsigned long rxMs = 0;
volatile uint8_t rxFlags = 0;
volatile int8_t rxX = 0;
volatile int8_t rxY = 0;
bool radioOk = false;
const uint8_t BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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
  writePulseUs(THROTTLE_PIN, normToUs(drive::deadbandComp(outThrottle, ESC_DEADBAND), 0, THROTTLE_REVERSED));
  writePulseUs(STEER_PIN, normToUs(DIFF_DRIVE ? drive::deadbandComp(outSteer, ESC_DEADBAND) : outSteer,
                                   STEER_TRIM_US, STEER_REVERSED));
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
    heartbeatSeen = true;
    return;
  }
  float t, s;
  if (parseCommand(line, &t, &s)) {
    cmdThrottle = t;
    cmdSteer = s;
    lastCmdMs = millis();
    cmdSeen = true;
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

// --- radio (remote) ---------------------------------------------------------
void onRadioReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (len != (int)sizeof(RemotePacket)) return;
  RemotePacket p;
  memcpy(&p, data, sizeof(p));
  if (p.pairId != HONEBOT_PAIR_ID || p.version != HONEBOT_LINK_VERSION) return;
  unsigned long now = millis();              // freshness is judged on the robot's own clock
  portENTER_CRITICAL(&remoteMux);
  rxSeen = true;
  rxMs = now;
  rxFlags = p.flags;
  rxX = p.x;
  rxY = p.y;
  portEXIT_CRITICAL(&remoteMux);
}

bool setupRadio() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(HONEBOT_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRadioReceive);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_ADDR, 6);
  peer.channel = HONEBOT_WIFI_CHANNEL;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

void sendRadioStatus() {
  if (!radioOk) return;
  RobotStatusPacket s;
  s.pairId = HONEBOT_PAIR_ID;
  s.version = HONEBOT_LINK_VERSION;
  s.flags = (failsafe ? STATUS_FLAG_FAILSAFE : 0) | (linkOk ? STATUS_FLAG_PI_LINK : 0) |
            (decision.source == drive::SRC_MANUAL ? STATUS_FLAG_MANUAL : 0) |
            (decision.remote == drive::REMOTE_RUN ? STATUS_FLAG_RUN : 0);
  esp_now_send(BROADCAST_ADDR, (const uint8_t *)&s, sizeof(s));
}

// --- control --------------------------------------------------------------
void updateOutputs(unsigned long now) {
  float dt = (now - lastLoopMs) * 0.001f;
  lastLoopMs = now;

  drive::Inputs in;
  in.nowMs = now;
  in.heartbeatSeen = heartbeatSeen;
  in.lastHeartbeatMs = lastHeartbeatMs;
  in.cmdSeen = cmdSeen;
  in.lastCmdMs = lastCmdMs;
  portENTER_CRITICAL(&remoteMux);
  in.remoteSeen = rxSeen;
  in.lastRemoteMs = rxMs;
  in.remoteRun = (rxFlags & REMOTE_FLAG_RUN) != 0;
  in.stickX = rxX / 100.0f;
  in.stickY = rxY / 100.0f;
  portEXIT_CRITICAL(&remoteMux);

  decision = drive::decide(DRIVE_CFG, in, &driveState);
  linkOk = decision.piLink;
  bool wasFailsafe = failsafe;
  failsafe = decision.source == drive::SRC_NONE;
  if (failsafe && !wasFailsafe) failsafeSinceMs = now;

  if (failsafe) {
    if (now < ARM_MS || (now - failsafeSinceMs) >= FAILSAFE_MAX_MS) {
      outThrottle = 0.0f;                  // boot window, or the ramp has had its time: hard neutral
      outSteer = 0.0f;
    } else if (DIFF_DRIVE) {
      drive::rampPairToZero(&outThrottle, &outSteer, FAILSAFE_RATE_PER_S * dt);   // keeps the arc
    } else {
      outThrottle = drive::rampToward(outThrottle, 0.0f, FAILSAFE_RATE_PER_S * dt);  // steering holds
    }
  } else {
    float tgtA, tgtB;
    if (decision.source == drive::SRC_MANUAL) {
      drive::arcadeMix(drive::shapeStick(in.stickX, STICK_DEADBAND), drive::shapeStick(in.stickY, STICK_DEADBAND),
                       MANUAL_LIMIT, MANUAL_TURN_SCALE, &tgtA, &tgtB);
    } else {
      tgtA = cmdThrottle;
      tgtB = cmdSteer;
    }
    float up = ACCEL_RATE_PER_S * dt, down = DECEL_RATE_PER_S * dt;
    outThrottle = drive::slewChannel(outThrottle, constrain(tgtA, -THROTTLE_LIMIT, THROTTLE_LIMIT), up, down);
    if (DIFF_DRIVE) {
      outSteer = drive::slewChannel(outSteer, constrain(tgtB, -THROTTLE_LIMIT, THROTTLE_LIMIT), up, down);
    } else {
      outSteer = (decision.source == drive::SRC_MANUAL) ? 0.0f : cmdSteer;   // manual drive is diff-drive only
    }
  }
  // Camera pan is independent of the drive fail-safe: it just follows the last command, slew-limited.
  outPan = drive::rampToward(outPan, cmdPan, PAN_RATE_PER_S * dt);
  applyOutputs();
}

void updateLed(unsigned long now) {
  bool on;
  if (!failsafe) on = true;
  else if (decision.remote == drive::REMOTE_STOP || decision.remote == drive::REMOTE_LOST) on = (now / 100) % 2;
  else if (linkOk) on = (now / 500) % 2;
  else on = false;
  digitalWrite(LED_PIN, on ? HIGH : LOW);
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

  radioOk = setupRadio();                  // if the radio fails and REMOTE_REQUIRED is set, the robot stays stopped
  lastLoopMs = millis();
  Serial.printf("V,%s\n", FW_VERSION);
}

void loop() {
  readSerial();

  unsigned long now = millis();
  updateOutputs(now);
  updateLed(now);

  if (now - lastMeasureMs >= MEASURE_PERIOD_MS) {
    lastMeasureMs = now;
    float d = readDistanceMeters();
    Serial.printf("R,%.3f,%d\n", d, linkOk ? 1 : 0);
  }
  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    Serial.printf("S,%d,%.3f,%.3f\n", failsafe ? 1 : 0, outThrottle, outSteer);
    Serial.printf("K,%d,%d,%d\n", REMOTE_REQUIRED ? 1 : 0, (int)decision.remote, (int)decision.source);
    sendRadioStatus();
  }
}
