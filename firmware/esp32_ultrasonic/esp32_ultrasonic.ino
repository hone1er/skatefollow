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
//                  G,<yaw_rad_s>,<usable>       every STATUS_PERIOD_MS when YAW_LOOP_ENABLED (2.3.0): gyro check
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
//   Optional gyro for the yaw-rate loop (firmware >= 2.3.0, YAW_LOOP_ENABLED): MPU6050 breakout (GY-521) on
//     VCC -> 3V3, GND -> GND, SDA -> GPIO 21, SCL -> GPIO 22, mounted flat (chip side up) anywhere on the deck.
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
#include <Wire.h>

const char *FW_VERSION = "2.4.0";

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
// Pan pulse (2.4.0): pan = +-1 -> neutral +- PAN_SPAN_US. Its own range, wider than the ESCs' 1000-2000 us, so the
// camera can reach a skater 45 deg off the nose (side shot). A standard servo (MG996R / DS3218) turns roughly
// 0.09 deg/us, so 778 us is about 70 deg, but servos differ: MEASURE the angle at pan = +-1 and set camera_aim's
// half_range_deg to it. PAN_MIN/MAX_US keep the pulse inside what such servos accept.
const int PAN_SPAN_US = 778;
const int PAN_MIN_US = 500;
const int PAN_MAX_US = 2500;

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

// --- Yaw-rate loop (2.3.0). OFF by default: with it off, or with no gyro answering, the robot behaves exactly
// like 2.2.1. Turn it on once an MPU6050 is wired (see the wiring list) and its sign is checked on the stand
// (docs/first_drive.md). Values tuned in the simulator (sim/README.md); confirm on the robot.
const bool YAW_LOOP_ENABLED = false;
const int GYRO_SDA = 21;
const int GYRO_SCL = 22;
const uint8_t GYRO_ADDR = 0x68;            // MPU6050 with AD0 low
const float GYRO_SIGN = 1.0f;              // +1: chip side up (turning left reads positive). -1 if mounted upside down
const uint16_t GYRO_I2C_TIMEOUT_MS = 2;    // a hung bus must never stall the loop: a failed read = no gyro
const unsigned long GYRO_PERIOD_MS = 5;    // 200 Hz
const unsigned long GYRO_STALE_MS = 50;    // no good reading for this long -> open loop
const uint32_t GYRO_BIAS_MIN_SAMPLES = 100;  // need at least ~0.5 s of good samples during ARM_MS to trust bias
const uint16_t GYRO_STUCK_SAMPLES = 100;   // 2.4.0: this many identical raw readings in a row (0.5 s) -> gyro stuck,
                                           // treated as absent until restart (a working chip always shows noise)
const float YAW_TRACK_WIDTH = 0.39f;       // m, MUST equal track_width in robot.yaml (the Pi converts with it)
const float YAW_KP = 0.12f;                // throttle per rad/s of error
const float YAW_KI = 0.8f;                 // throttle per rad of error
const float YAW_MAX_CORR = 0.2f;           // cap on the half-difference added (scrub breakaway is ~0.13)
const unsigned long YAW_SAT_MS = 1000;     // correction pinned this long -> loop off until the next fail-safe
const unsigned long YAW_WRONG_WAY_MS = 75;  // gyro reads a hard turn the other way (or the turn runs away against the
                                           // correction) this long -> loop off. 75 ms (2.4.0, was 150): once the
                                           // tyres break loose 150 ms is enough to roll over at speed (simulated)
const float MANUAL_MAX_YAW = 3.0f;         // rad/s at full side stick when the loop is on
const float MANUAL_TURN_EXPO = 0.5f;       // gentle turns near the stick centre when the loop is on

const drive::YawConfig YAW_CFG = {YAW_KP,          YAW_KI,     YAW_MAX_CORR,     FULL_SCALE_MPS,
                                  YAW_TRACK_WIDTH, YAW_SAT_MS, YAW_WRONG_WAY_MS, THROTTLE_LIMIT};
// with the loop on, the stick asks for a yaw rate: full side stick = MANUAL_MAX_YAW
const float MANUAL_TURN_SCALE_GYRO = MANUAL_MAX_YAW * YAW_TRACK_WIDTH / (2.0f * MANUAL_LIMIT * FULL_SCALE_MPS);

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
float driveA = 0.0f;                       // wheel commands after the yaw loop (= outThrottle/outSteer when it is off)
float driveB = 0.0f;

// Gyro (yaw-rate loop)
bool gyroPresent = false;
bool gyroBiasDone = false;
float gyroBias = 0.0f;
double gyroBiasSum = 0.0;
uint32_t gyroBiasN = 0;
float gyroZ = 0.0f;                        // rad/s, + = turning left, bias removed
unsigned long gyroLastOkMs = 0;
unsigned long lastGyroMs = 0;
drive::YawState yawState;
drive::StuckWatch gyroWatch;                // 2.4.0 stuck-gyro check

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
bool rxOverflow = false;

// Non-blocking HC-SR04 echo capture (CHANGE interrupt on ECHO_PIN); never stalls loop().
portMUX_TYPE echoMux = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long echoRiseUs = 0;
volatile unsigned long echoPulseUs = 0;
volatile bool echoArmed = false;
volatile bool echoRising = false;
volatile bool echoReady = false;
bool pingInFlight = false;
unsigned long pingStartUs = 0;

// --- servo output ---------------------------------------------------------
void writePulseUs(int pin, int us, int minUs, int maxUs) {
  us = constrain(us, minUs, maxUs);
  const uint32_t periodUs = 1000000UL / SERVO_FREQ_HZ;                 // 20000
  const uint32_t maxDuty = (1UL << SERVO_RES_BITS) - 1;
  uint32_t duty = (uint32_t)(((uint64_t)us * maxDuty) / periodUs);
  ledcWrite(pin, duty);
}

int normToUs(float norm, int trimUs, bool reversed, int spanUs) {
  norm = constrain(norm, -1.0f, 1.0f);
  if (reversed) norm = -norm;
  return PULSE_NEUTRAL_US + trimUs + (int)(norm * spanUs);
}

void applyOutputs() {
  const int escSpan = PULSE_MAX_US - PULSE_NEUTRAL_US;     // ESCs: 1000-2000 us, unchanged
  writePulseUs(THROTTLE_PIN, normToUs(drive::deadbandComp(driveA, ESC_DEADBAND), 0, THROTTLE_REVERSED, escSpan),
               PULSE_MIN_US, PULSE_MAX_US);
  writePulseUs(STEER_PIN, normToUs(DIFF_DRIVE ? drive::deadbandComp(driveB, ESC_DEADBAND) : driveB,
                                   STEER_TRIM_US, STEER_REVERSED, escSpan),
               PULSE_MIN_US, PULSE_MAX_US);
  writePulseUs(PAN_PIN, normToUs(outPan, PAN_TRIM_US, PAN_REVERSED, PAN_SPAN_US), PAN_MIN_US, PAN_MAX_US);
}

// --- sensor (non-blocking HC-SR04) ------------------------------------------
void IRAM_ATTR onEchoChange() {
  unsigned long nowUs = (unsigned long)esp_timer_get_time();
  bool high = gpio_get_level((gpio_num_t)ECHO_PIN) != 0;
  portENTER_CRITICAL_ISR(&echoMux);
  if (echoArmed) {
    if (high && !echoRising) {
      echoRiseUs = nowUs;
      echoRising = true;
    } else if (!high && echoRising) {
      echoPulseUs = nowUs - echoRiseUs;
      echoRising = false;
      echoReady = true;
      echoArmed = false;                   // ignore trailing bounce until next startPing()
    }
  }
  portEXIT_CRITICAL_ISR(&echoMux);
}

void startPing() {
  portENTER_CRITICAL(&echoMux);
  echoArmed = false;
  echoRising = false;
  echoReady = false;
  echoPulseUs = 0;
  portEXIT_CRITICAL(&echoMux);
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Arm after TRIG falls: HC-SR04 emits 8x 40 kHz cycles (~450 us) before raising ECHO.
  portENTER_CRITICAL(&echoMux);
  echoArmed = true;
  portEXIT_CRITICAL(&echoMux);
  pingInFlight = true;
  pingStartUs = micros();
}

void pollUltrasonic(unsigned long now) {
  if (pingInFlight) {
    bool ready, rising;
    unsigned long pulseUs, riseUs;
    portENTER_CRITICAL(&echoMux);
    ready = echoReady;
    rising = echoRising;
    pulseUs = echoPulseUs;
    riseUs = echoRiseUs;
    portEXIT_CRITICAL(&echoMux);

    unsigned long nowUs = micros();
    if (ready) {
      pingInFlight = false;
      float d = (pulseUs > 0 && pulseUs <= ECHO_TIMEOUT_US) ? (pulseUs * 0.000343f / 2.0f) : -1.0f;
      Serial.printf("R,%.3f,%d\n", d, linkOk ? 1 : 0);
    } else if ((rising && (unsigned long)(nowUs - riseUs) >= ECHO_TIMEOUT_US) ||
               (!rising && (unsigned long)(nowUs - pingStartUs) >= ECHO_TIMEOUT_US)) {
      portENTER_CRITICAL(&echoMux);
      echoArmed = false;
      echoRising = false;
      portEXIT_CRITICAL(&echoMux);
      pingInFlight = false;
      Serial.printf("R,%.3f,%d\n", -1.0f, linkOk ? 1 : 0);
    }
  }
  if (now - lastMeasureMs >= MEASURE_PERIOD_MS) {
    lastMeasureMs = now;
    if (pingInFlight) {
      // Only reachable if loop() stalled past ECHO_TIMEOUT_US; drop the unknown sample instead of
      // reporting -1 ("clear") so the Pi's range_timeout catches a sustained stall.
      portENTER_CRITICAL(&echoMux);
      echoArmed = false;
      echoRising = false;
      portEXIT_CRITICAL(&echoMux);
      pingInFlight = false;
    }
    startPing();
  }
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
      if (!rxOverflow) {
        rxBuf[rxLen] = '\0';
        handleLine(rxBuf);
      }
      rxLen = 0;
      rxOverflow = false;
    } else if (c != '\r') {
      if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
      else rxOverflow = true;              // overlong garbage: discard rest of line until '\n'
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
// --- gyro (MPU6050) ---------------------------------------------------------
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuReadGyroZ(int16_t *raw) {
  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(0x47);                        // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(GYRO_ADDR, (size_t)2) != 2) return false;
  int hi = Wire.read();
  int lo = Wire.read();
  if (hi < 0 || lo < 0) return false;
  *raw = (int16_t)((hi << 8) | lo);
  return true;
}

void setupGyro() {
  if (!YAW_LOOP_ENABLED) return;
  Wire.begin(GYRO_SDA, GYRO_SCL, 400000);
  Wire.setTimeOut(GYRO_I2C_TIMEOUT_MS);
  // wake up, 44 Hz low-pass, +/-1000 deg/s (32.8 LSB per deg/s)
  gyroPresent = mpuWrite(0x6B, 0x00) && mpuWrite(0x1A, 0x03) && mpuWrite(0x1B, 0x10);
  Serial.println(gyroPresent ? "gyro: MPU6050 found; calibrating during the arming window, keep the robot still"
                             : "gyro: no MPU6050 at 0x68; yaw loop off (open loop, as 2.2.1)");
}

// Reads the gyro every GYRO_PERIOD_MS. The bias is the average over the ARM_MS boot window (robot standing still).
void updateGyro(unsigned long now) {
  if (!gyroPresent || now - lastGyroMs < GYRO_PERIOD_MS) return;
  lastGyroMs = now;
  int16_t raw;
  if (!mpuReadGyroZ(&raw) || raw >= 32700 || raw <= -32700) return;     // failed or clipped: no fresh reading
  if (drive::stuckReading(&gyroWatch, raw, GYRO_STUCK_SAMPLES)) return;  // frozen chip: never fresh again
  float w = GYRO_SIGN * (float)raw / 32.8f * 0.01745329f;
  if (now < ARM_MS) {
    gyroBiasSum += w;
    gyroBiasN++;
    return;
  }
  if (!gyroBiasDone) {
    if (gyroBiasN < GYRO_BIAS_MIN_SAMPLES) return;   // not enough samples during arming: leave yaw loop off
    gyroBias = (float)(gyroBiasSum / gyroBiasN);
    gyroBiasDone = true;
  }
  gyroZ = w - gyroBias;
  gyroLastOkMs = now;
}

bool yawLoopUsable(unsigned long now) {
  return YAW_LOOP_ENABLED && gyroPresent && gyroBiasDone && (now - gyroLastOkMs) < GYRO_STALE_MS && !yawState.tripped;
}

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
      float turn = drive::shapeStick(in.stickY, STICK_DEADBAND);
      float scale = MANUAL_TURN_SCALE;
      if (yawLoopUsable(now)) {              // the stick asks for a yaw rate, with expo for gentle turns
        turn = drive::expo(turn, MANUAL_TURN_EXPO);
        scale = MANUAL_TURN_SCALE_GYRO;
      }
      drive::arcadeMix(drive::shapeStick(in.stickX, STICK_DEADBAND), turn, MANUAL_LIMIT, scale, &tgtA, &tgtB);
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
  // Yaw-rate loop: corrects only the left/right difference; passes through when off, in fail-safe or without gyro.
  if (failsafe) yawState.reset();
  driveA = outThrottle;
  driveB = outSteer;
  drive::yawLoop(YAW_CFG, &yawState, DIFF_DRIVE && !failsafe && yawLoopUsable(now), gyroZ, now, dt, &driveA, &driveB);
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
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), onEchoChange, CHANGE);
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

  setupGyro();
  radioOk = setupRadio();                  // if the radio fails and REMOTE_REQUIRED is set, the robot stays stopped
  lastLoopMs = millis();
  Serial.printf("V,%s\n", FW_VERSION);
}

void loop() {
  readSerial();

  unsigned long now = millis();
  updateGyro(now);
  updateOutputs(now);
  updateLed(now);

  pollUltrasonic(now);
  if (now - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = now;
    Serial.printf("S,%d,%.3f,%.3f\n", failsafe ? 1 : 0, driveA, driveB);
    Serial.printf("K,%d,%d,%d\n", REMOTE_REQUIRED ? 1 : 0, (int)decision.remote, (int)decision.source);
    if (YAW_LOOP_ENABLED) {                // gyro check line (bridge -> esp32/yaw_rate, side view): yaw rate, + = left; loop usable
      Serial.printf("G,%.2f,%d\n", gyroZ, yawLoopUsable(now) ? 1 : 0);
    }
    sendRadioStatus();
  }
}
