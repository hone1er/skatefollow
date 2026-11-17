// honebot handheld remote: kill switch + manual driving, on a second ESP32 with the 5-pin joystick module
// (GND, +5V, VRx, VRy, SW). Talks to the robot's ESP32 over ESP-NOW (no router, no pairing step: both
// sketches share HONEBOT_PAIR_ID in honebot_link.h).
//
// What it does:
//   - Boots in STOP. The robot will not move while the remote says STOP, or when the remote is off or out
//     of range (robot firmware >= 2.2.0 with REMOTE_REQUIRED = true).
//   - RUN: let go of the stick, then press and HOLD the stick's button for 1.5 s.
//   - STOP: click the button. Any click in RUN stops the robot (ramped stop at ~0.5 g).
//   - In RUN, moving the stick drives the robot by hand (slow: about 2.4 m/s at full stick) and overrides
//     the Pi. It works even if the Pi is off. Let go and, ~1 s later, the Pi drives again.
//
// Wiring (ESP32 DevKit):
//   joystick GND  -> ESP32 GND
//   joystick +5V  -> ESP32 3V3     (NOT 5 V / VIN: the stick outputs would exceed the 3.3 V the pins accept)
//   joystick VRx  -> GPIO 34       (ADC1; ADC2 pins do not work while the radio is on)
//   joystick VRy  -> GPIO 35       (ADC1)
//   joystick SW   -> GPIO 32       (button to GND; internal pull-up; 34-39 have no pull-ups, so not there)
//   Power: USB power bank into the DevKit's USB port.
//
// Onboard LED (GPIO 2):
//   solid            RUN and the robot answers
//   fast blink       STOP (robot answers)
//   slow blink       no answer from the robot (off, out of range, or wrong PAIR_ID)
//   3 quick flashes, pause, repeat   stick fault at power-up (loose wire or stick held): RUN is refused;
//                    fix the wiring / release the stick and press EN (reset)
//
// Serial monitor (115200) prints one line 5x per second for bench checks:
//   state=STOP x=+0.00 y=+0.00 btn=0 robot=1 pi=1 failsafe=1 manual=0
//
// Requires the esp32 Arduino core 3.x.

#include <math.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "honebot_link.h"
#include "remote_logic.h"

const char *FW_VERSION = "1.0.0";

const int PIN_VRX = 34;
const int PIN_VRY = 35;
const int PIN_SW = 32;
const int LED_PIN = 2;

// Which way is "forward"? Hold the remote the way you will use it, watch the serial line, and set these so
// that pushing the stick away from you gives x=+1.00 and pushing it left gives y=+1.00.
const bool SWAP_XY = false;                 // true if forward/back shows up on y
const bool X_REVERSED = false;
const bool Y_REVERSED = false;

const int ADC_MAX = 4095;
const float CENTRE_BAND = 0.15f;            // |stick| below this counts as centred (same as the robot's deadband)
const unsigned long HOLD_TO_RUN_MS = 1500;
const unsigned long DEBOUNCE_MS = 30;
const unsigned long SEND_PERIOD_MS = 20;    // 50 Hz
const unsigned long PRINT_PERIOD_MS = 200;
const unsigned long ROBOT_TIMEOUT_MS = 500;

const uint8_t BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

remote::ButtonState button;
int centreX = ADC_MAX / 2, centreY = ADC_MAX / 2;
bool stickFault = false;
bool radioOk = false;
uint16_t seq = 0;
unsigned long lastSendMs = 0, lastPrintMs = 0;
bool debounced = false, lastRaw = false;
unsigned long lastRawChangeMs = 0;

portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool statusSeen = false;
volatile unsigned long statusMs = 0;
volatile uint8_t statusFlags = 0;

void onRadioReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (len != (int)sizeof(RobotStatusPacket)) return;
  RobotStatusPacket p;
  memcpy(&p, data, sizeof(p));
  if (p.pairId != HONEBOT_PAIR_ID || p.version != HONEBOT_LINK_VERSION) return;
  unsigned long now = millis();
  portENTER_CRITICAL(&statusMux);
  statusSeen = true;
  statusMs = now;
  statusFlags = p.flags;
  portEXIT_CRITICAL(&statusMux);
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

int readAveraged(int pin, int n) {
  long sum = 0;
  for (int i = 0; i < n; i++) sum += analogRead(pin);
  return (int)(sum / n);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SW, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);            // full 0-3.3 V range

  delay(200);                                // let the stick supply settle
  centreX = readAveraged(PIN_VRX, 64);       // the stick must be at rest at power-up
  centreY = readAveraged(PIN_VRY, 64);
  stickFault = !remote::centreLooksValid(centreX, ADC_MAX) || !remote::centreLooksValid(centreY, ADC_MAX);

  radioOk = setupRadio();
  Serial.printf("honebot remote %s  centre x=%d y=%d  stick=%s radio=%s\n", FW_VERSION, centreX, centreY,
                stickFault ? "FAULT" : "ok", radioOk ? "ok" : "FAILED");
}

void loop() {
  unsigned long now = millis();

  // Stick
  float a = remote::scaleAxis(readAveraged(PIN_VRX, 4), centreX, ADC_MAX, false);
  float b = remote::scaleAxis(readAveraged(PIN_VRY, 4), centreY, ADC_MAX, false);
  float x = SWAP_XY ? b : a;
  float y = SWAP_XY ? a : b;
  if (X_REVERSED) x = -x;
  if (Y_REVERSED) y = -y;
  bool centred = fabsf(x) < CENTRE_BAND && fabsf(y) < CENTRE_BAND;

  // Button (active low), debounced
  bool raw = digitalRead(PIN_SW) == LOW;
  if (raw != lastRaw) {
    lastRaw = raw;
    lastRawChangeMs = now;
  }
  if ((now - lastRawChangeMs) >= DEBOUNCE_MS) debounced = raw;

  bool run = remote::updateRun(&button, debounced, centred && !stickFault, now, HOLD_TO_RUN_MS);
  if (stickFault || !radioOk) {
    button.run = false;
    run = false;
  }

  // Robot status
  bool robotSeen;
  unsigned long robotMs;
  uint8_t flags;
  portENTER_CRITICAL(&statusMux);
  robotSeen = statusSeen;
  robotMs = statusMs;
  flags = statusFlags;
  portEXIT_CRITICAL(&statusMux);
  bool robotLink = robotSeen && (now - robotMs) < ROBOT_TIMEOUT_MS;

  // Send
  if (radioOk && (now - lastSendMs) >= SEND_PERIOD_MS) {
    lastSendMs = now;
    RemotePacket p;
    p.pairId = HONEBOT_PAIR_ID;
    p.version = HONEBOT_LINK_VERSION;
    p.flags = run ? REMOTE_FLAG_RUN : 0;
    p.x = run ? remote::toWire(x) : 0;       // no stick data in STOP
    p.y = run ? remote::toWire(y) : 0;
    p.seq = seq++;
    esp_now_send(BROADCAST_ADDR, (const uint8_t *)&p, sizeof(p));
  }

  // LED
  bool led;
  if (stickFault || !radioOk) {
    unsigned long t = now % 1400;
    led = t < 600 && (t / 100) % 2 == 0;     // 3 quick flashes, pause
  } else if (!robotLink) {
    led = (now / 500) % 2;
  } else if (run) {
    led = true;
  } else {
    led = (now / 100) % 2;
  }
  digitalWrite(LED_PIN, led ? HIGH : LOW);

  if ((now - lastPrintMs) >= PRINT_PERIOD_MS) {
    lastPrintMs = now;
    Serial.printf("state=%s x=%+.2f y=%+.2f btn=%d robot=%d pi=%d failsafe=%d manual=%d\n",
                  stickFault ? "FAULT" : (run ? "RUN" : "STOP"), x, y, debounced ? 1 : 0, robotLink ? 1 : 0,
                  (flags & STATUS_FLAG_PI_LINK) ? 1 : 0, (flags & STATUS_FLAG_FAILSAFE) ? 1 : 0,
                  (flags & STATUS_FLAG_MANUAL) ? 1 : 0);
  }
  delay(2);
}
