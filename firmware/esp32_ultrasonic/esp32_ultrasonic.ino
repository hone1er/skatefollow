// honebot ESP32 firmware: ultrasonic sensor + Pi heartbeat watchdog.
//
// Serial protocol (115200 baud, one line per message, '\n' terminated):
//   ESP32 -> Pi :  R,<distance_m>,<link_ok>     e.g. "R,0.532,1"
//                  distance_m = -1 when no echo (nothing in range)
//                  link_ok    = 1 if a heartbeat arrived in the last WATCHDOG_MS
//   Pi -> ESP32 :  H                             heartbeat, sent ~10x per second
//
// The onboard LED is ON while the Pi link is healthy. Later, "link not ok"
// is where the motors get forced to neutral.
//
// Wiring (HC-SR04, ESP32 DevKit / WROOM-32):
//   HC-SR04 VCC  -> ESP32 VIN/5V
//   HC-SR04 GND  -> ESP32 GND
//   HC-SR04 Trig -> GPIO 5
//   HC-SR04 Echo -> 1k resistor -> GPIO 18, and GPIO 18 -> 2k resistor -> GND
//                  (divider drops the 5 V echo to ~3.3 V; skip it only for a
//                   3.3 V sensor such as HC-SR04P / RCWL-1601 powered from 3V3)

const int TRIG_PIN = 5;
const int ECHO_PIN = 18;
const int LED_PIN = 2;                     // onboard LED on most DevKits

const unsigned long MEASURE_PERIOD_MS = 60;  // ~16 Hz; HC-SR04 datasheet asks for >= 60 ms between pings
const unsigned long WATCHDOG_MS = 500;       // link lost after this long without "H"
const unsigned long ECHO_TIMEOUT_US = 25000; // ~4.3 m max range

unsigned long lastHeartbeatMs = 0;
unsigned long lastMeasureMs = 0;
bool linkOk = false;

char rxBuf[32];
size_t rxLen = 0;

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

void handleLine(const char *line) {
  if (strcmp(line, "H") == 0) {
    lastHeartbeatMs = millis();
  }
  // Future commands (e.g. "C,<speed>,<steer>") get handled here.
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

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  readSerial();

  unsigned long now = millis();
  linkOk = lastHeartbeatMs != 0 && (now - lastHeartbeatMs) < WATCHDOG_MS;
  digitalWrite(LED_PIN, linkOk ? HIGH : LOW);

  if (now - lastMeasureMs >= MEASURE_PERIOD_MS) {
    lastMeasureMs = now;
    float d = readDistanceMeters();
    Serial.printf("R,%.3f,%d\n", d, linkOk ? 1 : 0);
  }
}
