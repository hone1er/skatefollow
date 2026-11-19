// Minimal stand-ins for the Arduino / ESP32 APIs the sketches use, so a PC compiler can syntax- and
// type-check them (firmware/tests/run_tests.sh). This does NOT replace compiling with arduino-cli.
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define ESP_OK 0
#define constrain(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
typedef int esp_err_t;
unsigned long millis();
void delay(unsigned long);
void delayMicroseconds(unsigned int);
void pinMode(int, int);
void digitalWrite(int, int);
int digitalRead(int);
int analogRead(int);
void analogReadResolution(int);
enum adc_attenuation_t { ADC_11db };
void analogSetAttenuation(adc_attenuation_t);
unsigned long pulseIn(int, int, unsigned long);
bool ledcAttach(int, uint32_t, uint8_t);
bool ledcWrite(int, uint32_t);
struct SerialStub {
  void begin(long);
  int available();
  int read();
  int printf(const char *, ...) __attribute__((format(printf, 2, 3)));
  void println(const char *);
};
extern SerialStub Serial;
struct portMUX_TYPE { int x; };
#define portMUX_INITIALIZER_UNLOCKED {0}
void portENTER_CRITICAL(portMUX_TYPE *);
void portEXIT_CRITICAL(portMUX_TYPE *);
