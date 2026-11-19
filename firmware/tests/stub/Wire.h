// Host stub of the ESP32 Wire (I2C) API, for the sketch syntax check only.
#pragma once
#include <stddef.h>
#include <stdint.h>
class TwoWire {
 public:
  bool begin(int, int, uint32_t = 0) { return true; }
  void setTimeOut(uint16_t) {}
  void beginTransmission(uint8_t) {}
  size_t write(uint8_t) { return 1; }
  uint8_t endTransmission(bool = true) { return 0; }
  size_t requestFrom(uint8_t, size_t) { return 0; }
  int read() { return -1; }
};
extern TwoWire Wire;
