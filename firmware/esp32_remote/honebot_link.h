// Radio packets between the handheld remote and the robot (ESP-NOW broadcast).
// THIS FILE EXISTS TWICE and both copies must be identical:
//   firmware/esp32_ultrasonic/honebot_link.h   and   firmware/esp32_remote/honebot_link.h
// (an Arduino sketch can only include files from its own folder; firmware/tests/run_tests.sh checks they match).
#pragma once
#include <stdint.h>

// Change PAIR_ID to any other number if someone nearby runs the same firmware; both sketches must agree.
const uint32_t HONEBOT_PAIR_ID = 0x486F6E65;   // "Hone"
const uint8_t HONEBOT_LINK_VERSION = 1;
const uint8_t HONEBOT_WIFI_CHANNEL = 1;

const uint8_t REMOTE_FLAG_RUN = 0x01;          // 1 = RUN, 0 = STOP

struct __attribute__((packed)) RemotePacket {  // remote -> robot, 50 Hz
  uint32_t pairId;
  uint8_t version;
  uint8_t flags;
  int8_t x;                                    // forward, -100..100
  int8_t y;                                    // turn (+ = left), -100..100
  uint16_t seq;
};

const uint8_t STATUS_FLAG_FAILSAFE = 0x01;     // robot outputs forced toward neutral
const uint8_t STATUS_FLAG_PI_LINK = 0x02;      // robot hears the Pi's heartbeat
const uint8_t STATUS_FLAG_MANUAL = 0x04;       // robot is being driven by the remote's stick
const uint8_t STATUS_FLAG_RUN = 0x08;          // robot's view: remote fresh and in RUN

struct __attribute__((packed)) RobotStatusPacket {  // robot -> remote, 10 Hz
  uint32_t pairId;
  uint8_t version;
  uint8_t flags;
};
