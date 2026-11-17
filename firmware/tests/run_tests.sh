#!/usr/bin/env bash
# Firmware checks that run on any PC with g++ (no ESP32, no arduino-cli):
#   1. the two copies of honebot_link.h are identical
#   2. unit tests of the pure logic (drive_logic.h, remote_logic.h)
#   3. syntax/type check of both sketches against stub Arduino headers
# It does not replace `arduino-cli compile` (README in each sketch folder).
set -euo pipefail
cd "$(dirname "$0")"
cmp ../esp32_ultrasonic/honebot_link.h ../esp32_remote/honebot_link.h
echo "ok: honebot_link.h copies match"
out=$(mktemp -d)
g++ -std=c++17 -Wall -Wextra -Werror -o "$out/t" test_drive_logic.cpp && "$out/t"
for sketch in esp32_ultrasonic esp32_remote; do
  # Arduino adds the Arduino.h include and function prototypes itself; -include does the first.
  g++ -std=c++17 -fsyntax-only -Wall -Wextra -Wno-unused-function -x c++ -include stub/Arduino.h -I stub \
      -I "../$sketch" "../$sketch/$sketch.ino"
  echo "ok: $sketch.ino passes the host syntax check"
done
rm -rf "$out"
