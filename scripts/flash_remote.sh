#!/bin/bash
# Flash the handheld remote (and optionally the robot ESP32) from the Mac, then print the next steps for
# driving turtlesim with it. Run from anywhere in the repo clone on the Mac:
#
#   scripts/flash_remote.sh                       # list the boards and their ports
#   scripts/flash_remote.sh <remote_port>         # flash the remote only
#   scripts/flash_remote.sh <remote_port> <robot_port>   # flash both
#
# Ports look like /dev/cu.usbserial-0001. Both boards use the same USB chip, so plug them in one at a time
# if you are not sure which is which.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FQBN="esp32:esp32:esp32"
REMOTE_PORT="${1:-}"
ROBOT_PORT="${2:-}"

if ! command -v arduino-cli >/dev/null; then
  echo "arduino-cli not found (install: brew install arduino-cli, then the esp32:esp32 core)." >&2
  exit 1
fi

echo "Boards connected now:"
arduino-cli board list
echo

if [ -z "$REMOTE_PORT" ]; then
  echo "Usage: $0 <remote_port> [robot_port]"
  exit 0
fi

flash() {  # $1 = sketch folder, $2 = port
  echo "== Compiling $1"
  arduino-cli compile --fqbn "$FQBN" "$REPO/firmware/$1"
  echo "== Uploading $1 to $2"
  arduino-cli upload -p "$2" --fqbn "$FQBN" "$REPO/firmware/$1"
}

flash esp32_remote "$REMOTE_PORT"
if [ -n "$ROBOT_PORT" ]; then
  flash esp32_ultrasonic "$ROBOT_PORT"
fi

cat <<EOF

Flashed. Next:

1. Check the remote on the Mac (stick at rest when it powers up):
     arduino-cli monitor -p $REMOTE_PORT -c baudrate=115200
   You should see 5 lines a second like:
     state=STOP x=+0.00 y=+0.00 btn=0 robot=0 pi=0 failsafe=0 manual=0
   - Stick pushed away from you: x goes to +1.00. Stick pushed left: y goes to +1.00.
     (If not, set SWAP_XY / X_REVERSED / Y_REVERSED at the top of esp32_remote.ino and flash again.)
   - Hold the button 1.5 s with the stick centred: state=RUN. Click: state=STOP.
   - robot=1 only when the robot ESP32 is powered and answering over ESP-NOW.

2. Drive turtlesim on the Pi: see docs/remote_turtlesim.md
   (remote board on the Pi's USB:  ros2 launch honebot_remote_bridge bridge_launch.py
    robot ESP32 on the Pi's USB:   ros2 launch honebot_remote_bridge bridge_launch.py source:=esp32)
EOF
