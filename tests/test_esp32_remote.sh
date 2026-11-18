#!/bin/bash
# Drive turtlesim with the handheld remote, on the Raspberry Pi. Checks the setup first, then starts the
# launch. Run from the repo root (~/honebot_ws), on the Pi's HDMI console (turtlesim needs a screen).
#
#   tests/test_esp32_remote.sh fake                  # no ESP32: fake remote + automatic PASS/FAIL check
#   tests/test_esp32_remote.sh serial [port]         # remote board on the Pi's USB (default /dev/ttyUSB0)
#   tests/test_esp32_remote.sh esp32  [port]         # robot ESP32 on the Pi's USB, remote by radio
#   add  follow  as the last word to also run the follower turtle, e.g.:  tests/test_esp32_remote.sh serial /dev/ttyUSB0 follow
#
# Flash the boards first, from the Mac: firmware/tests/test_remote_turtle.sh. Guide: firmware/tests/ESP32_REMOTE_TESTING.md
set -uo pipefail

MODE="${1:-}"
PORT="${2:-/dev/ttyUSB0}"
FOLLOW=false
for a in "$@"; do [ "$a" = follow ] && FOLLOW=true; done
[ "$PORT" = follow ] && PORT=/dev/ttyUSB0
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAKE=/tmp/honebot_fake_remote

case "$MODE" in
  fake|serial|esp32) ;;
  *) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac

ok()   { echo "  ok    $*"; }
bad()  { echo "  FAIL  $*"; FAILED=1; }
warn() { echo "  note  $*"; }
FAILED=0

echo "== Checks"
if command -v ros2 >/dev/null; then ok "ros2 found"; else bad "ros2 not found: source /opt/ros/jazzy/setup.bash"; fi
if [ -f "$REPO/install/setup.bash" ]; then
  # ROS/colcon setup scripts read unset variables (COLCON_TRACE, AMENT_TRACE_SETUP_FILES, ...): relax -u for them
  set +u
  # shellcheck disable=SC1091
  source "$REPO/install/setup.bash"
  set -u
fi
if ros2 pkg prefix honebot_remote_bridge >/dev/null 2>&1; then
  ok "honebot_remote_bridge is built"
else
  bad "honebot_remote_bridge not built: cd $REPO && colcon build --symlink-install --packages-select honebot_remote_bridge honebot_hardware && source install/setup.bash"
fi
if [ -z "${DISPLAY:-}" ] && [ -z "${QT_QPA_PLATFORM:-}" ]; then
  export QT_QPA_PLATFORM=linuxfb
  warn "no DISPLAY: using QT_QPA_PLATFORM=linuxfb (works on the HDMI console; over SSH see CLAUDE.md 'sg video')"
fi

if [ "$MODE" != fake ]; then
  echo "  serial devices: $(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | tr '\n' ' ')"
  if [ -e "$PORT" ]; then ok "$PORT exists"; else bad "$PORT not found (plug the board in; check the list above)"; fi
  if systemctl is-active --quiet honebot 2>/dev/null; then
    bad "the honebot service is running and holds the robot ESP32's port: sudo systemctl stop honebot"
  fi
  if [ "$MODE" = serial ] && [ -e "$PORT" ]; then
    ok "remote board expected on $PORT: its lines look like 'state=STOP x=+0.00 y=+0.00 ...'"
  fi
fi
[ "$FAILED" = 0 ] || { echo "Fix the FAIL lines above and run again."; exit 1; }

ARGS=("source:=$([ "$MODE" = esp32 ] && echo esp32 || echo serial)" "follow:=$FOLLOW")
if [ "$MODE" = fake ]; then ARGS+=("port:=$FAKE"); else ARGS+=("port:=$PORT"); fi

echo
echo "== ros2 launch honebot_remote_bridge bridge_launch.py ${ARGS[*]}"
if [ "$MODE" != fake ]; then
  cat <<EOF
Then, with the remote (stick at rest when it powers up):
  - remote in STOP: the turtle does not move, even with the stick pushed
  - let go, hold the button 1.5 s -> RUN; stick away = forward, stick left = turn left
  - click: stops at once; remote off or unplugged: stops within 0.5 s
Ctrl+C to quit.
EOF
  exec ros2 launch honebot_remote_bridge bridge_launch.py "${ARGS[@]}"
fi

# fake: start the launch in its own process group, run the automatic check, then stop everything.
setsid ros2 launch honebot_remote_bridge bridge_launch.py "${ARGS[@]}" > /tmp/honebot_remote_launch.log 2>&1 &
LAUNCH_PID=$!
cleanup() { kill -TERM -- "-$LAUNCH_PID" 2>/dev/null; wait "$LAUNCH_PID" 2>/dev/null; }
trap cleanup EXIT
sleep 3
python3 "$REPO/tests/test_remote_turtlesim.py"
RESULT=$?
[ "$RESULT" = 0 ] || echo "Launch log: /tmp/honebot_remote_launch.log"
exit "$RESULT"
