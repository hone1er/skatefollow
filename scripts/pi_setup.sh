#!/usr/bin/env bash
# honebot Pi housekeeping: run ON THE PI, from the workspace, as your normal user. It asks for your sudo
# password itself. Safe to run again at any time (it only changes what differs).
#
#   cd ~/honebot_ws && ./scripts/pi_setup.sh            # udev rule + setserial + auto-start service
#   ./scripts/pi_setup.sh --no-service                  # everything except the auto-start service
#   ./scripts/pi_setup.sh --uninstall-service           # stop and remove the auto-start service
#
# What it does:
#   1. installs the 'setserial' package (needed by the udev rule)
#   2. installs the udev rule: /dev/esp32 symlink + no 5 s delay when the bridge closes the serial port
#   3. installs honebot.service so bringup (bridge + safety gate + Foxglove) starts at boot
# It does NOT touch the GoPro netplan file (already installed) or build the workspace.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RULE_SRC="$WS/src/honebot_hardware/udev/99-honebot-esp32.rules"
RULE_DST="/etc/udev/rules.d/99-honebot-esp32.rules"
UNIT_SRC="$WS/src/honebot_bringup/systemd/honebot.service"
UNIT_DST="/etc/systemd/system/honebot.service"
MODE="${1:-}"

say() { printf '\n== %s\n' "$*"; }

if [[ "$MODE" == "--uninstall-service" ]]; then
  say "Removing honebot.service"
  sudo systemctl disable --now honebot.service 2>/dev/null || true
  sudo rm -f "$UNIT_DST"
  sudo systemctl daemon-reload
  echo "done"
  exit 0
fi

[[ -f "$RULE_SRC" && -f "$UNIT_SRC" ]] || { echo "Run this from the workspace: $RULE_SRC not found" >&2; exit 1; }
[[ -f "$WS/install/setup.bash" ]] || { echo "Build first:  cd $WS && colcon build --symlink-install" >&2; exit 1; }

say "1/3 setserial"
if command -v setserial >/dev/null; then
  echo "already installed"
else
  sudo apt-get install -y setserial
fi

say "2/3 udev rule"
if sudo cmp -s "$RULE_SRC" "$RULE_DST"; then
  echo "already up to date"
else
  sudo install -m 0644 "$RULE_SRC" "$RULE_DST"
  sudo udevadm control --reload-rules
  sudo udevadm trigger --action=add --subsystem-match=tty
  echo "installed"
fi
sleep 1
if [[ -e /dev/esp32 ]]; then
  echo "ok: /dev/esp32 -> $(readlink -f /dev/esp32)"
else
  echo "note: /dev/esp32 not there yet (is the ESP32 plugged in? unplug and replug it once)"
fi

if [[ "$MODE" == "--no-service" ]]; then
  say "3/3 service skipped (--no-service)"
  exit 0
fi

say "3/3 auto-start service"
# The unit in the repo is written for user 'hone1er' and ~/honebot_ws; adapt it to where this copy lives.
TMP="$(mktemp)"
sed -e "s|^User=.*|User=$USER|" -e "s|/home/hone1er/honebot_ws|$WS|g" "$UNIT_SRC" > "$TMP"
if sudo cmp -s "$TMP" "$UNIT_DST"; then
  echo "unit already up to date"
else
  sudo install -m 0644 "$TMP" "$UNIT_DST"
  sudo systemctl daemon-reload
  echo "unit installed"
fi
rm -f "$TMP"
sudo systemctl enable honebot.service
sudo systemctl restart honebot.service
sleep 5
if systemctl is-active --quiet honebot.service; then
  echo "ok: honebot.service is running"
else
  echo "PROBLEM: honebot.service is not running. Look at:  journalctl -u honebot -n 50" >&2
  exit 1
fi

cat <<MSG

All done. Useful commands:
  journalctl -u honebot -f          watch the robot's log
  sudo systemctl stop honebot       stop it (do this BEFORE running 'ros2 launch honebot_bringup ...' by hand:
                                    two bridges cannot share the serial port)
  sudo systemctl start honebot      start it again
MSG
