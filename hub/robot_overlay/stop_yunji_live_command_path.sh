#!/usr/bin/env bash
# Remove every Yunji live command owner and obtain an explicit WATER zero ACK.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

receiver_units=(
  focus-yunji-v2-debug-v2.service
  focus-yunji-v2-live-v2.service
  focus-yunji-v2-debug-v3.service
  focus-yunji-v2-live-v3.service
)
bridge_units=(
  focus-yunji-water-bridge-debug-v1.service
  focus-yunji-water-bridge-live-v1.service
)

stop_units() {
  local unit
  for unit in "$@"; do
    sudo -n systemctl stop "$unit" >/dev/null 2>&1 || true
    sudo -n systemctl reset-failed "$unit" >/dev/null 2>&1 || true
  done
}

# First revoke the authenticated publisher.  Leave the bridge alive for one
# watchdog interval so it independently emits zero before it is removed.
stop_units "${receiver_units[@]}"
sleep 0.4

/bin/bash "$SCRIPT_DIR/run_yunji_tinynav_component.sh" bridge \
  --send-explicit-zero
stop_units "${bridge_units[@]}"
/bin/bash "$SCRIPT_DIR/run_yunji_tinynav_component.sh" bridge \
  --send-explicit-zero

if pgrep -af \
    'water_cmd_vel_bridge\.py|v2_wsj_receiver\.py.*--robot-id robot-1' \
    >/dev/null 2>&1; then
  echo "Yunji command process remained after explicit zero." >&2
  exit 1
fi

echo "YUNJI_EXPLICIT_ZERO_CONFIRMED"
