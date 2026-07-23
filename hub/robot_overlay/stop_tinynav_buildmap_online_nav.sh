#!/usr/bin/env bash
# Fail closed, finalize the fresh BuildMap, and keep camera/perception running.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${FOCUS_BUILDMAP_NAV_SESSION:-tinynav_semantic_nav_auto}"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
keep_buildmap="false"

usage() {
  echo "Usage: $0 [--session NAME] [--keep-buildmap]"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session) SESSION="$2"; shift 2 ;;
    --keep-buildmap) keep_buildmap="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

tmux has-session -t "$SESSION" 2>/dev/null || {
  echo "TinyNav session is not running: $SESSION" >&2
  exit 1
}
source "$SETUP_FILE"

# The latched pause and guarded zero are stop commands, not navigation goals.
ros2 topic pub --once /nav/paused std_msgs/msg/Bool '{data: true}' \
  >/dev/null 2>&1 || true
if ros2 topic list 2>/dev/null | grep -qx /focus_guarded_cmd_vel; then
  ros2 topic pub --once /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' \
    >/dev/null 2>&1 || true
fi

for window in go2-bridge v2-receiver; do
  if tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -qx "$window"; then
    tmux send-keys -t "$SESSION:$window" C-c >/dev/null 2>&1 || true
  fi
done
sleep 1
for window in go2-bridge v2-receiver control goal-router planning online-map; do
  tmux kill-window -t "$SESSION:$window" >/dev/null 2>&1 || true
done

if [[ "$keep_buildmap" == "true" ]]; then
  echo "Motion/planning path stopped; BuildMap remains active in $SESSION:maploc."
  exit 0
fi

if tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -qx maploc; then
  bash "$SCRIPT_DIR/save_go2_buildmap.sh" --session "$SESSION"
else
  echo "No active maploc window; there was no BuildMap process to finalize."
fi

echo "Online navigation stopped. Camera/perception remain available."
echo "No map, calibration, log, token or observation was deleted."
