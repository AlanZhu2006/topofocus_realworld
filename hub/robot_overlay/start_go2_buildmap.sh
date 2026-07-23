#!/usr/bin/env bash
# Add native TinyNav BuildMap to an already healthy observation-only session.
# This publishes no motion command; the human operator moves the robot.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
session="${FOCUS_REHEARSAL_SESSION:-focus_live_rehearsal}"
TINYNAV_ROOT="${TINYNAV_ROOT:-/home/nvidia/twork/tinynav}"
PATCHED_ROOT="${TINYNAV_PATCHED_ROOT:-/home/nvidia/twork/tinynav-topofocus}"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
PYTHON_BIN="${TINYNAV_PYTHON:-$TINYNAV_ROOT/.venv/bin/python}"
OUTPUT_DIR="${FOCUS_BUILDMAP_OUTPUT:-$HOME/.local/share/topofocus/maps/buildmap_$(date -u +%Y%m%dT%H%M%SZ)}"
repair_online_stack="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session) session="$2"; shift 2 ;;
    --output) OUTPUT_DIR="$2"; shift 2 ;;
    --repair-online-stack) repair_online_stack="true"; shift ;;
    -h|--help)
      echo "Usage: $0 [--session NAME] [--output DIR] [--repair-online-stack]"
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

tmux has-session -t "$session" 2>/dev/null || {
  echo "Observation session is not running: $session" >&2
  exit 1
}
tmux list-windows -t "$session" -F '#{window_name}' | grep -qx maploc && {
  echo "A maploc window already exists in $session" >&2
  exit 1
}
[[ ! -e "$OUTPUT_DIR" ]] || {
  echo "Refusing to overwrite existing output: $OUTPUT_DIR" >&2
  exit 1
}
if [[ "$repair_online_stack" == "true" ]]; then
  for required_window in online-map planning goal-router control; do
    tmux list-windows -t "$session" -F '#{window_name}' \
      | grep -qx "$required_window" || {
        echo "Repair requires existing $session:$required_window." >&2
        exit 1
      }
  done
  if tmux list-windows -t "$session" -F '#{window_name}' \
      | grep -qx go2-bridge \
     || pgrep -af 'go2_cmd_bridge|nav2_controller' >/dev/null 2>&1 \
     || pgrep -af 'v2_wsj_receiver\.py.*--enable-live-go2-motion' \
        >/dev/null 2>&1; then
    echo "Repair refuses any live physical command path." >&2
    exit 1
  fi
  source "$SETUP_FILE"
  ros2 topic pub --once /nav/paused std_msgs/msg/Bool '{data: true}' \
    >/dev/null 2>&1 || true
else
  if pgrep -af \
      'go2_cmd_bridge|cmd_vel_control|planning_node.py|nav2_controller' \
      >/dev/null 2>&1; then
    echo "Refusing BuildMap while a known actuation/planner process is running." >&2
    exit 1
  fi
fi

mkdir -p "$(dirname "$OUTPUT_DIR")"
log_file="$OUTPUT_DIR.log"
runner=(
  "$PYTHON_BIN" -u "$SCRIPT_DIR/run_tinynav_buildmap_live.py"
  --map-save-path "$OUTPUT_DIR"
  --quiet-timers
)
printf -v runner_text '%q ' "${runner[@]}"
tmux new-window -d -t "$session" -n maploc \
  "bash -lc 'source \"$SETUP_FILE\"; export PYTHONPATH=\"$PATCHED_ROOT\":\${PYTHONPATH:-}; cd \"$PATCHED_ROOT\"; set -o pipefail; $runner_text 2>&1 | tee \"$log_file\"'"

source "$SETUP_FILE"
deadline=$((SECONDS + 45))
until ros2 topic list 2>/dev/null | grep -qx '/benchmark/stop'; do
  (( SECONDS < deadline )) || {
    echo "BuildMap did not expose /benchmark/stop; inspect $log_file" >&2
    exit 1
  }
  sleep 1
done

echo "Native BuildMap is ready:"
echo "  output: $OUTPUT_DIR"
echo "  log:    $log_file"
if [[ "$repair_online_stack" == "true" ]]; then
  echo "  repair: restored missing maploc while the guarded bridge was absent"
fi
echo "Move the robot only under direct operator control."
echo "When finished: bash $SCRIPT_DIR/save_go2_buildmap.sh --session $session"
