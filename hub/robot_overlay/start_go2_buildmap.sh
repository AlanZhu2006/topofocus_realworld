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

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session) session="$2"; shift 2 ;;
    --output) OUTPUT_DIR="$2"; shift 2 ;;
    -h|--help) echo "Usage: $0 [--session NAME] [--output DIR]"; exit 0 ;;
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
if pgrep -af 'go2_cmd_bridge|cmd_vel_control|planning_node.py|nav2_controller' >/dev/null 2>&1; then
  echo "Refusing BuildMap while a known actuation/planner process is running." >&2
  exit 1
fi

mkdir -p "$(dirname "$OUTPUT_DIR")"
log_file="$OUTPUT_DIR.log"
command_text="source '$SETUP_FILE'; export PYTHONPATH='$PATCHED_ROOT':\${PYTHONPATH:-}; cd '$PATCHED_ROOT'; set -o pipefail; '$PYTHON_BIN' -u '$SCRIPT_DIR/run_tinynav_buildmap_live.py' --map-save-path '$OUTPUT_DIR' --quiet-timers 2>&1 | tee '$log_file'"
tmux new-window -d -t "$session" -n maploc "bash -lc \"$command_text\""

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
echo "Move the robot only under direct operator control."
echo "When finished: bash $SCRIPT_DIR/save_go2_buildmap.sh --session $session"
