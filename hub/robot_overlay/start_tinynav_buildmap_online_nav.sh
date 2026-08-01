#!/usr/bin/env bash
# Add fresh-session BuildMap navigation to an existing camera/perception session.
#
# This launcher deliberately starts no Go2 bridge. TinyNav may produce raw
# /cmd_vel only after an authenticated v2 receiver publishes a POI; physical
# output still requires the separate guarded bridge on /focus_guarded_cmd_vel.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${FOCUS_BUILDMAP_NAV_SESSION:-tinynav_semantic_nav_auto}"
TINYNAV_ROOT="${TINYNAV_ROOT:-$HOME/twork/tinynav}"
PATCHED_ROOT="${TINYNAV_PATCHED_ROOT:-$HOME/twork/tinynav-topofocus}"
SETUP_FILE="${TINYNAV_SETUP:-$HOME/twork/tinynav_setup.bash}"
PYTHON_BIN="${TINYNAV_PYTHON:-$TINYNAV_ROOT/.venv/bin/python}"
OUTPUT_DIR="${FOCUS_BUILDMAP_OUTPUT:-$HOME/.local/share/topofocus/maps/buildmap_online_$(date -u +%Y%m%dT%H%M%SZ)}"
FRAME_ID="world"
BASE_CAMERA_CALIBRATION_FILE="${FOCUS_WSJ_BASE_CAMERA_CALIBRATION_FILE:-${XDG_STATE_HOME:-$HOME/.local/state}/topofocus/calibration/robot0_camera_base.json}"
# The source occupancy mapper accepts a translational keyframe at 0.20 m and
# uses 0.05 m cells.  Let the latched grid bridge exactly one source keyframe
# interval plus one cell; a larger displacement still fails closed.
MAX_CACHED_MAP_MOTION_M="${FOCUS_MAX_CACHED_MAP_MOTION_M:-0.25}"
# Keep the router's freshness deadline identical to the independently checked
# data-plane limit.  The old 6 s/12 s split rejected a grid which startup had
# explicitly accepted, while the velocity gate and local depth planner
# remained healthy.
MAP_TIMEOUT_S="${FOCUS_WSJ_MAP_TIMEOUT_S:-12.0}"
# Observed on WSJ after the 2026-07-24 power cycle: /slam/odometry averages
# about 4 Hz but can have a 1.256 s inter-message gap.  Keep the router
# fail-closed while allowing that measured transient rather than rejecting the
# first supervised goal at the old 1.0 s default.
ODOMETRY_INPUT_TIMEOUT_S="${FOCUS_WSJ_ODOMETRY_INPUT_TIMEOUT_S:-3.0}"
# The graph supplies a known-free centreline; the endpoint footprint and
# TinyNav ESDF retain physical collision authority.  The bounded start search
# returns its complete path and never overrides observed occupied cells.
REACHABILITY_CLEARANCE_M="${FOCUS_WSJ_REACHABILITY_CLEARANCE_M:-0.05}"
PREFERRED_REACHABILITY_CLEARANCE_M="${FOCUS_WSJ_PREFERRED_REACHABILITY_CLEARANCE_M:-0.20}"
TERMINAL_OBSTACLE_CLEARANCE_M="${FOCUS_WSJ_TERMINAL_OBSTACLE_CLEARANCE_M:-0.50}"
LOOKAHEAD_M="${FOCUS_WSJ_LOOKAHEAD_M:-0.35}"
START_SNAP_RADIUS_M="${FOCUS_WSJ_START_SNAP_RADIUS_M:-0.75}"
START_FOOTPRINT_OVERRIDE_M="${FOCUS_WSJ_START_FOOTPRINT_OVERRIDE_M:-0.35}"
# Keep the semantic ARRIVED contract unchanged, but make online A* target its
# interior rather than the first 5 cm cell on the terminal rim.
SEMANTIC_TERMINAL_PLANNING_MARGIN_M="${FOCUS_WSJ_SEMANTIC_TERMINAL_PLANNING_MARGIN_M:-0.15}"
# Bound Python A* work so an unreachable component cannot starve fresh
# trajectory publication. The same contract is used by both robots.
MAX_PLAN_EXPANSIONS="${FOCUS_TINYNAV_MAX_PLAN_EXPANSIONS:-20000}"
MAX_PLAN_DURATION_S="${FOCUS_TINYNAV_MAX_PLAN_DURATION_S:-0.50}"

usage() {
  echo "Usage: $0 [--session NAME] [--output DIR] [--frame-id FRAME]"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session) SESSION="$2"; shift 2 ;;
    --output) OUTPUT_DIR="$2"; shift 2 ;;
    --frame-id) FRAME_ID="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "$FRAME_ID" ]] || {
  echo "Frame ID must not be empty." >&2
  exit 2
}
[[ "$MAX_PLAN_EXPANSIONS" =~ ^[1-9][0-9]*$ ]] || {
  echo "FOCUS_TINYNAV_MAX_PLAN_EXPANSIONS must be a positive integer." >&2
  exit 2
}
for required in \
  "$SETUP_FILE" \
  "$TINYNAV_ROOT/tinynav/core/planning_node.py" \
  "$TINYNAV_ROOT/tinynav/platforms/cmd_vel_control.py" \
  "$SCRIPT_DIR/run_tinynav_buildmap_live.py" \
  "$SCRIPT_DIR/run_tinynav_buildmap_online_mapping.py" \
  "$SCRIPT_DIR/navigation_occupancy_mapper.py" \
  "$SCRIPT_DIR/ros_continuous_depth_geometry_rgb.py" \
  "$SCRIPT_DIR/tinynav_source_contract.py" \
  "$SCRIPT_DIR/run_yunji_tinynav_planner.py" \
  "$SCRIPT_DIR/tinynav_buildmap_goal_router.py" \
  "$SCRIPT_DIR/yunji_tinynav_cmd_vel_control.py"; do
  [[ -f "$required" ]] || {
    echo "Required file is missing: $required" >&2
    exit 1
  }
done
[[ -r "$BASE_CAMERA_CALIBRATION_FILE" ]] || {
  echo "Required measured mount is missing: $BASE_CAMERA_CALIBRATION_FILE" >&2
  exit 1
}
[[ -x "$PYTHON_BIN" ]] || {
  echo "TinyNav Python is missing or not executable: $PYTHON_BIN" >&2
  exit 1
}
tmux has-session -t "$SESSION" 2>/dev/null || {
  echo "Camera/perception tmux session is not running: $SESSION" >&2
  exit 1
}
for required_window in camera perception; do
  tmux list-windows -t "$SESSION" -F '#{window_name}' \
    | grep -qx "$required_window" || {
      echo "Required observation window is missing: $SESSION:$required_window" >&2
      exit 1
    }
done
for forbidden_window in \
  map maploc online-map planning goal-router control go2-bridge rviz-goal static-map; do
  if tmux list-windows -t "$SESSION" -F '#{window_name}' \
    | grep -qx "$forbidden_window"; then
    echo "Refusing to replace existing window: $SESSION:$forbidden_window" >&2
    exit 1
  fi
done
[[ ! -e "$OUTPUT_DIR" && ! -e "$OUTPUT_DIR.online_occupancy" ]] || {
  echo "Refusing to overwrite an existing BuildMap output." >&2
  exit 1
}
if pgrep -f 'go2_cmd_bridge|cmd_vel_control|planning_node.py|run_yunji_tinynav_planner.py|map_node.py|nav2_controller' \
  >/dev/null 2>&1; then
  echo "Refusing startup while an old planner/controller/bridge is running." >&2
  exit 1
fi

FOCUS_REHEARSAL_SESSION="$SESSION" \
TINYNAV_ROOT="$TINYNAV_ROOT" \
TINYNAV_PATCHED_ROOT="$PATCHED_ROOT" \
TINYNAV_SETUP="$SETUP_FILE" \
TINYNAV_PYTHON="$PYTHON_BIN" \
bash "$SCRIPT_DIR/start_go2_buildmap.sh" \
  --session "$SESSION" \
  --output "$OUTPUT_DIR"

started_windows=()
cleanup_partial_start() {
  for window in "${started_windows[@]}"; do
    tmux kill-window -t "$SESSION:$window" >/dev/null 2>&1 || true
  done
  echo "Online navigation startup failed; BuildMap remains mapping-only in $SESSION:maploc." >&2
  echo "Finalize it with save_go2_buildmap.sh before stopping the observation session." >&2
}
trap cleanup_partial_start ERR

online_output="$OUTPUT_DIR.online_occupancy"
tmux new-window -d -t "$SESSION" -n online-map \
  "bash -lc 'source \"$SETUP_FILE\"; source \"$TINYNAV_ROOT/install/setup.bash\" 2>/dev/null || true; \"$PYTHON_BIN\" -u \"$SCRIPT_DIR/run_tinynav_buildmap_online_mapping.py\" --target-frame \"$FRAME_ID\" --output-directory \"$online_output\" --robot-id robot-0 --camera-frame camera --base-camera-calibration-file \"$BASE_CAMERA_CALIBRATION_FILE\"'"
started_windows+=("online-map")

tmux new-window -d -t "$SESSION" -n planning \
  "bash -lc 'source \"$SETUP_FILE\"; cd \"$TINYNAV_ROOT\"; uv run python \"$SCRIPT_DIR/run_yunji_tinynav_planner.py\" --robot-profile source-default'"
started_windows+=("planning")

tmux new-window -d -t "$SESSION" -n goal-router \
  "bash -lc 'source \"$SETUP_FILE\"; export PYTHONPATH=\"$SCRIPT_DIR/../src\":\${PYTHONPATH:-}; \"$PYTHON_BIN\" -u \"$SCRIPT_DIR/tinynav_buildmap_goal_router.py\" --frame-id \"$FRAME_ID\" --occupancy-topic /semantic_mapping/occupancy_bev --base-camera-calibration-file \"$BASE_CAMERA_CALIBRATION_FILE\" --lookahead-m \"$LOOKAHEAD_M\" --clearance-m \"$REACHABILITY_CLEARANCE_M\" --preferred-clearance-m \"$PREFERRED_REACHABILITY_CLEARANCE_M\" --terminal-obstacle-clearance-m \"$TERMINAL_OBSTACLE_CLEARANCE_M\" --semantic-terminal-planning-margin-m \"$SEMANTIC_TERMINAL_PLANNING_MARGIN_M\" --start-snap-radius-m \"$START_SNAP_RADIUS_M\" --start-footprint-override-m \"$START_FOOTPRINT_OVERRIDE_M\" --input-timeout-s \"$ODOMETRY_INPUT_TIMEOUT_S\" --map-timeout-s \"$MAP_TIMEOUT_S\" --max-cached-map-motion-m \"$MAX_CACHED_MAP_MOTION_M\" --max-plan-expansions \"$MAX_PLAN_EXPANSIONS\" --max-plan-duration-s \"$MAX_PLAN_DURATION_S\"'"
started_windows+=("goal-router")

tmux new-window -d -t "$SESSION" -n control \
  "bash -lc 'source \"$SETUP_FILE\"; cd \"$TINYNAV_ROOT\"; uv run python \"$SCRIPT_DIR/yunji_tinynav_cmd_vel_control.py\" --robot-profile source-default --robot-id robot-0 --base-camera-frame camera --base-camera-calibration-file \"$BASE_CAMERA_CALIBRATION_FILE\" --verified-forward-only-planner --rotate-first-on-reverse --stabilize-large-turn --linear-command-floor-mps 0.18 --rotate-first-max-angular-radps 0.35 --rotate-first-timeout-s 12.0'"
started_windows+=("control")

source "$SETUP_FILE"
deadline=$((SECONDS + 45))
for topic in \
  /benchmark/stop \
  /semantic_mapping/occupancy_bev \
  /mapping/cmd_pois \
  /planning/trajectory_path \
  /cmd_vel; do
  until ros2 topic list 2>/dev/null | grep -qx "$topic"; do
    (( SECONDS < deadline )) || {
      echo "Timed out waiting for online navigation topic: $topic" >&2
      exit 1
    }
    sleep 1
  done
done

if ros2 node list 2>/dev/null | grep -Eq 'go2|unitree.*bridge'; then
  echo "Unexpected Go2 bridge node appeared during no-bridge startup." >&2
  exit 1
fi

trap - ERR
echo "TinyNav fresh-session BuildMap navigation is ready (NO GO2 BRIDGE):"
echo "  session:          $SESSION"
echo "  BuildMap output:  $OUTPUT_DIR"
echo "  online occupancy: $online_output"
echo "  frame:            $FRAME_ID"
echo "  global source:    BuildMapNode + online known/free/occupied grid"
echo "  local planner:    Focus forward-only wrapper over TinyNav planning_node.py"
echo "  controller:       Focus-wrapped TinyNav cmd_vel_control.py -> raw /cmd_vel only"
echo "  target source:    versioned /mapping/cmd_pois only"
echo "  stale-map motion: <=${MAX_CACHED_MAP_MOTION_M} m (source keyframe + one cell)"
