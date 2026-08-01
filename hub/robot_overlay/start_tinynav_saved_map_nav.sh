#!/usr/bin/env bash
# Replace the non-actuating WSJ navigation core with saved-map relocalization.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${FOCUS_WSJ_NAV_SESSION:-tinynav_semantic_nav_auto}"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
TINYNAV_ROOT="${TINYNAV_ROOT:-/home/nvidia/twork/tinynav}"
PYTHON_BIN="${TINYNAV_PYTHON:-/home/nvidia/twork/tinynav/.venv/bin/python}"
BASE_CAMERA_CALIBRATION_FILE="${FOCUS_WSJ_BASE_CAMERA_CALIBRATION_FILE:-/home/nvidia/.local/state/topofocus/calibration/wsj_tinynav_camera_base_20260723_operator.json}"
LINEAR_COMMAND_FLOOR_MPS="${FOCUS_WSJ_LINEAR_COMMAND_FLOOR_MPS:-0.18}"
MAP_TIMEOUT_S="${FOCUS_WSJ_MAP_TIMEOUT_S:-12.0}"
ODOMETRY_INPUT_TIMEOUT_S="${FOCUS_WSJ_ODOMETRY_INPUT_TIMEOUT_S:-3.0}"
MAX_CACHED_MAP_MOTION_M="${FOCUS_MAX_CACHED_MAP_MOTION_M:-0.25}"
REACHABILITY_CLEARANCE_M="${FOCUS_WSJ_REACHABILITY_CLEARANCE_M:-0.05}"
PREFERRED_REACHABILITY_CLEARANCE_M="${FOCUS_WSJ_PREFERRED_REACHABILITY_CLEARANCE_M:-0.20}"
TERMINAL_OBSTACLE_CLEARANCE_M="${FOCUS_WSJ_TERMINAL_OBSTACLE_CLEARANCE_M:-0.50}"
LOOKAHEAD_M="${FOCUS_WSJ_LOOKAHEAD_M:-0.35}"
START_SNAP_RADIUS_M="${FOCUS_WSJ_START_SNAP_RADIUS_M:-0.75}"
START_FOOTPRINT_OVERRIDE_M="${FOCUS_WSJ_START_FOOTPRINT_OVERRIDE_M:-0.35}"
SEMANTIC_TERMINAL_PLANNING_MARGIN_M="${FOCUS_WSJ_SEMANTIC_TERMINAL_PLANNING_MARGIN_M:-0.15}"
MAX_PLAN_EXPANSIONS="${FOCUS_TINYNAV_MAX_PLAN_EXPANSIONS:-20000}"
MAX_PLAN_DURATION_S="${FOCUS_TINYNAV_MAX_PLAN_DURATION_S:-0.50}"
map_directory="${FOCUS_WSJ_TINYNAV_SAVED_MAP:-}"
map_manifest=""

usage() {
  cat <<'EOF'
Usage: start_tinynav_saved_map_nav.sh --map-directory DIR [options]
  --map-directory DIR
  --map-manifest FILE
  --session NAME

This script starts only localization, occupancy, planning and raw-controller
processes. It refuses an active receiver/chassis bridge, publishes a latched
pause and guarded zero, and never creates a Go2 SDK process.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map-directory) map_directory="$2"; shift 2 ;;
    --map-manifest) map_manifest="$2"; shift 2 ;;
    --session) SESSION="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$map_directory" = /* ]] || {
  echo "--map-directory must be an explicit absolute path." >&2
  exit 2
}
map_directory="$(readlink -f "$map_directory")"
[[ -n "$map_manifest" ]] \
  || map_manifest="$map_directory/focus_saved_map_manifest.json"
[[ "$map_manifest" = /* ]] || {
  echo "--map-manifest must be an explicit absolute path." >&2
  exit 2
}
map_manifest="$(readlink -f "$map_manifest")"
[[ "$map_manifest" == "$map_directory/focus_saved_map_manifest.json" ]] || {
  echo "Saved-map manifest must be the map's immutable in-directory contract." >&2
  exit 2
}
for required in \
  "$SETUP_FILE" \
  "$PYTHON_BIN" \
  "$BASE_CAMERA_CALIBRATION_FILE" \
  "$map_manifest" \
  "$SCRIPT_DIR/start_wsj_command_observation.sh" \
  "$SCRIPT_DIR/run_tinynav_saved_map_localization.py" \
  "$SCRIPT_DIR/tinynav_relocalized_odometry.py" \
  "$SCRIPT_DIR/run_tinynav_buildmap_online_mapping.py" \
  "$SCRIPT_DIR/tinynav_buildmap_goal_router.py" \
  "$SCRIPT_DIR/run_yunji_tinynav_planner.py" \
  "$SCRIPT_DIR/yunji_tinynav_cmd_vel_control.py"; do
  [[ -r "$required" ]] || {
    echo "Missing saved-map runtime input: $required" >&2
    exit 1
  }
done
tmux has-session -t "$SESSION" 2>/dev/null || {
  echo "TinyNav camera/perception session is unavailable: $SESSION" >&2
  exit 1
}
for window in camera perception; do
  [[ "$(tmux display-message -p -t "$SESSION:$window" '#{pane_dead}' \
      2>/dev/null || true)" == 0 ]] || {
    echo "Required TinyNav publisher window is unavailable: $window" >&2
    exit 1
  }
done
if pgrep -af \
    'go2_cmd_bridge|v2_wsj_receiver\.py|focus_water_cmd_vel_bridge' \
    >/dev/null 2>&1; then
  echo "Refusing saved-map core replacement while a command path exists." >&2
  exit 1
fi

source "$SETUP_FILE"
timeout 5 ros2 topic pub --once \
  /nav/paused std_msgs/msg/Bool '{data: true}' \
  >/dev/null 2>&1 || true
timeout 5 ros2 topic pub --once \
  /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' \
  >/dev/null 2>&1 || true

# Create the long-lived observation subscriber before its new relocalized pose
# publisher. This retains the verified publisher-last Fast DDS lifecycle.
bash "$SCRIPT_DIR/start_wsj_command_observation.sh" \
  --park-only \
  --session "$SESSION" \
  --pose-topic /focus/maploc/odometry_visual \
  --pose-frame map

for window in maploc map-odom online-map planning goal-router control; do
  tmux kill-window -t "$SESSION:$window" >/dev/null 2>&1 || true
done
deadline=$((SECONDS + 20))
while pgrep -af \
    'map_node.py|tinynav_relocalized_odometry.py|run_tinynav_buildmap_online_mapping.py|run_yunji_tinynav_planner.py|tinynav_buildmap_goal_router.py|yunji_tinynav_cmd_vel_control.py' \
    >/dev/null 2>&1; do
  (( SECONDS < deadline )) || {
    echo "An old navigation-core process survived its managed tmux window." >&2
    exit 1
  }
  sleep 1
done

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
state_root="/home/nvidia/.local/state/topofocus/saved-map-${stamp}"
scratch="$state_root/maploc-scratch"
occupancy_output="$state_root/online-occupancy"
mkdir -p "$state_root"

planning_command="bash -lc 'source \"$SETUP_FILE\"; cd \"$TINYNAV_ROOT\"; uv run python \"$SCRIPT_DIR/run_yunji_tinynav_planner.py\" --robot-profile source-default'"
goal_router_command="bash -lc 'source \"$SETUP_FILE\"; export PYTHONPATH=\"$SCRIPT_DIR/../src\":\${PYTHONPATH:-}; \"$PYTHON_BIN\" -u \"$SCRIPT_DIR/tinynav_buildmap_goal_router.py\" --frame-id map --odom-topic /focus/maploc/odometry --occupancy-topic /semantic_mapping/occupancy_bev --base-camera-calibration-file \"$BASE_CAMERA_CALIBRATION_FILE\" --lookahead-m \"$LOOKAHEAD_M\" --clearance-m \"$REACHABILITY_CLEARANCE_M\" --preferred-clearance-m \"$PREFERRED_REACHABILITY_CLEARANCE_M\" --terminal-obstacle-clearance-m \"$TERMINAL_OBSTACLE_CLEARANCE_M\" --semantic-terminal-planning-margin-m \"$SEMANTIC_TERMINAL_PLANNING_MARGIN_M\" --start-snap-radius-m \"$START_SNAP_RADIUS_M\" --start-footprint-override-m \"$START_FOOTPRINT_OVERRIDE_M\" --input-timeout-s \"$ODOMETRY_INPUT_TIMEOUT_S\" --map-timeout-s \"$MAP_TIMEOUT_S\" --max-cached-map-motion-m \"$MAX_CACHED_MAP_MOTION_M\" --max-plan-expansions \"$MAX_PLAN_EXPANSIONS\" --max-plan-duration-s \"$MAX_PLAN_DURATION_S\"'"
control_command="bash -lc 'source \"$SETUP_FILE\"; cd \"$TINYNAV_ROOT\"; uv run python \"$SCRIPT_DIR/yunji_tinynav_cmd_vel_control.py\" --robot-profile source-default --robot-id robot-0 --base-camera-frame camera --base-camera-calibration-file \"$BASE_CAMERA_CALIBRATION_FILE\" --verified-forward-only-planner --rotate-first-on-reverse --stabilize-large-turn --linear-command-floor-mps \"$LINEAR_COMMAND_FLOOR_MPS\" --rotate-first-max-angular-radps 0.35 --rotate-first-timeout-s 12.0 --ros-args -r /slam/odometry_visual:=/focus/maploc/odometry_visual'"
bridge_command="bash -lc 'source \"$SETUP_FILE\"; export PYTHONPATH=\"$SCRIPT_DIR/../src\":\${PYTHONPATH:-}; \"$PYTHON_BIN\" -u \"$SCRIPT_DIR/tinynav_relocalized_odometry.py\" --map-directory \"$map_directory\" --map-manifest \"$map_manifest\"'"
maploc_command="bash -lc 'source \"$SETUP_FILE\"; export PYTHONPATH=\"$SCRIPT_DIR/../src\":\${PYTHONPATH:-}; \"$PYTHON_BIN\" -u \"$SCRIPT_DIR/run_tinynav_saved_map_localization.py\" --map-directory \"$map_directory\" --map-manifest \"$map_manifest\" --scratch-directory \"$scratch\"'"
online_map_command="bash -lc 'source \"$SETUP_FILE\"; source \"$TINYNAV_ROOT/install/setup.bash\" 2>/dev/null || true; \"$PYTHON_BIN\" -u \"$SCRIPT_DIR/run_tinynav_buildmap_online_mapping.py\" --target-frame map --tracking-frame world --output-directory \"$occupancy_output\"'"

# Every consumer exists before the corresponding new publisher. No chassis
# bridge exists, and /nav/paused remains latched throughout startup.
tmux new-window -d -t "$SESSION" -n planning "$planning_command"
tmux new-window -d -t "$SESSION" -n goal-router "$goal_router_command"
tmux new-window -d -t "$SESSION" -n control "$control_command"
tmux new-window -d -t "$SESSION" -n map-odom "$bridge_command"
tmux new-window -d -t "$SESSION" -n maploc "$maploc_command"
tmux new-window -d -t "$SESSION" -n online-map "$online_map_command"

for window in planning goal-router control map-odom maploc online-map; do
  tmux set-option -w -t "$SESSION:$window" remain-on-exit on
done
deadline=$((SECONDS + 30))
required_nodes=(
  /planning_node
  /focus_tinynav_buildmap_goal_router
  /cmd_vel_control_node
  /focus_tinynav_relocalized_odometry
  /map_node
  /semantic_pointcloud_node
  /occupancy_mapper_node
)
while true; do
  nodes="$(timeout 5 ros2 node list 2>/dev/null || true)"
  missing=()
  for node in "${required_nodes[@]}"; do
    grep -qx "$node" <<<"$nodes" || missing+=("$node")
  done
  (( ${#missing[@]} == 0 )) && break
  for window in planning goal-router control map-odom maploc online-map; do
    if [[ "$(tmux display-message -p -t "$SESSION:$window" \
        '#{pane_dead}' 2>/dev/null || true)" == 1 ]]; then
      echo "Saved-map component exited during startup: $window" >&2
      tmux capture-pane -pt "$SESSION:$window" -S -100 >&2 || true
      exit 1
    fi
  done
  (( SECONDS < deadline )) || {
    echo "Timed out waiting for saved-map nodes: ${missing[*]}" >&2
    exit 1
  }
  sleep 1
done

echo "TinyNav saved-map core started in HOLD."
echo "  map:      $map_directory"
echo "  manifest: $map_manifest"
echo "  scratch:  $scratch"
echo "  pose:     /focus/maploc/odometry_visual (map -> camera)"
echo "  status:   /focus/maploc/status"
echo "Safety: no receiver or Go2 bridge was created; robot motion is impossible through this stack."
