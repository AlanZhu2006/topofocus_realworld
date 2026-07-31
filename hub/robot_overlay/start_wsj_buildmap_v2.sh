#!/usr/bin/env bash
# Start the minimal WSJ BuildMap/v2 stack in debug or explicitly armed live mode.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${FOCUS_WSJ_NAV_SESSION:-tinynav_semantic_nav_auto}"
ENV_FILE="${FOCUS_WSJ_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/topofocus/robot-0.env}"
if [[ -r "$ENV_FILE" ]]; then
  set -a
  # bootstrap_robot0_cleanroom.sh emits shell-quoted assignments only.
  # shellcheck disable=SC1090
  source "$ENV_FILE"
  set +a
fi
SETUP_FILE="${TINYNAV_SETUP:-$HOME/twork/tinynav_setup.bash}"
TINYNAV_ROOT="${TINYNAV_ROOT:-$HOME/twork/tinynav}"
PYTHON_BIN="${TINYNAV_PYTHON:-$TINYNAV_ROOT/.venv/bin/python}"
default_token_file="${XDG_CONFIG_HOME:-$HOME/.config}/topofocus/robot-0.token"
[[ -r "$default_token_file" ]] || default_token_file="$HOME/focus_sender/.token"
TOKEN_FILE="${FOCUS_ROBOT_TOKEN_FILE:-$default_token_file}"
CALIBRATION_FILE="${FOCUS_SHARED_CALIBRATION_FILE:-}"
BASE_CAMERA_CALIBRATION_FILE="${FOCUS_WSJ_BASE_CAMERA_CALIBRATION_FILE:-${XDG_STATE_HOME:-$HOME/.local/state}/topofocus/calibration/robot0_camera_base.json}"
TRANSFORM_VERSION="${FOCUS_WSJ_TRANSFORM_VERSION:-}"
CALIBRATION_ID="${FOCUS_SHARED_CALIBRATION_ID:-}"
HUB_URL="${FOCUS_HUB_BASE_URL:-http://127.0.0.1:18089}"
DEPLOYMENT_COMMIT="${FOCUS_DEPLOYMENT_COMMIT:-}"
FASTDDS_BUILTIN_TRANSPORTS_VALUE="${FOCUS_WSJ_FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
PATCHED_ROOT="${TINYNAV_PERCEPTION_PATCHED_ROOT:-$HOME/focus_sender/tinynav_imu_fix_worktree_20260721}"
PATCHED_COMMIT="${TINYNAV_PERCEPTION_PATCHED_COMMIT:-29f26bc058886ff450f02cdc0d6e9977e1c57010}"
PATCHED_PERCEPTION_SHA256="${TINYNAV_PERCEPTION_PATCHED_SHA256:-3a695d5210d60ea1f721549ca7458ba89e7bf32db5178cd1c312c633aef1c3b3}"
GO2_BRIDGE_SOURCE="${FOCUS_GO2_BRIDGE_SOURCE:-$TINYNAV_ROOT/tool/go2_cmd_bridge.py}"
GO2_BRIDGE_SOURCE_SIZE=11678
GO2_BRIDGE_SOURCE_SHA256="8b81107f89ed4013529f325f75a80b39295e828a67e2e1d87c432d860f19ebb2"
GO2_BRIDGE_RUNNER="${FOCUS_GO2_BRIDGE_RUNNER:-$TINYNAV_ROOT/scripts/run_go2_cmd_bridge.sh}"
GO2_BRIDGE_RUNNER_SIZE=3514
GO2_BRIDGE_RUNNER_SHA256="f0d06edb8d1ac59b497aac77b099927d415baf63700a6cd73d240cbd0d7b9c21"
# Keep these values identical to start_tinynav_buildmap_online_nav.sh.  The
# Mapper/planner can remain alive across supervised episodes, but the overlay
# goal router and velocity wrapper must be reloaded so they cannot retain
# pre-deployment Python code.
MAX_CACHED_MAP_MOTION_M="${FOCUS_MAX_CACHED_MAP_MOTION_M:-0.25}"
LINEAR_COMMAND_FLOOR_MPS="${FOCUS_WSJ_LINEAR_COMMAND_FLOOR_MPS:-0.18}"
MAP_TIMEOUT_S="${FOCUS_WSJ_MAP_TIMEOUT_S:-12.0}"
# TinyNav's goal router keeps a strict three-second odometry deadline.  It
# enters HOLD on a transient gap and the v2 receiver immediately closes its
# guarded velocity output; the receiver must remain alive slightly longer so
# the existing bounded router-recovery path can observe odometry returning.
ODOMETRY_INPUT_TIMEOUT_S="${FOCUS_WSJ_ODOMETRY_INPUT_TIMEOUT_S:-3.0}"
# Observed physical provenance (2026-07-25): WSJ had 2.227 s and 3.278 s
# visual-odometry gaps while SLAM diagnostics, occupancy, graph alignment, and
# the Go2 bridge stayed healthy.  Five seconds separates transient recovery
# from a persistent localization loss.  This is not a stale-motion allowance:
# the guarded trajectory output closes after 1.0 s and TinyNav/controller
# watchdogs retain final authority.
RECEIVER_LOCAL_DATA_TIMEOUT_S="${FOCUS_WSJ_RECEIVER_LOCAL_DATA_TIMEOUT_S:-5.0}"
# Keep the five-second physical gate unchanged.  If only the high-rate
# odometry publisher pauses, retain the immutable leg at zero velocity for a
# separately bounded window so it can use TinyNav's existing stopped router
# recovery instead of turning one transient publication gap into an episode
# failure.
RECEIVER_ODOMETRY_RECOVERY_GRACE_S="${FOCUS_WSJ_RECEIVER_ODOMETRY_RECOVERY_GRACE_S:-7.0}"
# Keep the cross-robot occupancy-liveness contract identical. After this wall
# deadline, both the receiver's 20 Hz gate and the TinyNav router may use the
# exact cached grid only until base displacement reaches
# MAX_CACHED_MAP_MOTION_M. A missing pose anchor fails closed. The separate
# recovery window applies only after that spatial bound closes the gate.
RECEIVER_OCCUPANCY_TIMEOUT_S="${FOCUS_WSJ_RECEIVER_OCCUPANCY_TIMEOUT_S:-5.0}"
RECEIVER_OCCUPANCY_RECOVERY_GRACE_S="${FOCUS_WSJ_RECEIVER_OCCUPANCY_RECOVERY_GRACE_S:-7.0}"
# A delayed Hub heartbeat still closes physical output after the receiver's
# unchanged 1.5 s delivery gate.  Keep an existing immutable leg stopped for a
# further bounded interval so one HTTP timing gap is not an episode verdict.
HEARTBEAT_DELIVERY_RECOVERY_GRACE_S="${FOCUS_WSJ_HEARTBEAT_DELIVERY_RECOVERY_GRACE_S:-3.0}"
# A collision report still zeros guarded velocity immediately.  The shared
# local occupancy/planner contract recovered from a live collision state after
# roughly six seconds on Robot 1; use the same bounded seven-second verdict on
# both platforms before the Hub permanently rejects the immutable leg.
PLANNER_COLLISION_REJECTION_S="${FOCUS_WSJ_PLANNER_COLLISION_REJECTION_S:-7.0}"
# The guarded velocity output is zeroed after one second without a fresh path.
# Physical legs observed a 1.900-1.921 s first-path delay and later 1.016 s and
# 3.365 s planner publication gaps while the router was still producing an
# online path. Keep the zero-output gate at one second, but use twelve-second
# terminal verdicts so a stopped local recovery is not misreported as an
# immediate leg failure. A router-owned bounded map-maturation wait takes
# precedence over both trajectory verdicts.
TRAJECTORY_START_GRACE_S="${FOCUS_WSJ_TRAJECTORY_START_GRACE_S:-12.0}"
TRAJECTORY_STALE_TIMEOUT_S="${FOCUS_WSJ_TRAJECTORY_STALE_TIMEOUT_S:-1.0}"
TRAJECTORY_RECOVERY_TIMEOUT_S="${FOCUS_WSJ_TRAJECTORY_RECOVERY_TIMEOUT_S:-12.0}"
NO_PROGRESS_TIMEOUT_S="${FOCUS_WSJ_NO_PROGRESS_TIMEOUT_S:-20.0}"
MINIMUM_GOAL_PROGRESS_M="${FOCUS_WSJ_MINIMUM_GOAL_PROGRESS_M:-0.05}"
# /slam/data is optimizer diagnostics rather than the controller's odometry
# input.  Its observed interval can also exceed 2 s under live perception load.
SLAM_DATA_TIMEOUT_S="${FOCUS_WSJ_SLAM_DATA_TIMEOUT_S:-3.0}"
# The Go2 half-width plus the existing local safety margin is approximately
# 0.20 m. The graph must not reduce that to a one-cell point route.
REACHABILITY_CLEARANCE_M="${FOCUS_WSJ_REACHABILITY_CLEARANCE_M:-0.20}"
# A seed outside the measured 0.35 m current footprint is a virtual pose jump,
# not a verified escape path. Keep the Hub, startup verifier and router equal.
START_SNAP_RADIUS_M="${FOCUS_WSJ_START_SNAP_RADIUS_M:-0.35}"
START_FOOTPRINT_OVERRIDE_M="${FOCUS_WSJ_START_FOOTPRINT_OVERRIDE_M:-0.35}"
# The source semantic mask keeps its radius-10-cell approach region unchanged.
# The selected semantic approach point is already on the source radius-10-cell
# (0.50 m) dilation around the raw object mask. Keep only the source FMM's
# three-cell (0.15 m) terminal tolerance around that point; a second 0.50 m
# tolerance can stop the base about 1 m before the semantic region.
SEMANTIC_ARRIVAL_RADIUS_M="${FOCUS_WSJ_SEMANTIC_ARRIVAL_RADIUS_M:-0.15}"
# Plan inside the terminal radius so boundary cells cannot chatter.
SEMANTIC_TERMINAL_PLANNING_MARGIN_M="${FOCUS_WSJ_SEMANTIC_TERMINAL_PLANNING_MARGIN_M:-0.15}"
MAX_PLAN_EXPANSIONS="${FOCUS_TINYNAV_MAX_PLAN_EXPANSIONS:-20000}"
MAX_PLAN_DURATION_S="${FOCUS_TINYNAV_MAX_PLAN_DURATION_S:-0.50}"
mode="debug"
confirmation=""
reuse_verified_debug_core="false"
startup_complete="false"
online_stack_started="false"

fail_closed_on_error() {
  local rc=$?
  if [[ "$rc" -ne 0 && "$mode" == live \
        && "$startup_complete" != true ]]; then
    set +u
    source "$SETUP_FILE" >/dev/null 2>&1 || true
    set -u
    timeout 5 ros2 topic pub --once \
      /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' \
      >/dev/null 2>&1 || true
    tmux kill-window -t "$SESSION:go2-bridge" >/dev/null 2>&1 || true
    tmux kill-window -t "$SESSION:v2-receiver" >/dev/null 2>&1 || true
  fi
  return "$rc"
}
trap fail_closed_on_error EXIT

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) mode="$2"; shift 2 ;;
    --operator-confirmation) confirmation="$2"; shift 2 ;;
    --reuse-verified-debug-core)
      reuse_verified_debug_core="true"
      shift
      ;;
    --session) SESSION="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: $0 --mode debug|live [--operator-confirmation OPERATOR_PRESENT_AND_WSJ_CLEAR] [--reuse-verified-debug-core]"
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ "$mode" == debug || "$mode" == live ]] || {
  echo "--mode must be debug or live." >&2
  exit 2
}
if [[ "$mode" == live && "$confirmation" != OPERATOR_PRESENT_AND_WSJ_CLEAR ]]; then
  echo "Live WSJ mode requires OPERATOR_PRESENT_AND_WSJ_CLEAR." >&2
  exit 2
fi
if [[ "$reuse_verified_debug_core" == true && "$mode" != live ]]; then
  echo "--reuse-verified-debug-core is valid only for live mode." >&2
  exit 2
fi
[[ "$MAX_PLAN_EXPANSIONS" =~ ^[1-9][0-9]*$ ]] || {
  echo "FOCUS_TINYNAV_MAX_PLAN_EXPANSIONS must be a positive integer." >&2
  exit 2
}
[[ "$TRANSFORM_VERSION" =~ ^[A-Za-z0-9_.-]+$ ]] || {
  echo "FOCUS_WSJ_TRANSFORM_VERSION must be explicit and filesystem-safe." >&2
  exit 2
}
[[ "$CALIBRATION_ID" =~ ^[A-Za-z0-9_.-]+$ ]] || {
  echo "FOCUS_SHARED_CALIBRATION_ID must be explicit and filesystem-safe." >&2
  exit 2
}
[[ "$DEPLOYMENT_COMMIT" =~ ^[0-9a-f]{40}$ ]] || {
  echo "FOCUS_DEPLOYMENT_COMMIT must be the explicit 40-character Git commit." >&2
  exit 2
}
[[ "$CALIBRATION_FILE" = /* ]] || {
  echo "FOCUS_SHARED_CALIBRATION_FILE must be an explicit absolute path." >&2
  exit 2
}
[[ "$HUB_URL" =~ ^http://127\.0\.0\.1:[0-9]+$ ]] || {
  echo "FOCUS_HUB_BASE_URL must remain loopback-only." >&2
  exit 2
}
[[ "$FASTDDS_BUILTIN_TRANSPORTS_VALUE" == UDPv4 ]] || {
  echo "WSJ overlay transport must be the verified UDPv4 profile." >&2
  exit 2
}
export FASTDDS_BUILTIN_TRANSPORTS="$FASTDDS_BUILTIN_TRANSPORTS_VALUE"
for required in \
  "$SCRIPT_DIR/start_wsj_command_observation.sh" \
  "$SCRIPT_DIR/start_go2_buildmap.sh" \
  "$SCRIPT_DIR/start_tinynav_buildmap_online_nav.sh" \
  "$SCRIPT_DIR/wsj_perception_entry.py" \
  "$SCRIPT_DIR/tinynav_source_contract.py" \
  "$SCRIPT_DIR/run_yunji_tinynav_planner.py" \
  "$SCRIPT_DIR/yunji_tinynav_cmd_vel_control.py" \
  "$SCRIPT_DIR/verify_tinynav_data_plane.py" \
  "$SCRIPT_DIR/v2_wsj_receiver.py" \
  "$CALIBRATION_FILE" \
  "$BASE_CAMERA_CALIBRATION_FILE" \
  "$TOKEN_FILE" \
  "$GO2_BRIDGE_SOURCE" \
  "$GO2_BRIDGE_RUNNER"; do
  [[ -r "$required" ]] || { echo "Missing required file: $required" >&2; exit 1; }
done

verify_file_contract() {
  local path="$1" expected_size="$2" expected_sha256="$3"
  local actual_size actual_sha256
  actual_size="$(stat -c %s "$path")"
  actual_sha256="$(sha256sum "$path" | awk '{print $1}')"
  [[ "$actual_size" == "$expected_size" \
      && "$actual_sha256" == "$expected_sha256" ]] || {
    echo "Runtime dependency contract mismatch: $path" >&2
    echo "expected size=$expected_size sha256=$expected_sha256" >&2
    echo "actual   size=$actual_size sha256=$actual_sha256" >&2
    return 1
  }
}

verify_file_contract \
  "$GO2_BRIDGE_SOURCE" "$GO2_BRIDGE_SOURCE_SIZE" \
  "$GO2_BRIDGE_SOURCE_SHA256"
verify_file_contract \
  "$GO2_BRIDGE_RUNNER" "$GO2_BRIDGE_RUNNER_SIZE" \
  "$GO2_BRIDGE_RUNNER_SHA256"
BASE_CAMERA_CALIBRATION_SHA256="$(
  sha256sum "$BASE_CAMERA_CALIBRATION_FILE" | awk '{print $1}'
)"

verify_patched_perception() {
  local actual_commit actual_sha perception_pid perception_cwd pane_start
  [[ -f "$PATCHED_ROOT/tinynav/core/perception_node.py" ]] || {
    echo "Missing live-tested TinyNav perception tree: $PATCHED_ROOT" >&2
    return 1
  }
  actual_commit="$(git -C "$PATCHED_ROOT" rev-parse HEAD 2>/dev/null || true)"
  [[ "$actual_commit" == "$PATCHED_COMMIT" ]] || {
    echo "TinyNav perception commit mismatch: $actual_commit" >&2
    return 1
  }
  [[ -z "$(git -C "$PATCHED_ROOT" status --porcelain 2>/dev/null)" ]] || {
    echo "Live-tested TinyNav perception worktree is dirty." >&2
    return 1
  }
  actual_sha="$(sha256sum "$PATCHED_ROOT/tinynav/core/perception_node.py" | awk '{print $1}')"
  [[ "$actual_sha" == "$PATCHED_PERCEPTION_SHA256" ]] || {
    echo "TinyNav perception file hash mismatch: $actual_sha" >&2
    return 1
  }
  pane_start="$(tmux display-message -p -t "$SESSION:perception" '#{pane_start_command}' 2>/dev/null || true)"
  [[ "$pane_start" == *"$PATCHED_ROOT"* \
     && "$pane_start" == *"$SCRIPT_DIR/wsj_perception_entry.py"* ]] || {
    echo "Refusing stale perception window; it is not the bounded deployment entry point." >&2
    return 1
  }
  perception_pid="$(
    pgrep -f "$PYTHON_BIN -u $SCRIPT_DIR/wsj_perception_entry.py" 2>/dev/null \
      | head -n 1
  )"
  [[ -n "$perception_pid" ]] || {
    echo "Bounded TinyNav perception process is not running." >&2
    return 1
  }
  perception_cwd="$(readlink -f "/proc/$perception_pid/cwd" 2>/dev/null || true)"
  [[ "$perception_cwd" == "$PATCHED_ROOT" ]] || {
    echo "TinyNav perception process cwd mismatch: $perception_cwd" >&2
    return 1
  }
}

verify_patched_perception

set +u
source "$SETUP_FILE"
set -u

required_windows=(maploc online-map planning goal-router control)
missing_windows=()
for window in "${required_windows[@]}"; do
  tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -qx "$window" \
    || missing_windows+=("$window")
done

verified_online_window() {
  local window="$1" pane_start
  pane_start="$(
    tmux display-message -p -t "$SESSION:$window" \
      '#{pane_start_command}' 2>/dev/null || true
  )"
  case "$window" in
    maploc)
      [[ "$pane_start" == *"$SCRIPT_DIR/run_tinynav_buildmap_live.py"* ]]
      ;;
    online-map)
      [[ "$pane_start" == \
        *"$SCRIPT_DIR/run_tinynav_buildmap_online_mapping.py"* ]]
      ;;
    planning)
      [[ "$pane_start" == *"run_yunji_tinynav_planner.py"* \
         || "$pane_start" == *"planning_node.py"* ]]
      ;;
    goal-router)
      [[ "$pane_start" == *"$SCRIPT_DIR/tinynav_buildmap_goal_router.py"* ]]
      ;;
    control)
      [[ "$pane_start" == *"yunji_tinynav_cmd_vel_control.py"* \
         || "$pane_start" == *"cmd_vel_control.py"* ]]
      ;;
    *)
      return 1
      ;;
  esac
}

rebuild_verified_partial_online_stack() {
  local window deadline
  if tmux list-windows -t "$SESSION" -F '#{window_name}' \
      | grep -qx go2-bridge \
     || pgrep -af 'go2_cmd_bridge|nav2_controller' >/dev/null 2>&1 \
     || pgrep -af 'v2_wsj_receiver\.py.*--enable-live-go2-motion' \
        >/dev/null 2>&1; then
    echo "Refusing partial-stack rebuild while a live command path exists." >&2
    return 1
  fi
  for window in "${required_windows[@]}"; do
    tmux list-windows -t "$SESSION" -F '#{window_name}' \
      | grep -qx "$window" || continue
    verified_online_window "$window" || {
      echo "Refusing unrecognized partial-stack window: $SESSION:$window" >&2
      return 1
    }
  done

  # The bridge and live receiver were proven absent above.  Latch pause/zero,
  # remove only the verified session-local planning windows, and reconstruct a
  # complete graph.  This turns an interrupted debug/live cleanup into an
  # idempotent next launch instead of requiring operator-side tmux surgery.
  timeout 5 ros2 topic pub --once \
    /nav/paused std_msgs/msg/Bool '{data: true}' \
    >/dev/null 2>&1 || true
  timeout 5 ros2 topic pub --once \
    /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' \
    >/dev/null 2>&1 || true
  echo "Rebuilding verified partial online stack; missing: ${missing_windows[*]}"
  for window in "${required_windows[@]}"; do
    tmux kill-window -t "$SESSION:$window" >/dev/null 2>&1 || true
  done
  deadline=$((SECONDS + 20))
  while pgrep -af \
      'planning_node.py|run_yunji_tinynav_planner.py|cmd_vel_control.py|tinynav_buildmap_goal_router.py|run_tinynav_buildmap_online_mapping.py|run_tinynav_buildmap_live.py' \
      >/dev/null 2>&1; do
    (( SECONDS < deadline )) || {
      echo "A verified partial-stack process survived its tmux window." >&2
      return 1
    }
    sleep 1
  done
  bash "$SCRIPT_DIR/start_tinynav_buildmap_online_nav.sh" --session "$SESSION"
  online_stack_started="true"
}

if [[ ${#missing_windows[@]} -eq 1 \
      && "${missing_windows[0]}" == "maploc" ]]; then
  bash "$SCRIPT_DIR/start_go2_buildmap.sh" \
    --session "$SESSION" \
    --repair-online-stack
elif [[ ${#missing_windows[@]} -eq ${#required_windows[@]} ]]; then
  bash "$SCRIPT_DIR/start_tinynav_buildmap_online_nav.sh" --session "$SESSION"
  online_stack_started="true"
elif [[ ${#missing_windows[@]} -ne 0 ]]; then
  rebuild_verified_partial_online_stack
fi

# Keep the formal DDS subscriber alive before any bounded publisher recovery.
# Parking only removes its Hub upload contract; it cannot publish a target or
# velocity.  The checked calibration contract is hot-loaded after the complete
# sensor/map graph has passed.
bash "$SCRIPT_DIR/start_wsj_command_observation.sh" \
  --park-only \
  --session "$SESSION"

component_contract_sha256() {
  local command="$1"
  printf '%s\0%s\0%s\0' \
    "$DEPLOYMENT_COMMIT" "$command" \
    "$BASE_CAMERA_CALIBRATION_SHA256" \
    | sha256sum | awk '{print $1}'
}

online_map_contract="$(
  printf '%s\0%s\0%s\0' \
    "continuous-depth-online-map-v1" \
    "$(sha256sum \
      "$SCRIPT_DIR/run_tinynav_buildmap_online_mapping.py" \
      "$SCRIPT_DIR/navigation_occupancy_mapper.py" \
      "$SCRIPT_DIR/../src/focus_hub/navigation_occupancy.py" \
      "$SCRIPT_DIR/../src/focus_hub/rate_aware_keyframes.py" \
      | awk '{print $1}')" \
    "$(sha256sum \
      "$SCRIPT_DIR/ros_continuous_depth_geometry_rgb.py" \
      | awk '{print $1}')" \
    | sha256sum \
    | awk '{print $1}'
)"

mark_component_contract() {
  local window="$1" command="$2"
  tmux set-option -w -t "$SESSION:$window" \
    @focus_deployment_commit "$DEPLOYMENT_COMMIT"
  tmux set-option -w -t "$SESSION:$window" \
    @focus_component_contract_sha256 \
    "$(component_contract_sha256 "$command")"
}

component_contract_matches() {
  local window="$1" command="$2" required_text="$3"
  local pane_dead pane_start deployment contract
  pane_dead="$(
    tmux display-message -p -t "$SESSION:$window" '#{pane_dead}' \
      2>/dev/null || true
  )"
  pane_start="$(
    tmux display-message -p -t "$SESSION:$window" '#{pane_start_command}' \
      2>/dev/null || true
  )"
  deployment="$(
    tmux show-options -w -v -t "$SESSION:$window" \
      @focus_deployment_commit 2>/dev/null || true
  )"
  contract="$(
    tmux show-options -w -v -t "$SESSION:$window" \
      @focus_component_contract_sha256 2>/dev/null || true
  )"
  [[ "$pane_dead" == 0 \
     && "$pane_start" == *"$required_text"* \
     && "$deployment" == "$DEPLOYMENT_COMMIT" \
     && "$contract" == "$(component_contract_sha256 "$command")" ]]
}

sensor_map_verifier=(
  "$PYTHON_BIN" -u "$SCRIPT_DIR/verify_tinynav_data_plane.py"
  --robot-id robot-0
  --mode "$mode"
  --sensor-map-only
  --frame-id world
  --camera-frame camera
  --odom-topic /slam/odometry_visual
  --fresh-camera-info-topic /camera/camera/color/camera_info
  --fresh-camera-info-topic /slam/camera_info
  --camera-info-topic /slam/camera_info
  --geometry-width 848
  --geometry-height 480
  --max-occupancy-age-s 12
  --max-cached-occupancy-motion-m "$MAX_CACHED_MAP_MOTION_M"
  --minimum-occupancy-updates 2
  --maximum-occupancy-update-interval-s 4.0
  --require-reachable-start
  --base-camera-calibration-file "$BASE_CAMERA_CALIBRATION_FILE"
  --reachability-clearance-m "$REACHABILITY_CLEARANCE_M"
  --start-snap-radius-m "$START_SNAP_RADIUS_M"
  --start-footprint-override-m "$START_FOOTPRINT_OVERRIDE_M"
)

recover_online_map_publisher() {
  local pane_start old_pid new_pid deadline graph
  if tmux list-windows -t "$SESSION" -F '#{window_name}' \
      | grep -qx go2-bridge \
     || pgrep -af 'v2_wsj_receiver\.py.*--enable-live-go2-motion' \
        >/dev/null 2>&1; then
    echo "Refusing online-map recovery while a live command path exists." >&2
    return 1
  fi
  pane_start="$(
    tmux display-message -p -t "$SESSION:online-map" \
      '#{pane_start_command}' 2>/dev/null || true
  )"
  [[ "$pane_start" == *"$SCRIPT_DIR/run_tinynav_buildmap_online_mapping.py"* ]] || {
    echo "Refusing to restart an unrecognized online-map publisher." >&2
    return 1
  }

  # Fast DDS was observed to retain an alive publisher process while new
  # subscribers saw no /semantic_mapping/occupancy_bev endpoint. Restart only
  # this non-actuating publisher, after every subscriber already exists. The
  # pause/zero witnesses and absent Go2 bridge keep the chassis path closed.
  timeout 5 ros2 topic pub --once \
    /nav/paused std_msgs/msg/Bool '{data: true}' \
    >/dev/null 2>&1 || true
  timeout 5 ros2 topic pub --once \
    /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' \
    >/dev/null 2>&1 || true
  old_pid="$(
    tmux display-message -p -t "$SESSION:online-map" '#{pane_pid}'
  )"
  tmux set-option -w -t "$SESSION:online-map" remain-on-exit on
  tmux send-keys -t "$SESSION:online-map" C-c
  deadline=$((SECONDS + 20))
  until [[ "$(tmux display-message -p -t "$SESSION:online-map" \
      '#{pane_dead}' 2>/dev/null || true)" == 1 ]]; do
    (( SECONDS < deadline )) || {
      echo "WSJ online-map publisher did not stop cleanly." >&2
      return 1
    }
    sleep 1
  done

  deadline=$((SECONDS + 20))
  while graph="$(
      timeout 5 ros2 topic info /semantic_mapping/occupancy_bev -v \
        2>/dev/null || true
    )" \
    && grep -q 'Endpoint type: PUBLISHER' <<<"$graph"; do
    (( SECONDS < deadline )) || {
      echo "Old WSJ occupancy publisher remained in DDS after shutdown." >&2
      return 1
    }
    sleep 1
  done

  tmux respawn-pane -t "$SESSION:online-map"
  tmux set-option -w -t "$SESSION:online-map" remain-on-exit off
  new_pid="$(
    tmux display-message -p -t "$SESSION:online-map" '#{pane_pid}'
  )"
  [[ -n "$new_pid" && "$new_pid" != "$old_pid" ]] || {
    echo "WSJ online-map publisher did not restart in a new process." >&2
    return 1
  }
  deadline=$((SECONDS + 30))
  until graph="$(
      timeout 5 ros2 topic info /semantic_mapping/occupancy_bev -v \
        2>/dev/null || true
    )" \
    && grep -Eq '^Publisher count: 1[[:space:]]*$' <<<"$graph" \
    && grep -Eq '^Node name: occupancy_mapper_node[[:space:]]*$' \
      <<<"$graph" \
    && ! grep -q '_NODE_.*_UNKNOWN_' <<<"$graph"; do
    if [[ "$(tmux display-message -p -t "$SESSION:online-map" \
        '#{pane_dead}')" == 1 ]]; then
      tmux capture-pane -pt "$SESSION:online-map" -S -100 >&2 || true
      return 1
    fi
    (( SECONDS < deadline )) || {
      echo "Timed out waiting for the restarted WSJ occupancy publisher." >&2
      printf '%s\n' "$graph" >&2
      return 1
    }
    sleep 1
  done
  echo "WSJ online-map publisher recovered publisher-last: $new_pid"
  mark_component_contract online-map "$online_map_contract"
}

# A persistent BuildMap session can outlive a deployment.  Bind the
# non-actuating online-map process to the same checked release as the
# planner/controller instead of accepting whichever Python bytes happened to
# be loaded when the session was first created.
if [[ "$online_stack_started" == true ]]; then
  mark_component_contract online-map "$online_map_contract"
elif ! component_contract_matches \
    online-map "$online_map_contract" \
    "run_tinynav_buildmap_online_mapping.py"; then
  if [[ "$reuse_verified_debug_core" == true ]]; then
    echo "WSJ warm online-map does not match the verified deployment contract." >&2
    exit 1
  fi
  echo "WSJ online-map contract changed; reloading its non-actuating publisher."
  recover_online_map_publisher
fi

# This single verifier replaces the former three serial `ros2 topic echo`
# witnesses. It subscribes to those same two CameraInfo streams and visual
# odometry together, then also validates geometry, occupancy and router state.
# No full-resolution Image subscriber is created, so the long-lived Fast DDS
# visual pipeline retains its measured startup behavior. A short failed probe
# triggers one bounded publisher-last recovery; the full check remains
# authoritative and still fails closed on every other sensor/map defect.
if ! "${sensor_map_verifier[@]}" --timeout-s 8; then
  echo "WSJ sensor/map fast probe failed; attempting one publisher-last recovery." >&2
  recover_online_map_publisher
fi
"${sensor_map_verifier[@]}" --timeout-s 35

bash "$SCRIPT_DIR/start_wsj_command_observation.sh" \
  --session "$SESSION" \
  --shared-tracking-calibration "$CALIBRATION_FILE" \
  --shared-frame-calibration-id "$CALIBRATION_ID" \
  --transform-version "$TRANSFORM_VERSION" \
  --hub-url "$HUB_URL"

planning_command="bash -lc 'source \"$SETUP_FILE\"; cd \"$TINYNAV_ROOT\"; uv run python \"$SCRIPT_DIR/run_yunji_tinynav_planner.py\" --robot-profile source-default'"
control_command="bash -lc 'source \"$SETUP_FILE\"; cd \"$TINYNAV_ROOT\"; uv run python \"$SCRIPT_DIR/yunji_tinynav_cmd_vel_control.py\" --robot-profile source-default --robot-id robot-0 --base-camera-frame camera --base-camera-calibration-file \"$BASE_CAMERA_CALIBRATION_FILE\" --verified-forward-only-planner --rotate-first-on-reverse --stabilize-large-turn --linear-command-floor-mps \"$LINEAR_COMMAND_FLOOR_MPS\" --rotate-first-max-angular-radps 0.35 --rotate-first-timeout-s 12.0'"

if [[ "$reuse_verified_debug_core" == true ]]; then
  component_contract_matches \
    planning "$planning_command" "run_yunji_tinynav_planner.py" || {
    echo "WSJ warm planner does not match the verified deployment contract." >&2
    exit 1
  }
  component_contract_matches \
    control "$control_command" "yunji_tinynav_cmd_vel_control.py" || {
    echo "WSJ warm controller does not match the verified deployment contract." >&2
    exit 1
  }
  echo "Reusing verified WSJ planner/controller without DDS participant churn."
else
# The persistent source planner includes a fixed reverse vocabulary. Replace
# it with the forward-only deployment wrapper while the chassis bridge is
# absent and navigation is paused; the controller can then resolve a bounded
# behind-heading segment by yawing in place before moving forward.
timeout 5 ros2 topic pub --once \
  /nav/paused std_msgs/msg/Bool '{data: true}' \
  >/dev/null 2>&1 || true
old_planning_pid="$(
  tmux display-message -p -t "$SESSION:planning" '#{pane_pid}'
)"
tmux set-option -w -t "$SESSION:planning" remain-on-exit on
tmux send-keys -t "$SESSION:planning" C-c
deadline=$((SECONDS + 15))
until [[ "$(tmux display-message -p -t "$SESSION:planning" \
    '#{pane_dead}' 2>/dev/null || true)" == 1 ]]; do
  (( SECONDS < deadline )) || {
    echo "WSJ planner did not stop cleanly; removing its isolated pane." >&2
    tmux kill-window -t "$SESSION:planning" >/dev/null 2>&1 || true
    break
  }
  sleep 1
done

deadline=$((SECONDS + 30))
while timeout 5 ros2 topic info /planning/trajectory_path -v 2>/dev/null \
    | grep -q 'Endpoint type: PUBLISHER'; do
  (( SECONDS < deadline )) || {
    echo "Timed out waiting for the old WSJ planner publisher to leave DDS." >&2
    exit 1
  }
  sleep 1
done

if tmux display-message -p -t "$SESSION:planning" >/dev/null 2>&1; then
  tmux respawn-pane -t "$SESSION:planning" "$planning_command"
  tmux set-option -w -t "$SESSION:planning" remain-on-exit off
else
  tmux new-window -d -t "$SESSION" -n planning "$planning_command"
fi
new_planning_pid="$(
  tmux display-message -p -t "$SESSION:planning" '#{pane_pid}'
)"
planning_start="$(
  tmux display-message -p -t "$SESSION:planning" '#{pane_start_command}'
)"
[[ -n "$new_planning_pid" \
   && "$new_planning_pid" != "$old_planning_pid" \
   && "$planning_start" == *"run_yunji_tinynav_planner.py"* \
   && "$planning_start" == *"--robot-profile source-default"* ]] || {
  echo "WSJ planner did not reload from the forward-only wrapper." >&2
  exit 1
}
deadline=$((SECONDS + 30))
until planner_graph="$(
    timeout 5 ros2 topic info /planning/trajectory_path -v 2>/dev/null || true
  )" \
  && grep -Eq '^Publisher count: 1[[:space:]]*$' <<<"$planner_graph" \
  && grep -Eq '^Node name: planning_node[[:space:]]*$' <<<"$planner_graph" \
  && ! grep -q '_NODE_.*_UNKNOWN_' <<<"$planner_graph"; do
  if [[ "$(tmux display-message -p -t "$SESSION:planning" \
      '#{pane_dead}')" == 1 ]]; then
    tmux capture-pane -pt "$SESSION:planning" -S -100 >&2 || true
    exit 1
  fi
  (( SECONDS < deadline )) || {
    echo "Timed out waiting for the forward-only WSJ planner publisher." >&2
    printf '%s\n' "$planner_graph" >&2
    exit 1
  }
  sleep 1
done
mark_component_contract planning "$planning_command"
echo "WSJ forward-only planner reloaded from the current deployment: $new_planning_pid"

# The BuildMap core persists between episodes, so its controller process may
# predate the checked deployment. Pause first, then replace only the raw
# /cmd_vel controller before any receiver or chassis bridge can be created.
# The latched pause makes the new process publish zero until the v2 receiver
# explicitly authorizes a fresh trajectory in live mode. Let rclpy shut down
# cleanly before respawning the pane: an abrupt tmux respawn was observed to
# leave a Fast DDS publisher whose participant identity stayed UNKNOWN, which
# the exclusive-route verifier must reject.
timeout 5 ros2 topic pub --once \
  /nav/paused std_msgs/msg/Bool '{data: true}' \
  >/dev/null 2>&1 || true
old_control_pid="$(
  tmux display-message -p -t "$SESSION:control" '#{pane_pid}'
)"
tmux set-option -w -t "$SESSION:control" remain-on-exit on
tmux send-keys -t "$SESSION:control" C-c
deadline=$((SECONDS + 15))
until [[ "$(tmux display-message -p -t "$SESSION:control" \
    '#{pane_dead}' 2>/dev/null || true)" == 1 ]]; do
  (( SECONDS < deadline )) || {
    echo "WSJ controller did not stop cleanly; removing its isolated pane." >&2
    tmux kill-window -t "$SESSION:control" >/dev/null 2>&1 || true
    break
  }
  sleep 1
done

# Do not let a stale, still-discovered publisher satisfy the new-process
# readiness check. No chassis bridge exists at this point, and the pause above
# remains latched throughout this bounded gap.
deadline=$((SECONDS + 30))
while timeout 5 ros2 topic info /cmd_vel -v 2>/dev/null \
    | grep -q 'Endpoint type: PUBLISHER'; do
  (( SECONDS < deadline )) || {
    echo "Timed out waiting for the old WSJ controller publisher to leave DDS." >&2
    exit 1
  }
  sleep 1
done

if tmux display-message -p -t "$SESSION:control" >/dev/null 2>&1; then
  tmux respawn-pane -t "$SESSION:control" "$control_command"
  tmux set-option -w -t "$SESSION:control" remain-on-exit off
else
  tmux new-window -d -t "$SESSION" -n control "$control_command"
fi
new_control_pid="$(
  tmux display-message -p -t "$SESSION:control" '#{pane_pid}'
)"
control_start="$(
  tmux display-message -p -t "$SESSION:control" '#{pane_start_command}'
)"
[[ -n "$new_control_pid" \
   && "$new_control_pid" != "$old_control_pid" \
   && "$control_start" == *"yunji_tinynav_cmd_vel_control.py"* \
   && "$control_start" == *"--robot-profile source-default"* ]] || {
  echo "WSJ velocity controller did not reload from the deployment wrapper." >&2
  exit 1
}
deadline=$((SECONDS + 30))
until controller_graph="$(
    timeout 5 ros2 topic info /cmd_vel -v 2>/dev/null || true
  )" \
  && grep -Eq '^Publisher count: 1[[:space:]]*$' <<<"$controller_graph" \
  && grep -Eq '^Node name: cmd_vel_control_node[[:space:]]*$' \
    <<<"$controller_graph" \
  && ! grep -q '_NODE_.*_UNKNOWN_' <<<"$controller_graph"; do
  if [[ "$(tmux display-message -p -t "$SESSION:control" '#{pane_dead}')" == 1 ]]; then
    tmux capture-pane -pt "$SESSION:control" -S -100 >&2 || true
    exit 1
  fi
  (( SECONDS < deadline )) || {
    echo "Timed out waiting for the named WSJ controller publisher." >&2
    printf '%s\n' "$controller_graph" >&2
    exit 1
  }
  sleep 1
done
mark_component_contract control "$control_command"
echo "WSJ velocity controller reloaded from the current deployment: $new_control_pid"
fi

if tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -qx v2-receiver; then
  if [[ "$mode" == debug ]] \
     && ! tmux list-windows -t "$SESSION" -F '#{window_name}' \
        | grep -qx go2-bridge \
     && pgrep -af 'v2_wsj_receiver\.py' >/dev/null 2>&1 \
     && ! pgrep -af 'v2_wsj_receiver\.py.*--enable-live-go2-motion' \
        >/dev/null 2>&1; then
    source "$SETUP_FILE"
    "$PYTHON_BIN" -u "$SCRIPT_DIR/verify_tinynav_data_plane.py" \
      --robot-id robot-0 \
      --mode debug \
      --command-graph-only \
      --camera-frame camera \
      --timeout-s 20
    echo "WSJ v2 BuildMap stack is already ready: mode=debug"
    echo "Safety: no Go2 bridge; physical motion is impossible through this stack."
    exit 0
  fi
  echo "Refusing to replace existing $SESSION:v2-receiver." >&2
  exit 1
fi
if pgrep -af 'v2_wsj_receiver.py' >/dev/null 2>&1; then
  echo "An untracked WSJ v2 receiver is already running." >&2
  exit 1
fi
if [[ "$mode" == debug ]] && pgrep -af 'go2_cmd_bridge' >/dev/null 2>&1; then
  echo "Debug mode refuses an active Go2 bridge." >&2
  exit 1
fi
if [[ "$mode" == live ]] \
   && ! ip -o -4 addr show dev "${UNITREE_NET_IF:-eth0}" >/dev/null 2>&1; then
  echo "Go2 interface ${UNITREE_NET_IF:-eth0} has no IPv4 address." >&2
  exit 1
fi

goal_router_command="bash -lc 'source \"$SETUP_FILE\"; export PYTHONPATH=\"$SCRIPT_DIR/../src\":\${PYTHONPATH:-}; \"$PYTHON_BIN\" -u \"$SCRIPT_DIR/tinynav_buildmap_goal_router.py\" --frame-id world --occupancy-topic /semantic_mapping/occupancy_bev --base-camera-calibration-file \"$BASE_CAMERA_CALIBRATION_FILE\" --clearance-m \"$REACHABILITY_CLEARANCE_M\" --semantic-terminal-planning-margin-m \"$SEMANTIC_TERMINAL_PLANNING_MARGIN_M\" --start-snap-radius-m \"$START_SNAP_RADIUS_M\" --start-footprint-override-m \"$START_FOOTPRINT_OVERRIDE_M\" --input-timeout-s \"$ODOMETRY_INPUT_TIMEOUT_S\" --map-timeout-s \"$MAP_TIMEOUT_S\" --max-cached-map-motion-m \"$MAX_CACHED_MAP_MOTION_M\" --max-plan-expansions \"$MAX_PLAN_EXPANSIONS\" --max-plan-duration-s \"$MAX_PLAN_DURATION_S\"'"
if [[ "$reuse_verified_debug_core" == true ]]; then
  component_contract_matches \
    goal-router "$goal_router_command" "tinynav_buildmap_goal_router.py" || {
    echo "WSJ warm goal router does not match the verified deployment contract." >&2
    exit 1
  }
  echo "Reusing verified WSJ goal router without a process restart."
else
  # A BuildMap session intentionally survives between deployments. Replace the
  # router only when establishing a new verified core; a debug->live mode
  # switch reuses the exact marked process and avoids needless DDS churn.
  old_goal_router_pid="$(
    tmux display-message -p -t "$SESSION:goal-router" '#{pane_pid}'
  )"
  tmux respawn-pane -k -t "$SESSION:goal-router" "$goal_router_command"
  new_goal_router_pid="$(
    tmux display-message -p -t "$SESSION:goal-router" '#{pane_pid}'
  )"
  [[ -n "$new_goal_router_pid" \
     && "$new_goal_router_pid" != "$old_goal_router_pid" ]] || {
    echo "WSJ goal-router did not reload into a new process." >&2
    exit 1
  }
  deadline=$((SECONDS + 15))
  until ros2 node list 2>/dev/null \
    | grep -qx /focus_tinynav_buildmap_goal_router; do
    if [[ "$(tmux display-message -p -t "$SESSION:goal-router" '#{pane_dead}')" == 1 ]]; then
      tmux capture-pane -pt "$SESSION:goal-router" -S -100 >&2 || true
      exit 1
    fi
    (( SECONDS < deadline )) || {
      echo "Timed out waiting for the reloaded WSJ goal-router." >&2
      exit 1
    }
    sleep 1
  done
  mark_component_contract goal-router "$goal_router_command"
  echo "WSJ goal-router reloaded from the current deployment: $new_goal_router_pid"
fi

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
state_dir="${FOCUS_ROBOT_STATE_DIR:-${XDG_STATE_HOME:-$HOME/.local/state}/topofocus}"
mkdir -p "$state_dir"
alignment="$state_dir/robot0-v2-buildmap-${mode}-${stamp}.json"
log="$state_dir/robot0-v2-buildmap-${mode}-${stamp}.jsonl"
bridge_log="$state_dir/robot0-go2-bridge-${stamp}.log"
receiver=(
  "$PYTHON_BIN" -u "$SCRIPT_DIR/v2_wsj_receiver.py"
  --base-url "$HUB_URL"
  --token-file "$TOKEN_FILE"
  --calibration-file "$CALIBRATION_FILE"
  --base-camera-calibration-file "$BASE_CAMERA_CALIBRATION_FILE"
  --transform-version "$TRANSFORM_VERSION"
  --shared-frame-calibration-id "$CALIBRATION_ID"
  --online-buildmap-world
  --tracking-frame world
  --tinynav-map-frame world
  --local-map-frame wsj/world
  --occupancy-topic /semantic_mapping/occupancy_bev
  --local-data-timeout-s "$RECEIVER_LOCAL_DATA_TIMEOUT_S"
  --odometry-recovery-grace-s "$RECEIVER_ODOMETRY_RECOVERY_GRACE_S"
  --occupancy-data-timeout-s "$RECEIVER_OCCUPANCY_TIMEOUT_S"
  --occupancy-recovery-grace-s "$RECEIVER_OCCUPANCY_RECOVERY_GRACE_S"
  --heartbeat-delivery-recovery-grace-s "$HEARTBEAT_DELIVERY_RECOVERY_GRACE_S"
  --planner-collision-rejection-s "$PLANNER_COLLISION_REJECTION_S"
  --max-cached-occupancy-motion-m "$MAX_CACHED_MAP_MOTION_M"
  --slam-data-timeout-s "$SLAM_DATA_TIMEOUT_S"
  --trajectory-start-grace-s "$TRAJECTORY_START_GRACE_S"
  --trajectory-stale-timeout-s "$TRAJECTORY_STALE_TIMEOUT_S"
  --trajectory-recovery-timeout-s "$TRAJECTORY_RECOVERY_TIMEOUT_S"
  --semantic-arrival-radius-m "$SEMANTIC_ARRIVAL_RADIUS_M"
  --no-progress-timeout-s "$NO_PROGRESS_TIMEOUT_S"
  --minimum-goal-progress-m "$MINIMUM_GOAL_PROGRESS_M"
  --reject-reverse-trajectory
  --reject-stalled-turn
  --reachability-clearance-m "$REACHABILITY_CLEARANCE_M"
  --start-snap-radius-m "$START_SNAP_RADIUS_M"
  --start-footprint-override-m "$START_FOOTPRINT_OVERRIDE_M"
  --alignment-output "$alignment"
  --log "$log"
)
if [[ "$mode" == live ]]; then
  receiver+=(
    --enable-live-go2-motion
    --operator-confirmation OPERATOR_PRESENT_AND_WSJ_CLEAR
  )
fi
printf -v receiver_text '%q ' "${receiver[@]}"
tmux new-window -d -t "$SESSION" -n v2-receiver \
  "bash -lc 'source \"$SETUP_FILE\"; export PYTHONPATH=\"$SCRIPT_DIR/../src\":\${PYTHONPATH:-}; $receiver_text'"

deadline=$((SECONDS + 40))
until [[ -s "$alignment" ]]; do
  if [[ "$(tmux display-message -p -t "$SESSION:v2-receiver" '#{pane_dead}')" == 1 ]]; then
    tmux capture-pane -pt "$SESSION:v2-receiver" -S -100 >&2 || true
    exit 1
  fi
  (( SECONDS < deadline )) || {
    echo "Timed out waiting for WSJ v2 alignment." >&2
    exit 1
  }
  sleep 1
done

# The lightweight sensor/map phase already passed independently of the Hub
# sender. Validate the complete non-actuating command route now, without
# creating an Image subscription. In live mode the chassis subscriber must
# still be absent until this check passes; the post-bridge check below then
# proves only the one endpoint that the bridge adds.
source "$SETUP_FILE"
pre_bridge_args=(--command-graph-only)
if [[ "$mode" == live ]]; then
  pre_bridge_args+=(--pre-bridge-command-check)
fi
"$PYTHON_BIN" -u "$SCRIPT_DIR/verify_tinynav_data_plane.py" \
  --robot-id robot-0 \
  --mode "$mode" \
  "${pre_bridge_args[@]}" \
  --camera-frame camera \
  --timeout-s 20

if [[ "$mode" == live ]]; then
  tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -qx go2-bridge && {
    echo "Refusing to replace existing Go2 bridge window." >&2
    exit 1
  }
  # Both remote dependencies were checked against the observed 2026-07-27
  # size/SHA-256 contracts before any ROS command path was created.
  # A confirmed guarded zero releases an active SportClient command exactly
  # once via Move(0)+StopMove. Subsequent zero messages are no-ops until a
  # fresh non-zero command reacquires control. This prevents a stale Sport
  # state from surviving a coordinated HOLD or planner retry.
  tmux new-window -d -t "$SESSION" -n go2-bridge \
    "bash -lc 'set -o pipefail; export GO2_CMD_TOPIC=/focus_guarded_cmd_vel GO2_MAX_VX=0.20 GO2_MAX_VY=0.00 GO2_MAX_WZ=0.50 GO2_MIN_CMD_V=\"$LINEAR_COMMAND_FLOOR_MPS\" GO2_MIN_CMD_W=0.30 GO2_REMOTE_PRIORITY=true GO2_SEND_ZERO_WHEN_IDLE=false GO2_LOG_COMMANDS=true GO2_LOG_INTERVAL_SEC=0.2; bash \"$GO2_BRIDGE_RUNNER\" 2>&1 | tee \"$bridge_log\"'"
  echo "WSJ Go2 bridge command log: $bridge_log"
  "$PYTHON_BIN" -u "$SCRIPT_DIR/verify_tinynav_data_plane.py" \
    --robot-id robot-0 \
    --mode live \
    --post-bridge-command-check \
    --camera-frame camera \
    --timeout-s 20
fi

startup_complete="true"
trap - EXIT
echo "WSJ v2 BuildMap stack ready: mode=$mode alignment=$alignment"
if [[ "$mode" == debug ]]; then
  echo "Safety: no Go2 bridge; physical motion is impossible through this stack."
fi
