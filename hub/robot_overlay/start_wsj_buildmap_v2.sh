#!/usr/bin/env bash
# Start the minimal WSJ BuildMap/v2 stack in debug or explicitly armed live mode.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${FOCUS_WSJ_NAV_SESSION:-tinynav_semantic_nav_auto}"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
TINYNAV_ROOT="${TINYNAV_ROOT:-/home/nvidia/twork/tinynav}"
PYTHON_BIN="${TINYNAV_PYTHON:-/home/nvidia/twork/tinynav/.venv/bin/python}"
TOKEN_FILE="${FOCUS_ROBOT_TOKEN_FILE:-/home/nvidia/focus_sender/.token}"
CALIBRATION_FILE="${FOCUS_SHARED_CALIBRATION_FILE:-}"
BASE_CAMERA_CALIBRATION_FILE="${FOCUS_WSJ_BASE_CAMERA_CALIBRATION_FILE:-/home/nvidia/.local/state/topofocus/calibration/wsj_tinynav_camera_base_20260723_operator.json}"
TRANSFORM_VERSION="${FOCUS_WSJ_TRANSFORM_VERSION:-}"
CALIBRATION_ID="${FOCUS_SHARED_CALIBRATION_ID:-}"
HUB_URL="${FOCUS_HUB_BASE_URL:-http://127.0.0.1:18089}"
PATCHED_ROOT="${TINYNAV_PERCEPTION_PATCHED_ROOT:-/home/nvidia/focus_sender/tinynav_imu_fix_worktree_20260721}"
PATCHED_COMMIT="${TINYNAV_PERCEPTION_PATCHED_COMMIT:-29f26bc058886ff450f02cdc0d6e9977e1c57010}"
PATCHED_PERCEPTION_SHA256="${TINYNAV_PERCEPTION_PATCHED_SHA256:-3a695d5210d60ea1f721549ca7458ba89e7bf32db5178cd1c312c633aef1c3b3}"
# Keep these values identical to start_tinynav_buildmap_online_nav.sh.  The
# Mapper/planner can remain alive across supervised episodes, but the overlay
# goal router and velocity wrapper must be reloaded so they cannot retain
# pre-deployment Python code.
MAX_CACHED_MAP_MOTION_M="${FOCUS_MAX_CACHED_MAP_MOTION_M:-0.25}"
MAP_TIMEOUT_S="${FOCUS_WSJ_MAP_TIMEOUT_S:-12.0}"
# The 2026-07-25 physical run observed one 2.227 s BuildMap odometry gap
# while SLAM diagnostics, occupancy, the local trajectory, and the Go2 bridge
# stayed healthy.  TinyNav's controller independently zeros velocity after
# 0.8 s without pose and the v2 receiver additionally closes its guarded
# output after 1.0 s without a fresh trajectory, so this three-second
# high-level liveness window cannot make the robot drive on stale localization.
ODOMETRY_INPUT_TIMEOUT_S="${FOCUS_WSJ_ODOMETRY_INPUT_TIMEOUT_S:-3.0}"
NO_PROGRESS_TIMEOUT_S="${FOCUS_WSJ_NO_PROGRESS_TIMEOUT_S:-20.0}"
MINIMUM_GOAL_PROGRESS_M="${FOCUS_WSJ_MINIMUM_GOAL_PROGRESS_M:-0.05}"
# /slam/data is optimizer diagnostics rather than the controller's odometry
# input.  Its observed interval can also exceed 2 s under live perception load.
SLAM_DATA_TIMEOUT_S="${FOCUS_WSJ_SLAM_DATA_TIMEOUT_S:-3.0}"
START_SNAP_RADIUS_M="${FOCUS_WSJ_START_SNAP_RADIUS_M:-0.75}"
START_FOOTPRINT_OVERRIDE_M="${FOCUS_WSJ_START_FOOTPRINT_OVERRIDE_M:-0.35}"
# The source semantic mask keeps its radius-10-cell approach region unchanged.
# On 2026-07-25 the local planner stopped producing a multi-pose trajectory at
# 0.32 m from the selected approach point, before the source-exact 0.15 m
# receiver terminal check. Use an explicit 0.50 m physical demo terminal
# radius; independent surveyed goal-region membership remains authoritative for
# reported SR/SPL.
SEMANTIC_ARRIVAL_RADIUS_M="${FOCUS_WSJ_SEMANTIC_ARRIVAL_RADIUS_M:-0.50}"
mode="debug"
confirmation=""
startup_complete="false"

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
    --session) SESSION="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: $0 --mode debug|live [--operator-confirmation OPERATOR_PRESENT_AND_WSJ_CLEAR]"
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
[[ "$TRANSFORM_VERSION" =~ ^[A-Za-z0-9_.-]+$ ]] || {
  echo "FOCUS_WSJ_TRANSFORM_VERSION must be explicit and filesystem-safe." >&2
  exit 2
}
[[ "$CALIBRATION_ID" =~ ^[A-Za-z0-9_.-]+$ ]] || {
  echo "FOCUS_SHARED_CALIBRATION_ID must be explicit and filesystem-safe." >&2
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
for required in \
  "$SCRIPT_DIR/start_wsj_command_observation.sh" \
  "$SCRIPT_DIR/start_go2_buildmap.sh" \
  "$SCRIPT_DIR/start_tinynav_buildmap_online_nav.sh" \
  "$SCRIPT_DIR/yunji_tinynav_cmd_vel_control.py" \
  "$SCRIPT_DIR/verify_tinynav_data_plane.py" \
  "$SCRIPT_DIR/v2_wsj_receiver.py" \
  "$CALIBRATION_FILE" \
  "$BASE_CAMERA_CALIBRATION_FILE" \
  "$TOKEN_FILE"; do
  [[ -r "$required" ]] || { echo "Missing required file: $required" >&2; exit 1; }
done

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
     && "$pane_start" == *"tinynav/core/perception_node.py"* ]] || {
    echo "Refusing stale perception window; it is not the live-tested patched entry point." >&2
    return 1
  }
  perception_pid="$(
    pgrep -f "$PYTHON_BIN -u tinynav/core/perception_node.py" 2>/dev/null \
      | head -n 1
  )"
  [[ -n "$perception_pid" ]] || {
    echo "Patched TinyNav perception process is not running." >&2
    return 1
  }
  perception_cwd="$(readlink -f "/proc/$perception_pid/cwd" 2>/dev/null || true)"
  [[ "$perception_cwd" == "$PATCHED_ROOT" ]] || {
    echo "TinyNav perception process cwd mismatch: $perception_cwd" >&2
    return 1
  }
}

verify_patched_perception

fresh_topic_once() {
  local topic="$1"
  # The RealSense RGB publisher is 848x480@30 Hz.  The ros2 CLI's default
  # reliable/deep subscription can overflow while deserializing full images
  # and then time out even though the publisher is healthy.  Subscribe like a
  # sensor consumer, keep only the newest sample and deserialize/print only
  # the small header used as freshness evidence.
  timeout -k 2 15 ros2 topic echo --once \
    --field header \
    --qos-reliability best_effort \
    --qos-durability volatile \
    --qos-depth 1 \
    "$topic" \
    >/dev/null 2>&1
}

set +u
source "$SETUP_FILE"
set -u
# RGB and processed depth are continuous streams and therefore prove that the
# calibrated camera/perception epoch is advancing. Keyframe topics are
# intentionally VOLATILE and event-driven; while the robot is stationary a
# newly-created subscriber can legitimately see no sample before this short
# startup deadline. The downstream strict map-freshness gate proves that a
# synchronized keyframe actually arrived, which is stronger than querying the
# ROS CLI's eventually-consistent graph cache for a publisher endpoint here.
for topic in \
  /camera/camera/color/image_raw \
  /slam/depth; do
  fresh_topic_once "$topic" || {
    echo "WSJ calibrated sensor epoch is stale at $topic." >&2
    echo "Refusing to restart camera/perception after calibration because that" \
      "would change the tracking origin; run a new board-calibration session." \
      >&2
    exit 1
  }
done

bash "$SCRIPT_DIR/start_wsj_command_observation.sh" \
  --session "$SESSION" \
  --shared-tracking-calibration "$CALIBRATION_FILE" \
  --shared-frame-calibration-id "$CALIBRATION_ID" \
  --transform-version "$TRANSFORM_VERSION" \
  --hub-url "$HUB_URL"

required_windows=(maploc online-map planning goal-router control)
missing_windows=()
for window in "${required_windows[@]}"; do
  tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -qx "$window" \
    || missing_windows+=("$window")
done
if [[ ${#missing_windows[@]} -eq 1 \
      && "${missing_windows[0]}" == "maploc" ]]; then
  bash "$SCRIPT_DIR/start_go2_buildmap.sh" \
    --session "$SESSION" \
    --repair-online-stack
elif [[ ${#missing_windows[@]} -eq ${#required_windows[@]} ]]; then
  bash "$SCRIPT_DIR/start_tinynav_buildmap_online_nav.sh" --session "$SESSION"
elif [[ ${#missing_windows[@]} -ne 0 ]]; then
  echo "Refusing ambiguous partial online stack; missing: ${missing_windows[*]}" >&2
  exit 1
fi

# The BuildMap core persists between episodes, so its controller process may
# predate the checked deployment. Pause first, then replace only the raw
# /cmd_vel controller before any receiver or chassis bridge can be created.
# The latched pause makes the new process publish zero until the v2 receiver
# explicitly authorizes a fresh trajectory in live mode.
timeout 5 ros2 topic pub --once \
  /nav/paused std_msgs/msg/Bool '{data: true}' \
  >/dev/null 2>&1 || true
old_control_pid="$(
  tmux display-message -p -t "$SESSION:control" '#{pane_pid}'
)"
tmux respawn-pane -k -t "$SESSION:control" \
  "bash -lc 'source \"$SETUP_FILE\"; cd \"$TINYNAV_ROOT\"; uv run python \"$SCRIPT_DIR/yunji_tinynav_cmd_vel_control.py\"'"
new_control_pid="$(
  tmux display-message -p -t "$SESSION:control" '#{pane_pid}'
)"
control_start="$(
  tmux display-message -p -t "$SESSION:control" '#{pane_start_command}'
)"
[[ -n "$new_control_pid" \
   && "$new_control_pid" != "$old_control_pid" \
   && "$control_start" == *"yunji_tinynav_cmd_vel_control.py"* ]] || {
  echo "WSJ velocity controller did not reload from the deployment wrapper." >&2
  exit 1
}
deadline=$((SECONDS + 15))
until ros2 node list 2>/dev/null | grep -qx /cmd_vel_control_node; do
  if [[ "$(tmux display-message -p -t "$SESSION:control" '#{pane_dead}')" == 1 ]]; then
    tmux capture-pane -pt "$SESSION:control" -S -100 >&2 || true
    exit 1
  fi
  (( SECONDS < deadline )) || {
    echo "Timed out waiting for the reloaded WSJ velocity controller." >&2
    exit 1
  }
  sleep 1
done
echo "WSJ velocity controller reloaded from the current deployment: $new_control_pid"

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
      --frame-id world \
      --camera-frame camera \
      --fresh-image-topic /camera/camera/color/image_raw \
      --fresh-image-topic /slam/depth \
      --geometry-image-topic /slam/depth \
      --camera-info-topic /slam/camera_info \
      --max-occupancy-age-s 12 \
      --max-cached-occupancy-motion-m "$MAX_CACHED_MAP_MOTION_M" \
      --timeout-s 35
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

# A BuildMap session intentionally survives between experiments.  Its Python
# goal-router process therefore may predate a newly deployed overlay even when
# the files on disk are byte-identical.  Reload only that non-actuating process
# before creating a receiver or bridge.  The replacement starts with no goal
# and publishes HOLD; the persistent mapper and TinyNav planner are untouched.
old_goal_router_pid="$(
  tmux display-message -p -t "$SESSION:goal-router" '#{pane_pid}'
)"
tmux respawn-pane -k -t "$SESSION:goal-router" \
  "bash -lc 'source \"$SETUP_FILE\"; export PYTHONPATH=\"$SCRIPT_DIR/../src\":\${PYTHONPATH:-}; \"$PYTHON_BIN\" -u \"$SCRIPT_DIR/tinynav_buildmap_goal_router.py\" --frame-id world --occupancy-topic /semantic_mapping/occupancy_bev --base-camera-calibration-file \"$BASE_CAMERA_CALIBRATION_FILE\" --clearance-m 0.05 --start-snap-radius-m \"$START_SNAP_RADIUS_M\" --start-footprint-override-m \"$START_FOOTPRINT_OVERRIDE_M\" --input-timeout-s \"$ODOMETRY_INPUT_TIMEOUT_S\" --map-timeout-s \"$MAP_TIMEOUT_S\" --max-cached-map-motion-m \"$MAX_CACHED_MAP_MOTION_M\"'"
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
echo "WSJ goal-router reloaded from the current deployment: $new_goal_router_pid"

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
alignment="/home/nvidia/.local/state/topofocus/wsj-v2-buildmap-${mode}-${stamp}.json"
log="/home/nvidia/.local/state/topofocus/wsj-v2-buildmap-${mode}-${stamp}.jsonl"
bridge_log="/home/nvidia/.local/state/topofocus/wsj-go2-bridge-${stamp}.log"
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
  --local-data-timeout-s "$ODOMETRY_INPUT_TIMEOUT_S"
  --slam-data-timeout-s "$SLAM_DATA_TIMEOUT_S"
  --semantic-arrival-radius-m "$SEMANTIC_ARRIVAL_RADIUS_M"
  --no-progress-timeout-s "$NO_PROGRESS_TIMEOUT_S"
  --minimum-goal-progress-m "$MINIMUM_GOAL_PROGRESS_M"
  --start-snap-radius-m 0.75
  --start-footprint-override-m 0.35
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

# The Unitree SDK2 bridge and ROS 2 coexist in one Python process. On the
# deployed Go2, starting that participant can delay discovery for newly
# created high-bandwidth image subscribers even though the existing mapper,
# sender and controller subscriptions continue normally. Prove every sensor,
# geometry, map-freshness and non-actuating command-route invariant before the
# SDK2 participant exists. The post-bridge check below then proves only the
# one endpoint that the bridge adds.
source "$SETUP_FILE"
pre_bridge_args=()
if [[ "$mode" == live ]]; then
  pre_bridge_args+=(--pre-bridge-full-check)
fi
"$PYTHON_BIN" -u "$SCRIPT_DIR/verify_tinynav_data_plane.py" \
  --robot-id robot-0 \
  --mode "$mode" \
  "${pre_bridge_args[@]}" \
  --frame-id world \
  --camera-frame camera \
  --fresh-image-topic /camera/camera/color/image_raw \
  --fresh-image-topic /slam/depth \
  --geometry-image-topic /slam/depth \
  --camera-info-topic /slam/camera_info \
  --max-occupancy-age-s 12 \
  --max-cached-occupancy-motion-m "$MAX_CACHED_MAP_MOTION_M" \
  --timeout-s 35

if [[ "$mode" == live ]]; then
  tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -qx go2-bridge && {
    echo "Refusing to replace existing Go2 bridge window." >&2
    exit 1
  }
  tmux new-window -d -t "$SESSION" -n go2-bridge \
    "bash -lc 'set -o pipefail; export GO2_CMD_TOPIC=/focus_guarded_cmd_vel GO2_MAX_VX=0.20 GO2_MAX_VY=0.00 GO2_MAX_WZ=0.50 GO2_MIN_CMD_V=0.15 GO2_MIN_CMD_W=0.30 GO2_REMOTE_PRIORITY=true GO2_LOG_COMMANDS=true GO2_LOG_INTERVAL_SEC=0.2; bash /home/nvidia/twork/tinynav/scripts/run_go2_cmd_bridge.sh 2>&1 | tee \"$bridge_log\"'"
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
