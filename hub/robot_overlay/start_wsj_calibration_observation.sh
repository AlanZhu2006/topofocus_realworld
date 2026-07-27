#!/usr/bin/env bash
# Start a fresh, mapping-only WSJ observation epoch for board calibration.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${FOCUS_WSJ_NAV_SESSION:-tinynav_semantic_nav_auto}"
ENV_FILE="${FOCUS_WSJ_ENV_FILE:-/home/nvidia/focus_sender/go2_20260723.env}"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
PYTHON_BIN="${TINYNAV_PYTHON:-/home/nvidia/twork/tinynav/.venv/bin/python}"
TOKEN_FILE="${FOCUS_ROBOT_TOKEN_FILE:-/home/nvidia/focus_sender/.token}"
HUB_URL="${FOCUS_HUB_BASE_URL:-http://127.0.0.1:18089}"
PREVIEW_URL="${FOCUS_FOXGLOVE_PREVIEW_URL:-http://127.0.0.1:18766}"
DEPLOYMENT_COMMIT="${FOCUS_DEPLOYMENT_COMMIT:-}"
STATE_DIR="${FOCUS_ROBOT_STATE_DIR:-/home/nvidia/.local/state/topofocus}"
RUNTIME_RECEIPT_FILE="${FOCUS_WSJ_RUNTIME_RECEIPT_FILE:-$STATE_DIR/wsj-command-observation-receipt.json}"
REANCHOR_REQUIRED_FILE="${FOCUS_WSJ_REANCHOR_REQUIRED_FILE:-$STATE_DIR/wsj-tracking-reanchor-required.json}"
TRANSFORM_VERSION=""
CONFIRMATION=""

usage() {
  cat <<'EOF'
Usage: start_wsj_calibration_observation.sh \
  --transform-version UNIQUE_RAW_TRANSFORM \
  --operator-confirmation OPERATOR_PRESENT_AND_BOARD_ONLY \
  [--session NAME] [--env FILE]

This command latches navigation pause, removes receiver/planner/bridge windows,
recovers the D435i/TinyNav sensor epoch before any board frame is captured,
then starts only a mapping-only Hub sender and camera preview. Calibration uses
the native rectified infra1 image, depth, intrinsics and pose in the same
optical frame, so no RGB-to-depth mosaic can create a second board. It never
starts a GOAL receiver or Go2 bridge.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --transform-version) TRANSFORM_VERSION="$2"; shift 2 ;;
    --operator-confirmation) CONFIRMATION="$2"; shift 2 ;;
    --session) SESSION="$2"; shift 2 ;;
    --env) ENV_FILE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$TRANSFORM_VERSION" =~ ^[A-Za-z0-9_.-]+$ ]] || {
  echo "A filesystem-safe raw --transform-version is required." >&2
  exit 2
}
[[ "$CONFIRMATION" == OPERATOR_PRESENT_AND_BOARD_ONLY ]] || {
  echo "Calibration observation requires OPERATOR_PRESENT_AND_BOARD_ONLY." >&2
  exit 2
}
[[ "$HUB_URL" =~ ^http://127\.0\.0\.1:[0-9]+$ ]] || {
  echo "Hub URL must remain loopback-only." >&2
  exit 2
}
[[ "$PREVIEW_URL" =~ ^http://127\.0\.0\.1:[0-9]+$ ]] || {
  echo "Preview URL must remain loopback-only." >&2
  exit 2
}
[[ "$DEPLOYMENT_COMMIT" =~ ^[0-9a-f]{40}$ ]] || {
  echo "FOCUS_DEPLOYMENT_COMMIT must be the explicit 40-character Git commit." >&2
  exit 2
}
for required in \
  "$ENV_FILE" "$SETUP_FILE" "$PYTHON_BIN" "$TOKEN_FILE" \
  "$SCRIPT_DIR/focus_ros_sender.py" "$SCRIPT_DIR/wsj_camera_preview.py" \
  "$SCRIPT_DIR/start_wsj_command_observation.sh" \
  "$SCRIPT_DIR/start_go2_observation.sh" \
  "$SCRIPT_DIR/verify_ros_geometry_profile.py" \
  "$SCRIPT_DIR/verify_tinynav_data_plane.py"; do
  [[ -r "$required" ]] || {
    echo "Missing calibration-observation input: $required" >&2
    exit 1
  }
done

source "$SETUP_FILE"
timeout 5 ros2 topic pub --once /nav/paused std_msgs/msg/Bool '{data: true}' \
  >/dev/null 2>&1 || true
if ros2 topic list 2>/dev/null | grep -qx /focus_guarded_cmd_vel; then
  timeout 5 ros2 topic pub --once /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' \
    >/dev/null 2>&1 || true
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  for window in \
    go2-bridge v2-receiver control goal-router planning online-map maploc \
    calibration-sender; do
    if tmux list-windows -t "$SESSION" -F '#{window_name}' \
        | grep -qx "$window"; then
      tmux send-keys -t "$SESSION:$window" C-c >/dev/null 2>&1 || true
    fi
  done
  sleep 2
  for window in \
    go2-bridge v2-receiver control goal-router planning online-map maploc \
    calibration-sender; do
    tmux kill-window -t "$SESSION:$window" >/dev/null 2>&1 || true
  done
  for required_window in camera perception; do
    tmux list-windows -t "$SESSION" -F '#{window_name}' \
      | grep -qx "$required_window" || {
        echo "Existing session lacks $required_window; stop it and retry." >&2
        exit 1
      }
  done
else
  bash "$SCRIPT_DIR/start_go2_observation.sh" \
    --env "$ENV_FILE" \
    --session "$SESSION"
fi

if pgrep -af \
  'go2_cmd_bridge|cmd_vel_control|planning_node.py|v2_wsj_receiver.py|nav2_controller' \
  >/dev/null 2>&1; then
  echo "A WSJ planner/receiver/bridge remains after fail-closed cleanup." >&2
  exit 1
fi

for legacy_session in focus_wsj_camera_preview_20260723 focus_wsj_mapping; do
  if tmux has-session -t "$legacy_session" 2>/dev/null; then
    for window in sender sender_rgb; do
      tmux kill-window -t "$legacy_session:$window" >/dev/null 2>&1 || true
    done
  fi
done

# Establish the persistent formal subscriber before touching either publisher.
# It is parked (no Hub upload), but its DDS readers are already present when
# camera/perception are restarted below. Calibration uses a second keyframe
# sender, also created before those publishers.
FOCUS_DEPLOYMENT_COMMIT="$DEPLOYMENT_COMMIT" \
FOCUS_HUB_BASE_URL="$HUB_URL" \
bash "$SCRIPT_DIR/start_wsj_command_observation.sh" \
  --session "$SESSION" \
  --park-only
runtime_sender_pid() {
  local pid executable
  while IFS= read -r pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    executable="$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)"
    [[ "${executable##*/}" == python* ]] || continue
    printf '%s\n' "$pid"
    return 0
  done < <(
    pgrep -f \
      'focus_ros_sender\.py.*--runtime-command-contract-file' \
      2>/dev/null || true
  )
}
persistent_sender_pid="$(runtime_sender_pid)"
[[ -n "$persistent_sender_pid" ]] || {
  echo "Persistent WSJ sender disappeared after parking." >&2
  exit 1
}

token="$(<"$TOKEN_FILE")"
initial_json="$(
  curl -fsS --max-time 5 -H "X-Robot-Token: $token" \
    "$HUB_URL/v1/robots/robot-0/observations/latest"
)"
initial_sequence="$(
  FOCUS_SEQUENCE_JSON="$initial_json" python3 -c \
    'import json,os; print(int(json.loads(os.environ["FOCUS_SEQUENCE_JSON"])["last_sequence"]))'
)"
unset token initial_json

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
state_dir="$STATE_DIR"
mkdir -p "$state_dir"
sender_log="$state_dir/wsj-calibration-sender-$stamp.log"
metrics="$state_dir/wsj-calibration-sender-$stamp.json"
sender=(
  "$PYTHON_BIN" -u "$SCRIPT_DIR/focus_ros_sender.py"
  --base-url "$HUB_URL"
  --robot-id robot-0
  --transform-version "$TRANSFORM_VERSION"
  --rgb-topic /camera/camera/infra1/image_rect_raw
  --depth-topic /slam/keyframe_depth
  --info-topic /slam/camera_info
  --pose-topic /slam/keyframe_odom
  --camera-frame camera
  --capture-time-source header
  --rate-hz 2.0
  --max-frames 0
  --metrics-out "$metrics"
)
printf -v sender_text '%q ' "${sender[@]}"
tmux new-window -d -t "$SESSION" -n calibration-sender \
  "bash -lc 'source \"$SETUP_FILE\"; export FOCUS_ROBOT_TOKEN=\"\$(<\"$TOKEN_FILE\")\"; export PYTHONPATH=\"$SCRIPT_DIR/../src\":\${PYTHONPATH:-}; set -o pipefail; $sender_text 2>&1 | tee \"$sender_log\"'"
deadline=$((SECONDS + 20))
until pgrep -af \
    'focus_ros_sender\.py.*--depth-topic /slam/keyframe_depth' \
    >/dev/null 2>&1; do
  if [[ "$(tmux display-message -p -t "$SESSION:calibration-sender" \
      '#{pane_dead}' 2>/dev/null || true)" == 1 ]]; then
    tmux capture-pane -pt "$SESSION:calibration-sender" -S -100 >&2 || true
    exit 1
  fi
  (( SECONDS < deadline )) || {
    echo "Timed out prewarming the WSJ calibration DDS participant." >&2
    exit 1
  }
  sleep 1
done
echo "WSJ_DDS_SUBSCRIBERS_READY_BEFORE_PUBLISHERS"

fresh_topic_once() {
  local topic="$1"
  # Avoid false camera recovery when the ros2 CLI cannot drain full
  # 848x480@30 Hz RGB messages quickly enough.  A one-sample best-effort
  # subscription to the header proves that a current sensor message arrived.
  timeout -k 2 15 ros2 topic echo --once \
    --field header \
    --qos-reliability best_effort \
    --qos-durability volatile \
    --qos-depth 1 \
    "$topic" \
    >/dev/null 2>&1
}

wait_for_fresh_topic() {
  local topic="$1" description="$2" deadline
  deadline=$((SECONDS + 75))
  until fresh_topic_once "$topic"; do
    (( SECONDS < deadline )) || {
      echo "Timed out waiting for fresh $description ($topic)." >&2
      return 1
    }
    sleep 1
  done
}

parked_tuple_count() {
  FOCUS_RECEIPT="$RUNTIME_RECEIPT_FILE" \
  FOCUS_EXPECT_PID="$persistent_sender_pid" \
  "$PYTHON_BIN" - <<'PY'
import json
import os
from pathlib import Path

payload = json.loads(Path(os.environ["FOCUS_RECEIPT"]).read_text())
if payload.get("status") != "parked":
    raise SystemExit("persistent WSJ sender is not parked")
if int(payload.get("pid", -1)) != int(os.environ["FOCUS_EXPECT_PID"]):
    raise SystemExit("persistent WSJ sender receipt PID changed")
print(int(payload.get("parked_geometry_tuples", -1)))
PY
}

wait_for_persistent_sender_tuple() {
  local baseline="$1" deadline current current_pid
  deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    current_pid="$(runtime_sender_pid)"
    [[ "$current_pid" == "$persistent_sender_pid" ]] || {
      echo "Persistent WSJ sender PID changed during publisher restart." >&2
      return 1
    }
    current="$(parked_tuple_count 2>/dev/null || true)"
    if [[ "$current" =~ ^[0-9]+$ ]] && (( current > baseline )); then
      return 0
    fi
    sleep 1
  done
  echo "Persistent WSJ sender received no post-restart synchronized tuple." >&2
  return 1
}

# Fast DDS recovery has one authoritative order:
#   1. both sender participants already exist (proved above);
#   2. stop the old perception publisher;
#   3. restart camera;
#   4. start a fresh perception publisher.
# Calibration intentionally creates a new tracking epoch, so doing this once
# here is safe and eliminates the observed "publisher visible, no samples"
# state. Never invert this order or replace either sender afterward.
tracking_boot_id="wsj-camera-perception-calibration-$(date -u +%Y%m%dT%H%M%S)_${RANDOM}"
reanchor_marker="$REANCHOR_REQUIRED_FILE"
FOCUS_MARKER="$reanchor_marker" \
FOCUS_BOOT_ID="$tracking_boot_id" \
FOCUS_DEPLOYMENT_COMMIT="$DEPLOYMENT_COMMIT" \
FOCUS_SENDER_PID="$persistent_sender_pid" \
"$PYTHON_BIN" - <<'PY'
import json
import os
from pathlib import Path
import time

destination = Path(os.environ["FOCUS_MARKER"])
destination.parent.mkdir(parents=True, exist_ok=True)
payload = {
    "schema_version": "focus-wsj-tracking-reanchor-required-v1",
    "tracking_restart_boot_id": os.environ["FOCUS_BOOT_ID"],
    "deployment_commit": os.environ["FOCUS_DEPLOYMENT_COMMIT"],
    "sender_pid_preserved": int(os.environ["FOCUS_SENDER_PID"]),
    "recovery_status": "board_calibration_publishers_restarting",
    "publisher_order": [
        "persistent_sender_parked",
        "calibration_sender_running",
        "old_perception_stopped",
        "camera_restarted",
        "perception_restarted",
    ],
    "publisher_order_complete": False,
    "resolution": "validated_new_board_calibration_required",
    "classification": "source_derived_calibration_epoch_started",
    "robot_commands_issued": False,
    "written_at_ns": time.time_ns(),
}
temporary = destination.with_name(f".{destination.name}.tmp.{os.getpid()}")
temporary.write_text(
    json.dumps(payload, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
os.chmod(temporary, 0o600)
os.replace(temporary, destination)
PY

tmux set-option -w -t "$SESSION:perception" remain-on-exit on
if [[ "$(tmux display-message -p -t "$SESSION:perception" \
    '#{pane_dead}' 2>/dev/null || true)" == 0 ]]; then
  tmux send-keys -t "$SESSION:perception" C-c
  deadline=$((SECONDS + 20))
  until [[ "$(tmux display-message -p -t "$SESSION:perception" \
      '#{pane_dead}' 2>/dev/null || true)" == 1 ]]; do
    (( SECONDS < deadline )) || {
      echo "Old perception publisher did not stop; refusing camera restart." >&2
      exit 1
    }
    sleep 1
  done
fi

sleep 1
parked_tuple_baseline="$(parked_tuple_count)"
tmux respawn-pane -k -t "$SESSION:camera"
wait_for_fresh_topic \
  /camera/camera/infra1/image_rect_raw "WSJ infra1 after ordered camera restart"
tmux respawn-pane -k -t "$SESSION:perception"
camera_restarted=true
perception_restarted=true

wait_for_fresh_topic /slam/depth "TinyNav processed depth"
wait_for_fresh_topic /slam/odometry_visual "TinyNav continuous visual odometry"
wait_for_persistent_sender_tuple "$parked_tuple_baseline"
wait_for_fresh_topic /slam/camera_info "TinyNav camera intrinsics"
wait_for_fresh_topic \
  /camera/camera/infra1/image_rect_raw "RealSense rectified infra1 image"
"$PYTHON_BIN" -u "$SCRIPT_DIR/verify_ros_geometry_profile.py" \
  --image-topic /slam/depth \
  --camera-info-topic /slam/camera_info \
  --expected-frame camera \
  --timeout-s 15

# Require a second processed frame after a short soak.  One retained/startup
# frame is not proof that the IMU watermark continues to advance.
sleep 5
wait_for_fresh_topic /slam/depth "stable TinyNav processed depth"
wait_for_fresh_topic \
  /slam/odometry_visual "stable TinyNav continuous visual odometry"
echo "WSJ_CALIBRATION_SENSOR_EPOCH_READY:" \
  "camera_restarted=$camera_restarted" \
  "perception_restarted=$perception_restarted" \
  "dds_order=senders_then_camera_then_perception" \
  "keyframe_tuple_gate=sender_sequence"

# `/slam/keyframe_depth` and `/slam/keyframe_odom` are deliberately absent
# from the continuous sensor-epoch gate above. TinyNav publishes that exact
# pair only when `keyframe_check(...)` accepts a frame (or its sparse-keyframe
# timeout expires), so treating either topic as a heartbeat can repeatedly
# restart a healthy perception process while a stationary robot is waiting
# for board calibration. The prewarmed calibration sender synchronizes the exact
# keyframe depth/pose tuple, and the mandatory Hub sequence advance remains
# the end-to-end proof that a fresh tuple was actually captured.

tmux kill-window -t "$SESSION:foxglove-preview" >/dev/null 2>&1 || true
sleep 1
if pgrep -af 'wsj_camera_preview\.py' >/dev/null 2>&1; then
  echo "An untracked WSJ camera preview is still running." >&2
  exit 1
fi
preview_log="$state_dir/wsj-calibration-preview-$stamp.log"
tmux new-window -d -t "$SESSION" -n foxglove-preview \
  "bash -lc 'source \"$SETUP_FILE\"; export FOCUS_ROBOT_TOKEN=\"\$(<\"$TOKEN_FILE\")\"; exec \"$PYTHON_BIN\" -u \"$SCRIPT_DIR/wsj_camera_preview.py\" --relay-url \"$PREVIEW_URL\" --name wsj --rgb-topic /camera/camera/infra1/image_rect_raw --max-rate-hz 5 2>&1 | tee \"$preview_log\"'"

deadline=$((SECONDS + 60))
latest_sequence="$initial_sequence"
while (( SECONDS < deadline )); do
  token="$(<"$TOKEN_FILE")"
  latest_json="$(
    curl -fsS --max-time 5 -H "X-Robot-Token: $token" \
      "$HUB_URL/v1/robots/robot-0/observations/latest" 2>/dev/null || true
  )"
  unset token
  if [[ -n "$latest_json" ]]; then
    latest_sequence="$(
      FOCUS_SEQUENCE_JSON="$latest_json" python3 -c \
        'import json,os; print(int(json.loads(os.environ["FOCUS_SEQUENCE_JSON"])["last_sequence"]))'
    )"
    (( latest_sequence > initial_sequence )) && break
  fi
  sleep 1
done
(( latest_sequence > initial_sequence )) || {
  echo "No fresh WSJ calibration observation arrived." >&2
  exit 1
}

echo "WSJ calibration observation ready: $initial_sequence -> $latest_sequence"
echo "Safety: navigation paused; no planner, receiver or Go2 bridge is running."
