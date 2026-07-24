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
then starts only a mapping-only Hub sender and camera preview. It never starts
a GOAL receiver or Go2 bridge.
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
for required in \
  "$ENV_FILE" "$SETUP_FILE" "$PYTHON_BIN" "$TOKEN_FILE" \
  "$SCRIPT_DIR/focus_ros_sender.py" "$SCRIPT_DIR/wsj_camera_preview.py" \
  "$SCRIPT_DIR/start_go2_observation.sh"; do
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
    hub-sender calibration-sender; do
    if tmux list-windows -t "$SESSION" -F '#{window_name}' \
        | grep -qx "$window"; then
      tmux send-keys -t "$SESSION:$window" C-c >/dev/null 2>&1 || true
    fi
  done
  sleep 2
  for window in \
    go2-bridge v2-receiver control goal-router planning online-map maploc \
    hub-sender calibration-sender; do
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

fresh_topic_once() {
  local topic="$1"
  timeout -k 2 8 ros2 topic echo --once "$topic" \
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

# Camera and perception have separate process lifetimes.  If RealSense is
# respawned while perception keeps its old IMU watermark, RGB remains live but
# every stereo pair can be rejected forever.  Recovery is safe only here,
# before the board defines the new tracking epoch.
camera_restarted=false
perception_restarted=false
if [[ "$(tmux display-message -p -t "$SESSION:camera" '#{pane_dead}')" != 0 ]] \
   || ! fresh_topic_once /camera/camera/color/image_raw; then
  tmux respawn-pane -k -t "$SESSION:camera"
  camera_restarted=true
  wait_for_fresh_topic \
    /camera/camera/color/image_raw "WSJ RGB after camera recovery"
fi

if [[ "$(tmux display-message -p -t "$SESSION:perception" '#{pane_dead}')" != 0 ]] \
   || [[ "$camera_restarted" == true ]] \
   || ! fresh_topic_once /slam/depth \
   || ! fresh_topic_once /slam/keyframe_depth \
   || ! fresh_topic_once /slam/keyframe_odom; then
  tmux respawn-pane -k -t "$SESSION:perception"
  perception_restarted=true
fi

wait_for_fresh_topic /slam/depth "TinyNav processed depth"
wait_for_fresh_topic /slam/keyframe_depth "TinyNav keyframe depth"
wait_for_fresh_topic /slam/keyframe_odom "TinyNav keyframe odometry"
wait_for_fresh_topic /slam/camera_info "TinyNav camera intrinsics"
wait_for_fresh_topic \
  /camera/camera/color/camera_info "RealSense RGB camera intrinsics"

# Require a second processed frame after a short soak.  One retained/startup
# frame is not proof that the IMU watermark continues to advance.
sleep 5
wait_for_fresh_topic /slam/depth "stable TinyNav processed depth"
wait_for_fresh_topic /slam/keyframe_odom "stable TinyNav keyframe odometry"
echo "WSJ_CALIBRATION_SENSOR_EPOCH_READY:" \
  "camera_restarted=$camera_restarted" \
  "perception_restarted=$perception_restarted"

for legacy_session in focus_wsj_camera_preview_20260723 focus_wsj_mapping; do
  if tmux has-session -t "$legacy_session" 2>/dev/null; then
    for window in sender sender_rgb; do
      tmux kill-window -t "$legacy_session:$window" >/dev/null 2>&1 || true
    done
  fi
done
if pgrep -af 'focus_ros_sender(_rgb)?\.py' >/dev/null 2>&1; then
  echo "An untracked WSJ Hub sender is still running." >&2
  exit 1
fi

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
state_dir="/home/nvidia/.local/state/topofocus"
mkdir -p "$state_dir"
sender_log="$state_dir/wsj-calibration-sender-$stamp.log"
metrics="$state_dir/wsj-calibration-sender-$stamp.json"
sender=(
  "$PYTHON_BIN" -u "$SCRIPT_DIR/focus_ros_sender.py"
  --base-url "$HUB_URL"
  --robot-id robot-0
  --transform-version "$TRANSFORM_VERSION"
  --rgb-topic /camera/camera/color/image_raw
  --depth-topic /slam/keyframe_depth
  --info-topic /slam/camera_info
  --pose-topic /slam/keyframe_odom
  --camera-frame camera
  --register-rgb-to-depth
  --rgb-info-topic /camera/camera/color/camera_info
  --rgb-optical-frame camera_color_optical_frame
  --depth-optical-frame camera_infra1_optical_frame
  --registration-min-coverage 0.45
  --capture-time-source header
  --rate-hz 2.0
  --max-frames 0
  --metrics-out "$metrics"
)
printf -v sender_text '%q ' "${sender[@]}"
tmux new-window -d -t "$SESSION" -n calibration-sender \
  "bash -lc 'source \"$SETUP_FILE\"; export FOCUS_ROBOT_TOKEN=\"\$(<\"$TOKEN_FILE\")\"; export PYTHONPATH=\"$SCRIPT_DIR/../src\":\${PYTHONPATH:-}; set -o pipefail; $sender_text 2>&1 | tee \"$sender_log\"'"

if ! pgrep -af 'wsj_camera_preview\.py' >/dev/null 2>&1; then
  tmux kill-window -t "$SESSION:foxglove-preview" >/dev/null 2>&1 || true
  preview_log="$state_dir/wsj-calibration-preview-$stamp.log"
  tmux new-window -d -t "$SESSION" -n foxglove-preview \
    "bash -lc 'source \"$SETUP_FILE\"; export FOCUS_ROBOT_TOKEN=\"\$(<\"$TOKEN_FILE\")\"; exec \"$PYTHON_BIN\" -u \"$SCRIPT_DIR/wsj_camera_preview.py\" --relay-url \"$PREVIEW_URL\" --name wsj --rgb-topic /camera/camera/color/image_raw --max-rate-hz 5 2>&1 | tee \"$preview_log\"'"
fi

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
