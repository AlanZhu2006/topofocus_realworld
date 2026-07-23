#!/usr/bin/env bash
# Restart only the WSJ observation sender with measured command-capable metadata.
#
# This does not start a receiver, planner, controller, bridge or robot command.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${FOCUS_WSJ_NAV_SESSION:-tinynav_semantic_nav_auto}"
LEGACY_SESSION="${FOCUS_WSJ_SENDER_SESSION:-focus_wsj_camera_preview_20260723}"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
PYTHON_BIN="${TINYNAV_PYTHON:-/home/nvidia/twork/tinynav/.venv/bin/python}"
TOKEN_FILE="${FOCUS_ROBOT_TOKEN_FILE:-/home/nvidia/focus_sender/.token}"
BASE_CAMERA_CALIBRATION="${FOCUS_WSJ_BASE_CAMERA_CALIBRATION:-/home/nvidia/.local/state/topofocus/calibration/wsj_tinynav_camera_base_20260723_operator.json}"
SHARED_TRACKING_CALIBRATION="${FOCUS_SHARED_CALIBRATION_FILE:-/home/nvidia/.local/state/topofocus/calibration/shared_board_odin1_20260723_v3_yunji_powercycle_v6.json}"
SHARED_FRAME_CALIBRATION_ID="${FOCUS_SHARED_CALIBRATION_ID:-shared-board-odin1-20260723-v3}"
TRANSFORM_VERSION="${FOCUS_WSJ_TRANSFORM_VERSION:-wsj-tinynav-depth-20260723-powercycle-v3}"
HUB_URL="${FOCUS_HUB_BASE_URL:-http://127.0.0.1:18089}"
PREVIEW_URL="${FOCUS_FOXGLOVE_PREVIEW_URL:-http://127.0.0.1:18766}"
PREVIEW_WINDOW="${FOCUS_WSJ_PREVIEW_WINDOW:-foxglove-preview}"

usage() {
  cat <<'EOF'
Usage: start_wsj_command_observation.sh [options]
  --session NAME
  --legacy-session NAME
  --base-camera-calibration FILE
  --shared-tracking-calibration FILE
  --shared-frame-calibration-id ID
  --transform-version ID
  --hub-url http://127.0.0.1:PORT
  --preview-url http://127.0.0.1:PORT
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session) SESSION="$2"; shift 2 ;;
    --legacy-session) LEGACY_SESSION="$2"; shift 2 ;;
    --base-camera-calibration) BASE_CAMERA_CALIBRATION="$2"; shift 2 ;;
    --shared-tracking-calibration) SHARED_TRACKING_CALIBRATION="$2"; shift 2 ;;
    --shared-frame-calibration-id) SHARED_FRAME_CALIBRATION_ID="$2"; shift 2 ;;
    --transform-version) TRANSFORM_VERSION="$2"; shift 2 ;;
    --hub-url) HUB_URL="$2"; shift 2 ;;
    --preview-url) PREVIEW_URL="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$HUB_URL" =~ ^http://127\.0\.0\.1:[0-9]+$ ]] || {
  echo "Hub URL must remain loopback-only." >&2
  exit 2
}
[[ "$PREVIEW_URL" =~ ^http://127\.0\.0\.1:[0-9]+$ ]] || {
  echo "Foxglove preview URL must remain loopback-only." >&2
  exit 2
}
for required in \
  "$SCRIPT_DIR/focus_ros_sender.py" \
  "$SCRIPT_DIR/wsj_camera_preview.py" \
  "$SETUP_FILE" \
  "$PYTHON_BIN" \
  "$TOKEN_FILE" \
  "$BASE_CAMERA_CALIBRATION" \
  "$SHARED_TRACKING_CALIBRATION"; do
  [[ -r "$required" ]] || {
    echo "Missing required observation input: $required" >&2
    exit 1
  }
done
tmux has-session -t "$SESSION" 2>/dev/null || {
  echo "WSJ camera/perception session is not running: $SESSION" >&2
  exit 1
}

ensure_camera_preview() {
  local preview_log deadline
  if pgrep -af 'wsj_camera_preview\.py' >/dev/null 2>&1; then
    return 0
  fi
  if tmux list-windows -t "$SESSION" -F '#{window_name}' \
      | grep -qx "$PREVIEW_WINDOW"; then
    tmux kill-window -t "$SESSION:$PREVIEW_WINDOW" >/dev/null 2>&1 || true
  fi
  preview_log="/home/nvidia/.local/state/topofocus/wsj-camera-preview-$(date -u +%Y%m%dT%H%M%SZ).log"
  tmux new-window -d -t "$SESSION" -n "$PREVIEW_WINDOW" \
    "bash -lc 'source \"$SETUP_FILE\"; export FOCUS_ROBOT_TOKEN=\"\$(<\"$TOKEN_FILE\")\"; exec \"$PYTHON_BIN\" -u \"$SCRIPT_DIR/wsj_camera_preview.py\" --relay-url \"$PREVIEW_URL\" --name wsj --rgb-topic /camera/camera/color/image_raw --max-rate-hz 5 2>&1 | tee \"$preview_log\"'"
  deadline=$((SECONDS + 20))
  until pgrep -af 'wsj_camera_preview\.py' >/dev/null 2>&1; do
    if [[ "$(tmux display-message -p -t "$SESSION:$PREVIEW_WINDOW" '#{pane_dead}')" == 1 ]]; then
      tmux capture-pane -pt "$SESSION:$PREVIEW_WINDOW" -S -80 >&2 || true
      return 1
    fi
    (( SECONDS < deadline )) || {
      echo "Timed out waiting for WSJ Foxglove camera preview." >&2
      return 1
    }
    sleep 1
  done
  echo "WSJ Foxglove camera preview is active (read-only)."
}

ensure_camera_preview
if pgrep -af 'focus_ros_sender.*--enable-command-capable-observations' \
  >/dev/null 2>&1; then
  echo "WSJ command-capable observation sender is already running."
  exit 0
fi
if pgrep -af 'go2_cmd_bridge' >/dev/null 2>&1; then
  echo "Refusing to replace observation metadata while a Go2 bridge is active." >&2
  exit 1
fi

if tmux has-session -t "$LEGACY_SESSION" 2>/dev/null \
   && tmux list-windows -t "$LEGACY_SESSION" -F '#{window_name}' \
      | grep -qx sender_rgb; then
  tmux send-keys -t "$LEGACY_SESSION:sender_rgb" C-c >/dev/null 2>&1 || true
  sleep 2
  tmux kill-window -t "$LEGACY_SESSION:sender_rgb" >/dev/null 2>&1 || true
fi
if tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -qx hub-sender; then
  echo "Refusing to replace existing $SESSION:hub-sender." >&2
  exit 1
fi
if pgrep -af 'focus_ros_sender(_rgb)?\.py' >/dev/null 2>&1; then
  echo "An untracked WSJ observation sender is still running." >&2
  exit 1
fi

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
metrics="/home/nvidia/.local/state/topofocus/wsj-command-observation-${stamp}.json"
log="/home/nvidia/.local/state/topofocus/wsj-command-observation-${stamp}.log"
command=(
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
  --enable-command-capable-observations
  --activation-confirmation COMMAND_CAPABLE_OBSERVATION_ONLY
  --base-camera-calibration-file "$BASE_CAMERA_CALIBRATION"
  --shared-tracking-calibration-file "$SHARED_TRACKING_CALIBRATION"
  --shared-frame-calibration-id "$SHARED_FRAME_CALIBRATION_ID"
  --heartbeat-hz 0
)
printf -v command_text '%q ' "${command[@]}"
tmux new-window -d -t "$SESSION" -n hub-sender \
  "bash -lc 'source \"$SETUP_FILE\"; export FOCUS_ROBOT_TOKEN=\"\$(<\"$TOKEN_FILE\")\"; export PYTHONPATH=\"$SCRIPT_DIR/../src\":\${PYTHONPATH:-}; set -o pipefail; $command_text 2>&1 | tee \"$log\"'"

deadline=$((SECONDS + 30))
until pgrep -af 'focus_ros_sender\.py.*--enable-command-capable-observations' \
  >/dev/null 2>&1; do
  if [[ "$(tmux display-message -p -t "$SESSION:hub-sender" '#{pane_dead}')" == 1 ]]; then
    tmux capture-pane -pt "$SESSION:hub-sender" -S -80 >&2 || true
    exit 1
  fi
  (( SECONDS < deadline )) || {
    echo "Timed out waiting for the WSJ command-capable sender." >&2
    exit 1
  }
  sleep 1
done

echo "WSJ command-capable observation metadata is active (NO MOTION PATH)."
echo "  transform: $TRANSFORM_VERSION"
echo "  mount:     $BASE_CAMERA_CALIBRATION"
echo "  shared:    $SHARED_TRACKING_CALIBRATION"
echo "  metrics:   $metrics"
echo "  log:       $log"
