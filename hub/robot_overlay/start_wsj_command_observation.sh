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
DEPLOYMENT_COMMIT="${FOCUS_DEPLOYMENT_COMMIT:-}"
PREVIEW_URL="${FOCUS_FOXGLOVE_PREVIEW_URL:-http://127.0.0.1:18766}"
PREVIEW_WINDOW="${FOCUS_WSJ_PREVIEW_WINDOW:-foxglove-preview}"
COLOR_PREVIEW_TOPIC="/camera/camera/color/image_raw"
# Observed with the deployed D435 profile on 2026-07-25: calibrated valid-depth
# overlap is stable at 0.412--0.418 because the color imager has a narrower FOV
# than infra1. 0.38 keeps a measured margin while still rejecting a grossly
# wrong intrinsic/extrinsic profile.
REGISTRATION_MIN_COVERAGE="${FOCUS_WSJ_REGISTRATION_MIN_COVERAGE:-0.38}"
# TinyNav keyframes are motion/content selected and can be much sparser while
# the robot is stationary.  A newly attached five-topic synchronizer was
# observed to need 17 s for its first accepted Hub upload on 2026-07-27.
# Allow that healthy first tuple to arrive instead of restarting just before
# it; this returns immediately on sequence advance, so it does not add delay
# to the normal path.  One bounded read-only sender restart remains the only
# self-heal if the sequence genuinely stalls.
SENDER_ADVANCE_TIMEOUT_S="${FOCUS_WSJ_SENDER_ADVANCE_TIMEOUT_S:-30}"

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
[[ "$SENDER_ADVANCE_TIMEOUT_S" =~ ^[1-9][0-9]*$ ]] || {
  echo "FOCUS_WSJ_SENDER_ADVANCE_TIMEOUT_S must be a positive integer." >&2
  exit 2
}
[[ "$DEPLOYMENT_COMMIT" =~ ^[0-9a-f]{40}$ ]] || {
  echo "FOCUS_DEPLOYMENT_COMMIT must be the explicit 40-character Git commit." >&2
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
  local preview_log deadline preview_processes
  preview_processes="$(
    pgrep -af '[w]sj_camera_preview\.py' 2>/dev/null || true
  )"
  if [[ -n "$preview_processes" ]] \
     && ! grep -Fv -- "--rgb-topic $COLOR_PREVIEW_TOPIC" \
       <<<"$preview_processes" >/dev/null; then
    return 0
  fi
  if tmux list-windows -t "$SESSION" -F '#{window_name}' \
      | grep -qx "$PREVIEW_WINDOW"; then
    tmux kill-window -t "$SESSION:$PREVIEW_WINDOW" >/dev/null 2>&1 || true
    sleep 1
  fi
  preview_processes="$(
    pgrep -af '[w]sj_camera_preview\.py' 2>/dev/null || true
  )"
  if [[ -n "$preview_processes" ]]; then
    if ! grep -Fv -- "--rgb-topic $COLOR_PREVIEW_TOPIC" \
      <<<"$preview_processes" >/dev/null; then
      return 0
    fi
    echo "An untracked non-color WSJ preview is still running:" >&2
    printf '%s\n' "$preview_processes" >&2
    return 1
  fi
  preview_log="/home/nvidia/.local/state/topofocus/wsj-camera-preview-$(date -u +%Y%m%dT%H%M%SZ).log"
  tmux new-window -d -t "$SESSION" -n "$PREVIEW_WINDOW" \
    "bash -lc 'source \"$SETUP_FILE\"; export FOCUS_ROBOT_TOKEN=\"\$(<\"$TOKEN_FILE\")\"; exec \"$PYTHON_BIN\" -u \"$SCRIPT_DIR/wsj_camera_preview.py\" --relay-url \"$PREVIEW_URL\" --name wsj --rgb-topic \"$COLOR_PREVIEW_TOPIC\" --max-rate-hz 5 2>&1 | tee \"$preview_log\"'"
  deadline=$((SECONDS + 20))
  until pgrep -af '[w]sj_camera_preview\.py' \
      | grep -F -- "--rgb-topic $COLOR_PREVIEW_TOPIC" >/dev/null; do
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

hub_latest_sequence() {
  local token payload
  token="$(<"$TOKEN_FILE")"
  payload="$(
    curl -fsS --max-time 5 -H "X-Robot-Token: $token" \
      "$HUB_URL/v1/robots/robot-0/observations/latest"
  )"
  unset token
  FOCUS_SEQUENCE_JSON="$payload" python3 -c \
    'import json,os; print(int(json.loads(os.environ["FOCUS_SEQUENCE_JSON"])["last_sequence"]))'
}

wait_for_hub_sequence_advance() {
  local baseline="$1" deadline token payload candidate
  latest_sequence="$baseline"
  deadline=$((SECONDS + SENDER_ADVANCE_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    token="$(<"$TOKEN_FILE")"
    payload="$(
      curl -fsS --max-time 5 -H "X-Robot-Token: $token" \
        "$HUB_URL/v1/robots/robot-0/observations/latest" \
        2>/dev/null || true
    )"
    unset token
    if [[ -n "$payload" ]]; then
      candidate="$(
        FOCUS_SEQUENCE_JSON="$payload" python3 -c \
          'import json,os; print(int(json.loads(os.environ["FOCUS_SEQUENCE_JSON"])["last_sequence"]))' \
          2>/dev/null || true
      )"
      if [[ "$candidate" =~ ^-?[0-9]+$ ]]; then
        latest_sequence="$candidate"
        (( latest_sequence > baseline )) && return 0
      fi
    fi
    sleep 1
  done
  return 1
}

stop_tracked_sender() {
  local deadline
  if tmux list-windows -t "$SESSION" -F '#{window_name}' \
      | grep -qx hub-sender; then
    tmux send-keys -t "$SESSION:hub-sender" C-c >/dev/null 2>&1 || true
  fi
  deadline=$((SECONDS + 8))
  while pgrep -af \
      'focus_ros_sender\.py.*--enable-command-capable-observations' \
      >/dev/null 2>&1; do
    (( SECONDS < deadline )) || break
    sleep 1
  done
  if tmux list-windows -t "$SESSION" -F '#{window_name}' \
      | grep -qx hub-sender; then
    tmux kill-window -t "$SESSION:hub-sender" >/dev/null 2>&1 || true
  fi
  deadline=$((SECONDS + 5))
  while pgrep -af \
      'focus_ros_sender\.py.*--enable-command-capable-observations' \
      >/dev/null 2>&1; do
    (( SECONDS < deadline )) || {
      echo "Tracked WSJ observation sender did not stop." >&2
      return 1
    }
    sleep 1
  done
}

launch_sender() {
  local stamp command_text deadline
  local -a command
  stamp="$(date -u +%Y%m%dT%H%M%S_%N)"
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
    --registration-min-coverage "$REGISTRATION_MIN_COVERAGE"
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
  tmux set-option -w -t "$SESSION:hub-sender" \
    @focus_deployment_commit "$DEPLOYMENT_COMMIT"

  deadline=$((SECONDS + 30))
  until pgrep -af \
      'focus_ros_sender\.py.*--enable-command-capable-observations' \
      >/dev/null 2>&1; do
    if [[ "$(tmux display-message -p -t "$SESSION:hub-sender" \
        '#{pane_dead}' 2>/dev/null || true)" == 1 ]]; then
      tmux capture-pane -pt "$SESSION:hub-sender" -S -80 >&2 || true
      return 1
    fi
    (( SECONDS < deadline )) || {
      echo "Timed out waiting for the WSJ command-capable sender." >&2
      return 1
    }
    sleep 1
  done
}

ensure_camera_preview
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
sender_processes="$(
  pgrep -af 'focus_ros_sender(_rgb)?\.py' 2>/dev/null || true
)"
sender_window="false"
if tmux list-windows -t "$SESSION" -F '#{window_name}' \
    | grep -qx hub-sender; then
  sender_window="true"
fi
if [[ -n "$sender_processes" ]] \
   && grep -Fv -- "--enable-command-capable-observations" \
     <<<"$sender_processes" >/dev/null; then
  echo "An incompatible WSJ observation sender is still running:" >&2
  printf '%s\n' "$sender_processes" >&2
  exit 1
fi
if [[ -n "$sender_processes" && "$sender_window" != true ]]; then
  echo "An untracked WSJ observation sender is still running." >&2
  exit 1
fi
if [[ -z "$sender_processes" && "$sender_window" == true ]]; then
  tmux kill-window -t "$SESSION:hub-sender" >/dev/null 2>&1 || true
  sender_window="false"
fi
if [[ "$sender_window" == true ]]; then
  sender_deployment_commit="$(
    tmux show-options -w -v -t "$SESSION:hub-sender" \
      @focus_deployment_commit 2>/dev/null || true
  )"
  if [[ "$sender_deployment_commit" != "$DEPLOYMENT_COMMIT" ]]; then
    echo "WSJ read-only sender belongs to deployment ${sender_deployment_commit:-unmarked}; reloading it once for $DEPLOYMENT_COMMIT."
    stop_tracked_sender
    sender_window="false"
    sender_processes="$(
      pgrep -af 'focus_ros_sender(_rgb)?\.py' 2>/dev/null || true
    )"
    [[ -z "$sender_processes" ]] || {
      echo "An old WSJ observation sender survived deployment reload." >&2
      printf '%s\n' "$sender_processes" >&2
      exit 1
    }
  fi
fi

initial_sequence="$(hub_latest_sequence)"
attempt_baseline="$initial_sequence"
latest_sequence="$initial_sequence"
metrics=""
log=""
sender_restarts=0
sender_ready="false"

if [[ "$sender_window" != true ]]; then
  launch_sender
fi

# Attempt zero observes the existing/new sender. Attempt one is the sole
# self-heal: restart only this read-only process, never camera/perception or a
# command component. A second failure is terminal and remains fail-closed.
for attempt in 0 1; do
  if wait_for_hub_sequence_advance "$attempt_baseline"; then
    sender_ready="true"
    break
  fi
  if [[ "$attempt" == 1 ]]; then
    current_sequence="$(hub_latest_sequence)"
    if (( current_sequence > attempt_baseline )); then
      latest_sequence="$current_sequence"
      sender_ready="true"
    fi
    break
  fi
  current_sequence="$(hub_latest_sequence)"
  if (( current_sequence > attempt_baseline )); then
    latest_sequence="$current_sequence"
    sender_ready="true"
    break
  fi
  echo "WSJ observation sequence did not advance from $attempt_baseline; restarting only the read-only sender once." >&2
  stop_tracked_sender
  attempt_baseline="$current_sequence"
  launch_sender
  sender_restarts=1
done

[[ "$sender_ready" == true ]] || {
  echo "WSJ observation sender failed to advance after one bounded read-only restart." >&2
  exit 1
}

echo "WSJ command-capable observation metadata is active (NO MOTION PATH)."
echo "  transform: $TRANSFORM_VERSION"
echo "  mount:     $BASE_CAMERA_CALIBRATION"
echo "  shared:    $SHARED_TRACKING_CALIBRATION"
echo "  deployment: $DEPLOYMENT_COMMIT"
echo "  Hub sequence: $initial_sequence -> $latest_sequence"
echo "  read-only sender restarts: $sender_restarts"
if [[ -n "$metrics" ]]; then
  echo "  metrics:   $metrics"
  echo "  log:       $log"
fi
