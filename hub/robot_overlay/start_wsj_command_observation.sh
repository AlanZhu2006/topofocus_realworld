#!/usr/bin/env bash
# Keep one WSJ ROS/DDS observation participant alive and atomically switch its
# checked metadata contract. This script never starts a planner or actuator.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${FOCUS_WSJ_NAV_SESSION:-tinynav_semantic_nav_auto}"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
PYTHON_BIN="${TINYNAV_PYTHON:-/home/nvidia/twork/tinynav/.venv/bin/python}"
TOKEN_FILE="${FOCUS_ROBOT_TOKEN_FILE:-/home/nvidia/focus_sender/.token}"
STATE_DIR="${FOCUS_ROBOT_STATE_DIR:-/home/nvidia/.local/state/topofocus}"
BASE_CAMERA_CALIBRATION="${FOCUS_WSJ_BASE_CAMERA_CALIBRATION:-${FOCUS_WSJ_BASE_CAMERA_CALIBRATION_FILE:-/home/nvidia/.local/state/topofocus/calibration/wsj_tinynav_camera_base_20260723_operator.json}}"
SHARED_TRACKING_CALIBRATION="${FOCUS_SHARED_CALIBRATION_FILE:-}"
SHARED_FRAME_CALIBRATION_ID="${FOCUS_SHARED_CALIBRATION_ID:-}"
TRANSFORM_VERSION="${FOCUS_WSJ_TRANSFORM_VERSION:-}"
GOAL_CATEGORY="${FOCUS_WSJ_GOAL_CATEGORY:-chair}"
HUB_URL="${FOCUS_HUB_BASE_URL:-http://127.0.0.1:18089}"
DEPLOYMENT_COMMIT="${FOCUS_DEPLOYMENT_COMMIT:-}"
SENDER_PROCESS_DEPLOYMENT_COMMIT=""
PREVIEW_URL="${FOCUS_FOXGLOVE_PREVIEW_URL:-http://127.0.0.1:18766}"
PREVIEW_WINDOW="${FOCUS_WSJ_PREVIEW_WINDOW:-foxglove-preview}"
RUNTIME_CONTRACT_FILE="${FOCUS_WSJ_RUNTIME_CONTRACT_FILE:-$STATE_DIR/wsj-command-observation-contract.json}"
RUNTIME_RECEIPT_FILE="${FOCUS_WSJ_RUNTIME_RECEIPT_FILE:-$STATE_DIR/wsj-command-observation-receipt.json}"
REANCHOR_REQUIRED_FILE="${FOCUS_WSJ_REANCHOR_REQUIRED_FILE:-$STATE_DIR/wsj-tracking-reanchor-required.json}"
COLOR_PREVIEW_TOPIC="/camera/camera/color/image_raw"
REGISTRATION_MIN_COVERAGE="${FOCUS_WSJ_REGISTRATION_MIN_COVERAGE:-0.38}"
RGB_CACHE_SIZE="${FOCUS_WSJ_RGB_CACHE_SIZE:-90}"
LATEST_RGB_MAX_SKEW_S="${FOCUS_WSJ_LATEST_RGB_MAX_SKEW_S:-0.05}"
POSE_TOPIC="${FOCUS_WSJ_OBSERVATION_POSE_TOPIC:-/slam/odometry_visual}"
POSE_FRAME="${FOCUS_WSJ_OBSERVATION_POSE_FRAME:-world}"
# A stale persistent DataReader can expose every publisher endpoint yet stop
# delivering synchronized tuples.  Twelve seconds covers the observed healthy
# cadence without spending most of a test window on a dead reader.  A fresh
# read-only participant must prove the complete RGB-D/pose tuple before only
# the sender is replaced; tracking publishers are never touched by that path.
SENDER_ADVANCE_TIMEOUT_S="${FOCUS_WSJ_SENDER_ADVANCE_TIMEOUT_S:-12}"
SYNC_PROBE_TIMEOUT_S="${FOCUS_WSJ_SYNC_PROBE_TIMEOUT_S:-12}"
FASTDDS_BUILTIN_TRANSPORTS_VALUE="${FOCUS_WSJ_FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
park_only=false

usage() {
  cat <<'EOF'
Usage: start_wsj_command_observation.sh [options]
  --park-only
  --session NAME
  --base-camera-calibration FILE
  --shared-tracking-calibration FILE
  --shared-frame-calibration-id ID
  --transform-version ID
  --goal-category CATEGORY
  --pose-topic /slam/odometry_visual|/focus/maploc/odometry_visual
  --pose-frame world|map
  --hub-url http://127.0.0.1:PORT
  --preview-url http://127.0.0.1:PORT

--park-only creates or preserves the long-lived DDS subscriber but removes its
upload contract. Run it before starting/restarting camera and perception
publishers. A later normal invocation validates and hot-loads calibration
without replacing that DDS participant.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --park-only) park_only=true; shift ;;
    --session) SESSION="$2"; shift 2 ;;
    --base-camera-calibration) BASE_CAMERA_CALIBRATION="$2"; shift 2 ;;
    --shared-tracking-calibration) SHARED_TRACKING_CALIBRATION="$2"; shift 2 ;;
    --shared-frame-calibration-id) SHARED_FRAME_CALIBRATION_ID="$2"; shift 2 ;;
    --transform-version) TRANSFORM_VERSION="$2"; shift 2 ;;
    --goal-category) GOAL_CATEGORY="$2"; shift 2 ;;
    --pose-topic) POSE_TOPIC="$2"; shift 2 ;;
    --pose-frame) POSE_FRAME="$2"; shift 2 ;;
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
case "$POSE_TOPIC:$POSE_FRAME" in
  /slam/odometry_visual:world|/focus/maploc/odometry_visual:map) ;;
  *)
    echo "Unsupported WSJ observation pose contract: $POSE_TOPIC frame=$POSE_FRAME" >&2
    exit 2
    ;;
esac
[[ "$DEPLOYMENT_COMMIT" =~ ^[0-9a-f]{40}$ ]] || {
  echo "FOCUS_DEPLOYMENT_COMMIT must be the explicit 40-character Git commit." >&2
  exit 2
}
[[ "$SENDER_ADVANCE_TIMEOUT_S" =~ ^[1-9][0-9]*$ ]] || {
  echo "FOCUS_WSJ_SENDER_ADVANCE_TIMEOUT_S must be a positive integer." >&2
  exit 2
}
[[ "$SYNC_PROBE_TIMEOUT_S" =~ ^[1-9][0-9]*$ ]] || {
  echo "FOCUS_WSJ_SYNC_PROBE_TIMEOUT_S must be a positive integer." >&2
  exit 2
}
[[ "$LATEST_RGB_MAX_SKEW_S" =~ ^0\.[0-9]*[1-9][0-9]*$ ]] || {
  echo "FOCUS_WSJ_LATEST_RGB_MAX_SKEW_S must be a positive subsecond decimal." >&2
  exit 2
}
[[ "$RGB_CACHE_SIZE" =~ ^[1-9][0-9]*$ ]] || {
  echo "FOCUS_WSJ_RGB_CACHE_SIZE must be a positive integer." >&2
  exit 2
}
[[ "$FASTDDS_BUILTIN_TRANSPORTS_VALUE" == UDPv4 ]] || {
  echo "WSJ observation transport must be the verified UDPv4 profile." >&2
  exit 2
}
for required in \
  "$SCRIPT_DIR/focus_ros_sender.py" \
  "$SCRIPT_DIR/probe_wsj_observation_sync.py" \
  "$SCRIPT_DIR/wsj_camera_preview.py" \
  "$SETUP_FILE" \
  "$PYTHON_BIN" \
  "$TOKEN_FILE"; do
  [[ -r "$required" ]] || {
    echo "Missing required observation input: $required" >&2
    exit 1
  }
done
if [[ "$park_only" != true ]]; then
  [[ "$TRANSFORM_VERSION" =~ ^[A-Za-z0-9_.-]+$ ]] || {
    echo "An explicit filesystem-safe transform version is required." >&2
    exit 2
  }
  [[ "$SHARED_FRAME_CALIBRATION_ID" =~ ^[A-Za-z0-9_.-]+$ ]] || {
    echo "An explicit filesystem-safe shared calibration ID is required." >&2
    exit 2
  }
  [[ "$GOAL_CATEGORY" =~ ^[A-Za-z0-9_.-]+$ ]] || {
    echo "A filesystem-safe goal category is required." >&2
    exit 2
  }
  for required in \
    "$BASE_CAMERA_CALIBRATION" "$SHARED_TRACKING_CALIBRATION"; do
    [[ -r "$required" ]] || {
      echo "Missing required checked calibration: $required" >&2
      exit 1
    }
  done
fi
tmux has-session -t "$SESSION" 2>/dev/null || {
  echo "WSJ camera/perception session is not running: $SESSION" >&2
  exit 1
}

PROCESS_CONTRACT_SHA256="$(
  {
    printf '%s\0' \
      "$HUB_URL" \
      "$RUNTIME_CONTRACT_FILE" \
      "$RUNTIME_RECEIPT_FILE" \
      "$REGISTRATION_MIN_COVERAGE" \
      "$RGB_CACHE_SIZE" \
      "$LATEST_RGB_MAX_SKEW_S" \
      "$POSE_TOPIC" \
      "$POSE_FRAME" \
      "$FASTDDS_BUILTIN_TRANSPORTS_VALUE"
    sha256sum "$SCRIPT_DIR/focus_ros_sender.py"
  } | sha256sum | awk '{print $1}'
)"

legacy_process_contract_sha256() {
  local process_deployment_commit="$1"
  {
    printf '%s\0' \
      "$process_deployment_commit" \
      "$HUB_URL" \
      "$RUNTIME_CONTRACT_FILE" \
      "$RUNTIME_RECEIPT_FILE" \
      "$REGISTRATION_MIN_COVERAGE" \
      "$RGB_CACHE_SIZE" \
      "$LATEST_RGB_MAX_SKEW_S" \
      "$FASTDDS_BUILTIN_TRANSPORTS_VALUE"
    sha256sum "$SCRIPT_DIR/focus_ros_sender.py"
  } | sha256sum | awk '{print $1}'
}

sender_process_rows() {
  local pid executable
  while IFS= read -r pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    executable="$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)"
    [[ "${executable##*/}" == python* ]] || continue
    ps -p "$pid" -o pid=,args=
  done < <(
    pgrep -f 'focus_ros_sender(_rgb)?\.py' 2>/dev/null || true
  )
}

sender_pid() {
  sender_process_rows \
    | grep -E -- 'focus_ros_sender\.py.*--runtime-command-contract-file' \
    | awk 'NR == 1 {print $1}'
}

stop_tracked_sender() {
  local deadline
  if tmux list-windows -t "$SESSION" -F '#{window_name}' \
      | grep -qx hub-sender; then
    tmux send-keys -t "$SESSION:hub-sender" C-c >/dev/null 2>&1 || true
  fi
  deadline=$((SECONDS + 8))
  while pgrep -af \
      'focus_ros_sender\.py.*(--runtime-command-contract-file|--enable-command-capable-observations)' \
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
      'focus_ros_sender\.py.*(--runtime-command-contract-file|--enable-command-capable-observations)' \
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
  metrics="$STATE_DIR/wsj-command-observation-${stamp}.json"
  log="$STATE_DIR/wsj-command-observation-${stamp}.log"
  command=(
    env "FASTDDS_BUILTIN_TRANSPORTS=$FASTDDS_BUILTIN_TRANSPORTS_VALUE"
    "$PYTHON_BIN" -u "$SCRIPT_DIR/focus_ros_sender.py"
    --base-url "$HUB_URL"
    --robot-id robot-0
    --transform-version wsj-runtime-parked-v1
    --goal-category "$GOAL_CATEGORY"
    --rgb-topic /camera/camera/color/image_raw
    --depth-topic /slam/depth
    --info-topic /slam/camera_info
    --pose-topic "$POSE_TOPIC"
    --camera-frame camera
    --register-rgb-to-depth
    --rgb-info-topic /camera/camera/color/camera_info
    --latest-rgb-for-depth
    --rgb-cache-size "$RGB_CACHE_SIZE"
    --latest-rgb-max-skew-s "$LATEST_RGB_MAX_SKEW_S"
    --rgb-optical-frame camera_color_optical_frame
    --depth-optical-frame camera_infra1_optical_frame
    --registration-min-coverage "$REGISTRATION_MIN_COVERAGE"
    --capture-time-source header
    --rate-hz 2.0
    --max-frames 0
    --metrics-out "$metrics"
    --heartbeat-hz 0
    --runtime-command-contract-file "$RUNTIME_CONTRACT_FILE"
    --runtime-command-receipt-file "$RUNTIME_RECEIPT_FILE"
    --deployment-commit "$DEPLOYMENT_COMMIT"
  )
  printf -v command_text '%q ' "${command[@]}"
  tmux new-window -d -t "$SESSION" -n hub-sender \
    "bash -lc 'source \"$SETUP_FILE\"; export FOCUS_ROBOT_TOKEN=\"\$(<\"$TOKEN_FILE\")\"; export PYTHONPATH=\"$SCRIPT_DIR/../src\":\${PYTHONPATH:-}; set -o pipefail; $command_text 2>&1 | tee \"$log\"'"
  tmux set-option -w -t "$SESSION:hub-sender" \
    @focus_deployment_commit "$DEPLOYMENT_COMMIT"
  tmux set-option -w -t "$SESSION:hub-sender" \
    @focus_sender_process_contract_sha256 "$PROCESS_CONTRACT_SHA256"
  tmux set-option -w -t "$SESSION:hub-sender" \
    @focus_fastrtps_builtin_transports "$FASTDDS_BUILTIN_TRANSPORTS_VALUE"
  SENDER_PROCESS_DEPLOYMENT_COMMIT="$DEPLOYMENT_COMMIT"
  deadline=$((SECONDS + 30))
  until [[ -n "$(sender_pid)" ]]; do
    if [[ "$(tmux display-message -p -t "$SESSION:hub-sender" \
        '#{pane_dead}' 2>/dev/null || true)" == 1 ]]; then
      tmux capture-pane -pt "$SESSION:hub-sender" -S -100 >&2 || true
      return 1
    fi
    (( SECONDS < deadline )) || {
      echo "Timed out waiting for the persistent WSJ DDS participant." >&2
      return 1
    }
    sleep 1
  done
}

ensure_sender_process() {
  local sender_window=false deployment process_contract legacy_contract processes
  local runtime_processes incompatible_processes runtime_count
  if tmux list-windows -t "$SESSION" -F '#{window_name}' \
      | grep -qx hub-sender; then
    sender_window=true
  fi
  processes="$(sender_process_rows)"
  runtime_processes="$(
    grep -E -- 'focus_ros_sender\.py.*--runtime-command-contract-file' \
      <<<"$processes" || true
  )"
  incompatible_processes="$(
    grep -Ev -- 'focus_ros_sender\.py.*--runtime-command-contract-file' \
      <<<"$processes" || true
  )"
  [[ -z "$incompatible_processes" ]] || {
    echo "An incompatible WSJ observation sender is running:" >&2
    printf '%s\n' "$incompatible_processes" >&2
    return 1
  }
  runtime_count=0
  if [[ -n "$runtime_processes" ]]; then
    runtime_count="$(wc -l <<<"$runtime_processes")"
  fi
  [[ "$runtime_count" -le 1 ]] || {
    echo "Multiple runtime-configurable WSJ senders are running:" >&2
    printf '%s\n' "$runtime_processes" >&2
    return 1
  }
  if [[ "$runtime_count" -eq 1 && "$sender_window" != true ]]; then
    echo "An untracked WSJ observation sender is running:" >&2
    printf '%s\n' "$runtime_processes" >&2
    return 1
  fi
  if [[ "$runtime_count" -eq 0 && "$sender_window" == true ]]; then
    tmux kill-window -t "$SESSION:hub-sender" >/dev/null 2>&1 || true
    sender_window=false
  fi
  if [[ "$sender_window" == true ]]; then
    deployment="$(
      tmux show-options -w -v -t "$SESSION:hub-sender" \
        @focus_deployment_commit 2>/dev/null || true
    )"
    process_contract="$(
      tmux show-options -w -v -t "$SESSION:hub-sender" \
        @focus_sender_process_contract_sha256 2>/dev/null || true
    )"
    legacy_contract="$(
      legacy_process_contract_sha256 "$deployment"
    )"
    if [[ ! "$deployment" =~ ^[0-9a-f]{40}$ \
          || ( "$process_contract" != "$PROCESS_CONTRACT_SHA256" \
               && ( "$POSE_TOPIC" != /slam/odometry_visual \
                    || "$process_contract" != "$legacy_contract" ) ) ]]; then
      echo "Replacing the WSJ sender once for a changed process contract; the verified UDPv4 subscriber must receive a fresh frame before use."
      stop_tracked_sender
      sender_window=false
    else
      # Older launchers included the repository commit in this process
      # identity. Adopt that exact running process once when its immutable
      # sender code and DDS arguments still match. Session-specific commit,
      # calibration and goal metadata are hot-loaded below.
      tmux set-option -w -t "$SESSION:hub-sender" \
        @focus_sender_process_contract_sha256 "$PROCESS_CONTRACT_SHA256"
      SENDER_PROCESS_DEPLOYMENT_COMMIT="$deployment"
    fi
  fi
  if [[ "$sender_window" != true ]]; then
    launch_sender
  fi
  [[ "$SENDER_PROCESS_DEPLOYMENT_COMMIT" =~ ^[0-9a-f]{40}$ ]] || {
    echo "WSJ sender process deployment identity is unavailable." >&2
    return 1
  }
}

write_parked_contract() {
  mkdir -p "$STATE_DIR"
  if [[ -e "$RUNTIME_CONTRACT_FILE" ]]; then
    unlink "$RUNTIME_CONTRACT_FILE"
  fi
}

write_active_contract() {
  FOCUS_CONTRACT_PATH="$RUNTIME_CONTRACT_FILE" \
  FOCUS_ROBOT_ID=robot-0 \
  FOCUS_CAMERA_FRAME=camera \
  FOCUS_PROCESS_DEPLOYMENT_COMMIT="$SENDER_PROCESS_DEPLOYMENT_COMMIT" \
  FOCUS_SESSION_DEPLOYMENT_COMMIT="$DEPLOYMENT_COMMIT" \
  FOCUS_TRANSFORM_VERSION="$TRANSFORM_VERSION" \
  FOCUS_CALIBRATION_ID="$SHARED_FRAME_CALIBRATION_ID" \
  FOCUS_GOAL_CATEGORY="$GOAL_CATEGORY" \
  FOCUS_POSE_TOPIC="$POSE_TOPIC" \
  FOCUS_POSE_FRAME="$POSE_FRAME" \
  FOCUS_BASE_CALIBRATION="$BASE_CAMERA_CALIBRATION" \
  FOCUS_SHARED_CALIBRATION="$SHARED_TRACKING_CALIBRATION" \
  FOCUS_RESOLVED_RESTART_BOOT_ID="$resolved_restart_boot_id" \
  "$PYTHON_BIN" - <<'PY'
import hashlib
import json
import os
from pathlib import Path

def artifact(raw):
    path = Path(raw).resolve()
    data = path.read_bytes()
    return {
        "path": str(path),
        "size_bytes": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
    }

destination = Path(os.environ["FOCUS_CONTRACT_PATH"])
destination.parent.mkdir(parents=True, exist_ok=True)
payload = {
    "schema_version": "focus-wsj-command-observation-contract-v1",
    "activation_confirmation": "COMMAND_CAPABLE_OBSERVATION_ONLY",
    "robot_id": os.environ["FOCUS_ROBOT_ID"],
    "camera_frame": os.environ["FOCUS_CAMERA_FRAME"],
    "deployment_commit": os.environ["FOCUS_PROCESS_DEPLOYMENT_COMMIT"],
    "session_deployment_commit": os.environ["FOCUS_SESSION_DEPLOYMENT_COMMIT"],
    "transform_version": os.environ["FOCUS_TRANSFORM_VERSION"],
    "shared_frame_calibration_id": os.environ["FOCUS_CALIBRATION_ID"],
    "goal_category": os.environ["FOCUS_GOAL_CATEGORY"],
    "pose_topic": os.environ["FOCUS_POSE_TOPIC"],
    "pose_frame": os.environ["FOCUS_POSE_FRAME"],
    "pose_source_status": (
        "observed_validated_saved_map_relocalization"
        if os.environ["FOCUS_POSE_FRAME"] == "map"
        else "observed_tracking_odometry"
    ),
    "base_camera_calibration": artifact(
        os.environ["FOCUS_BASE_CALIBRATION"]
    ),
    "shared_tracking_calibration": artifact(
        os.environ["FOCUS_SHARED_CALIBRATION"]
    ),
    "resolves_tracking_restart_boot_id": (
        os.environ["FOCUS_RESOLVED_RESTART_BOOT_ID"] or None
    ),
}
encoded = (json.dumps(payload, sort_keys=True) + "\n").encode()
temporary = destination.with_name(f".{destination.name}.tmp.{os.getpid()}")
temporary.write_bytes(encoded)
os.chmod(temporary, 0o600)
os.replace(temporary, destination)
print(hashlib.sha256(encoded).hexdigest())
PY
}

validate_reanchor_marker() {
  resolved_restart_boot_id=""
  [[ -e "$REANCHOR_REQUIRED_FILE" ]] || return 0
  [[ -r "$REANCHOR_REQUIRED_FILE" ]] || {
    echo "WSJ tracking-restart marker exists but is unreadable." >&2
    return 1
  }
  resolved_restart_boot_id="$(
    FOCUS_MARKER="$REANCHOR_REQUIRED_FILE" \
    FOCUS_CALIBRATION="$SHARED_TRACKING_CALIBRATION" \
    "$PYTHON_BIN" - <<'PY'
import json
import os
from pathlib import Path

marker = json.loads(Path(os.environ["FOCUS_MARKER"]).read_text())
calibration = json.loads(Path(os.environ["FOCUS_CALIBRATION"]).read_text())
boot_id = marker.get("tracking_restart_boot_id")
if not isinstance(boot_id, str) or not boot_id:
    raise SystemExit("tracking-restart marker has no boot ID")
if calibration.get("passed") is not True:
    raise SystemExit("shared calibration is not passed")

def contains(value):
    if value == boot_id:
        return True
    if isinstance(value, dict):
        return any(contains(item) for item in value.values())
    if isinstance(value, list):
        return any(contains(item) for item in value)
    return False

if not contains(calibration):
    raise SystemExit(
        "shared calibration does not resolve the pending tracking restart"
    )
print(boot_id)
PY
  )" || {
    echo "WSJ tracking was restarted; a matching validated stationary re-anchor or new board calibration is required." >&2
    return 1
  }
}

wait_for_receipt() {
  local expected_status="$1" expected_sha="$2" expected_pid deadline
  expected_pid="$(sender_pid)"
  deadline=$((SECONDS + 15))
  while (( SECONDS < deadline )); do
    if FOCUS_RECEIPT="$RUNTIME_RECEIPT_FILE" \
       FOCUS_EXPECT_STATUS="$expected_status" \
       FOCUS_EXPECT_SHA="$expected_sha" \
       FOCUS_EXPECT_PID="$expected_pid" \
       FOCUS_EXPECT_COMMIT="$SENDER_PROCESS_DEPLOYMENT_COMMIT" \
       "$PYTHON_BIN" - <<'PY' >/dev/null 2>&1
import json
import os
from pathlib import Path

payload = json.loads(Path(os.environ["FOCUS_RECEIPT"]).read_text())
if payload.get("status") != os.environ["FOCUS_EXPECT_STATUS"]:
    raise SystemExit(1)
expected_sha = os.environ["FOCUS_EXPECT_SHA"]
if payload.get("contract_sha256") != (expected_sha or None):
    raise SystemExit(1)
if int(payload.get("pid", -1)) != int(os.environ["FOCUS_EXPECT_PID"]):
    raise SystemExit(1)
if payload.get("deployment_commit") != os.environ["FOCUS_EXPECT_COMMIT"]:
    raise SystemExit(1)
PY
    then
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for WSJ sender receipt: $expected_status." >&2
  [[ ! -r "$RUNTIME_RECEIPT_FILE" ]] \
    || "$PYTHON_BIN" -m json.tool "$RUNTIME_RECEIPT_FILE" >&2
  return 1
}

active_receipt_frames_seen() {
  local expected_sha="$1" expected_pid
  expected_pid="$(sender_pid)"
  FOCUS_RECEIPT="$RUNTIME_RECEIPT_FILE" \
  FOCUS_EXPECT_SHA="$expected_sha" \
  FOCUS_EXPECT_PID="$expected_pid" \
  "$PYTHON_BIN" - <<'PY'
import json
import os
from pathlib import Path

payload = json.loads(Path(os.environ["FOCUS_RECEIPT"]).read_text())
if payload.get("status") != "active":
    raise SystemExit("WSJ sender receipt is not active")
if payload.get("contract_sha256") != os.environ["FOCUS_EXPECT_SHA"]:
    raise SystemExit("WSJ sender receipt contract mismatch")
if int(payload.get("pid", -1)) != int(os.environ["FOCUS_EXPECT_PID"]):
    raise SystemExit("WSJ sender receipt PID mismatch")
print(int(payload.get("frames_seen", -1)))
PY
}

sender_frame_error_since() {
  local baseline_frames="$1" expected_sha="$2" expected_pid
  expected_pid="$(sender_pid)"
  FOCUS_RECEIPT="$RUNTIME_RECEIPT_FILE" \
  FOCUS_EXPECT_SHA="$expected_sha" \
  FOCUS_EXPECT_PID="$expected_pid" \
  FOCUS_BASELINE_FRAMES="$baseline_frames" \
  "$PYTHON_BIN" - <<'PY'
import json
import os
from pathlib import Path

payload = json.loads(Path(os.environ["FOCUS_RECEIPT"]).read_text())
if payload.get("status") != "active":
    raise SystemExit(1)
if payload.get("contract_sha256") != os.environ["FOCUS_EXPECT_SHA"]:
    raise SystemExit(1)
if int(payload.get("pid", -1)) != int(os.environ["FOCUS_EXPECT_PID"]):
    raise SystemExit(1)
if int(payload.get("frames_seen", -1)) <= int(
    os.environ["FOCUS_BASELINE_FRAMES"]
):
    raise SystemExit(1)
error = payload.get("last_frame_error")
if not isinstance(error, str) or not error:
    raise SystemExit(1)
print(error)
PY
}

hub_latest_sequence() {
  local token payload
  token="$(<"$TOKEN_FILE")"
  payload="$(
    curl -fsS --max-time 5 -H "X-Robot-Token: $token" \
      "$HUB_URL/v1/robots/robot-0/observations/latest"
  )"
  unset token
  FOCUS_SEQUENCE_JSON="$payload" "$PYTHON_BIN" -c \
    'import json,os; print(int(json.loads(os.environ["FOCUS_SEQUENCE_JSON"])["last_sequence"]))'
}

wait_for_hub_sequence_advance() {
  local baseline="$1" deadline candidate
  latest_sequence="$baseline"
  deadline=$((SECONDS + SENDER_ADVANCE_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    candidate="$(hub_latest_sequence 2>/dev/null || true)"
    if [[ "$candidate" =~ ^-?[0-9]+$ ]]; then
      latest_sequence="$candidate"
      (( latest_sequence > baseline )) && return 0
    fi
    sleep 1
  done
  return 1
}

fresh_reader_can_assemble_observation() {
  env "FASTDDS_BUILTIN_TRANSPORTS=$FASTDDS_BUILTIN_TRANSPORTS_VALUE" \
    "$PYTHON_BIN" -u "$SCRIPT_DIR/probe_wsj_observation_sync.py" \
      --timeout-s "$SYNC_PROBE_TIMEOUT_S" \
      --minimum-tuples 3 \
      --sync-queue-size 50 \
      --sync-slop-s 0.05 \
      --rgb-cache-size "$RGB_CACHE_SIZE" \
      --latest-rgb-max-skew-s "$LATEST_RGB_MAX_SKEW_S" \
      --odometry-topic "$POSE_TOPIC"
}

recover_stale_sender_reader() {
  local old_pid new_pid baseline receipt_frames frame_error
  old_pid="$(sender_pid)"
  [[ "$old_pid" =~ ^[1-9][0-9]*$ ]] || {
    echo "Persistent WSJ sender PID is unavailable for reader recovery." >&2
    return 1
  }
  if pgrep -af 'go2_cmd_bridge|v2_wsj_receiver\.py' >/dev/null 2>&1; then
    echo "Refusing sender reader recovery while a command path exists." >&2
    return 1
  fi

  # Remove command-capable metadata before replacing the observation-only
  # process.  No camera/perception publisher or tracking epoch is touched.
  write_parked_contract
  wait_for_receipt parked ""
  stop_tracked_sender
  launch_sender
  new_pid="$(sender_pid)"
  [[ "$new_pid" =~ ^[1-9][0-9]*$ && "$new_pid" != "$old_pid" ]] || {
    echo "WSJ sender reader was not replaced by a new process." >&2
    return 1
  }
  wait_for_receipt parked ""

  baseline="$(hub_latest_sequence)"
  contract_sha256="$(write_active_contract)"
  wait_for_receipt active "$contract_sha256"
  receipt_frames="$(active_receipt_frames_seen "$contract_sha256")"
  if ! wait_for_hub_sequence_advance "$baseline"; then
    if frame_error="$(
      sender_frame_error_since "$receipt_frames" "$contract_sha256"
    )"; then
      echo "WSJ sender receives synchronized tuples, but Hub rejected frame processing: $frame_error" >&2
      echo "Activate a Hub session with the same transform/calibration contract before retrying the sender." >&2
      return 1
    fi
    echo "Fresh WSJ sender reader did not advance after a passing sync probe." >&2
    return 1
  fi
  initial_sequence="$baseline"
  echo "WSJ_STALE_SENDER_READER_RECOVERED:$old_pid->$new_pid"
  echo "  tracking publishers restarted: false"
  echo "  robot commands issued: false"
}

ensure_camera_preview() {
  local preview_log deadline preview_processes preview_transport
  preview_processes="$(
    pgrep -af '[w]sj_camera_preview\.py' 2>/dev/null || true
  )"
  preview_transport="$(
    tmux show-options -w -v -t "$SESSION:$PREVIEW_WINDOW" \
      @focus_fastrtps_builtin_transports 2>/dev/null || true
  )"
  if [[ -n "$preview_processes" ]] \
     && ! grep -Fv -- "--rgb-topic $COLOR_PREVIEW_TOPIC" \
       <<<"$preview_processes" >/dev/null \
     && [[ "$preview_transport" == "$FASTDDS_BUILTIN_TRANSPORTS_VALUE" ]]; then
    return 0
  fi
  tmux kill-window -t "$SESSION:$PREVIEW_WINDOW" >/dev/null 2>&1 || true
  sleep 1
  preview_processes="$(
    pgrep -af '[w]sj_camera_preview\.py' 2>/dev/null || true
  )"
  [[ -z "$preview_processes" ]] || {
    echo "An untracked non-color WSJ preview is still running." >&2
    return 1
  }
  preview_log="$STATE_DIR/wsj-camera-preview-$(date -u +%Y%m%dT%H%M%SZ).log"
  tmux new-window -d -t "$SESSION" -n "$PREVIEW_WINDOW" \
    "bash -lc 'source \"$SETUP_FILE\"; export FASTDDS_BUILTIN_TRANSPORTS=\"$FASTDDS_BUILTIN_TRANSPORTS_VALUE\"; export FOCUS_ROBOT_TOKEN=\"\$(<\"$TOKEN_FILE\")\"; exec \"$PYTHON_BIN\" -u \"$SCRIPT_DIR/wsj_camera_preview.py\" --relay-url \"$PREVIEW_URL\" --name wsj --rgb-topic \"$COLOR_PREVIEW_TOPIC\" --max-rate-hz 5 2>&1 | tee \"$preview_log\"'"
  tmux set-option -w -t "$SESSION:$PREVIEW_WINDOW" \
    @focus_fastrtps_builtin_transports "$FASTDDS_BUILTIN_TRANSPORTS_VALUE"
  deadline=$((SECONDS + 20))
  until pgrep -af '[w]sj_camera_preview\.py' \
      | grep -F -- "--rgb-topic $COLOR_PREVIEW_TOPIC" >/dev/null; do
    (( SECONDS < deadline )) || {
      echo "Timed out waiting for WSJ Foxglove camera preview." >&2
      return 1
    }
    sleep 1
  done
}

if pgrep -af 'go2_cmd_bridge' >/dev/null 2>&1; then
  echo "Refusing to change observation metadata while a Go2 bridge is active." >&2
  exit 1
fi

# Remove only the two known pre-v2 observation windows. Unknown senders remain
# a hard error in ensure_sender_process rather than being killed by pattern.
for legacy_session in focus_wsj_camera_preview_20260723 focus_wsj_mapping; do
  if tmux has-session -t "$legacy_session" 2>/dev/null; then
    for legacy_window in sender sender_rgb; do
      tmux kill-window -t "$legacy_session:$legacy_window" \
        >/dev/null 2>&1 || true
    done
  fi
done

if [[ "$park_only" == true ]]; then
  write_parked_contract
  ensure_sender_process
  wait_for_receipt parked ""
  echo "WSJ_DDS_UDP_PARTICIPANT_PARKED"
  echo "  process contract: $PROCESS_CONTRACT_SHA256"
  echo "  robot commands issued: false"
  exit 0
fi

# Stop the temporary native-infra1 calibration uploader before activating the
# persistent color sender; otherwise both could claim the same sequence.
if tmux list-windows -t "$SESSION" -F '#{window_name}' \
    | grep -qx calibration-sender; then
  tmux send-keys -t "$SESSION:calibration-sender" C-c >/dev/null 2>&1 || true
  deadline=$((SECONDS + 10))
  while pgrep -af \
      'focus_ros_sender\.py.*--rgb-topic /camera/camera/infra1/image_rect_raw' \
      >/dev/null 2>&1; do
    (( SECONDS < deadline )) || break
    sleep 1
  done
  tmux kill-window -t "$SESSION:calibration-sender" >/dev/null 2>&1 || true
  if pgrep -af \
      'focus_ros_sender\.py.*--rgb-topic /camera/camera/infra1/image_rect_raw' \
      >/dev/null 2>&1; then
    echo "Calibration sender did not stop; runtime activation refused." >&2
    exit 1
  fi
fi
ensure_camera_preview
ensure_sender_process
validate_reanchor_marker
initial_sequence="$(hub_latest_sequence)"
contract_sha256="$(write_active_contract)"
wait_for_receipt active "$contract_sha256"
receipt_frames="$(active_receipt_frames_seen "$contract_sha256")"
if ! wait_for_hub_sequence_advance "$initial_sequence"; then
  if frame_error="$(
    sender_frame_error_since "$receipt_frames" "$contract_sha256"
  )"; then
    echo "WSJ sender receives synchronized tuples, but Hub rejected frame processing: $frame_error" >&2
    echo "Activate a Hub session with the same transform/calibration contract before retrying the sender." >&2
    exit 1
  fi
  echo "WSJ persistent sender did not advance; probing the publisher tuple with a fresh read-only participant." >&2
  if fresh_reader_can_assemble_observation; then
    echo "Fresh reader proved the publishers healthy; replacing only the stale observation sender." >&2
    recover_stale_sender_reader
  else
    echo "WSJ sender and fresh sync probe both received no complete observation tuple." >&2
    echo "Preserving the DDS participant; do not restart it. If publisher recovery is authorized, restart camera/perception only after this sender exists." >&2
    exit 1
  fi
fi
if [[ -n "$resolved_restart_boot_id" \
      && -e "$REANCHOR_REQUIRED_FILE" ]]; then
  unlink "$REANCHOR_REQUIRED_FILE"
fi

echo "WSJ command-capable observations activated without a DDS restart."
echo "  transform: $TRANSFORM_VERSION"
echo "  calibration: $SHARED_TRACKING_CALIBRATION"
echo "  deployment: $DEPLOYMENT_COMMIT"
echo "  sender process deployment: $SENDER_PROCESS_DEPLOYMENT_COMMIT"
echo "  Hub sequence: $initial_sequence -> $latest_sequence"
echo "  process contract: $PROCESS_CONTRACT_SHA256"
echo "  runtime contract: $contract_sha256"
echo "  Fast DDS transport: $FASTDDS_BUILTIN_TRANSPORTS_VALUE"
echo "  robot commands issued: false"
