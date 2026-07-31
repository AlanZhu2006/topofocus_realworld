#!/usr/bin/env bash
# Recover WSJ camera/perception publishers only after the persistent DDS
# observation participant exists. This creates a new tracking epoch and writes
# a fail-closed marker that only a matching re-anchor/new board calibration can
# resolve. It never starts a planner, receiver, or chassis bridge.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${FOCUS_WSJ_NAV_SESSION:-tinynav_semantic_nav_auto}"
SETUP_FILE="${TINYNAV_SETUP:-$HOME/twork/tinynav_setup.bash}"
PYTHON_BIN="${TINYNAV_PYTHON:-$HOME/twork/tinynav/.venv/bin/python}"
default_token_file="${XDG_CONFIG_HOME:-$HOME/.config}/topofocus/robot-0.token"
[[ -r "$default_token_file" ]] || default_token_file="$HOME/focus_sender/.token"
TOKEN_FILE="${FOCUS_ROBOT_TOKEN_FILE:-$default_token_file}"
HUB_URL="${FOCUS_HUB_BASE_URL:-http://127.0.0.1:18089}"
STATE_DIR="${FOCUS_ROBOT_STATE_DIR:-${XDG_STATE_HOME:-$HOME/.local/state}/topofocus}"
DEPLOYMENT_COMMIT="${FOCUS_DEPLOYMENT_COMMIT:-}"
MARKER_FILE="${FOCUS_WSJ_REANCHOR_REQUIRED_FILE:-$STATE_DIR/wsj-tracking-reanchor-required.json}"
RECEIPT_FILE="${FOCUS_WSJ_RUNTIME_RECEIPT_FILE:-$STATE_DIR/wsj-command-observation-receipt.json}"
CAPTURE_RECEIPT_FILE="${FOCUS_WSJ_REANCHOR_CAPTURE_RECEIPT_FILE:-$STATE_DIR/wsj-stationary-reanchor-capture.json}"
CONFIRMATION=""
PERCEPTION_ONLY=false
REANCHOR_TRANSFORM_VERSION=""
REANCHOR_SAMPLE_COUNT="${FOCUS_WSJ_REANCHOR_SAMPLE_COUNT:-5}"
REANCHOR_SOAK_COUNT="${FOCUS_WSJ_REANCHOR_SOAK_COUNT:-10}"
FASTDDS_BUILTIN_TRANSPORTS_VALUE="${FOCUS_WSJ_FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

usage() {
  cat <<'EOF'
Usage: recover_wsj_publishers_after_sender.sh \
  --operator-confirmation OPERATOR_PRESENT_AND_WSJ_STATIONARY \
  [--perception-only] \
  [--capture-stationary-reanchor TRANSFORM_VERSION]

The sender must already be the runtime-configurable hub-sender. The script
stops perception, restarts camera, then restarts perception. It leaves all
motion paths closed and emits a tracking boot ID that must be represented by a
validated stationary re-anchor (or superseded by a new board calibration).

--perception-only preserves a currently healthy camera publisher and restarts
only TinyNav perception. It still creates a new tracking epoch and therefore
retains the same mandatory stationary re-anchor gate.

--capture-stationary-reanchor is valid only with --perception-only. It creates
a mapping-only evidence subscriber before perception is touched, retains five
stable old-epoch samples, then retains five new-epoch samples after a bounded
soak. The temporary sender has no planner, receiver, or actuator output and is
left running until the validated re-anchor is atomically activated.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --operator-confirmation) CONFIRMATION="$2"; shift 2 ;;
    --perception-only) PERCEPTION_ONLY=true; shift ;;
    --capture-stationary-reanchor)
      REANCHOR_TRANSFORM_VERSION="$2"
      shift 2
      ;;
    --session) SESSION="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$CONFIRMATION" == OPERATOR_PRESENT_AND_WSJ_STATIONARY ]] || {
  echo "Publisher recovery requires OPERATOR_PRESENT_AND_WSJ_STATIONARY." >&2
  exit 2
}
[[ "$DEPLOYMENT_COMMIT" =~ ^[0-9a-f]{40}$ ]] || {
  echo "FOCUS_DEPLOYMENT_COMMIT must be the explicit deployed Git commit." >&2
  exit 2
}
[[ "$FASTDDS_BUILTIN_TRANSPORTS_VALUE" == UDPv4 ]] || {
  echo "WSJ recovery transport must be the verified UDPv4 profile." >&2
  exit 2
}
if [[ -n "$REANCHOR_TRANSFORM_VERSION" ]]; then
  [[ "$PERCEPTION_ONLY" == true ]] || {
    echo "--capture-stationary-reanchor requires --perception-only." >&2
    exit 2
  }
  [[ "$REANCHOR_TRANSFORM_VERSION" =~ ^[A-Za-z0-9_.-]+$ ]] || {
    echo "Stationary re-anchor transform version is not filesystem-safe." >&2
    exit 2
  }
  [[ "$HUB_URL" =~ ^http://127\.0\.0\.1:[0-9]+$ ]] || {
    echo "Stationary re-anchor Hub URL must remain loopback-only." >&2
    exit 2
  }
  [[ "$REANCHOR_SAMPLE_COUNT" =~ ^[0-9]+$ ]] \
    && (( REANCHOR_SAMPLE_COUNT >= 3 )) || {
    echo "FOCUS_WSJ_REANCHOR_SAMPLE_COUNT must be an integer >= 3." >&2
    exit 2
  }
  [[ "$REANCHOR_SOAK_COUNT" =~ ^[1-9][0-9]*$ ]] \
    && (( REANCHOR_SOAK_COUNT >= REANCHOR_SAMPLE_COUNT )) || {
      echo "FOCUS_WSJ_REANCHOR_SOAK_COUNT must be >= the sample count." >&2
      exit 2
    }
fi
export FASTDDS_BUILTIN_TRANSPORTS="$FASTDDS_BUILTIN_TRANSPORTS_VALUE"
for required in "$SETUP_FILE" "$SCRIPT_DIR/start_wsj_command_observation.sh"; do
  [[ -r "$required" ]] || {
    echo "Missing WSJ recovery input: $required" >&2
    exit 1
  }
done
if [[ -n "$REANCHOR_TRANSFORM_VERSION" ]]; then
  for required in \
    "$PYTHON_BIN" "$TOKEN_FILE" "$SCRIPT_DIR/focus_ros_sender.py"; do
    [[ -r "$required" ]] || {
      echo "Missing stationary re-anchor capture input: $required" >&2
      exit 1
    }
  done
fi
tmux has-session -t "$SESSION" 2>/dev/null || {
  echo "Missing WSJ session: $SESSION" >&2
  exit 1
}
for window in camera perception hub-sender; do
  tmux list-windows -t "$SESSION" -F '#{window_name}' \
    | grep -qx "$window" || {
      echo "Missing required WSJ window: $window" >&2
      exit 1
    }
done

source "$SETUP_FILE"
timeout 5 ros2 topic pub --once \
  /nav/paused std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true
timeout 5 ros2 topic pub --once \
  /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' \
  >/dev/null 2>&1 || true
tmux kill-window -t "$SESSION:go2-bridge" >/dev/null 2>&1 || true
tmux kill-window -t "$SESSION:v2-receiver" >/dev/null 2>&1 || true
if pgrep -af 'go2_cmd_bridge|v2_wsj_receiver\.py' >/dev/null 2>&1; then
  echo "A WSJ motion path survived fail-closed cleanup." >&2
  exit 1
fi

# Remove the upload contract and prove the long-lived subscriber is parked
# before changing any publisher. This call may create/upgrade the sender, but
# it never starts a planner, receiver, or bridge.
FOCUS_DEPLOYMENT_COMMIT="$DEPLOYMENT_COMMIT" \
bash "$SCRIPT_DIR/start_wsj_command_observation.sh" \
  --session "$SESSION" \
  --park-only

runtime_sender_pids() {
  local pid executable command_line
  while IFS= read -r pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    executable="$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)"
    [[ "${executable##*/}" == python* ]] || continue
    command_line="$(
      tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true
    )"
    [[ "$command_line" == *"$SCRIPT_DIR/focus_ros_sender.py"* \
       && "$command_line" == *"--runtime-command-contract-file"* ]] \
      || continue
    printf '%s\n' "$pid"
  done < <(
    pgrep -f 'focus_ros_sender\.py.*--runtime-command-contract-file' \
      2>/dev/null || true
  )
}

sender_pids="$(runtime_sender_pids)"
[[ "$(wc -w <<<"$sender_pids")" -eq 1 ]] || {
  echo "Expected exactly one persistent runtime-configurable WSJ sender." >&2
  [[ -z "$sender_pids" ]] || printf '%s\n' "$sender_pids" >&2
  exit 1
}
sender_pid="$(
  printf '%s\n' "$sender_pids"
)"
[[ -n "$sender_pid" ]] || {
  echo "Persistent runtime-configurable WSJ sender is not running." >&2
  exit 1
}
sender_deployment="$(
  tmux show-options -w -v -t "$SESSION:hub-sender" \
    @focus_deployment_commit 2>/dev/null || true
)"
[[ "$sender_deployment" =~ ^[0-9a-f]{40}$ ]] || {
  echo "WSJ sender process deployment identity is unavailable." >&2
  exit 1
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

capture_sender_pid() {
  local pid executable command_line
  while IFS= read -r pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    executable="$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)"
    [[ "${executable##*/}" == python* ]] || continue
    command_line="$(
      tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true
    )"
    [[ "$command_line" == *"$SCRIPT_DIR/focus_ros_sender.py"* \
       && "$command_line" == *"--rgb-topic /camera/camera/infra1/image_rect_raw"* \
       && "$command_line" != *"--runtime-command-contract-file"* ]] \
      || continue
    printf '%s\n' "$pid"
  done < <(
    pgrep -f \
      'focus_ros_sender\.py.*--rgb-topic /camera/camera/infra1/image_rect_raw' \
      2>/dev/null || true
  )
}

wait_for_hub_sequence_at_least() {
  local minimum="$1" deadline candidate
  latest_capture_sequence=-1
  deadline=$((SECONDS + 45))
  while (( SECONDS < deadline )); do
    candidate="$(hub_latest_sequence 2>/dev/null || true)"
    if [[ "$candidate" =~ ^-?[0-9]+$ ]]; then
      latest_capture_sequence="$candidate"
      (( latest_capture_sequence >= minimum )) && return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for stationary re-anchor evidence sequence $minimum." >&2
  return 1
}

start_stationary_reanchor_capture() {
  local stamp sender_text deadline
  local -a sender
  if tmux list-windows -t "$SESSION" -F '#{window_name}' \
      | grep -qx calibration-sender; then
    echo "A calibration sender already exists; refusing ambiguous re-anchor capture." >&2
    return 1
  fi
  if [[ -n "$(capture_sender_pid)" ]]; then
    echo "An untracked calibration sender already exists." >&2
    return 1
  fi
  capture_initial_sequence="$(hub_latest_sequence)"
  stamp="$(date -u +%Y%m%dT%H%M%SZ)"
  capture_log="$STATE_DIR/wsj-stationary-reanchor-sender-$stamp.log"
  capture_metrics="$STATE_DIR/wsj-stationary-reanchor-sender-$stamp.json"
  sender=(
    env "FASTDDS_BUILTIN_TRANSPORTS=$FASTDDS_BUILTIN_TRANSPORTS_VALUE"
    "$PYTHON_BIN" -u "$SCRIPT_DIR/focus_ros_sender.py"
    --base-url "$HUB_URL"
    --robot-id robot-0
    --transform-version "$REANCHOR_TRANSFORM_VERSION"
    --rgb-topic /camera/camera/infra1/image_rect_raw
    --depth-topic /slam/depth
    --info-topic /slam/camera_info
    --pose-topic /slam/odometry_visual
    --camera-frame camera
    --capture-time-source header
    --rate-hz 2.0
    --max-frames 0
    --metrics-out "$capture_metrics"
  )
  printf -v sender_text '%q ' "${sender[@]}"
  tmux new-window -d -t "$SESSION" -n calibration-sender \
    "bash -lc 'source \"$SETUP_FILE\"; export FOCUS_ROBOT_TOKEN=\"\$(<\"$TOKEN_FILE\")\"; export PYTHONPATH=\"$SCRIPT_DIR/../src\":\${PYTHONPATH:-}; set -o pipefail; $sender_text 2>&1 | tee \"$capture_log\"'"
  tmux set-option -w -t "$SESSION:calibration-sender" \
    @focus_stationary_reanchor_transform_version \
    "$REANCHOR_TRANSFORM_VERSION"
  deadline=$((SECONDS + 20))
  until [[ -n "$(capture_sender_pid)" ]]; do
    if [[ "$(tmux display-message -p -t "$SESSION:calibration-sender" \
        '#{pane_dead}' 2>/dev/null || true)" == 1 ]]; then
      tmux capture-pane -pt "$SESSION:calibration-sender" -S -100 >&2 || true
      return 1
    fi
    (( SECONDS < deadline )) || {
      echo "Timed out prewarming the stationary re-anchor subscriber." >&2
      return 1
    }
    sleep 1
  done
  reanchor_capture_sender_pid="$(capture_sender_pid)"
  [[ "$(wc -w <<<"$reanchor_capture_sender_pid")" -eq 1 ]] || {
    echo "Expected exactly one stationary re-anchor capture sender." >&2
    return 1
  }
  wait_for_hub_sequence_at_least \
    "$((capture_initial_sequence + REANCHOR_SAMPLE_COUNT))"
  echo "WSJ_STATIONARY_REANCHOR_SUBSCRIBER_PREWARMED:$reanchor_capture_sender_pid"
}

fresh_topic_once() {
  timeout -k 2 15 ros2 topic echo --once \
    --field header \
    --qos-reliability best_effort \
    --qos-durability volatile \
    --qos-depth 1 \
    "$1" >/dev/null 2>&1
}

wait_for_topic() {
  local topic="$1" deadline
  deadline=$((SECONDS + 75))
  until fresh_topic_once "$topic"; do
    (( SECONDS < deadline )) || {
      echo "Timed out waiting for $topic after ordered publisher recovery." >&2
      return 1
    }
    sleep 1
  done
}

parked_tuple_count() {
  FOCUS_RECEIPT="$RECEIPT_FILE" \
  FOCUS_EXPECT_PID="$sender_pid" \
  python3 - <<'PY'
import json
import os
from pathlib import Path

payload = json.loads(Path(os.environ["FOCUS_RECEIPT"]).read_text())
if payload.get("status") != "parked":
    raise SystemExit("WSJ sender is not parked")
if int(payload.get("pid", -1)) != int(os.environ["FOCUS_EXPECT_PID"]):
    raise SystemExit("WSJ sender receipt PID changed")
print(int(payload.get("parked_geometry_tuples", -1)))
PY
}

wait_for_sender_tuple_advance() {
  local baseline="$1" deadline current current_pid
  deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    current_pid="$(runtime_sender_pids)"
    [[ "$current_pid" == "$sender_pid" ]] || {
      echo "WSJ DDS sender PID changed during publisher recovery." >&2
      return 1
    }
    current="$(parked_tuple_count 2>/dev/null || true)"
    if [[ "$current" =~ ^[0-9]+$ ]] && (( current > baseline )); then
      return 0
    fi
    sleep 1
  done
  echo "The preserved WSJ sender received no post-restart synchronized tuple." >&2
  return 1
}

write_reanchor_marker() {
  local status="$1"
  FOCUS_MARKER="$MARKER_FILE" \
  FOCUS_BOOT_ID="$boot_id" \
  FOCUS_DEPLOYMENT_COMMIT="$DEPLOYMENT_COMMIT" \
  FOCUS_SENDER_DEPLOYMENT_COMMIT="$sender_deployment" \
  FOCUS_SENDER_PID="$sender_pid" \
  FOCUS_RECOVERY_STATUS="$status" \
  FOCUS_PERCEPTION_ONLY="$PERCEPTION_ONLY" \
  FOCUS_REANCHOR_TRANSFORM_VERSION="$REANCHOR_TRANSFORM_VERSION" \
  FOCUS_REANCHOR_PRE_FIRST="${reanchor_pre_first:-}" \
  FOCUS_REANCHOR_PRE_LAST="${reanchor_pre_last:-}" \
  FOCUS_REANCHOR_POST_FIRST="${reanchor_post_first:-}" \
  FOCUS_REANCHOR_POST_LAST="${reanchor_post_last:-}" \
  python3 - <<'PY'
import json
import os
from pathlib import Path
import time

destination = Path(os.environ["FOCUS_MARKER"])
destination.parent.mkdir(parents=True, exist_ok=True)
status = os.environ["FOCUS_RECOVERY_STATUS"]
perception_only = os.environ["FOCUS_PERCEPTION_ONLY"] == "true"
publisher_order = [
    "persistent_sender_parked",
    "old_perception_stopped",
]
if not perception_only:
    publisher_order.append("camera_restarted")
publisher_order.append("perception_restarted")
payload = {
    "schema_version": "focus-wsj-tracking-reanchor-required-v1",
    "tracking_restart_boot_id": os.environ["FOCUS_BOOT_ID"],
    "deployment_commit": os.environ["FOCUS_DEPLOYMENT_COMMIT"],
    "sender_process_deployment_commit": os.environ[
        "FOCUS_SENDER_DEPLOYMENT_COMMIT"
    ],
    "sender_pid_preserved": int(os.environ["FOCUS_SENDER_PID"]),
    "recovery_status": status,
    "recovery_scope": (
        "perception_only" if perception_only else "camera_and_perception"
    ),
    "camera_preserved": perception_only,
    "publisher_order": publisher_order,
    "publisher_order_complete": status == "publishers_recovered",
    "operator_confirmation": "OPERATOR_PRESENT_AND_WSJ_STATIONARY",
    "classification": (
        "observed_ordered_read_only_publisher_restart_reanchor_required"
        if status == "publishers_recovered"
        else "source_derived_recovery_started_reanchor_required"
    ),
    "robot_commands_issued": False,
    "written_at_ns": time.time_ns(),
}
transform_version = os.environ["FOCUS_REANCHOR_TRANSFORM_VERSION"]
if transform_version:
    payload["stationary_reanchor_capture"] = {
        "transform_version": transform_version,
        "pre_first": (
            int(os.environ["FOCUS_REANCHOR_PRE_FIRST"])
            if os.environ["FOCUS_REANCHOR_PRE_FIRST"]
            else None
        ),
        "pre_last": (
            int(os.environ["FOCUS_REANCHOR_PRE_LAST"])
            if os.environ["FOCUS_REANCHOR_PRE_LAST"]
            else None
        ),
        "post_first": (
            int(os.environ["FOCUS_REANCHOR_POST_FIRST"])
            if os.environ["FOCUS_REANCHOR_POST_FIRST"]
            else None
        ),
        "post_last": (
            int(os.environ["FOCUS_REANCHOR_POST_LAST"])
            if os.environ["FOCUS_REANCHOR_POST_LAST"]
            else None
        ),
    }
temporary = destination.with_name(f".{destination.name}.tmp.{os.getpid()}")
temporary.write_text(
    json.dumps(payload, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
os.chmod(temporary, 0o600)
os.replace(temporary, destination)
PY
}

write_capture_receipt() {
  FOCUS_CAPTURE_RECEIPT="$CAPTURE_RECEIPT_FILE" \
  FOCUS_BOOT_ID="$boot_id" \
  FOCUS_DEPLOYMENT_COMMIT="$DEPLOYMENT_COMMIT" \
  FOCUS_CAPTURE_SENDER_PID="$reanchor_capture_sender_pid" \
  FOCUS_TRANSFORM_VERSION="$REANCHOR_TRANSFORM_VERSION" \
  FOCUS_PRE_FIRST="$reanchor_pre_first" \
  FOCUS_PRE_LAST="$reanchor_pre_last" \
  FOCUS_POST_FIRST="$reanchor_post_first" \
  FOCUS_POST_LAST="$reanchor_post_last" \
  "$PYTHON_BIN" - <<'PY'
import json
import os
from pathlib import Path
import time

destination = Path(os.environ["FOCUS_CAPTURE_RECEIPT"])
payload = {
    "schema_version": "focus-wsj-stationary-reanchor-capture-v1",
    "tracking_restart_boot_id": os.environ["FOCUS_BOOT_ID"],
    "deployment_commit": os.environ["FOCUS_DEPLOYMENT_COMMIT"],
    "robot_id": "robot-0",
    "transform_version": os.environ["FOCUS_TRANSFORM_VERSION"],
    "capture_sender_pid": int(os.environ["FOCUS_CAPTURE_SENDER_PID"]),
    "pre_restart_observations": {
        "first_sequence": int(os.environ["FOCUS_PRE_FIRST"]),
        "last_sequence": int(os.environ["FOCUS_PRE_LAST"]),
    },
    "post_restart_observations": {
        "first_sequence": int(os.environ["FOCUS_POST_FIRST"]),
        "last_sequence": int(os.environ["FOCUS_POST_LAST"]),
    },
    "camera_preserved": True,
    "perception_restarted": True,
    "publisher_order": [
        "persistent_sender_parked",
        "stationary_reanchor_subscriber_started",
        "old_perception_stopped",
        "perception_restarted",
    ],
    "classification": (
        "observed_stationary_pre_and_post_tracking_epoch_pose_evidence"
    ),
    "robot_commands_issued": False,
    "written_at_ns": time.time_ns(),
}
temporary = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
temporary.write_text(
    json.dumps(payload, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
os.chmod(temporary, 0o600)
os.replace(temporary, destination)
PY
}

reanchor_capture_sender_pid=""
reanchor_pre_first=""
reanchor_pre_last=""
reanchor_post_first=""
reanchor_post_last=""
if [[ -n "$REANCHOR_TRANSFORM_VERSION" ]]; then
  start_stationary_reanchor_capture
fi

if [[ "$PERCEPTION_ONLY" == true ]]; then
  boot_id="wsj-perception-$(date -u +%Y%m%dT%H%M%S)_${RANDOM}"
else
  boot_id="wsj-camera-perception-$(date -u +%Y%m%dT%H%M%S)_${RANDOM}"
fi
# Write the fail-closed marker before the first publisher is touched. If any
# later step fails, old calibration cannot silently become command-capable.
write_reanchor_marker recovery_started

# Stop the downstream publisher first, while the sender remains alive.
tmux set-option -w -t "$SESSION:perception" remain-on-exit on
if [[ "$(tmux display-message -p -t "$SESSION:perception" \
    '#{pane_dead}' 2>/dev/null || true)" == 0 ]]; then
  tmux send-keys -t "$SESSION:perception" C-c
  deadline=$((SECONDS + 20))
  until [[ "$(tmux display-message -p -t "$SESSION:perception" \
      '#{pane_dead}' 2>/dev/null || true)" == 1 ]]; do
    (( SECONDS < deadline )) || {
      echo "Old perception publisher did not stop; camera was not restarted." >&2
      exit 1
    }
    sleep 1
  done
fi

# Drain any already-queued old-epoch callback before taking the receipt
# baseline. A later increase must therefore come from the restarted publishers.
sleep 2
if [[ -n "$REANCHOR_TRANSFORM_VERSION" ]]; then
  reanchor_pre_last="$(hub_latest_sequence)"
  (( reanchor_pre_last >= capture_initial_sequence + REANCHOR_SAMPLE_COUNT )) || {
    echo "Insufficient stable pre-restart evidence." >&2
    exit 1
  }
  reanchor_pre_first="$((reanchor_pre_last - REANCHOR_SAMPLE_COUNT + 1))"
fi
parked_tuple_baseline="$(parked_tuple_count)"
if [[ "$PERCEPTION_ONLY" != true ]]; then
  tmux respawn-pane -k -t "$SESSION:camera"
  wait_for_topic /camera/camera/infra1/image_rect_raw
fi
tmux respawn-pane -k -t "$SESSION:perception"
wait_for_topic /slam/depth
wait_for_topic /slam/odometry_visual
wait_for_sender_tuple_advance "$parked_tuple_baseline"
if [[ -n "$REANCHOR_TRANSFORM_VERSION" ]]; then
  wait_for_hub_sequence_at_least \
    "$((reanchor_pre_last + REANCHOR_SOAK_COUNT))"
  reanchor_post_last="$latest_capture_sequence"
  reanchor_post_first="$((reanchor_post_last - REANCHOR_SAMPLE_COUNT + 1))"
  write_capture_receipt
fi

write_reanchor_marker publishers_recovered

echo "WSJ_PUBLISHERS_RECOVERED_AFTER_PERSISTENT_SENDER:$boot_id"
echo "REANCHOR_REQUIRED_MARKER:$MARKER_FILE"
if [[ -n "$REANCHOR_TRANSFORM_VERSION" ]]; then
  echo "STATIONARY_REANCHOR_CAPTURE_RECEIPT:$CAPTURE_RECEIPT_FILE"
  echo "STATIONARY_REANCHOR_PRE_RANGE:$reanchor_pre_first:$reanchor_pre_last"
  echo "STATIONARY_REANCHOR_POST_RANGE:$reanchor_post_first:$reanchor_post_last"
fi
echo "Safety: navigation paused; receiver and chassis bridge are absent."
