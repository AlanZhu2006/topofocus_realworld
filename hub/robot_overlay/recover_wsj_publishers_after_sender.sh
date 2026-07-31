#!/usr/bin/env bash
# Recover WSJ camera/perception publishers only after the persistent DDS
# observation participant exists. This creates a new tracking epoch and writes
# a fail-closed marker that only a matching re-anchor/new board calibration can
# resolve. It never starts a planner, receiver, or chassis bridge.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${FOCUS_WSJ_NAV_SESSION:-tinynav_semantic_nav_auto}"
SETUP_FILE="${TINYNAV_SETUP:-$HOME/twork/tinynav_setup.bash}"
STATE_DIR="${FOCUS_ROBOT_STATE_DIR:-${XDG_STATE_HOME:-$HOME/.local/state}/topofocus}"
DEPLOYMENT_COMMIT="${FOCUS_DEPLOYMENT_COMMIT:-}"
MARKER_FILE="${FOCUS_WSJ_REANCHOR_REQUIRED_FILE:-$STATE_DIR/wsj-tracking-reanchor-required.json}"
RECEIPT_FILE="${FOCUS_WSJ_RUNTIME_RECEIPT_FILE:-$STATE_DIR/wsj-command-observation-receipt.json}"
CONFIRMATION=""
PERCEPTION_ONLY=false
FASTDDS_BUILTIN_TRANSPORTS_VALUE="${FOCUS_WSJ_FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

usage() {
  cat <<'EOF'
Usage: recover_wsj_publishers_after_sender.sh \
  --operator-confirmation OPERATOR_PRESENT_AND_WSJ_STATIONARY \
  [--perception-only]

The sender must already be the runtime-configurable hub-sender. The script
stops perception, restarts camera, then restarts perception. It leaves all
motion paths closed and emits a tracking boot ID that must be represented by a
validated stationary re-anchor (or superseded by a new board calibration).

--perception-only preserves a currently healthy camera publisher and restarts
only TinyNav perception. It still creates a new tracking epoch and therefore
retains the same mandatory stationary re-anchor gate.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --operator-confirmation) CONFIRMATION="$2"; shift 2 ;;
    --perception-only) PERCEPTION_ONLY=true; shift ;;
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
export FASTDDS_BUILTIN_TRANSPORTS="$FASTDDS_BUILTIN_TRANSPORTS_VALUE"
[[ -r "$SETUP_FILE" ]] || {
  echo "Missing TinyNav setup: $SETUP_FILE" >&2
  exit 1
}
[[ -r "$SCRIPT_DIR/start_wsj_command_observation.sh" ]] || {
  echo "Missing persistent WSJ sender launcher." >&2
  exit 1
}
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
temporary = destination.with_name(f".{destination.name}.tmp.{os.getpid()}")
temporary.write_text(
    json.dumps(payload, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
os.chmod(temporary, 0o600)
os.replace(temporary, destination)
PY
}

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
sleep 1
parked_tuple_baseline="$(parked_tuple_count)"
if [[ "$PERCEPTION_ONLY" != true ]]; then
  tmux respawn-pane -k -t "$SESSION:camera"
  wait_for_topic /camera/camera/infra1/image_rect_raw
fi
tmux respawn-pane -k -t "$SESSION:perception"
wait_for_topic /slam/depth
wait_for_topic /slam/odometry_visual
wait_for_sender_tuple_advance "$parked_tuple_baseline"

write_reanchor_marker publishers_recovered

echo "WSJ_PUBLISHERS_RECOVERED_AFTER_PERSISTENT_SENDER:$boot_id"
echo "REANCHOR_REQUIRED_MARKER:$MARKER_FILE"
echo "Safety: navigation paused; receiver and chassis bridge are absent."
