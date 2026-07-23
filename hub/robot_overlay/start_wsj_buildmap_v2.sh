#!/usr/bin/env bash
# Start the minimal WSJ BuildMap/v2 stack in debug or explicitly armed live mode.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${FOCUS_WSJ_NAV_SESSION:-tinynav_semantic_nav_auto}"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
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
mode="debug"
confirmation=""

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

if tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -qx v2-receiver; then
  if [[ "$mode" == debug ]] \
     && ! tmux list-windows -t "$SESSION" -F '#{window_name}' \
        | grep -qx go2-bridge \
     && pgrep -af 'v2_wsj_receiver\.py' >/dev/null 2>&1 \
     && ! pgrep -af 'v2_wsj_receiver\.py.*--enable-live-go2-motion' \
        >/dev/null 2>&1; then
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

if [[ "$mode" == live ]]; then
  tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -qx go2-bridge && {
    echo "Refusing to replace existing Go2 bridge window." >&2
    exit 1
  }
  tmux new-window -d -t "$SESSION" -n go2-bridge \
    "bash -lc 'set -o pipefail; export GO2_CMD_TOPIC=/focus_guarded_cmd_vel GO2_MAX_VX=0.20 GO2_MAX_VY=0.00 GO2_MAX_WZ=0.50 GO2_MIN_CMD_V=0.15 GO2_MIN_CMD_W=0.30 GO2_REMOTE_PRIORITY=true GO2_LOG_COMMANDS=true GO2_LOG_INTERVAL_SEC=0.2; bash /home/nvidia/twork/tinynav/scripts/run_go2_cmd_bridge.sh 2>&1 | tee \"$bridge_log\"'"
  echo "WSJ Go2 bridge command log: $bridge_log"
fi

echo "WSJ v2 BuildMap stack ready: mode=$mode alignment=$alignment"
if [[ "$mode" == debug ]]; then
  echo "Safety: no Go2 bridge; physical motion is impossible through this stack."
fi
