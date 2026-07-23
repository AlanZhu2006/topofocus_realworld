#!/usr/bin/env bash
# Operator-present, observation-only WSJ startup for a fresh mapping session.
# Starts D435i + patched TinyNav perception, the mapping-only Hub sender and
# Foxglove preview. It structurally never starts planner/control/Unitree code.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
env_file=""
session="focus_wsj_mapping"
session_from_cli="false"
transform_version=""
base_url="http://127.0.0.1:18089"
preview_url="http://127.0.0.1:18766"
robot_id="robot-0"
rate_hz="2.0"

usage() {
  cat <<'EOF'
Usage: bash start_wsj_mapping_session.sh --env FILE \
  --transform-version UNIQUE_NEW_SESSION_ID [--session NAME]

Operator presence is mandatory because this starts the physical D435i and
TinyNav perception. It starts no planner, cmd_vel controller, Unitree bridge,
Hub decision receiver or other actuation path.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --env) env_file="$2"; shift 2 ;;
    --session) session="$2"; session_from_cli="true"; shift 2 ;;
    --transform-version) transform_version="$2"; shift 2 ;;
    --base-url) base_url="$2"; shift 2 ;;
    --preview-url) preview_url="$2"; shift 2 ;;
    --rate-hz) rate_hz="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$env_file" || ! -f "$env_file" ]]; then
  echo "A real deployment --env file is required." >&2
  exit 2
fi
if [[ -z "$transform_version" || "$transform_version" == *20260722* ]]; then
  echo "Use a unique post-reboot transform version; do not reuse a July 22 session ID." >&2
  exit 2
fi
for file in "$SCRIPT_DIR/.token" "$SCRIPT_DIR/focus_ros_sender.py" \
            "$SCRIPT_DIR/wsj_camera_preview.py" "$SCRIPT_DIR/start_go2_observation.sh"; do
  [[ -f "$file" ]] || { echo "Missing deployment file: $file" >&2; exit 2; }
done
if pgrep -af 'go2_cmd_bridge|cmd_vel_control|planning_node.py|nav2_controller' >/dev/null 2>&1; then
  echo "Refusing to start while an actuation/planner process exists." >&2
  exit 1
fi

set -a
source "$env_file"
set +a
if [[ "$session_from_cli" != "true" ]]; then
  session="${FOCUS_REHEARSAL_SESSION:-$session}"
fi
setup_file="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
python_bin="${TINYNAV_PYTHON:-/home/nvidia/twork/tinynav/.venv/bin/python}"
state_dir="${FOCUS_ROBOT_STATE_DIR:-$HOME/.local/state/topofocus}"
[[ -x "$python_bin" ]] || {
  echo "Missing TinyNav Python: $python_bin" >&2
  exit 2
}
mkdir -p "$state_dir"

curl -fsS --max-time 5 "$base_url/healthz" >/dev/null
curl -fsS --max-time 5 "$preview_url/healthz" >/dev/null
token="$(<"$SCRIPT_DIR/.token")"
initial_json="$(curl -fsS --max-time 5 -H "X-Robot-Token: $token" \
  "$base_url/v1/robots/$robot_id/observations/latest")"
initial_sequence="$(FOCUS_SEQUENCE_JSON="$initial_json" python3 -c '
import json, os
print(int(json.loads(os.environ["FOCUS_SEQUENCE_JSON"])["last_sequence"]))
')"
unset token initial_json

launch_complete="false"
cleanup_on_error() {
  if [[ "$launch_complete" != "true" ]]; then
    tmux kill-session -t "$session" >/dev/null 2>&1 || true
  fi
}
trap cleanup_on_error EXIT

bash "$SCRIPT_DIR/start_go2_observation.sh" --env "$env_file" --session "$session"
source_setup="source '$setup_file'"
stamp="$(date -u +%Y%m%dT%H%M%SZ)"
sender_log="$state_dir/wsj-sender-$stamp.log"
preview_log="$state_dir/wsj-preview-$stamp.log"

tmux new-window -d -t "$session" -n sender \
  "bash -lc \"$source_setup; cd '$SCRIPT_DIR'; set -o pipefail; FOCUS_ROBOT_TOKEN=\$(cat .token) '$python_bin' -u focus_ros_sender.py --base-url '$base_url' --robot-id '$robot_id' --transform-version '$transform_version' --rgb-topic /camera/camera/color/image_raw --depth-topic /slam/keyframe_depth --info-topic /slam/camera_info --pose-topic /slam/keyframe_odom --camera-frame camera --register-rgb-to-depth --rgb-info-topic /camera/camera/color/camera_info --rgb-optical-frame camera_color_optical_frame --depth-optical-frame camera_infra1_optical_frame --registration-min-coverage 0.45 --capture-time-source header --rate-hz '$rate_hz' --max-frames 0 --metrics-out '$state_dir/wsj-sender-$stamp-metrics.json' 2>&1 | tee '$sender_log'\""

tmux new-window -d -t "$session" -n preview \
  "bash -lc \"$source_setup; cd '$SCRIPT_DIR'; set -o pipefail; FOCUS_ROBOT_TOKEN=\$(cat .token) python3 -u wsj_camera_preview.py --relay-url '$preview_url' --name wsj --rgb-topic /camera/camera/color/image_raw --max-rate-hz 5 2>&1 | tee '$preview_log'\""

deadline=$((SECONDS + 90))
latest_sequence="$initial_sequence"
while (( SECONDS < deadline )); do
  token="$(<"$SCRIPT_DIR/.token")"
  latest_json="$(curl -fsS --max-time 5 -H "X-Robot-Token: $token" \
    "$base_url/v1/robots/$robot_id/observations/latest" || true)"
  unset token
  if [[ -n "$latest_json" ]]; then
    latest_sequence="$(FOCUS_SEQUENCE_JSON="$latest_json" python3 -c '
import json, os
print(int(json.loads(os.environ["FOCUS_SEQUENCE_JSON"])["last_sequence"]))
')"
    if (( latest_sequence > initial_sequence )); then
      break
    fi
  fi
  sleep 1
done
if (( latest_sequence <= initial_sequence )); then
  echo "No fresh Hub observation arrived within 90 seconds." >&2
  exit 1
fi

launch_complete="true"
trap - EXIT
echo "WSJ mapping-only session is live: $session"
echo "  transform_version: $transform_version"
echo "  first observed Hub advance: $initial_sequence -> $latest_sequence"
echo "  sender log:  $sender_log"
echo "  preview log: $preview_log"
echo "No planner, command receiver, cmd_vel or Unitree bridge was started."
