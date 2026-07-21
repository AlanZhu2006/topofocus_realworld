#!/usr/bin/env bash
# Request native BuildMap persistence and wait for its positive acknowledgement.
set -euo pipefail

session="${FOCUS_REHEARSAL_SESSION:-focus_live_rehearsal}"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session) session="$2"; shift 2 ;;
    -h|--help) echo "Usage: $0 [--session NAME]"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

source "$SETUP_FILE"
ros2 topic list | grep -qx '/benchmark/stop' || {
  echo "No active BuildMap /benchmark/stop topic." >&2
  exit 1
}

ack_file="$(mktemp /tmp/topofocus-buildmap-ack.XXXXXX)"
watch_pid=""
cleanup() {
  [[ -z "$watch_pid" ]] || kill "$watch_pid" >/dev/null 2>&1 || true
  rm -f -- "$ack_file"
}
trap cleanup EXIT

timeout 180 ros2 topic echo --once /benchmark/data_saved >"$ack_file" &
watch_pid=$!
sleep 1
ros2 topic pub --once /benchmark/stop std_msgs/msg/Bool '{data: true}' >/dev/null
wait "$watch_pid"
watch_pid=""

grep -Eq '^data: true$' "$ack_file" || {
  echo "BuildMap did not acknowledge data_saved=true." >&2
  cat "$ack_file" >&2
  exit 1
}

deadline=$((SECONDS + 30))
while tmux list-windows -t "$session" -F '#{window_name}' 2>/dev/null | grep -qx maploc; do
  (( SECONDS < deadline )) || {
    echo "Save acknowledged, but maploc has not exited; inspect its log." >&2
    exit 1
  }
  sleep 1
done

echo "BuildMap acknowledged data_saved=true and exited cleanly."
echo "Camera/perception remain running; stop them separately when finished."
