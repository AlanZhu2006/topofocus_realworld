#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
SESSION_NAME="${TINYNAV_SEMANTIC_NAV_SESSION:-tinynav_semantic_nav_auto}"
SAVE_SERVICE="${TINYNAV_SEMANTIC_SAVE_SERVICE:-/semantic_mapping/save_map}"
SEMANTIC_SAVE_SERVICE="${TINYNAV_SEMANTIC_VOXEL_SAVE_SERVICE:-/semantic_mapping/save_semantic_map}"
checkpoint_timeout_s=30
service_discovery_timeout_s="${TINYNAV_SEMANTIC_SERVICE_DISCOVERY_TIMEOUT:-5}"
checkpoint="true"

usage() {
  cat <<EOF
Usage: $0 [--session NAME] [--timeout SEC] [--no-checkpoint]

Checkpoints the live semantic occupancy map, then stops the copied TinyNav
semantic navigation tmux session. It does not modify or stop original TinyNav
sessions with other names.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session) SESSION_NAME="$2"; shift 2 ;;
    --timeout) checkpoint_timeout_s="$2"; shift 2 ;;
    --no-checkpoint) checkpoint="false"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 1 ;;
  esac
done

if ! [[ "$checkpoint_timeout_s" =~ ^[0-9]+$ ]] || (( checkpoint_timeout_s <= 0 )); then
  echo "--timeout must be a positive integer" >&2
  exit 1
fi
if ! [[ "$service_discovery_timeout_s" =~ ^[0-9]+$ ]] \
  || (( service_discovery_timeout_s <= 0 )); then
  echo "TINYNAV_SEMANTIC_SERVICE_DISCOVERY_TIMEOUT must be a positive integer" >&2
  exit 1
fi

if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  echo "Semantic navigation session is not running: $SESSION_NAME"
  exit 0
fi

set +u
source "$SETUP_FILE"
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

checkpoint_service() {
  local service_name="$1"
  local map_name="$2"
  local attempts=$((service_discovery_timeout_s * 2))

  for ((attempt = 0; attempt < attempts; attempt++)); do
    if ros2 service list 2>/dev/null | grep -qx "$service_name"; then
      echo "Checkpointing $map_name through $service_name ..."
      if ! timeout "$checkpoint_timeout_s" ros2 service call \
        "$service_name" std_srvs/srv/Trigger "{}"; then
        echo "Warning: $map_name checkpoint failed or timed out." >&2
      fi
      return
    fi
    sleep 0.5
  done

  echo "Warning: checkpoint service is unavailable: $service_name" >&2
}

if [[ "$checkpoint" == "true" ]]; then
  checkpoint_service "$SAVE_SERVICE" "occupancy map"
  checkpoint_service "$SEMANTIC_SAVE_SERVICE" "semantic voxel map"
fi

if tmux list-windows -t "$SESSION_NAME" -F '#{window_name}' | grep -qx occupancy-map; then
  tmux send-keys -t "$SESSION_NAME:occupancy-map" C-c
  for ((attempt = 0; attempt < checkpoint_timeout_s * 2; attempt++)); do
    if ! tmux list-windows -t "$SESSION_NAME" -F '#{window_name}' \
      | grep -qx occupancy-map; then
      break
    fi
    sleep 0.5
  done
fi

tmux kill-session -t "$SESSION_NAME"
echo "Stopped semantic navigation session: $SESSION_NAME"
