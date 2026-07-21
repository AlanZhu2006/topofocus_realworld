#!/usr/bin/env bash
# Stops the tmux session started by focus_hub_up.sh and confirms the hub
# API, GLM server and pipeline daemon actually released their ports/GPU
# memory. Never deletes runtime/spool, runtime/state or the token files —
# those are the hub's durable data and secrets, not scratch output.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HUB_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SESSION_NAME="${FOCUS_HUB_SESSION:-focus_hub}"
port="8088"
glm_port="31511"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session) SESSION_NAME="$2"; shift 2 ;;
    --port) port="$2"; shift 2 ;;
    --glm-port) glm_port="$2"; shift 2 ;;
    -h|--help) echo "Usage: $0 [--session NAME] [--port N] [--glm-port N]"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 1 ;;
  esac
done

echo "Stopping tmux session: $SESSION_NAME"
if tmux has-session -t "$SESSION_NAME" >/dev/null 2>&1; then
  for window in hub glm pipeline; do
    tmux send-keys -t "$SESSION_NAME:$window" C-c >/dev/null 2>&1 || true
  done
  sleep 2
  tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
else
  echo "  (session was not running)"
fi

wait_for_port_free() {
  local port="$1" timeout_s="${2:-15}" start
  start="$(date +%s)"
  while ss -tln 2>/dev/null | grep -q ":$port "; do
    if (( $(date +%s) - start >= timeout_s )); then
      echo "  WARNING: port $port is still listening after ${timeout_s}s" >&2
      return 1
    fi
    sleep 1
  done
}

echo "Checking ports released..."
wait_for_port_free "$port" 15 && echo "  hub port $port: free"
wait_for_port_free "$glm_port" 15 && echo "  glm port $glm_port: free"

if command -v nvidia-smi >/dev/null 2>&1; then
  echo "GPU memory:"
  nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader | sed 's/^/  /'
fi

echo
echo "Done. Data preserved: $HUB_DIR/runtime/{spool,state,map_out,tokens.json,admin_token}"
