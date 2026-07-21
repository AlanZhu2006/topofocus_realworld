#!/usr/bin/env bash
# Stop the observation-only tmux session without deleting maps or recordings.
set -euo pipefail

session="${FOCUS_REHEARSAL_SESSION:-focus_live_rehearsal}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --session) session="$2"; shift 2 ;;
    -h|--help) echo "Usage: $0 [--session NAME]"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

if pgrep -af 'run_tinynav_buildmap_live.py' >/dev/null 2>&1; then
  echo "A BuildMap save wrapper is active. Publish /benchmark/stop and wait for" >&2
  echo "/benchmark/data_saved=true before stopping the tmux session." >&2
  exit 1
fi

if tmux has-session -t "$session" 2>/dev/null; then
  tmux kill-session -t "$session"
  echo "Stopped observation session: $session"
else
  echo "Observation session was not running: $session"
fi
echo "No map or recording was deleted."
