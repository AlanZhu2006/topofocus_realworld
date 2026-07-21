#!/usr/bin/env bash
# Stops the live rehearsal session started by run_live_rehearsal.sh and
# removes its throwaway temp directories. Never touches TinyNav's own map,
# bag or output directories — only the rehearsal_* scratch dirs this script's
# companion created under this overlay directory.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION_NAME="${FOCUS_REHEARSAL_SESSION:-focus_live_rehearsal}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session) SESSION_NAME="$2"; shift 2 ;;
    -h|--help) echo "Usage: $0 [--session NAME]"; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 1 ;;
  esac
done

echo "Stopping tmux session: $SESSION_NAME"
tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || echo "  (session was not running)"

sleep 1
echo "Cleaning rehearsal scratch directories under $SCRIPT_DIR..."
for dir in "$SCRIPT_DIR"/rehearsal_tinynav_db_* "$SCRIPT_DIR"/rehearsal_semantic_output_*; do
  if [[ -e "$dir" ]]; then
    rm -rf --one-file-system "$dir"
    echo "  removed: $dir"
  fi
done

echo "Done. TinyNav's own map/bag/output directories were never touched."
