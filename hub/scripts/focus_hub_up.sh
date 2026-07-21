#!/usr/bin/env bash
# One-click hub-side startup, styled after TinyNav's own
# tinynav_semantic_auto_nav.sh (tmux session, health-check waits, usage
# banner, clean stop companion) so both ends of the system start the same way.
#
# Starts (in tmux session '$SESSION_NAME'):
#   glm         GLM-4V-9B offline decision server (skip with --no-glm or
#               point at an already-running one with --glm-url)
#   hub         focus_hub FastAPI transport/registry (auth, spool, decisions)
#   pipeline    incremental RedNet mapping + periodic GLM frontier decision
#               daemon (skip with --no-pipeline for hub-API-only testing)
#   monitor     GPU memory / hub health / spool size at a glance
#
# Safety default preserved: whatever hub/config/robots.json (or
# --robots-config) says is what ships — this script never flips
# allow_goal itself. Ships with allow_goal=false for both robots.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HUB_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$(cd "$HUB_DIR/.." && pwd)"
PYTHON_BIN="${FOCUS_HUB_PYTHON:-$HUB_DIR/.venv/bin/python}"
SESSION_NAME="${FOCUS_HUB_SESSION:-focus_hub}"

host="127.0.0.1"
port="8088"
glm_port="31511"
glm_url=""
start_glm="true"
start_pipeline="true"
robots_config="$HUB_DIR/config/robots.local.json"
[[ -f "$robots_config" ]] || robots_config="$HUB_DIR/config/robots.json"
tokens_file="$HUB_DIR/runtime/tokens.json"
spool_dir="$HUB_DIR/runtime/spool"
state_dir="$HUB_DIR/runtime/state"
decision_interval="60"
goal_category="chair"

usage() {
  cat <<EOF
Usage: $0 [--port N] [--glm-port N] [--glm-url URL] [--no-glm] [--no-pipeline]
          [--robots-config FILE] [--tokens-file FILE] [--decision-interval S]
          [--goal-category NAME] [--session NAME]

Robot tokens: read from --tokens-file (JSON object {robot_id: token}); if the
file is missing, a fresh random token per robot in --robots-config is
generated, saved there (chmod 600) and PRINTED ONCE — copy it to the robot's
FOCUS_ROBOT_TOKEN before deploying a sender.

Stop:
  bash $SCRIPT_DIR/focus_hub_down.sh --session $SESSION_NAME
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) port="$2"; shift 2 ;;
    --glm-port) glm_port="$2"; shift 2 ;;
    --glm-url) glm_url="$2"; start_glm="false"; shift 2 ;;
    --no-glm) start_glm="false"; shift ;;
    --no-pipeline) start_pipeline="false"; shift ;;
    --robots-config) robots_config="$2"; shift 2 ;;
    --tokens-file) tokens_file="$2"; shift 2 ;;
    --decision-interval) decision_interval="$2"; shift 2 ;;
    --goal-category) goal_category="$2"; shift 2 ;;
    --session) SESSION_NAME="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 1 ;;
  esac
done

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Python environment missing: $PYTHON_BIN (run hub/scripts/create_g1_env.sh first)" >&2
  exit 1
fi
if [[ ! -f "$robots_config" ]]; then
  echo "Robot config not found: $robots_config" >&2
  exit 1
fi

mkdir -p "$HUB_DIR/runtime" "$spool_dir" "$state_dir"

# --- token bootstrap ---------------------------------------------------
if [[ ! -f "$tokens_file" ]]; then
  echo "No token file at $tokens_file; generating fresh per-robot tokens."
  "$PYTHON_BIN" - "$robots_config" "$tokens_file" <<'PYEOF'
import json, secrets, sys
robots_config, tokens_file = sys.argv[1], sys.argv[2]
robots = json.load(open(robots_config))["robots"]
tokens = {robot_id: secrets.token_hex(24) for robot_id in robots}
with open(tokens_file, "w") as f:
    json.dump(tokens, f, indent=2)
import os
os.chmod(tokens_file, 0o600)
print("Generated tokens (copy each to the matching robot's FOCUS_ROBOT_TOKEN):")
for robot_id, token in tokens.items():
    print(f"  {robot_id}: {token}")
PYEOF
fi
compact_tokens_file="$state_dir/.robot_tokens_compact.json"
"$PYTHON_BIN" -c "import json; json.dump(json.load(open('$tokens_file')), open('$compact_tokens_file', 'w'))"
chmod 600 "$compact_tokens_file"

if [[ ! -f "$HUB_DIR/runtime/admin_token" ]]; then
  "$PYTHON_BIN" -c "import secrets; print(secrets.token_hex(24))" > "$HUB_DIR/runtime/admin_token"
  chmod 600 "$HUB_DIR/runtime/admin_token"
  echo "Generated admin token: $HUB_DIR/runtime/admin_token (chmod 600)"
fi
admin_token="$(cat "$HUB_DIR/runtime/admin_token")"

wait_for_http() {
  local url="$1" timeout_s="${2:-60}" start
  start="$(date +%s)"
  until curl -s -o /dev/null -w '' "$url" 2>/dev/null; do
    if (( $(date +%s) - start >= timeout_s )); then
      echo "Timed out waiting for $url" >&2
      return 1
    fi
    sleep 1
  done
}

tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
tmux new-session -d -s "$SESSION_NAME" -n hub \
  "bash -lc 'cd \"$WORKSPACE\" && export FOCUS_HUB_ROBOT_CONFIG=\"$robots_config\" && export FOCUS_HUB_ROBOT_TOKENS_JSON=\"\$(cat \"$compact_tokens_file\")\" && export FOCUS_HUB_ADMIN_TOKEN=\"$admin_token\" && export FOCUS_HUB_SPOOL_DIR=\"$spool_dir\" && export FOCUS_HUB_STATE_DIR=\"$state_dir\" && \"$PYTHON_BIN\" -m uvicorn focus_hub.api:app --host \"$host\" --port \"$port\" 2>&1 | tee \"$HUB_DIR/runtime/hub.log\"'"
tmux set-option -g remain-on-exit on

echo "Waiting for hub API on $host:$port..."
wait_for_http "http://$host:$port/healthz" 30

if [[ "$start_glm" == "true" ]]; then
  tmux new-window -t "$SESSION_NAME" -n glm \
    "bash -lc 'cd \"$WORKSPACE\" && FOCUS_GLM_PORT=\"$glm_port\" bash hub/scripts/run_glm_offline.sh 2>&1 | tee \"$HUB_DIR/runtime/glm.log\"'"
  glm_url="http://127.0.0.1:$glm_port/v1"
  echo "Waiting for GLM-4V on port $glm_port (model load takes ~20-30s)..."
  wait_for_http "$glm_url/models" 180
fi

if [[ "$start_pipeline" == "true" ]]; then
  pipeline_args=(--spool "$spool_dir" --hub-url "http://$host:$port" \
    --admin-token-file "$HUB_DIR/runtime/admin_token" \
    --decision-interval "$decision_interval" --goal-category "$goal_category" \
    --log "$HUB_DIR/runtime/pipeline.jsonl" --out-dir "$HUB_DIR/runtime/map_out")
  [[ -n "$glm_url" ]] && pipeline_args+=(--glm-url "$glm_url")
  printf -v pipeline_args_str '%q ' "${pipeline_args[@]}"
  tmux new-window -t "$SESSION_NAME" -n pipeline \
    "bash -lc 'cd \"$WORKSPACE\" && \"$PYTHON_BIN\" hub/tools/hub_pipeline_daemon.py $pipeline_args_str 2>&1 | tee \"$HUB_DIR/runtime/pipeline_window.log\"'"
fi

tmux new-window -t "$SESSION_NAME" -n monitor \
  "bash -lc 'watch -n 3 \"curl -s http://$host:$port/healthz; echo; echo GPU:; nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader 2>/dev/null; echo; echo spool:; find \\\"$spool_dir\\\" -mindepth 2 -maxdepth 2 -type d 2>/dev/null | wc -l\"'"

tmux select-window -t "$SESSION_NAME":hub >/dev/null 2>&1 || true

echo
echo "Focus hub running (session: $SESSION_NAME)."
echo "  hub API:     http://$host:$port  (robots_config=$robots_config)"
[[ -n "$glm_url" ]] && echo "  GLM-4V:      $glm_url"
[[ "$start_pipeline" == "true" ]] && echo "  pipeline:    mapping + decision every ${decision_interval}s -> $HUB_DIR/runtime/map_out"
echo "  admin token: $HUB_DIR/runtime/admin_token"
echo "  robot tokens: $tokens_file"
echo
echo "For a robot to reach this hub over SSH (test transport, not production TLS/VPN):"
echo "  ssh -R 127.0.0.1:<remote-port>:127.0.0.1:$port <this-host>"
echo
echo "Attach:  tmux attach -d -t $SESSION_NAME"
echo "Stop:    bash $SCRIPT_DIR/focus_hub_down.sh --session $SESSION_NAME"
