#!/usr/bin/env bash
# Persistent-session one-click debug or supervised physical episode.
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HUB_DIR="$WORKSPACE/hub"
PYTHON_BIN="$HUB_DIR/.venv/bin/python"
SESSION_MANAGER="$HUB_DIR/tools/manage_realworld_session.py"
session_file="current"
mode=""
goal_category="chair"
scene_id=""
episode_id=""
confirmation=""
full_preflight="false"
session_env_file=""
live_cleanup_required="false"
cleanup_started="false"
SSH_PROBE_TIMEOUT_S="${FOCUS_SSH_PROBE_TIMEOUT_S:-15}"
oneclick_started="$SECONDS"

usage() {
  cat <<'EOF'
Usage:
  bash hub/scripts/realworld_oneclick.sh --session-file current --mode debug \
    [--goal-category chair] [--scene-id debug-chair]

  bash hub/scripts/realworld_oneclick.sh --session-file current --mode live \
    --scene-id SCENE --episode-id EPISODE --goal-category chair \
    --operator-confirmation OPERATOR_PRESENT_AND_ROBOTS_CLEAR \
    [--full-preflight]

debug restarts a clean Hub epoch, verifies exact session/map/Foxglove
identities, runs both robot receivers read-only, freezes fresh synchronized
inputs, runs the real VLM, and records a no-motion gate for this Git commit.

live is unlocked only by that same-session debug record. By default it proves
that each tracking process is the same process that passed debug and reuses
the warm TinyNav/map/Foxglove core. --full-preflight forces the slower
read-only stack recovery without weakening the tracking-epoch proof. Live
clears any old Hub decision epoch, arms both local paths with no GOAL present,
proves fresh receiver heartbeats, then runs the source 0/24/49/... decision
loop. Every round is separated by an acknowledged robot-local HOLD; semantic
ARRIVED automatically ends the episode and seals terminal RGB-D/map evidence.
The exit trap removes both chassis command paths and leaves only the warm
read-only observation/map core.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session-file) session_file="$2"; shift 2 ;;
    --mode) mode="$2"; shift 2 ;;
    --goal-category) goal_category="$2"; shift 2 ;;
    --scene-id) scene_id="$2"; shift 2 ;;
    --episode-id) episode_id="$2"; shift 2 ;;
    --operator-confirmation) confirmation="$2"; shift 2 ;;
    --full-preflight) full_preflight="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$mode" == debug || "$mode" == live ]] || {
  echo "--mode must be debug or live." >&2
  exit 2
}
case "$goal_category" in
  chair|bed|plant|toilet|tv|sofa) ;;
  *) echo "Unsupported HPC ObjectNav target: $goal_category" >&2; exit 2 ;;
esac
if [[ "$mode" == live ]]; then
  [[ "$confirmation" == OPERATOR_PRESENT_AND_ROBOTS_CLEAR ]] || {
    echo "Live mode requires OPERATOR_PRESENT_AND_ROBOTS_CLEAR." >&2
    exit 2
  }
  [[ -n "$scene_id" && -n "$episode_id" ]] || {
    echo "Live mode requires --scene-id and --episode-id." >&2
    exit 2
  }
fi
scene_id="${scene_id:-debug-${goal_category}}"
[[ "$scene_id" =~ ^[a-z0-9][a-z0-9_-]{0,63}$ ]] || {
  echo "Scene ID must match [a-z0-9][a-z0-9_-]{0,63}." >&2
  exit 2
}
[[ "${episode_id:-debug}" =~ ^[a-z0-9][a-z0-9_.-]{0,127}$ ]] || {
  echo "Episode ID must be lowercase and filesystem-safe." >&2
  exit 2
}
[[ -x "$PYTHON_BIN" ]] || {
  echo "Missing Hub Python environment: $PYTHON_BIN" >&2
  exit 1
}
[[ "$SSH_PROBE_TIMEOUT_S" =~ ^[1-9][0-9]*$ ]] || {
  echo "FOCUS_SSH_PROBE_TIMEOUT_S must be a positive integer." >&2
  exit 2
}
runtime_status="$(
  git -C "$WORKSPACE" status --porcelain --untracked-files=normal \
    -- hub source dependencies
)"
if [[ -n "$runtime_status" ]]; then
  echo "One-click requires committed runtime code under hub/, source/, and dependencies/:" >&2
  printf '%s\n' "$runtime_status" >&2
  exit 1
fi

mkdir -p "$HUB_DIR/runtime"
session_env_file="$(mktemp "$HUB_DIR/runtime/.oneclick-session.XXXXXX")"
trap 'rm -f "$session_env_file"' EXIT
"$PYTHON_BIN" "$SESSION_MANAGER" resolve \
  --session-file "$session_file" \
  --mode "$mode" \
  --allow-map-rebuild \
  --format shell >"$session_env_file"
# The manager emits only shlex-quoted assignments after validating every
# local path remains in this workspace and every remote endpoint is loopback.
# shellcheck disable=SC1090
source "$session_env_file"

HUB_URL="http://127.0.0.1:$FOCUS_HUB_PORT"
WSJ_TMUX_TARGET="$FOCUS_WSJ_SSH_TMUX_RESOLVED"
YUNJI_TMUX_TARGET="$FOCUS_YUNJI_SSH_TMUX_RESOLVED"
WSJ_ROOT="$FOCUS_WSJ_ROOT_RESOLVED"
YUNJI_ROOT="$FOCUS_YUNJI_ROOT_RESOLVED"
WSJ_MAP="$FOCUS_WSJ_MAP"
YUNJI_MAP="$FOCUS_YUNJI_MAP"
HUB_SESSION="$FOCUS_HUB_SESSION"
MAP_SESSION="$FOCUS_MAP_SESSION"
FOXGLOVE_SESSION="$FOCUS_FOXGLOVE_SESSION"
GLM_URL="$FOCUS_GLM_URL"

for required in \
  "$FOCUS_ADMIN_TOKEN_FILE" \
  "$FOCUS_ROBOT_CONFIG" \
  "$FOCUS_DEBUG_ROBOT_CONFIG" \
  "$FOCUS_LIVE_ROBOT_CONFIG"; do
  [[ -e "$required" ]] || {
    echo "Missing session-bound input: $required" >&2
    exit 1
  }
done

probe_ssh_tmux_shell() {
  local target="$1" token line deadline pane_state output
  token="FOCUS_SSH_PROBE_$(date +%s%N)_${RANDOM}"
  printf -v line 'printf "\\n__%s_SHELL_READY__\\n"' "$token"
  tmux send-keys -t "$target" -l "$line"
  tmux send-keys -t "$target" Enter
  deadline=$((SECONDS + SSH_PROBE_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    pane_state="$(
      tmux display-message -p -t "$target" \
        '#{pane_dead}:#{pane_current_command}' 2>/dev/null || true
    )"
    if [[ "$pane_state" == 0:ssh ]]; then
      output="$(
        tmux capture-pane -pJt "$target" -S -80 2>/dev/null || true
      )"
      if grep -Eq "^__${token}_SHELL_READY__[[:space:]]*$" <<<"$output"; then
        echo "SSH_TMUX_SHELL_READY: $target"
        return 0
      fi
    elif [[ "$pane_state" == 1:* ]]; then
      echo "Existing SSH/tmux shell disconnected during its probe: $target" >&2
      tmux capture-pane -pJt "$target" -S -30 2>/dev/null \
        | tail -n 30 >&2 || true
      return 1
    fi
    sleep 1
  done
  echo "Existing SSH/tmux shell did not answer its probe: $target" >&2
  tmux capture-pane -pJt "$target" -S -30 2>/dev/null \
    | tail -n 30 >&2 || true
  return 1
}

ensure_ssh_tmux_shell() {
  local target="$1" pane_state start_command
  pane_state="$(
    tmux display-message -p -t "$target" \
      '#{pane_dead}:#{pane_current_command}' 2>/dev/null || true
  )"
  [[ -n "$pane_state" ]] || {
    echo "Existing SSH/tmux target is unavailable: $target" >&2
    return 1
  }
  start_command="$(
    tmux display-message -p -t "$target" \
      '#{pane_start_command}' 2>/dev/null || true
  )"
  [[ "$start_command" == *"ssh "* ]] || {
    echo "Existing tmux target is not the expected SSH pane: $target" >&2
    return 1
  }
  if [[ "$pane_state" == 1:* ]]; then
    echo "Respawning disconnected existing SSH/tmux pane: $target"
    tmux respawn-pane -k -t "$target"
    echo "SSH_TMUX_PANE_RESPAWNED: $target"
  elif [[ "$pane_state" != 0:ssh ]]; then
    echo "Existing SSH/tmux target has unexpected state '$pane_state': $target" >&2
    return 1
  fi
  probe_ssh_tmux_shell "$target"
}

ensure_ssh_tmux_shell "$WSJ_TMUX_TARGET"
ensure_ssh_tmux_shell "$YUNJI_TMUX_TARGET"

remote_run() {
  local target="$1" command="$2" token
  remote_begin "$target" "$command" token
  remote_finish "$target" "$token"
}

remote_begin() {
  local target="$1" command="$2" result_name="$3" run_token line
  [[ "$result_name" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]] || {
    echo "Invalid remote token variable: $result_name" >&2
    return 2
  }
  run_token="FOCUS_$(date +%s%N)_${RANDOM}"
  printf -v line \
    'bash -lc %q; rc=$?; echo; echo __%s_RC=$rc' \
    "$command" "$run_token"
  tmux send-keys -t "$target" "$line" Enter
  printf -v "$result_name" '%s' "$run_token"
}

remote_finish() {
  local target="$1" token="$2" timeout_s="${3:-180}"
  local deadline output rc cancel_deadline
  deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    output="$(tmux capture-pane -pJt "$target" -S -260 2>/dev/null || true)"
    rc="$(
      sed -n "s/^__${token}_RC=\\([0-9][0-9]*\\)[[:space:]]*$/\\1/p" \
        <<<"$output" | tail -n 1
    )"
    if [[ -n "$rc" ]]; then
      if [[ "$rc" != 0 ]]; then
        echo "$output" | tail -n 120 >&2
        return "$rc"
      fi
      return 0
    fi
    sleep 1
  done
  echo "Remote command timed out on $target" >&2
  # Make timeout fail closed in the remote shell as well. Otherwise the
  # foreground launcher remains active and every cleanup command is only
  # queued behind it, which can turn one bounded failure into another full
  # timeout.
  tmux send-keys -t "$target" C-c
  cancel_deadline=$((SECONDS + 10))
  while (( SECONDS < cancel_deadline )); do
    output="$(tmux capture-pane -pJt "$target" -S -260 2>/dev/null || true)"
    rc="$(
      sed -n "s/^__${token}_RC=\([0-9][0-9]*\)[[:space:]]*$/\1/p" \
        <<<"$output" | tail -n 1
    )"
    if [[ -n "$rc" ]]; then
      echo "REMOTE_TIMEOUT_INTERRUPTED: $target rc=$rc" >&2
      return 124
    fi
    sleep 1
  done
  echo "Timed-out command did not release $target; respawning its existing SSH pane." >&2
  tmux respawn-pane -k -t "$target"
  probe_ssh_tmux_shell "$target" || true
  return 124
}

remote_pair() {
  local wsj_command="$1" yunji_command="$2"
  local wsj_token yunji_token wsj_rc=0 yunji_rc=0
  remote_begin "$WSJ_TMUX_TARGET" "$wsj_command" wsj_token
  remote_begin "$YUNJI_TMUX_TARGET" "$yunji_command" yunji_token
  remote_finish "$WSJ_TMUX_TARGET" "$wsj_token" || wsj_rc=$?
  remote_finish "$YUNJI_TMUX_TARGET" "$yunji_token" || yunji_rc=$?
  if [[ "$wsj_rc" != 0 || "$yunji_rc" != 0 ]]; then
    echo "Parallel robot operation failed: WSJ=$wsj_rc Yunji=$yunji_rc" >&2
    return 1
  fi
}

remote_queue() {
  local target="$1" command="$2" line
  printf -v line 'bash -lc %q' "$command"
  # Each queued line stays below the remote PTY canonical-input limit. The
  # shell executes them in order; the following marked remote_run is the
  # completion barrier and still performs the authoritative checksum check.
  tmux send-keys -t "$target" -l "$line"
  tmux send-keys -t "$target" Enter
}

verify_remote_release() {
  local target="$1" root="$2" manifest encoded quoted_root suffix
  local remote_encoded remote_manifest chunk
  manifest="$(
    cd "$WORKSPACE"
    git ls-files -z hub/src/focus_hub hub/robot_overlay \
      | sort -z \
      | xargs -0 -r sha256sum
  )"
  [[ -n "$manifest" ]] || {
    echo "No tracked robot deployment files were found." >&2
    return 1
  }
  encoded="$(printf '%s\n' "$manifest" | base64 -w0)"
  printf -v quoted_root '%q' "$root"
  suffix="$(date +%s%N)_${RANDOM}"
  remote_encoded="/tmp/focus-release-${suffix}.b64"
  remote_manifest="/tmp/focus-release-${suffix}.sha256"
  remote_run "$target" "umask 077; : > '$remote_encoded'"
  while IFS= read -r chunk || [[ -n "$chunk" ]]; do
    remote_queue "$target" \
      "printf '%s' '$chunk' >> '$remote_encoded'"
  # Keep each tmux/PTY line well below the observed remote canonical-input
  # limit. Larger chunks can be accepted by tmux but silently truncated by
  # the interactive SSH terminal.
  done < <(printf '%s' "$encoded" | fold -w 1000)
  remote_run "$target" \
    "set +e; base64 -d '$remote_encoded' > '$remote_manifest'; rc=\$?; if [ \"\$rc\" -eq 0 ]; then (cd $quoted_root && sha256sum --quiet -c '$remote_manifest'); rc=\$?; fi; unlink '$remote_encoded'; unlink '$remote_manifest'; exit \"\$rc\""
}

verify_remote_release_pair() {
  local wsj_pid yunji_pid wsj_rc=0 yunji_rc=0 started
  started="$SECONDS"
  (
    trap - EXIT INT TERM
    verify_remote_release "$WSJ_TMUX_TARGET" "$WSJ_ROOT"
  ) &
  wsj_pid=$!
  (
    trap - EXIT INT TERM
    verify_remote_release "$YUNJI_TMUX_TARGET" "$YUNJI_ROOT"
  ) &
  yunji_pid=$!
  wait "$wsj_pid" || wsj_rc=$?
  wait "$yunji_pid" || yunji_rc=$?
  echo "ONECLICK_TIMING phase=dual_release_verification elapsed_s=$((SECONDS - started))"
  if [[ "$wsj_rc" != 0 || "$yunji_rc" != 0 ]]; then
    echo "Parallel release verification failed: WSJ=$wsj_rc Yunji=$yunji_rc" >&2
    return 1
  fi
}

printf -v WSJ_ENV \
  'FOCUS_SHARED_CALIBRATION_FILE=%q FOCUS_WSJ_BASE_CAMERA_CALIBRATION_FILE=%q FOCUS_WSJ_TRANSFORM_VERSION=%q FOCUS_SHARED_CALIBRATION_ID=%q FOCUS_HUB_BASE_URL=%q FOCUS_FOXGLOVE_PREVIEW_URL=%q FOCUS_DEPLOYMENT_COMMIT=%q' \
  "$FOCUS_WSJ_REMOTE_CALIBRATION" \
  "$FOCUS_WSJ_REMOTE_BASE_CAMERA" \
  "$FOCUS_WSJ_TRANSFORM" \
  "$FOCUS_CALIBRATION_ID" \
  "$FOCUS_WSJ_REMOTE_HUB_URL" \
  "$FOCUS_WSJ_REMOTE_PREVIEW_URL" \
  "$FOCUS_SESSION_CODE_COMMIT"
printf -v YUNJI_ENV \
  'FOCUS_YUNJI_SHARED_CALIBRATION_FILE=%q FOCUS_YUNJI_BASE_CAMERA_CALIBRATION=%q FOCUS_YUNJI_TRANSFORM_VERSION=%q FOCUS_SHARED_CALIBRATION_ID=%q FOCUS_HUB_BASE_URL=%q FOCUS_DEPLOYMENT_COMMIT=%q' \
  "$FOCUS_YUNJI_REMOTE_CALIBRATION" \
  "$FOCUS_YUNJI_REMOTE_BASE_CAMERA" \
  "$FOCUS_YUNJI_TRANSFORM" \
  "$FOCUS_CALIBRATION_ID" \
  "$FOCUS_YUNJI_REMOTE_HUB_URL" \
  "$FOCUS_SESSION_CODE_COMMIT"
printf -v WSJ_LAUNCHER '%q' \
  "$WSJ_ROOT/hub/robot_overlay/start_wsj_buildmap_v2.sh"
printf -v YUNJI_LAUNCHER '%q' \
  "$YUNJI_ROOT/hub/robot_overlay/start_yunji_v2.sh"

stop_managed_hub() {
  local rows session start deadline
  rows="$(
    tmux list-panes -a -F $'#{session_name}\t#{pane_start_command}' \
      2>/dev/null || true
  )"
  while IFS=$'\t' read -r session start; do
    [[ -n "$session" ]] || continue
    if [[ "$session" == "$HUB_SESSION" ]] \
       || [[ "$start" == *"focus_hub.api:app"* \
             && "$start" == *"--port"* \
             && "$start" == *"$FOCUS_HUB_PORT"* ]]; then
      tmux kill-session -t "$session" >/dev/null 2>&1 || true
    fi
  done <<<"$rows"
  deadline=$((SECONDS + 15))
  while ss -tln 2>/dev/null | grep -q ":$FOCUS_HUB_PORT "; do
    (( SECONDS < deadline )) || {
      echo "Hub port $FOCUS_HUB_PORT is owned by an unmanaged process." >&2
      return 1
    }
    sleep 1
  done
}

restart_hub() {
  local config="$1" expected_goal="$2" health_json
  stop_managed_hub
  bash "$HUB_DIR/scripts/focus_hub_up.sh" \
    --port "$FOCUS_HUB_PORT" \
    --no-glm \
    --no-pipeline \
    --session "$HUB_SESSION" \
    --robots-config "$config"
  health_json="$(curl -fsS --max-time 5 "$HUB_URL/healthz")"
  FOCUS_HEALTH_JSON="$health_json" FOCUS_EXPECT_GOAL="$expected_goal" \
    "$PYTHON_BIN" -c '
import json
import os

health = json.loads(os.environ["FOCUS_HEALTH_JSON"])
expected = os.environ["FOCUS_EXPECT_GOAL"] == "true"
if set(health.get("robots", [])) != {"robot-0", "robot-1"}:
    raise SystemExit("Hub robot identity mismatch")
enabled = health.get("goal_output_enabled", {})
if any(enabled.get(robot) is not expected for robot in ("robot-0", "robot-1")):
    raise SystemExit("Hub GOAL policy mismatch")
'
}

map_window_matches() {
  local window="$1" robot_id="$2" map_dir="$3" transform="$4" boundary="$5"
  local dead start
  dead="$(
    tmux display-message -p -t "$MAP_SESSION:$window" '#{pane_dead}' \
      2>/dev/null || true
  )"
  [[ "$dead" == 0 ]] || return 1
  start="$(
    tmux display-message -p -t "$MAP_SESSION:$window" \
      '#{pane_start_command}' 2>/dev/null || true
  )"
  [[ "$start" == *"hub_pipeline_daemon.py"* \
     && "$start" == *"--robot-id $robot_id"* \
     && "$start" == *"--out-dir '$map_dir'"* \
     && "$start" == *"--expected-transform-version '$transform'"* \
     && "$start" == *"--shared-frame-calibration-id '$FOCUS_CALIBRATION_ID'"* \
     && "$start" == *"$FOCUS_SESSION_CODE_COMMIT"* \
     && "$start" == *"--start-after-sequence '$boundary'"* \
     && "$start" == *"--semantic-backend '$FOCUS_SEMANTIC_BACKEND'"* ]]
}

ensure_maps() {
  local rows session start deadline
  local -a map_resume_args=()
  if map_window_matches \
      wsj robot-0 "$WSJ_MAP" "$FOCUS_WSJ_TRANSFORM" \
      "$FOCUS_WSJ_START_AFTER" \
     && map_window_matches \
      yunji robot-1 "$YUNJI_MAP" "$FOCUS_YUNJI_TRANSFORM" \
      "$FOCUS_YUNJI_START_AFTER"; then
    return 0
  fi
  # A process without the exact session Git marker is rebuilt from the
  # immutable sequence boundary once. Later runs can reuse that proven daemon.
  tmux kill-session -t "$MAP_SESSION" >/dev/null 2>&1 || true
  rows="$(
    tmux list-panes -a -F $'#{session_name}\t#{pane_start_command}' \
      2>/dev/null || true
  )"
  while IFS=$'\t' read -r session start; do
    [[ "$start" == *"hub_pipeline_daemon.py"* ]] || continue
    if [[ "$start" == *"$WSJ_MAP"* || "$start" == *"$YUNJI_MAP"* ]]; then
      tmux kill-session -t "$session" >/dev/null 2>&1 || true
    fi
  done <<<"$rows"
  deadline=$((SECONDS + 15))
  while pgrep -af 'hub_pipeline_daemon[.]py' 2>/dev/null \
      | grep -F -e "$WSJ_MAP" -e "$YUNJI_MAP" >/dev/null; do
    (( SECONDS < deadline )) || {
      echo "A map writer outside the managed tmux session owns a session map." >&2
      return 1
    }
    sleep 1
  done
  if [[ -e "$WSJ_MAP" && -e "$YUNJI_MAP" ]]; then
    map_resume_args=(--resume-existing)
  elif [[ -e "$WSJ_MAP" || -e "$YUNJI_MAP" ]]; then
    echo "Refusing a partial session map pair; both maps must exist or both be absent." >&2
    return 1
  fi
  bash "$HUB_DIR/scripts/start_fresh_dual_maps.sh" \
    --session-tag "$FOCUS_SESSION_ID" \
    --calibration-id "$FOCUS_CALIBRATION_ID" \
    --wsj-transform "$FOCUS_WSJ_TRANSFORM" \
    --yunji-transform "$FOCUS_YUNJI_TRANSFORM" \
    --wsj-start-after "$FOCUS_WSJ_START_AFTER" \
    --yunji-start-after "$FOCUS_YUNJI_START_AFTER" \
    --goal-category "$FOCUS_MAP_GOAL_CATEGORY" \
    --semantic-backend "$FOCUS_SEMANTIC_BACKEND" \
    --code-commit "$FOCUS_SESSION_CODE_COMMIT" \
    --hub-url "$HUB_URL" \
    "${map_resume_args[@]}"
  map_window_matches \
    wsj robot-0 "$WSJ_MAP" "$FOCUS_WSJ_TRANSFORM" \
    "$FOCUS_WSJ_START_AFTER"
  map_window_matches \
    yunji robot-1 "$YUNJI_MAP" "$FOCUS_YUNJI_TRANSFORM" \
    "$FOCUS_YUNJI_START_AFTER"
}

ensure_glm() {
  local desired_port start deadline rows session candidate_start
  desired_port="$(
    GLM_URL_VALUE="$GLM_URL" "$PYTHON_BIN" -c '
from urllib.parse import urlparse
import os
print(urlparse(os.environ["GLM_URL_VALUE"]).port)
'
  )"
  if curl -fsS --max-time 5 "$GLM_URL/models" >/dev/null 2>&1; then
    start="$(
      tmux display-message -p -t "$FOCUS_GLM_SESSION" \
        '#{pane_start_command}' 2>/dev/null || true
    )"
    if [[ "$start" == *"run_glm_offline.sh"* \
          && "$start" == *"$desired_port"* ]]; then
      return 0
    fi
    # A healthy stateless GLM may still be owned by the previous experiment's
    # tmux session. Adopt only a pane whose launch command proves both the
    # repository launcher and exact port; never trust the endpoint alone.
    rows="$(
      tmux list-panes -a \
        -F $'#{session_name}\t#{pane_start_command}' 2>/dev/null || true
    )"
    while IFS=$'\t' read -r session candidate_start; do
      [[ "$candidate_start" == *"run_glm_offline.sh"* \
         && "$candidate_start" == *"$desired_port"* ]] || continue
      if tmux has-session -t "$FOCUS_GLM_SESSION" 2>/dev/null; then
        echo "GLM endpoint is live but the session tmux is already occupied." >&2
        return 1
      fi
      tmux rename-session -t "$session" "$FOCUS_GLM_SESSION"
      return 0
    done <<<"$rows"
    echo "GLM endpoint is live but not owned by a verified GLM tmux." >&2
    return 1
  fi
  tmux kill-session -t "$FOCUS_GLM_SESSION" >/dev/null 2>&1 || true
  if ss -tln 2>/dev/null | grep -q ":$desired_port "; then
    echo "GLM port $desired_port is owned by an unhealthy process." >&2
    return 1
  fi
  tmux new-session -d -s "$FOCUS_GLM_SESSION" -n server \
    "bash -lc 'cd \"$WORKSPACE\"; FOCUS_GLM_PORT=\"$desired_port\" exec bash hub/scripts/run_glm_offline.sh'"
  deadline=$((SECONDS + 180))
  until curl -fsS --max-time 5 "$GLM_URL/models" >/dev/null 2>&1; do
    (( SECONDS < deadline )) || {
      tmux capture-pane -pt "$FOCUS_GLM_SESSION:server" -S -120 >&2 || true
      return 1
    }
    sleep 2
  done
}

foxglove_source_sha256() {
  "$PYTHON_BIN" - "$WORKSPACE" \
    "$HUB_DIR/tools/foxglove_relay.py" \
    "$HUB_DIR/src/focus_hub/map_visualization.py" <<'PY'
import hashlib
from pathlib import Path
import sys

workspace = Path(sys.argv[1]).resolve()
digest = hashlib.sha256()
for raw_path in sys.argv[2:]:
    path = Path(raw_path).resolve()
    digest.update(str(path.relative_to(workspace)).encode("utf-8"))
    digest.update(b"\0")
    digest.update(path.read_bytes())
    digest.update(b"\0")
print(digest.hexdigest())
PY
}

foxglove_matches() {
  local dead start health expected_sha
  dead="$(
    tmux display-message -p -t "$FOXGLOVE_SESSION:relay" '#{pane_dead}' \
      2>/dev/null || true
  )"
  [[ "$dead" == 0 ]] || return 1
  start="$(
    tmux display-message -p -t "$FOXGLOVE_SESSION:relay" \
      '#{pane_start_command}' 2>/dev/null || true
  )"
  [[ "$start" == *"tools/foxglove_relay.py"* \
     && "$start" == *"$WSJ_MAP"* \
     && "$start" == *"$YUNJI_MAP"* \
     && "$start" == *"--port $FOCUS_FOXGLOVE_PORT"* \
     && "$start" == *"--preview-port $FOCUS_PREVIEW_PORT"* \
     && "$start" == *"--fuse"* ]] || return 1
  health="$(
    curl -fsS --max-time 3 \
      "http://127.0.0.1:$FOCUS_PREVIEW_PORT/healthz" 2>/dev/null
  )" || return 1
  expected_sha="$(foxglove_source_sha256)" || return 1
  FOCUS_FOXGLOVE_HEALTH="$health" \
    FOCUS_FOXGLOVE_EXPECTED_SHA="$expected_sha" \
    "$PYTHON_BIN" - <<'PY'
import json
import os

health = json.loads(os.environ["FOCUS_FOXGLOVE_HEALTH"])
if health.get("semantic_overview_contract") != "focus-semantic-overview-v2":
    raise SystemExit(1)
if (
    health.get("loaded_relay_source_sha256")
    != os.environ["FOCUS_FOXGLOVE_EXPECTED_SHA"]
):
    raise SystemExit(1)
robots = health.get("robots")
if not isinstance(robots, dict):
    raise SystemExit(1)
for name in ("wsj", "yunji"):
    if not isinstance(robots.get(name), dict):
        raise SystemExit(1)
    if robots[name].get("semantic_overview_ready") is not True:
        raise SystemExit(1)
fused = health.get("fused")
if not isinstance(fused, dict):
    raise SystemExit(1)
if (
    fused.get("enabled") is not True
    or fused.get("semantic_overview_ready") is not True
):
    raise SystemExit(1)
PY
}

ensure_foxglove() {
  local rows session start deadline
  foxglove_matches && return 0
  rows="$(
    tmux list-panes -a -F $'#{session_name}\t#{pane_start_command}' \
      2>/dev/null || true
  )"
  while IFS=$'\t' read -r session start; do
    [[ "$start" == *"tools/foxglove_relay.py"* ]] || continue
    if [[ "$session" == "$FOXGLOVE_SESSION" ]] \
       || [[ "$start" == *"$FOCUS_FOXGLOVE_PORT"* ]] \
       || [[ "$start" == *"$FOCUS_PREVIEW_PORT"* ]]; then
      tmux kill-session -t "$session" >/dev/null 2>&1 || true
    fi
  done <<<"$rows"
  deadline=$((SECONDS + 15))
  while ss -tln 2>/dev/null \
      | grep -Eq ":($FOCUS_FOXGLOVE_PORT|$FOCUS_PREVIEW_PORT) "; do
    (( SECONDS < deadline )) || {
      echo "Foxglove ports are owned by an unmanaged process." >&2
      return 1
    }
    sleep 1
  done
  tmux new-session -d -s "$FOXGLOVE_SESSION" -n relay \
    "bash -lc 'cd \"$HUB_DIR\"; exec .venv/bin/python -u tools/foxglove_relay.py --robot robot-0:wsj:\"$WSJ_MAP\" --robot robot-1:yunji:\"$YUNJI_MAP\" --host 0.0.0.0 --port $FOCUS_FOXGLOVE_PORT --preview-port $FOCUS_PREVIEW_PORT --fuse'"
  # A fresh pair of SegFormer-backed maps normally needs roughly 20-40
  # seconds to pass startup stability, infer its first semantic frame and
  # publish the first fused overview.  Readiness remains content-based; this
  # longer bound only prevents a healthy first launch from losing a race by a
  # few seconds.
  deadline=$((SECONDS + 90))
  until foxglove_matches; do
    (( SECONDS < deadline )) || {
      tmux capture-pane -pt "$FOXGLOVE_SESSION:relay" -S -120 >&2 || true
      return 1
    }
    sleep 1
  done
}

start_read_only_robots() {
  remote_pair \
    "source /home/nvidia/twork/tinynav_setup.bash; timeout 5 ros2 topic pub --once /nav/paused std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true; timeout 5 ros2 topic pub --once /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:go2-bridge >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:v2-receiver >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:hub-sender >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:calibration-sender >/dev/null 2>&1 || true; $WSJ_ENV bash $WSJ_LAUNCHER --mode debug" \
    "for unit in focus-yunji-calibration-observation-v1.service focus-yunji-v2-debug-v2.service focus-yunji-v2-live-v2.service focus-yunji-command-observation-v2.service; do sudo -n systemctl stop \"\$unit\" >/dev/null 2>&1 || true; sudo -n systemctl reset-failed \"\$unit\" >/dev/null 2>&1 || true; done; $YUNJI_ENV bash $YUNJI_LAUNCHER --mode debug"
}

ensure_local_services_parallel() {
  local glm_pid foxglove_pid glm_rc=0 foxglove_rc=0 started
  started="$SECONDS"
  (
    trap - EXIT INT TERM
    phase_started="$SECONDS"
    ensure_glm
    echo "ONECLICK_TIMING phase=glm_ready elapsed_s=$((SECONDS - phase_started))"
  ) &
  glm_pid=$!
  (
    trap - EXIT INT TERM
    phase_started="$SECONDS"
    ensure_foxglove
    echo "ONECLICK_TIMING phase=foxglove_ready elapsed_s=$((SECONDS - phase_started))"
  ) &
  foxglove_pid=$!
  wait "$glm_pid" || glm_rc=$?
  wait "$foxglove_pid" || foxglove_rc=$?
  echo "ONECLICK_TIMING phase=parallel_local_services elapsed_s=$((SECONDS - started))"
  if [[ "$glm_rc" != 0 || "$foxglove_rc" != 0 ]]; then
    echo "Parallel local readiness failed: GLM=$glm_rc Foxglove=$foxglove_rc" >&2
    return 1
  fi
}

recover_full_readonly_runtime_parallel() {
  local glm_pid robots_pid foxglove_pid
  local glm_rc=0 robots_rc=0 foxglove_rc=0 started
  started="$SECONDS"
  (
    trap - EXIT INT TERM
    phase_started="$SECONDS"
    ensure_glm
    echo "ONECLICK_TIMING phase=glm_ready elapsed_s=$((SECONDS - phase_started))"
  ) &
  glm_pid=$!
  (
    trap - EXIT INT TERM
    phase_started="$SECONDS"
    start_read_only_robots
    echo "ONECLICK_TIMING phase=dual_readonly_robots_ready elapsed_s=$((SECONDS - phase_started))"
  ) &
  robots_pid=$!
  (
    trap - EXIT INT TERM
    phase_started="$SECONDS"
    ensure_foxglove
    echo "ONECLICK_TIMING phase=foxglove_ready elapsed_s=$((SECONDS - phase_started))"
  ) &
  foxglove_pid=$!
  wait "$glm_pid" || glm_rc=$?
  wait "$robots_pid" || robots_rc=$?
  wait "$foxglove_pid" || foxglove_rc=$?
  echo "ONECLICK_TIMING phase=parallel_full_readonly_runtime elapsed_s=$((SECONDS - started))"
  if [[ "$glm_rc" != 0 || "$robots_rc" != 0 || "$foxglove_rc" != 0 ]]; then
    echo "Parallel full runtime recovery failed: GLM=$glm_rc robots=$robots_rc Foxglove=$foxglove_rc" >&2
    return 1
  fi
}

verify_tracking_epoch_continuity() {
  local stamp wsj_probe yunji_probe
  [[ "$FOCUS_DEBUG_PASSED_AT_NS" =~ ^[1-9][0-9]*$ ]] || {
    echo "The session has no strict debug timestamp to anchor fast reuse." >&2
    return 1
  }
  stamp="$(date -u +%Y%m%dT%H%M%SZ)"
  wsj_probe="/home/nvidia/.local/state/topofocus/tracking-epoch-${FOCUS_SESSION_ID}-${stamp}.json"
  yunji_probe="/home/nyu/.local/state/topofocus/tracking-epoch-${FOCUS_SESSION_ID}-${stamp}.json"
  remote_pair \
    "python3 '$WSJ_ROOT/hub/robot_overlay/probe_tracking_epoch.py' --robot-id robot-0 --debug-passed-at-ns '$FOCUS_DEBUG_PASSED_AT_NS' --output '$wsj_probe'" \
    "python3 '$YUNJI_ROOT/hub/robot_overlay/probe_tracking_epoch.py' --robot-id robot-1 --debug-passed-at-ns '$FOCUS_DEBUG_PASSED_AT_NS' --output '$yunji_probe'"
  echo "TRACKING_EPOCH_CONTINUITY_PASSED: both odometry owners predate strict debug."
  echo "  WSJ evidence:   $wsj_probe"
  echo "  Yunji evidence: $yunji_probe"
}

prepare_warm_live_reuse() {
  # Remove any stale command path first, then prove only the persistent
  # perception/planning cores needed for a fast mode switch remain alive.
  # This operation cannot move either robot.
  remote_pair \
    "source /home/nvidia/twork/tinynav_setup.bash; timeout 5 ros2 topic pub --once /nav/paused std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true; timeout 5 ros2 topic pub --once /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:go2-bridge >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:v2-receiver >/dev/null 2>&1 || true; for window in maploc online-map planning goal-router control; do tmux list-windows -t tinynav_semantic_nav_auto -F '#{window_name}' | grep -qx \"\$window\" || exit 1; done" \
    "for unit in focus-yunji-v2-debug-v2.service focus-yunji-v2-live-v2.service focus-yunji-v2-debug-v3.service focus-yunji-v2-live-v3.service focus-yunji-water-bridge-debug-v1.service focus-yunji-water-bridge-live-v1.service; do sudo -n systemctl stop \"\$unit\" >/dev/null 2>&1 || true; sudo -n systemctl reset-failed \"\$unit\" >/dev/null 2>&1 || true; done; systemctl is-active --quiet focus-yunji-odin1-driver.service || exit 1; for unit in focus-yunji-tinynav-adapter-v1.service focus-yunji-tinynav-occupancy-v1.service focus-yunji-tinynav-planner-v1.service focus-yunji-tinynav-router-v1.service focus-yunji-tinynav-controller-v1.service; do systemctl is-active --quiet \"\$unit\" || exit 1; environment=\$(systemctl show --property Environment --value \"\$unit\"); [[ \" \$environment \" == *\" FOCUS_DEPLOYMENT_COMMIT=$FOCUS_SESSION_CODE_COMMIT \"* ]] || exit 1; done"
}

arm_live_robots() {
  live_cleanup_required="true"
  remote_pair \
    "source /home/nvidia/twork/tinynav_setup.bash; timeout 5 ros2 topic pub --once /nav/paused std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true; timeout 5 ros2 topic pub --once /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:go2-bridge >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:v2-receiver >/dev/null 2>&1 || true; $WSJ_ENV bash $WSJ_LAUNCHER --mode live --operator-confirmation OPERATOR_PRESENT_AND_WSJ_CLEAR" \
    "for unit in focus-yunji-v2-debug-v2.service focus-yunji-v2-live-v2.service; do sudo -n systemctl stop \"\$unit\" >/dev/null 2>&1 || true; sudo -n systemctl reset-failed \"\$unit\" >/dev/null 2>&1 || true; done; $YUNJI_ENV bash $YUNJI_LAUNCHER --mode live --operator-confirmation OPERATOR_PRESENT_AND_YUNJI_CLEAR --reuse-verified-debug-core"
}

wait_for_live_readiness() {
  local deadline admin_token status
  admin_token="$(<"$FOCUS_ADMIN_TOKEN_FILE")"
  deadline=$((SECONDS + 25))
  while (( SECONDS < deadline )); do
    if status="$(
      FOCUS_HUB_URL="$HUB_URL" \
      FOCUS_ADMIN_TOKEN="$admin_token" \
      FOCUS_EPOCH_NS="$final_hub_epoch_ns" \
      "$PYTHON_BIN" - <<'PY'
import json
import os
import urllib.request

sequences = []
for robot_id in ("robot-0", "robot-1"):
    request = urllib.request.Request(
        os.environ["FOCUS_HUB_URL"]
        + f"/v2/admin/robots/{robot_id}/runtime-readiness",
        headers={"X-Admin-Token": os.environ["FOCUS_ADMIN_TOKEN"]},
    )
    with urllib.request.urlopen(request, timeout=3) as response:
        payload = json.load(response)
    if payload.get("ready_for_goal") is not True:
        raise SystemExit(1)
    if payload.get("health_source") != "heartbeat":
        raise SystemExit(1)
    if int(payload.get("last_observation_received_at_ns", 0)) < int(
        os.environ["FOCUS_EPOCH_NS"]
    ):
        raise SystemExit(1)
    sequence = int(payload.get("last_observation_sequence", -1))
    if sequence < 0:
        raise SystemExit(1)
    sequences.append(sequence)
print(*sequences)
PY
    )"; then
      read -r wsj_epoch_sequence yunji_epoch_sequence <<<"$status"
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for both live heartbeats and clean-epoch observations." >&2
  return 1
}

disarm_live_stack() {
  [[ "$live_cleanup_required" == "true" ]] || return 0
  [[ "$cleanup_started" == "false" ]] || return 0
  cleanup_started="true"
  echo "Fail-closed cleanup: disabling Hub GOAL and both robot command paths."
  restart_hub "$FOCUS_DEBUG_ROBOT_CONFIG" false \
    || echo "WARNING: Hub debug restart failed." >&2
  remote_pair \
    "source /home/nvidia/twork/tinynav_setup.bash; timeout 5 ros2 topic pub --once /nav/paused std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true; timeout 5 ros2 topic pub --once /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:go2-bridge >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:v2-receiver >/dev/null 2>&1 || true" \
    "for unit in focus-yunji-v2-debug-v2.service focus-yunji-v2-live-v2.service focus-yunji-v2-debug-v3.service focus-yunji-v2-live-v3.service focus-yunji-water-bridge-debug-v1.service focus-yunji-water-bridge-live-v1.service; do sudo -n systemctl stop \"\$unit\" >/dev/null 2>&1 || true; sudo -n systemctl reset-failed \"\$unit\" >/dev/null 2>&1 || true; done" \
    || echo "WARNING: one robot command path did not confirm its guarded stop." >&2
  echo "WARM_READONLY_CORE_PRESERVED: observation, maps and Foxglove keep running."
}

cleanup_on_exit() {
  local rc=$?
  trap - EXIT INT TERM
  set +e
  disarm_live_stack
  [[ -z "$session_env_file" ]] || rm -f "$session_env_file"
  exit "$rc"
}
trap cleanup_on_exit EXIT INT TERM

echo "Session $FOCUS_SESSION_ID: validating the exact committed deployment."
echo "Verifying that both robot release roots match this Git checkout."
verify_remote_release_pair

# Once a session has passed strict debug, the live path must prove that the
# processes owning both odometry origins still predate that debug. A chassis
# power cycle does not change either process and therefore needs no board.
# A compute/SLAM/Odin restart fails here before GOAL can be enabled.
if [[ "$FOCUS_DEBUG_PASSED_AT_NS" != 0 ]]; then
  verify_tracking_epoch_continuity || {
    echo "TRACKING_EPOCH_CHANGED: do not reuse this calibration directly." >&2
    echo "If the robot was stationary, create a validated stationary re-anchor; otherwise run board calibration." >&2
    exit 1
  }
fi

wait_for_hub_epoch() {
  local deadline admin_token status
  admin_token="$(<"$FOCUS_ADMIN_TOKEN_FILE")"
  deadline=$((SECONDS + 30))
  while (( SECONDS < deadline )); do
    if status="$(
      FOCUS_HUB_URL="$HUB_URL" \
      FOCUS_ADMIN_TOKEN="$admin_token" \
      FOCUS_EPOCH_NS="$final_hub_epoch_ns" \
      "$PYTHON_BIN" - <<'PY'
import json
import os
import urllib.request

rows = []
for robot_id in ("robot-0", "robot-1"):
    request = urllib.request.Request(
        os.environ["FOCUS_HUB_URL"]
        + f"/v2/admin/robots/{robot_id}/runtime-readiness",
        headers={"X-Admin-Token": os.environ["FOCUS_ADMIN_TOKEN"]},
    )
    with urllib.request.urlopen(request, timeout=3) as response:
        payload = json.load(response)
    if int(payload.get("last_observation_received_at_ns", 0)) < int(
        os.environ["FOCUS_EPOCH_NS"]
    ):
        raise SystemExit(1)
    sequence = int(payload.get("last_observation_sequence", -1))
    if sequence < 0:
        raise SystemExit(1)
    rows.append(sequence)
print(*rows)
PY
    )"; then
      read -r wsj_epoch_sequence yunji_epoch_sequence <<<"$status"
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for both robots in the clean Hub epoch." >&2
  return 1
}

fast_live_reuse="false"
if [[ "$mode" == live && "$full_preflight" != true ]]; then
  if prepare_warm_live_reuse; then
    fast_live_reuse="true"
    echo "FAST_LIVE_REUSE_READY: tracking, TinyNav cores and command-path stop are proven."
  else
    echo "Warm core is incomplete; falling back to full read-only recovery."
  fi
fi

if [[ "$fast_live_reuse" == true ]]; then
  phase_started="$SECONDS"
  ensure_maps
  echo "ONECLICK_TIMING phase=maps_ready elapsed_s=$((SECONDS - phase_started))"
  ensure_local_services_parallel
else
  echo "FULL_DEBUG_RUNTIME_RECOVERY: starting a clean fail-closed Hub epoch."
  phase_started="$SECONDS"
  restart_hub "$FOCUS_DEBUG_ROBOT_CONFIG" false
  echo "ONECLICK_TIMING phase=debug_hub_restart elapsed_s=$((SECONDS - phase_started))"
  phase_started="$SECONDS"
  ensure_maps
  echo "ONECLICK_TIMING phase=maps_ready elapsed_s=$((SECONDS - phase_started))"
  recover_full_readonly_runtime_parallel
fi

if [[ "$mode" == live ]]; then
  # Start a clean GOAL-capable Hub with no decision state, then arm both
  # robot-local paths in parallel. No target exists while the launchers run,
  # so both receivers remain in NO_GOAL/HOLD until their verified heartbeats
  # and fresh observations have entered this exact Hub epoch.
  live_cleanup_required="true"
  phase_started="$SECONDS"
  restart_hub "$FOCUS_LIVE_ROBOT_CONFIG" true
  echo "ONECLICK_TIMING phase=live_hub_restart elapsed_s=$((SECONDS - phase_started))"
  final_hub_epoch_ns="$(date +%s%N)"
  phase_started="$SECONDS"
  arm_live_robots
  echo "ONECLICK_TIMING phase=dual_live_arm elapsed_s=$((SECONDS - phase_started))"
  phase_started="$SECONDS"
  wait_for_live_readiness
  echo "ONECLICK_TIMING phase=clean_epoch_readiness elapsed_s=$((SECONDS - phase_started))"
  echo "LIVE_RECEIVERS_READY_NO_GOAL: starting continuous source episode."
else
  final_hub_epoch_ns="$(date +%s%N)"
  wait_for_hub_epoch
fi

echo "ONECLICK_TIMING phase=pre_episode_total elapsed_s=$((SECONDS - oneclick_started))"
stamp="$(date +%Y%m%d_%H%M%S_%N)"
run_dir="$HUB_DIR/runtime/oneclick_${FOCUS_SESSION_ID}_${mode}_${scene_id}_${stamp}"
mkdir "$run_dir"

if [[ "$mode" == live ]]; then
  if "$PYTHON_BIN" -u "$HUB_DIR/tools/run_v2_source_episode.py" \
      --session-file "$FOCUS_SESSION_FILE" \
      --output "$run_dir" \
      --scene-id "$scene_id" \
      --episode-id "$episode_id" \
      --goal-category "$goal_category" \
      --hub-url "$HUB_URL" \
      --glm-url "$GLM_URL" \
      --admin-token-file "$FOCUS_ADMIN_TOKEN_FILE" \
      --registry-state "$HUB_DIR/runtime/state/registry_state.json" \
      --robot-config "$FOCUS_LIVE_ROBOT_CONFIG" \
      --robot-0-min-sequence "$wsj_epoch_sequence" \
      --robot-1-min-sequence "$yunji_epoch_sequence" \
      --round-input-timeout-s 45 \
      --enable-live-goal-publication \
      --operator-confirmation OPERATOR_PRESENT_AND_ROBOTS_CLEAR; then
    echo "LIVE_SOURCE_EPISODE_FINISHED: $run_dir/episode_report.json"
    echo "Terminal RGB-D/map evidence was sealed automatically."
    echo "Official SR/SPL still needs the independent target/goal-region annotation."
    exit 0
  else
    rc=$?
    echo "LIVE_SOURCE_EPISODE_STOPPED: $run_dir/episode_report.json" >&2
    exit "$rc"
  fi
fi

accepted_dir="$run_dir/accepted"
freeze_log="$run_dir/freeze_rejections.log"
deadline=$((SECONDS + 300))
while ! "$PYTHON_BIN" "$HUB_DIR/tools/freeze_realworld_inputs.py" \
    --session-file "$FOCUS_SESSION_FILE" \
    --output "$accepted_dir" \
    --max-input-age-s 60 \
    --max-sync-skew-s 5 \
    --robot-0-min-sequence "$wsj_epoch_sequence" \
    --robot-1-min-sequence "$yunji_epoch_sequence" \
    >"$run_dir/freeze_result.json" 2>"$freeze_log"; do
  echo "Waiting for strict synchronized maps: $(tail -n 1 "$freeze_log")"
  (( SECONDS < deadline )) || {
    echo "Timed out waiting for strict synchronized maps." >&2
    exit 1
  }
  sleep 2
done

read -r wsj_source wsj_map_sha yunji_source yunji_map_sha < <(
  "$PYTHON_BIN" - "$accepted_dir/accepted_inputs.json" <<'PY'
import json
import sys

payload = json.load(open(sys.argv[1]))
rows = {row["robot_id"]: row for row in payload["robots"]}
print(
    rows["robot-0"]["source_sequence"],
    rows["robot-0"]["map_sha256"],
    rows["robot-1"]["source_sequence"],
    rows["robot-1"]["map_sha256"],
)
PY
)

shadow_dir="$run_dir/shadow"
trusted=(
  chair sofa plant bed toilet tv bathtub shower fireplace appliances
  towel sink chest_of_drawers table stairs
)
shadow_args=(
  "$PYTHON_BIN" -u "$HUB_DIR/tools/live_vlm_shadow.py"
  --robot "robot-0:wsj:$accepted_dir/wsj"
  --robot "robot-1:yunji:$accepted_dir/yunji"
  --spool "$FOCUS_SPOOL_DIR"
  --output "$shadow_dir"
  --goal-category "$goal_category"
  --expected-shared-frame-calibration-id "$FOCUS_CALIBRATION_ID"
  --realworld-session-id "$FOCUS_SESSION_ID"
  --realworld-session-contract-sha256 "$FOCUS_SESSION_CONTRACT_SHA256"
  --expected-source-sequence "robot-0:$wsj_source"
  --expected-source-sequence "robot-1:$yunji_source"
  --expected-map-sha256 "robot-0:$wsj_map_sha"
  --expected-map-sha256 "robot-1:$yunji_map_sha"
  --glm-url "$GLM_URL"
  --hub-url "$HUB_URL"
  --admin-token-file "$FOCUS_ADMIN_TOKEN_FILE"
  --registry-state "$HUB_DIR/runtime/state/registry_state.json"
  --max-input-age-s 60
  --max-sync-skew-s 5
  --publish-hold
  --write-foxglove-targets
)
for category in "${trusted[@]}"; do
  shadow_args+=(--trusted-category "$category")
done
"${shadow_args[@]}"

"$PYTHON_BIN" "$SESSION_MANAGER" mark-debug \
  --session-file "$FOCUS_SESSION_FILE" \
  --shadow-manifest "$shadow_dir/shadow_manifest.json" \
  --debug-safety-confirmation DEBUG_STACK_NO_MOTION_VERIFIED
echo "DEBUG_FULLSTACK_READY"
echo "Session:  $FOCUS_SESSION_FILE"
echo "Foxglove: ws://$(hostname -I | awk '{print $1}'):$FOCUS_FOXGLOVE_PORT"
echo "Evidence: $shadow_dir/shadow_manifest.json"
echo "Safety: Hub GOAL=false; both receivers read-only; no Go2 bridge/WATER move."
