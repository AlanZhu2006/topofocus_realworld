#!/usr/bin/env bash
# Interactive board calibration -> persistent session -> strict debug workflow.
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HUB_DIR="$WORKSPACE/hub"
PYTHON_BIN="$HUB_DIR/.venv/bin/python"
HUB_PORT="${FOCUS_HUB_PORT:-8188}"
HUB_SESSION="${FOCUS_HUB_SESSION:-focus_hub_realworld}"
WSJ_TMUX_TARGET="${FOCUS_WSJ_SSH_TMUX:-focus_wsj_tunnel_20260722:sensor-audit}"
YUNJI_TMUX_TARGET="${FOCUS_YUNJI_SSH_TMUX:-focus_yunji_tunnel_20260722:sensor-audit}"
WSJ_ROOT="${FOCUS_WSJ_RELEASE_ROOT:-/home/nvidia/topofocus_buildmap_v2_20260723}"
YUNJI_ROOT="${FOCUS_YUNJI_RELEASE_ROOT:-/home/nyu/topofocus_buildmap_v2_20260723}"
WSJ_ENV_FILE="${FOCUS_WSJ_ENV_FILE:-/home/nvidia/focus_sender/go2_20260723.env}"
WSJ_BASE_CAMERA="${FOCUS_WSJ_BASE_CAMERA_CALIBRATION:-/home/nvidia/.local/state/topofocus/calibration/wsj_tinynav_camera_base_20260723_operator.json}"
YUNJI_BASE_CAMERA="${FOCUS_YUNJI_BASE_CAMERA_CALIBRATION:-/home/nyu/.local/state/topofocus/calibration/yunji_odin1_base_camera_20260723_operator.json}"
session_id=""
confirmation=""
goal_category="chair"
run_debug="true"
MIN_BOARD_SPACING_PX="7.0"
SSH_PROBE_TIMEOUT_S="${FOCUS_SSH_PROBE_TIMEOUT_S:-15}"

usage() {
  cat <<'EOF'
Usage: bash hub/scripts/calibrate_realworld_session.sh \
  --session-id YYYYMMDD-lab01 \
  --operator-confirmation OPERATOR_PRESENT_AND_BOARD_ONLY \
  [--goal-category chair] [--no-debug]

The robots must remain stationary. The script:
  1. starts raw mapping-only observation streams and one Foxglove preview;
  2. waits until both camera previews are live, then asks for Enter once the
     complete 7x10 board is visible in both views;
  3. computes the initial fit, asks the operator to move only the board, then
     waits for a second Enter and validates the independent holdout;
  4. copies the checksummed calibration to both robots through existing SSH/tmux;
  5. starts calibrated read-only stacks and completely fresh central maps;
  6. saves hub/runtime/sessions/<id>/session.json and makes it current;
  7. runs strict no-motion full-stack debug unless --no-debug is supplied.

It never starts a Go2 bridge or WATER live receiver. Formal motion remains a
separate realworld_oneclick.sh --mode live command with its own confirmation.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session-id) session_id="$2"; shift 2 ;;
    --operator-confirmation) confirmation="$2"; shift 2 ;;
    --goal-category) goal_category="$2"; shift 2 ;;
    --no-debug) run_debug="false"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$session_id" =~ ^[a-z0-9][a-z0-9_.-]{0,63}$ ]] || {
  echo "A lowercase filesystem-safe --session-id is required." >&2
  exit 2
}
[[ "$confirmation" == OPERATOR_PRESENT_AND_BOARD_ONLY ]] || {
  echo "Calibration requires OPERATOR_PRESENT_AND_BOARD_ONLY." >&2
  exit 2
}
case "$goal_category" in
  chair|bed|plant|toilet|tv|sofa) ;;
  *) echo "Unsupported HPC goal: $goal_category" >&2; exit 2 ;;
esac
[[ -t 0 ]] || {
  echo "Interactive calibration requires a terminal on stdin." >&2
  exit 2
}
[[ -x "$PYTHON_BIN" ]] || {
  echo "Missing Hub Python: $PYTHON_BIN" >&2
  exit 1
}
[[ "$SSH_PROBE_TIMEOUT_S" =~ ^[1-9][0-9]*$ ]] || {
  echo "FOCUS_SSH_PROBE_TIMEOUT_S must be a positive integer." >&2
  exit 2
}
for required in \
  "$HUB_DIR/runtime/tokens.json" \
  "$HUB_DIR/runtime/admin_token" \
  "$HUB_DIR/scripts/focus_hub_up.sh" \
  "$HUB_DIR/scripts/start_fresh_dual_maps.sh"; do
  [[ -r "$required" ]] || {
    echo "Missing local deployment input: $required" >&2
    exit 1
  }
done
"$PYTHON_BIN" "$HUB_DIR/tools/manage_realworld_session.py" \
  resolve --session-file current --mode status >/dev/null 2>&1 || true
repository_status="$(
  git -C "$WORKSPACE" status --porcelain --untracked-files=normal
)"
runtime_status="$(
  git -C "$WORKSPACE" status --porcelain --untracked-files=normal \
    -- hub source dependencies
)"
if [[ -n "$runtime_status" ]]; then
  echo "Calibration requires clean runtime code under hub/, source/, and dependencies/:" >&2
  printf '%s\n' "$runtime_status" >&2
  exit 1
fi
if [[ -n "$repository_status" ]]; then
  echo "WARNING: non-runtime repository changes will be recorded but will not block calibration:"
  printf '%s\n' "$repository_status"
fi
"$PYTHON_BIN" - "$WORKSPACE" <<'PY'
import subprocess
from pathlib import Path
import sys

root = Path(sys.argv[1])
dirty = subprocess.check_output(
    [
        "git",
        "status",
        "--porcelain",
        "--untracked-files=normal",
        "--",
        "hub",
        "source",
        "dependencies",
    ],
    cwd=root, text=True,
).strip()
if dirty:
    raise SystemExit(
        "Calibration requires clean runtime code; commit and verify "
        "hub/, source/, and dependencies/ first."
    )
PY
code_commit="$(git -C "$WORKSPACE" rev-parse HEAD)"

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

work_dir="$HUB_DIR/runtime/calibration_sessions/$session_id"
if [[ -e "$work_dir" ]]; then
  [[ -d "$work_dir" && ! -L "$work_dir" ]] || {
    echo "Calibration work path is not a normal directory: $work_dir" >&2
    exit 1
  }
  if [[ -e "$work_dir/shared_frame.json" ]]; then
    echo "Refusing to replace a completed calibration directory: $work_dir" >&2
    exit 1
  fi
  failed_root="$HUB_DIR/runtime/calibration_sessions/failed"
  failed_stamp="$(date -u +%Y%m%dT%H%M%SZ)-$$"
  failed_archive="$failed_root/${session_id}-${failed_stamp}"
  mkdir -p "$failed_root"
  mv -- "$work_dir" "$failed_archive"
  echo "Archived incomplete calibration attempt: $failed_archive"
fi
mkdir -p "$work_dir"

"$PYTHON_BIN" - "$WORKSPACE" "$work_dir/repository_state.json" <<'PY'
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

root = Path(sys.argv[1])
output = Path(sys.argv[2])
runtime_paths = ("hub", "source", "dependencies")

def git(*args: str) -> str:
    return subprocess.check_output(
        ["git", *args], cwd=root, text=True
    ).strip()

full_status = git("status", "--porcelain", "--untracked-files=normal")
runtime_status = git(
    "status",
    "--porcelain",
    "--untracked-files=normal",
    "--",
    *runtime_paths,
)
if runtime_status:
    raise SystemExit("runtime code changed during calibration preflight")
payload = {
    "schema_version": "focus-calibration-repository-state-v1",
    "git_commit": git("rev-parse", "HEAD"),
    "runtime_paths": list(runtime_paths),
    "runtime_worktree_clean": True,
    "full_worktree_clean": not bool(full_status),
    "nonruntime_status": full_status.splitlines(),
    "nonruntime_status_sha256": hashlib.sha256(
        full_status.encode("utf-8")
    ).hexdigest(),
    "classification": (
        "observed local repository state before physical calibration"
    ),
}
temporary = output.with_name(f".{output.name}.{os.getpid()}.tmp")
temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
os.replace(temporary, output)
PY

wsj_raw_transform="wsj-tinynav-depth-${session_id}-raw-v1"
yunji_raw_transform="yunji-odin1-${session_id}-raw-v1"
yunji_final_transform="yunji-odin1-board-${session_id}-v1"
calibration_id="shared-board-odin1-${session_id}-v1"
raw_config="$work_dir/robots_raw.json"
final_debug_config="$work_dir/robots_debug.json"
calibration_file="$work_dir/shared_frame.json"
fit_only_calibration="$work_dir/fit_only_unvalidated.json"
fit_pair="$work_dir/fit_pair.json"
holdout_pair="$work_dir/holdout_pair.json"
map_session="shared_maps_${session_id}"
foxglove_session="foxglove_relay_${session_id}"
wsj_map="$HUB_DIR/runtime/map_out_wsj_${session_id}"
yunji_map="$HUB_DIR/runtime/map_out_yunji_${session_id}"
wsj_remote_calibration="/home/nvidia/.local/state/topofocus/calibration/${session_id}_shared_frame.json"
yunji_remote_calibration="/home/nyu/.local/state/topofocus/calibration/${session_id}_shared_frame.json"

"$PYTHON_BIN" - "$raw_config" "$final_debug_config" \
  "$wsj_raw_transform" "$yunji_raw_transform" \
  "$yunji_final_transform" <<'PY'
import json
import os
from pathlib import Path
import sys

raw_path, final_path = map(Path, sys.argv[1:3])
wsj_raw, yunji_raw, yunji_final = sys.argv[3:6]

def write(path, wsj, yunji):
    payload = {
        "schema_version": "1.0",
        "shared_frame": "shared_world",
        "robots": {
            "robot-0": {"transform_version": wsj, "allow_goal": False},
            "robot-1": {"transform_version": yunji, "allow_goal": False},
        },
    }
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n")
    os.replace(temporary, path)

write(raw_path, wsj_raw, yunji_raw)
write(final_path, wsj_raw, yunji_final)
PY

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
  local deadline output rc
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
  # Queue bounded PTY lines without waiting for one round trip per manifest
  # chunk.  The marked remote_run below remains the completion/checksum
  # barrier, exactly as in realworld_oneclick.sh.
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
  [[ -n "$manifest" ]] || return 1
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
  echo "CALIBRATION_TIMING phase=dual_release_verification elapsed_s=$((SECONDS - started))"
  if [[ "$wsj_rc" != 0 || "$yunji_rc" != 0 ]]; then
    echo "Parallel release verification failed: WSJ=$wsj_rc Yunji=$yunji_rc" >&2
    return 1
  fi
}

stop_managed_hub() {
  local deadline line session start
  tmux kill-session -t "$HUB_SESSION" >/dev/null 2>&1 || true
  while IFS=$'\t' read -r session start; do
    [[ "$start" == *"focus_hub.api:app"* \
       && "$start" == *"--port"* \
       && "$start" == *"$HUB_PORT"* ]] \
      || continue
    tmux kill-session -t "$session" >/dev/null 2>&1 || true
  done < <(tmux list-panes -a -F $'#{session_name}\t#{pane_start_command}' 2>/dev/null || true)
  deadline=$((SECONDS + 15))
  while ss -tln 2>/dev/null | grep -q ":$HUB_PORT "; do
    (( SECONDS < deadline )) || {
      echo "Hub port $HUB_PORT is owned by an unmanaged process." >&2
      return 1
    }
    sleep 1
  done
}

start_hub_config() {
  local config="$1"
  stop_managed_hub
  bash "$HUB_DIR/scripts/focus_hub_up.sh" \
    --port "$HUB_PORT" \
    --no-glm \
    --no-pipeline \
    --session "$HUB_SESSION" \
    --robots-config "$config"
}

ensure_calibration_relay() {
  local deadline line session start
  while IFS=$'\t' read -r session start; do
    [[ "$start" == *"tools/foxglove_relay.py"* ]] || continue
    tmux kill-session -t "$session" >/dev/null 2>&1 || true
  done < <(tmux list-panes -a -F $'#{session_name}\t#{pane_start_command}' 2>/dev/null || true)
  deadline=$((SECONDS + 15))
  while ss -tln 2>/dev/null | grep -Eq ':(8765|8766) '; do
    (( SECONDS < deadline )) || {
      echo "Foxglove ports are owned by an unmanaged process." >&2
      return 1
    }
    sleep 1
  done
  mkdir -p "$work_dir/preview-wsj" "$work_dir/preview-yunji"
  tmux new-session -d -s "foxglove_calibration_${session_id}" -n relay \
    "bash -lc 'cd \"$HUB_DIR\"; exec .venv/bin/python -u tools/foxglove_relay.py --robot robot-0:wsj:\"$work_dir/preview-wsj\" --robot robot-1:yunji:\"$work_dir/preview-yunji\" --host 0.0.0.0 --port 8765 --preview-port 8766'"
  deadline=$((SECONDS + 20))
  until curl -fsS --max-time 2 http://127.0.0.1:8766/healthz >/dev/null 2>&1; do
    (( SECONDS < deadline )) || {
      tmux capture-pane -pt "foxglove_calibration_${session_id}:relay" -S -100 >&2 || true
      return 1
    }
    sleep 1
  done
}

wait_for_calibration_cameras() {
  local deadline health
  deadline=$((SECONDS + 90))
  while (( SECONDS < deadline )); do
    health="$(
      curl -fsS --max-time 3 \
        http://127.0.0.1:8766/healthz 2>/dev/null
    )" || {
      sleep 1
      continue
    }
    if FOCUS_CALIBRATION_PREVIEW_HEALTH="$health" \
      "$PYTHON_BIN" - <<'PY'
import json
import os

health = json.loads(os.environ["FOCUS_CALIBRATION_PREVIEW_HEALTH"])
robots = health.get("robots")
if not isinstance(robots, dict):
    raise SystemExit(1)
for name in ("wsj", "yunji"):
    row = robots.get(name)
    if not isinstance(row, dict) or row.get("camera_ready") is not True:
        raise SystemExit(1)
PY
    then
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for both Foxglove camera previews." >&2
  curl -sS --max-time 3 http://127.0.0.1:8766/healthz >&2 || true
  return 1
}

latest_sequence() {
  local robot_id="$1" token
  token="$(
    FOCUS_ROBOT_ID="$robot_id" "$PYTHON_BIN" \
      - "$HUB_DIR/runtime/tokens.json" <<'PY'
import json
import os
import sys

print(json.load(open(sys.argv[1]))[os.environ["FOCUS_ROBOT_ID"]])
PY
  )"
  curl -fsS --max-time 5 \
    -H "X-Robot-Token: $token" \
    "http://127.0.0.1:$HUB_PORT/v1/robots/$robot_id/observations/latest" \
    | "$PYTHON_BIN" -c 'import json,sys; print(int(json.load(sys.stdin)["last_sequence"]))'
}

capture_pair() {
  local label="$1" output="$2" reference_after="$3" other_after="$4"
  local deadline log_file
  log_file="$work_dir/${label}_selection.log"
  deadline=$((SECONDS + 60))
  while true; do
    if "$PYTHON_BIN" "$HUB_DIR/tools/select_live_board_pair.py" \
      --spool "$HUB_DIR/runtime/spool" \
      --reference-after-sequence "$reference_after" \
      --other-after-sequence "$other_after" \
      --reference-transform-version "$wsj_raw_transform" \
      --other-transform-version "$yunji_raw_transform" \
      --min-board-spacing-px "$MIN_BOARD_SPACING_PX" \
      --max-age-s 30 \
      --output "$output" >"$log_file" 2>&1; then
      return 0
    fi
    if grep -q "BOARD_TOO_SMALL" "$log_file"; then
      echo "Board geometry is visible but too small for repeatable PnP:" >&2
      grep "BOARD_TOO_SMALL" "$log_file" | tail -n 2 >&2
      echo "Move the complete board closer to both cameras and start a new calibration session." >&2
      return 1
    fi
    # Always make one final selection after crossing the deadline. A slow WSJ
    # keyframe can arrive during the last detector invocation or sleep.
    (( SECONDS >= deadline )) && break
    sleep 2
  done
  echo "Timed out finding a synchronized board pair; detector log:" >&2
  tail -n 100 "$log_file" >&2 || true
  return 1
}

deploy_calibration() {
  local target="$1" remote_path="$2" encoded expected remote_dir
  local suffix remote_encoded chunk
  expected="$(sha256sum "$calibration_file" | awk '{print $1}')"
  encoded="$(base64 -w0 "$calibration_file")"
  remote_dir="${remote_path%/*}"
  suffix="$(date +%s%N)_${RANDOM}"
  remote_encoded="${remote_path}.focus-${suffix}.b64"
  remote_run "$target" \
    "set -e; install -d -m 700 '$remote_dir'; umask 077; : > '$remote_encoded'"
  while IFS= read -r chunk || [[ -n "$chunk" ]]; do
    remote_queue "$target" \
      "printf '%s' '$chunk' >> '$remote_encoded'"
  done < <(printf '%s' "$encoded" | fold -w 1000)
  remote_run "$target" \
    "set -e; base64 -d '$remote_encoded' > '${remote_path}.tmp'; test \"\$(sha256sum '${remote_path}.tmp' | awk '{print \$1}')\" = '$expected'; python3 -m json.tool '${remote_path}.tmp' >/dev/null; chmod 600 '${remote_path}.tmp'; mv '${remote_path}.tmp' '$remote_path'; unlink '$remote_encoded'; test \"\$(sha256sum '$remote_path' | awk '{print \$1}')\" = '$expected'"
}

deploy_calibration_pair() {
  local wsj_pid yunji_pid wsj_rc=0 yunji_rc=0 started
  started="$SECONDS"
  (
    trap - EXIT INT TERM
    deploy_calibration "$WSJ_TMUX_TARGET" "$wsj_remote_calibration"
  ) &
  wsj_pid=$!
  (
    trap - EXIT INT TERM
    deploy_calibration "$YUNJI_TMUX_TARGET" "$yunji_remote_calibration"
  ) &
  yunji_pid=$!
  wait "$wsj_pid" || wsj_rc=$?
  wait "$yunji_pid" || yunji_rc=$?
  echo "CALIBRATION_TIMING phase=dual_calibration_deploy elapsed_s=$((SECONDS - started))"
  if [[ "$wsj_rc" != 0 || "$yunji_rc" != 0 ]]; then
    echo "Parallel calibration deployment failed: WSJ=$wsj_rc Yunji=$yunji_rc" >&2
    return 1
  fi
}

calibration_cleanup_required="false"
cleanup_calibration_failure() {
  local rc=$?
  trap - EXIT INT TERM
  if [[ "$rc" != 0 && "$calibration_cleanup_required" == "true" ]]; then
    echo "Calibration failed; stopping mapping-only calibration streams."
    tmux kill-session -t "foxglove_calibration_${session_id}" \
      >/dev/null 2>&1 || true
    remote_run "$WSJ_TMUX_TARGET" \
      "tmux kill-window -t tinynav_semantic_nav_auto:calibration-sender >/dev/null 2>&1 || true; source /home/nvidia/twork/tinynav_setup.bash; timeout 5 ros2 topic pub --once /nav/paused std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true" \
      || true
    remote_run "$YUNJI_TMUX_TARGET" \
      "sudo -n systemctl stop focus-yunji-calibration-observation-v1.service >/dev/null 2>&1 || true" \
      || true
  fi
  exit "$rc"
}
trap cleanup_calibration_failure EXIT INT TERM

echo "Verifying byte-identical robot release roots before calibration."
verify_remote_release_pair
calibration_cleanup_required="true"
echo "Starting fail-closed raw calibration observation."
start_hub_config "$raw_config"
ensure_calibration_relay
raw_start_s="$SECONDS"
remote_pair \
  "FOCUS_WSJ_ENV_FILE='$WSJ_ENV_FILE' FOCUS_HUB_BASE_URL=http://127.0.0.1:18089 FOCUS_FOXGLOVE_PREVIEW_URL=http://127.0.0.1:18766 bash '$WSJ_ROOT/hub/robot_overlay/start_wsj_calibration_observation.sh' --transform-version '$wsj_raw_transform' --operator-confirmation OPERATOR_PRESENT_AND_BOARD_ONLY" \
  "bash '$YUNJI_ROOT/hub/robot_overlay/start_yunji_calibration_observation.sh' --transform-version '$yunji_raw_transform' --operator-confirmation OPERATOR_PRESENT_AND_BOARD_ONLY"
echo "CALIBRATION_TIMING phase=dual_raw_observation_start elapsed_s=$((SECONDS - raw_start_s))"

echo "Foxglove: ws://$(hostname -I | awk '{print $1}'):8765"
wait_for_calibration_cameras
echo "CALIBRATION_PREVIEW_READY: both WSJ and Yunji camera previews are live."
echo "WSJ intentionally uses the native grayscale infra1 calibration view (no RGB/depth mosaic)."
read -r -p "Confirm the COMPLETE 7x10 board is clearly visible and reasonably large in BOTH previews, then press Enter to compute the initial fit. "
fit_after_wsj="$(latest_sequence robot-0)"
fit_after_yunji="$(latest_sequence robot-1)"
capture_pair fit "$fit_pair" "$fit_after_wsj" "$fit_after_yunji"

read -r fit_wsj fit_yunji < <(
  "$PYTHON_BIN" - "$fit_pair" <<'PY'
import json
import sys

fit = json.load(open(sys.argv[1]))
print(
    int(fit["reference"]["sequence"]),
    int(fit["other"]["sequence"]),
)
PY
)

"$PYTHON_BIN" "$HUB_DIR/tools/calibrate_gravity_shared_frame_via_board.py" \
  --spool "$HUB_DIR/runtime/spool" \
  --reference-robot robot-0 \
  --other-robot robot-1 \
  --reference-sequence "$fit_wsj" \
  --other-sequence "$fit_yunji" \
  --other-pose-is-camera \
  --min-board-spacing-px "$MIN_BOARD_SPACING_PX" \
  --transform-version "$yunji_final_transform" \
  --calibration-id "$calibration_id" \
  --output "$fit_only_calibration"

"$PYTHON_BIN" - "$fit_only_calibration" <<'PY'
import json
import sys

payload = json.load(open(sys.argv[1]))
frame = payload["calibration_frame"]
gravity = payload["gravity_validation"]
print(
    "INITIAL_BOARD_FIT_READY: "
    f"sync_skew={frame['sync_skew_s']:.3f}s, "
    f"center_residual={frame['board_center_translation_residual_m']:.4f}m, "
    f"normal_residual={frame['board_normal_residual_deg']:.3f}deg, "
    f"tilt={gravity['shared_transform_tilt_deg']:.6f}deg"
)
PY

read -r -p "Move ONLY the board, preferably at least 30 cm sideways without tilting it. When the COMPLETE board is again clearly visible and large in BOTH previews, press Enter to validate and finish. "
holdout_after_wsj="$(latest_sequence robot-0)"
holdout_after_yunji="$(latest_sequence robot-1)"
capture_pair holdout "$holdout_pair" "$holdout_after_wsj" "$holdout_after_yunji"

read -r holdout_wsj holdout_yunji < <(
  "$PYTHON_BIN" - "$holdout_pair" <<'PY'
import json
import sys

holdout = json.load(open(sys.argv[1]))
print(
    int(holdout["reference"]["sequence"]),
    int(holdout["other"]["sequence"]),
)
PY
)

"$PYTHON_BIN" "$HUB_DIR/tools/calibrate_gravity_shared_frame_via_board.py" \
  --spool "$HUB_DIR/runtime/spool" \
  --reference-robot robot-0 \
  --other-robot robot-1 \
  --reference-sequence "$fit_wsj" \
  --other-sequence "$fit_yunji" \
  --holdout-reference-sequence "$holdout_wsj" \
  --holdout-other-sequence "$holdout_yunji" \
  --other-pose-is-camera \
  --min-board-spacing-px "$MIN_BOARD_SPACING_PX" \
  --transform-version "$yunji_final_transform" \
  --calibration-id "$calibration_id" \
  --output "$calibration_file"

echo "CALIBRATION_HOLDOUT_PASSED: deploying the checked shared transform."
deploy_calibration_pair

echo "Switching both robots to calibrated read-only observation."
start_hub_config "$final_debug_config"
debug_start_s="$SECONDS"
remote_pair \
  "tmux kill-window -t tinynav_semantic_nav_auto:calibration-sender >/dev/null 2>&1 || true; FOCUS_SHARED_CALIBRATION_FILE='$wsj_remote_calibration' FOCUS_WSJ_BASE_CAMERA_CALIBRATION_FILE='$WSJ_BASE_CAMERA' FOCUS_WSJ_TRANSFORM_VERSION='$wsj_raw_transform' FOCUS_SHARED_CALIBRATION_ID='$calibration_id' FOCUS_DEPLOYMENT_COMMIT='$code_commit' bash '$WSJ_ROOT/hub/robot_overlay/start_wsj_buildmap_v2.sh' --mode debug" \
  "FOCUS_YUNJI_SHARED_CALIBRATION_FILE='$yunji_remote_calibration' FOCUS_YUNJI_BASE_CAMERA_CALIBRATION='$YUNJI_BASE_CAMERA' FOCUS_YUNJI_TRANSFORM_VERSION='$yunji_final_transform' FOCUS_SHARED_CALIBRATION_ID='$calibration_id' FOCUS_DEPLOYMENT_COMMIT='$code_commit' bash '$YUNJI_ROOT/hub/robot_overlay/start_yunji_v2.sh' --mode debug"
echo "CALIBRATION_TIMING phase=dual_calibrated_debug_start elapsed_s=$((SECONDS - debug_start_s))"

deadline=$((SECONDS + 90))
while true; do
  wsj_start_after="$(latest_sequence robot-0)"
  yunji_start_after="$(latest_sequence robot-1)"
  if "$PYTHON_BIN" - "$HUB_DIR/runtime/spool" \
    "$wsj_start_after" "$yunji_start_after" \
    "$wsj_raw_transform" "$yunji_final_transform" <<'PY'
import json
from pathlib import Path
import sys
spool = Path(sys.argv[1])
for robot, sequence, expected in (
    ("robot-0", int(sys.argv[2]), sys.argv[4]),
    ("robot-1", int(sys.argv[3]), sys.argv[5]),
):
    path = spool / robot / f"{sequence:020d}" / "metadata.json"
    payload = json.loads(path.read_text())
    if payload["pose"]["transform_version"] != expected:
        raise SystemExit(1)
    if payload.get("mapping_only") is not False:
        raise SystemExit(1)
PY
  then
    break
  fi
  (( SECONDS < deadline )) || {
    echo "Timed out waiting for calibrated command-capable observations." >&2
    exit 1
  }
  sleep 1
done

tmux kill-session -t "foxglove_calibration_${session_id}" >/dev/null 2>&1 || true
deadline=$((SECONDS + 15))
while ss -tln 2>/dev/null | grep -Eq ':(8765|8766) '; do
  (( SECONDS < deadline )) || {
    echo "Calibration Foxglove relay did not stop." >&2
    exit 1
  }
  sleep 1
done

bash "$HUB_DIR/scripts/start_fresh_dual_maps.sh" \
  --session-tag "$session_id" \
  --calibration-id "$calibration_id" \
  --wsj-transform "$wsj_raw_transform" \
  --yunji-transform "$yunji_final_transform" \
  --wsj-start-after "$wsj_start_after" \
  --yunji-start-after "$yunji_start_after" \
  --goal-category "$goal_category" \
  --code-commit "$code_commit" \
  --hub-url "http://127.0.0.1:$HUB_PORT"

"$PYTHON_BIN" "$HUB_DIR/tools/manage_realworld_session.py" create \
  --session-id "$session_id" \
  --calibration-file "$calibration_file" \
  --wsj-map "$wsj_map" \
  --yunji-map "$yunji_map" \
  --wsj-start-after "$wsj_start_after" \
  --yunji-start-after "$yunji_start_after" \
  --wsj-remote-root "$WSJ_ROOT" \
  --yunji-remote-root "$YUNJI_ROOT" \
  --wsj-remote-calibration "$wsj_remote_calibration" \
  --yunji-remote-calibration "$yunji_remote_calibration" \
  --wsj-base-camera-calibration "$WSJ_BASE_CAMERA" \
  --yunji-base-camera-calibration "$YUNJI_BASE_CAMERA" \
  --wsj-ssh-tmux "$WSJ_TMUX_TARGET" \
  --yunji-ssh-tmux "$YUNJI_TMUX_TARGET" \
  --hub-port "$HUB_PORT" \
  --hub-session "$HUB_SESSION" \
  --glm-session "glm_${session_id}" \
  --map-session "$map_session" \
  --foxglove-session "$foxglove_session" \
  --map-goal-category "$goal_category" \
  --set-current

deadline=$((SECONDS + 240))
until "$PYTHON_BIN" "$HUB_DIR/tools/manage_realworld_session.py" \
  resolve --session-file current --mode debug >/dev/null 2>&1; do
  for window in wsj yunji; do
    if [[ "$(tmux display-message -p -t "$map_session:$window" '#{pane_dead}' 2>/dev/null)" == 1 ]]; then
      tmux capture-pane -pt "$map_session:$window" -S -120 >&2 || true
      exit 1
    fi
  done
  (( SECONDS < deadline )) || {
    echo "Timed out waiting for fresh calibrated maps." >&2
    exit 1
  }
  sleep 2
done

echo "Calibration session prepared: hub/runtime/sessions/$session_id/session.json"
calibration_cleanup_required="false"
if [[ "$run_debug" == "true" ]]; then
  bash "$HUB_DIR/scripts/realworld_oneclick.sh" \
    --session-file current \
    --mode debug \
    --scene-id "debug-${session_id}" \
    --goal-category "$goal_category"
else
  echo "Run strict debug before live:"
  echo "  bash hub/scripts/realworld_oneclick.sh --session-file current --mode debug --goal-category $goal_category"
fi
