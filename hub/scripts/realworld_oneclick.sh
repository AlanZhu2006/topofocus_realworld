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
session_env_file=""
live_cleanup_required="false"
cleanup_started="false"

usage() {
  cat <<'EOF'
Usage:
  bash hub/scripts/realworld_oneclick.sh --session-file current --mode debug \
    [--goal-category chair] [--scene-id debug-chair]

  bash hub/scripts/realworld_oneclick.sh --session-file current --mode live \
    --scene-id SCENE --episode-id EPISODE --goal-category chair \
    --operator-confirmation OPERATOR_PRESENT_AND_ROBOTS_CLEAR

debug restarts a clean Hub epoch, verifies exact session/map/Foxglove
identities, runs both robot receivers read-only, freezes fresh synchronized
inputs, runs the real VLM, and records a no-motion gate for this Git commit.

live is unlocked only by that same-session debug record. It clears any old
Hub decision epoch, prepares a fresh shadow decision with the robots still
read-only, then arms the local TinyNav/WATER paths for one supervised episode.
The exit trap always returns both robots and Hub to fail-closed debug mode.
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
for target in "$WSJ_TMUX_TARGET" "$YUNJI_TMUX_TARGET"; do
  tmux display-message -p -t "$target" '#{pane_current_command}' \
    >/dev/null 2>&1 || {
      echo "Existing SSH/tmux target is unavailable: $target" >&2
      exit 1
    }
done

remote_run() {
  local target="$1" command="$2" token line deadline output rc
  token="FOCUS_$(date +%s%N)_${RANDOM}"
  printf -v line \
    'bash -lc %q; rc=$?; echo; echo __%s_RC=$rc' \
    "$command" "$token"
  tmux send-keys -t "$target" "$line" Enter
  deadline=$((SECONDS + 180))
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
    remote_run "$target" \
      "printf '%s' '$chunk' >> '$remote_encoded'" || {
        remote_run "$target" \
          "test ! -e '$remote_encoded' || unlink '$remote_encoded'" || true
        return 1
      }
  # Keep each tmux/PTY line well below the observed remote canonical-input
  # limit. Larger chunks can be accepted by tmux but silently truncated by
  # the interactive SSH terminal.
  done < <(printf '%s' "$encoded" | fold -w 1000)
  remote_run "$target" \
    "set +e; base64 -d '$remote_encoded' > '$remote_manifest'; rc=\$?; if [ \"\$rc\" -eq 0 ]; then (cd $quoted_root && sha256sum --quiet -c '$remote_manifest'); rc=\$?; fi; unlink '$remote_encoded'; unlink '$remote_manifest'; exit \"\$rc\""
}

printf -v WSJ_ENV \
  'FOCUS_SHARED_CALIBRATION_FILE=%q FOCUS_WSJ_BASE_CAMERA_CALIBRATION_FILE=%q FOCUS_WSJ_TRANSFORM_VERSION=%q FOCUS_SHARED_CALIBRATION_ID=%q FOCUS_HUB_BASE_URL=%q FOCUS_FOXGLOVE_PREVIEW_URL=%q' \
  "$FOCUS_WSJ_REMOTE_CALIBRATION" \
  "$FOCUS_WSJ_REMOTE_BASE_CAMERA" \
  "$FOCUS_WSJ_TRANSFORM" \
  "$FOCUS_CALIBRATION_ID" \
  "$FOCUS_WSJ_REMOTE_HUB_URL" \
  "$FOCUS_WSJ_REMOTE_PREVIEW_URL"
printf -v YUNJI_ENV \
  'FOCUS_YUNJI_SHARED_CALIBRATION_FILE=%q FOCUS_YUNJI_BASE_CAMERA_CALIBRATION=%q FOCUS_YUNJI_TRANSFORM_VERSION=%q FOCUS_SHARED_CALIBRATION_ID=%q FOCUS_HUB_BASE_URL=%q' \
  "$FOCUS_YUNJI_REMOTE_CALIBRATION" \
  "$FOCUS_YUNJI_REMOTE_BASE_CAMERA" \
  "$FOCUS_YUNJI_TRANSFORM" \
  "$FOCUS_CALIBRATION_ID" \
  "$FOCUS_YUNJI_REMOTE_HUB_URL"
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
  deadline=$((SECONDS + 20))
  until foxglove_matches; do
    (( SECONDS < deadline )) || {
      tmux capture-pane -pt "$FOXGLOVE_SESSION:relay" -S -120 >&2 || true
      return 1
    }
    sleep 1
  done
}

start_read_only_robots() {
  remote_run "$WSJ_TMUX_TARGET" \
    "source /home/nvidia/twork/tinynav_setup.bash; timeout 5 ros2 topic pub --once /nav/paused std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true; timeout 5 ros2 topic pub --once /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:go2-bridge >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:v2-receiver >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:hub-sender >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:calibration-sender >/dev/null 2>&1 || true; $WSJ_ENV bash $WSJ_LAUNCHER --mode debug"
  remote_run "$YUNJI_TMUX_TARGET" \
    "for unit in focus-yunji-calibration-observation-v1.service focus-yunji-v2-debug-v2.service focus-yunji-v2-live-v2.service focus-yunji-command-observation-v2.service; do sudo -n systemctl stop \"\$unit\" >/dev/null 2>&1 || true; sudo -n systemctl reset-failed \"\$unit\" >/dev/null 2>&1 || true; done; $YUNJI_ENV bash $YUNJI_LAUNCHER --mode debug"
}

arm_live_robots() {
  live_cleanup_required="true"
  remote_run "$WSJ_TMUX_TARGET" \
    "source /home/nvidia/twork/tinynav_setup.bash; timeout 5 ros2 topic pub --once /nav/paused std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true; timeout 5 ros2 topic pub --once /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:go2-bridge >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:v2-receiver >/dev/null 2>&1 || true; $WSJ_ENV bash $WSJ_LAUNCHER --mode live --operator-confirmation OPERATOR_PRESENT_AND_WSJ_CLEAR"
  remote_run "$YUNJI_TMUX_TARGET" \
    "for unit in focus-yunji-v2-debug-v2.service focus-yunji-v2-live-v2.service; do sudo -n systemctl stop \"\$unit\" >/dev/null 2>&1 || true; sudo -n systemctl reset-failed \"\$unit\" >/dev/null 2>&1 || true; done; $YUNJI_ENV bash $YUNJI_LAUNCHER --mode live --operator-confirmation OPERATOR_PRESENT_AND_YUNJI_CLEAR"
}

disarm_live_stack() {
  [[ "$live_cleanup_required" == "true" ]] || return 0
  [[ "$cleanup_started" == "false" ]] || return 0
  cleanup_started="true"
  echo "Fail-closed cleanup: disabling Hub GOAL and both robot command paths."
  restart_hub "$FOCUS_DEBUG_ROBOT_CONFIG" false \
    || echo "WARNING: Hub debug restart failed." >&2
  remote_run "$WSJ_TMUX_TARGET" \
    "source /home/nvidia/twork/tinynav_setup.bash; timeout 5 ros2 topic pub --once /nav/paused std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true; timeout 5 ros2 topic pub --once /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:go2-bridge >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:v2-receiver >/dev/null 2>&1 || true; $WSJ_ENV bash $WSJ_LAUNCHER --mode debug" \
    || echo "WARNING: WSJ guarded stop ran but debug receiver restart failed." >&2
  remote_run "$YUNJI_TMUX_TARGET" \
    "for unit in focus-yunji-v2-live-v2.service focus-yunji-v2-debug-v2.service; do sudo -n systemctl stop \"\$unit\" >/dev/null 2>&1 || true; sudo -n systemctl reset-failed \"\$unit\" >/dev/null 2>&1 || true; done; $YUNJI_ENV bash $YUNJI_LAUNCHER --mode debug" \
    || echo "WARNING: Yunji debug receiver restart failed." >&2
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

echo "Session $FOCUS_SESSION_ID: restarting a clean fail-closed Hub epoch."
echo "Verifying that both robot release roots match this Git checkout."
verify_remote_release "$WSJ_TMUX_TARGET" "$WSJ_ROOT"
verify_remote_release "$YUNJI_TMUX_TARGET" "$YUNJI_ROOT"
restart_hub "$FOCUS_DEBUG_ROBOT_CONFIG" false
ensure_maps
ensure_glm
ensure_foxglove
start_read_only_robots

final_hub_epoch_ns="$(date +%s%N)"
if [[ "$mode" == live ]]; then
  # The Hub is armed only while both robot receivers are still read-only. A
  # fresh process has no old v2 decisions. The accepted observation/history
  # epoch is established below before either motion receiver is started.
  live_cleanup_required="true"
  restart_hub "$FOCUS_LIVE_ROBOT_CONFIG" true
  final_hub_epoch_ns="$(date +%s%N)"
fi

wait_for_hub_epoch() {
  local deadline admin_token status
  admin_token="$(<"$FOCUS_ADMIN_TOKEN_FILE")"
  deadline=$((SECONDS + 90))
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
wait_for_hub_epoch

stamp="$(date +%Y%m%d_%H%M%S_%N)"
run_dir="$HUB_DIR/runtime/oneclick_${FOCUS_SESSION_ID}_${mode}_${scene_id}_${stamp}"
mkdir "$run_dir"
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

if [[ "$mode" == debug ]]; then
  "$PYTHON_BIN" "$SESSION_MANAGER" mark-debug \
    --session-file "$FOCUS_SESSION_FILE" \
    --shadow-manifest "$shadow_dir/shadow_manifest.json" \
    --debug-safety-confirmation DEBUG_STACK_NO_MOTION_VERIFIED
  echo "DEBUG_FULLSTACK_READY"
  echo "Session:  $FOCUS_SESSION_FILE"
  echo "Foxglove: ws://$(hostname -I | awk '{print $1}'):$FOCUS_FOXGLOVE_PORT"
  echo "Evidence: $shadow_dir/shadow_manifest.json"
  echo "Safety: Hub GOAL=false; both receivers read-only; no Go2 bridge/WATER move."
  exit 0
fi

arm_live_robots
episode_dir="$run_dir/episode"
"$PYTHON_BIN" -u "$HUB_DIR/tools/run_v2_supervised_episode.py" \
  --manifest "$shadow_dir/shadow_manifest.json" \
  --registry-state "$HUB_DIR/runtime/state/registry_state.json" \
  --robot-config "$FOCUS_LIVE_ROBOT_CONFIG" \
  --scene-id "$scene_id" \
  --episode-id "$episode_id" \
  --output "$episode_dir" \
  --hub-url "$HUB_URL" \
  --admin-token-file "$FOCUS_ADMIN_TOKEN_FILE" \
  --enable-live-goal-publication \
  --operator-confirmation OPERATOR_PRESENT_AND_ROBOTS_CLEAR

echo "LIVE_EPISODE_FINISHED: $episode_dir/episode_report.json"
echo "SR/SPL remains pending until terminal target/goal-region evidence is added."
