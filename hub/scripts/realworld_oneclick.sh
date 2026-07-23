#!/usr/bin/env bash
# Minimal two-mode entry point for the current dual-robot real-world session.
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HUB_DIR="$WORKSPACE/hub"
PYTHON_BIN="$HUB_DIR/.venv/bin/python"
HUB_PORT=8188
HUB_URL="http://127.0.0.1:$HUB_PORT"
GLM_URL="http://127.0.0.1:31511/v1"
HUB_SESSION="focus_hub_observation_20260723"
WSJ_TMUX_TARGET="${FOCUS_WSJ_SSH_TMUX:-focus_wsj_tunnel_20260722:sensor-audit}"
YUNJI_TMUX_TARGET="${FOCUS_YUNJI_SSH_TMUX:-focus_yunji_tunnel_20260722:sensor-audit}"
WSJ_ROOT="${FOCUS_WSJ_RELEASE_ROOT:-/home/nvidia/topofocus_buildmap_v2_20260723}"
YUNJI_ROOT="${FOCUS_YUNJI_RELEASE_ROOT:-/home/nyu/topofocus_buildmap_v2_20260723}"
WSJ_MAP="$HUB_DIR/runtime/map_out_wsj_20260724_rebuild_v12_router025"
YUNJI_MAP="$HUB_DIR/runtime/map_out_yunji_20260724_rebuild_v12_router025"
MAP_SESSION="shared_maps_20260724_rebuild_v12_router025"
FOXGLOVE_SESSION="foxglove_relay_20260724_rebuild_v12_router025"
CALIBRATION_ID="shared-board-odin1-20260723-v3"
mode=""
goal_category="chair"
scene_id=""
episode_id=""
confirmation=""

usage() {
  cat <<'EOF'
Usage:
  bash hub/scripts/realworld_oneclick.sh --mode debug \
    [--goal-category chair] [--scene-id debug-chair]

  bash hub/scripts/realworld_oneclick.sh --mode live \
    --scene-id SCENE --episode-id EPISODE --goal-category chair \
    --operator-confirmation OPERATOR_PRESENT_AND_ROBOTS_CLEAR

debug: Hub GOAL disabled, both receivers read-only, no Go2 bridge, real VLM
       shadow + Foxglove display only.
live:  exact operator confirmation, fresh receiver heartbeats, expiring v2
       GOALs, TinyNav/WATER local planners and one supervised episode.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
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
  *) echo "Unsupported HPC target: $goal_category" >&2; exit 2 ;;
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
[[ "$scene_id" =~ ^[A-Za-z0-9_.-]+$ ]] || {
  echo "Scene ID must be filesystem-safe." >&2
  exit 2
}
[[ -x "$PYTHON_BIN" ]] || { echo "Missing Hub Python." >&2; exit 1; }
for path in "$WSJ_MAP" "$YUNJI_MAP" "$HUB_DIR/runtime/admin_token"; do
  [[ -e "$path" ]] || { echo "Missing current-session input: $path" >&2; exit 1; }
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
  printf -v line 'bash -lc %q; rc=$?; echo __%s_RC=$rc' "$command" "$token"
  tmux send-keys -t "$target" "$line" Enter
  deadline=$((SECONDS + 150))
  while (( SECONDS < deadline )); do
    output="$(tmux capture-pane -pt "$target" -S -220 2>/dev/null || true)"
    rc="$(
      sed -n "s/^__${token}_RC=\\([0-9][0-9]*\\)$/\\1/p" \
        <<<"$output" | tail -n 1
    )"
    if [[ -n "$rc" ]]; then
      if [[ "$rc" != 0 ]]; then
        echo "$output" | tail -n 100 >&2
        return "$rc"
      fi
      return 0
    fi
    sleep 1
  done
  echo "Remote command timed out on $target" >&2
  return 124
}

live_cleanup_started="false"
disarm_live_stack() {
  [[ "$mode" == live ]] || return 0
  [[ "$live_cleanup_started" == false ]] || return 0
  live_cleanup_started="true"
  echo "Fail-closed cleanup: disabling both robot command paths and Hub GOAL."
  remote_run "$WSJ_TMUX_TARGET" \
    "source /home/nvidia/twork/tinynav_setup.bash; ros2 topic pub --once /nav/paused std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:go2-bridge >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:v2-receiver >/dev/null 2>&1 || true; bash '$WSJ_ROOT/hub/robot_overlay/start_wsj_buildmap_v2.sh' --mode debug" \
    || echo "WARNING: WSJ automatic debug restart failed; guarded bridge kill was attempted." >&2
  remote_run "$YUNJI_TMUX_TARGET" \
    "bash '$YUNJI_ROOT/hub/robot_overlay/start_yunji_v2.sh' --mode debug" \
    || echo "WARNING: Yunji automatic debug restart failed." >&2
  tmux kill-session -t "$HUB_SESSION" >/dev/null 2>&1 || true
  cleanup_deadline=$((SECONDS + 15))
  while ss -tln 2>/dev/null | grep -q ":$HUB_PORT "; do
    if (( SECONDS >= cleanup_deadline )); then
      echo "WARNING: Hub port did not close during fail-closed cleanup." >&2
      break
    fi
    sleep 1
  done
  if ! ss -tln 2>/dev/null | grep -q ":$HUB_PORT "; then
    bash "$HUB_DIR/scripts/focus_hub_up.sh" \
      --port "$HUB_PORT" \
      --no-glm \
      --no-pipeline \
      --session "$HUB_SESSION" \
      --robots-config \
      "$HUB_DIR/config/experiments/robots_20260723_debug.json" \
      || echo "WARNING: Hub debug restart failed." >&2
  fi
}

cleanup_on_exit() {
  local rc=$?
  trap - EXIT
  disarm_live_stack
  exit "$rc"
}
trap cleanup_on_exit EXIT

desired_config="$HUB_DIR/config/experiments/robots_20260723_${mode}.json"
desired_goal="false"
[[ "$mode" == live ]] && desired_goal="true"
hub_matches="false"
hub_start_command="$(
  tmux display-message -p -t "$HUB_SESSION" '#{pane_start_command}' \
    2>/dev/null || true
)"
if [[ "$hub_start_command" == *"$desired_config"* ]] \
   && health_json="$(curl -fsS --max-time 3 "$HUB_URL/healthz" 2>/dev/null)"; then
  if FOCUS_HEALTH_JSON="$health_json" FOCUS_EXPECT_GOAL="$desired_goal" \
    "$PYTHON_BIN" -c '
import json, os
health = json.loads(os.environ["FOCUS_HEALTH_JSON"])
expected = os.environ["FOCUS_EXPECT_GOAL"] == "true"
raise SystemExit(0 if all(
    health.get("goal_output_enabled", {}).get(robot_id) is expected
    for robot_id in ("robot-0", "robot-1")
) else 1)
'; then
    hub_matches="true"
  fi
fi
if [[ "$hub_matches" != "true" ]]; then
  tmux kill-session -t "$HUB_SESSION" >/dev/null 2>&1 || true
  deadline=$((SECONDS + 15))
  while ss -tln 2>/dev/null | grep -q ":$HUB_PORT "; do
    (( SECONDS < deadline )) || {
      echo "Hub port $HUB_PORT did not close." >&2
      exit 1
    }
    sleep 1
  done
  bash "$HUB_DIR/scripts/focus_hub_up.sh" \
    --port "$HUB_PORT" \
    --no-glm \
    --no-pipeline \
    --session "$HUB_SESSION" \
    --robots-config "$desired_config"
fi

curl -fsS --max-time 5 "$HUB_URL/healthz" >/dev/null
if ! curl -fsS --max-time 5 "$GLM_URL/models" >/dev/null; then
  tmux has-session -t glm_offline_20260723_fullstack 2>/dev/null || \
    tmux new-session -d -s glm_offline_20260723_fullstack -n server \
      "bash -lc 'cd \"$WORKSPACE\"; FOCUS_GLM_PORT=31511 exec bash hub/scripts/run_glm_offline.sh'"
  deadline=$((SECONDS + 180))
  until curl -fsS --max-time 5 "$GLM_URL/models" >/dev/null 2>&1; do
    (( SECONDS < deadline )) || { echo "GLM startup timed out." >&2; exit 1; }
    sleep 2
  done
fi
for window in wsj yunji; do
  [[ "$(tmux display-message -p -t "$MAP_SESSION:$window" '#{pane_dead}' 2>/dev/null)" == 0 ]] || {
    echo "Current $window map daemon is not running." >&2
    exit 1
  }
done
if ! ss -tln 2>/dev/null | grep -q ':8765 '; then
  tmux new-session -d -s "$FOXGLOVE_SESSION" -n relay \
    "bash -lc 'cd \"$HUB_DIR\"; exec .venv/bin/python -u tools/foxglove_relay.py --robot robot-0:wsj:\"$WSJ_MAP\" --robot robot-1:yunji:\"$YUNJI_MAP\" --host 0.0.0.0 --port 8765 --preview-port 8766 --fuse'"
fi

if [[ "$mode" == live ]]; then
  remote_run "$WSJ_TMUX_TARGET" \
    "source /home/nvidia/twork/tinynav_setup.bash; ros2 topic pub --once /nav/paused std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:go2-bridge >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:v2-receiver >/dev/null 2>&1 || true; bash '$WSJ_ROOT/hub/robot_overlay/start_wsj_buildmap_v2.sh' --mode live --operator-confirmation OPERATOR_PRESENT_AND_WSJ_CLEAR"
  remote_run "$YUNJI_TMUX_TARGET" \
    "bash '$YUNJI_ROOT/hub/robot_overlay/start_yunji_v2.sh' --mode live --operator-confirmation OPERATOR_PRESENT_AND_YUNJI_CLEAR"
else
  remote_run "$WSJ_TMUX_TARGET" \
    "source /home/nvidia/twork/tinynav_setup.bash; ros2 topic pub --once /nav/paused std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:go2-bridge >/dev/null 2>&1 || true; tmux kill-window -t tinynav_semantic_nav_auto:v2-receiver >/dev/null 2>&1 || true; bash '$WSJ_ROOT/hub/robot_overlay/start_wsj_buildmap_v2.sh' --mode debug"
  remote_run "$YUNJI_TMUX_TARGET" \
    "bash '$YUNJI_ROOT/hub/robot_overlay/start_yunji_v2.sh' --mode debug"
fi

# Let both restarted senders contribute fresh synchronized command metadata.
sleep 4
if [[ "$mode" == live ]]; then
  sync_deadline=$((SECONDS + 60))
  while true; do
    if sync_status="$(
      "$PYTHON_BIN" - "$WSJ_MAP" "$YUNJI_MAP" "$HUB_DIR/runtime/spool" <<'PY'
import json
from pathlib import Path
import sys
import time

map_paths = [Path(sys.argv[1]), Path(sys.argv[2])]
spool = Path(sys.argv[3])
robot_ids = ("robot-0", "robot-1")
try:
    rows = []
    blockers = []
    for robot_id, map_path in zip(robot_ids, map_paths):
        live_status = json.loads((map_path / "live_status.json").read_text())
        blocked = live_status.get("mapping_blocked_reason")
        if blocked is not None:
            blockers.append(f"{robot_id}:map_blocked={blocked}")
        summary = json.loads((map_path / "map_summary.json").read_text())
        sequence = int(
            summary["semantic_mapping"]["yolo_reinforcement"]["last_sequence"]
        )
        metadata_path = spool / robot_id / f"{sequence:020d}" / "metadata.json"
        metadata = json.loads(metadata_path.read_text())
        rows.append((sequence, int(metadata["capture_time_ns"])))
    now_ns = time.time_ns()
    skew_s = abs(rows[0][1] - rows[1][1]) / 1e9
    ages_s = [(now_ns - capture_ns) / 1e9 for _, capture_ns in rows]
    ready = not blockers and skew_s <= 5.0 and max(ages_s) <= 60.0
    print(
        "ready={} wsj={} yunji={} skew_s={:.3f} ages_s={:.3f}/{:.3f}{}".format(
            int(ready),
            rows[0][0],
            rows[1][0],
            skew_s,
            ages_s[0],
            ages_s[1],
            "" if not blockers else " blockers=" + "|".join(blockers),
        )
    )
    raise SystemExit(0 if ready else 1)
except Exception as exc:
    print(f"ready=0 input_error={type(exc).__name__}:{exc}")
    raise SystemExit(1)
PY
    )"; then
      echo "Strict live VLM input window: $sync_status"
      break
    fi
    echo "Waiting for strict live VLM input window: $sync_status"
    (( SECONDS < sync_deadline )) || {
      echo "Timed out waiting for fresh cross-robot VLM inputs." >&2
      exit 1
    }
    sleep 1
  done
fi
stamp="$(date +%Y%m%d_%H%M%S)"
run_dir="$HUB_DIR/runtime/oneclick_${mode}_${scene_id}_${stamp}"
shadow_dir="$run_dir/shadow"
trusted=(
  chair sofa plant bed toilet tv bathtub shower fireplace appliances
  towel sink chest_of_drawers table stairs
)
shadow_args=(
  "$PYTHON_BIN" -u "$HUB_DIR/tools/live_vlm_shadow.py"
  --robot "robot-0:wsj:$WSJ_MAP"
  --robot "robot-1:yunji:$YUNJI_MAP"
  --spool "$HUB_DIR/runtime/spool"
  --output "$shadow_dir"
  --goal-category "$goal_category"
  --expected-shared-frame-calibration-id "$CALIBRATION_ID"
  --glm-url "$GLM_URL"
  --hub-url "$HUB_URL"
  --admin-token-file "$HUB_DIR/runtime/admin_token"
  --registry-state "$HUB_DIR/runtime/state/registry_state.json"
  --max-input-age-s 60
  --max-sync-skew-s 5
  --publish-hold
  --write-foxglove-targets
)
for category in "${trusted[@]}"; do
  shadow_args+=(--trusted-category "$category")
done
if [[ "$mode" == debug ]]; then
  shadow_args+=(
    --allow-blocked-shadow-input
    --allow-stale-shadow-input
  )
fi
"${shadow_args[@]}"

if [[ "$mode" == debug ]]; then
  echo "DEBUG_FULLSTACK_READY"
  echo "Foxglove: ws://$(hostname -I | awk '{print $1}'):8765"
  echo "Manifest: $shadow_dir/shadow_manifest.json"
  echo "Safety: Hub GOAL=false; receivers read-only; WSJ has no Go2 bridge."
  exit 0
fi

episode_dir="$run_dir/episode"
"$PYTHON_BIN" -u "$HUB_DIR/tools/run_v2_supervised_episode.py" \
  --manifest "$shadow_dir/shadow_manifest.json" \
  --registry-state "$HUB_DIR/runtime/state/registry_state.json" \
  --robot-config "$desired_config" \
  --scene-id "$scene_id" \
  --episode-id "$episode_id" \
  --output "$episode_dir" \
  --hub-url "$HUB_URL" \
  --admin-token-file "$HUB_DIR/runtime/admin_token" \
  --enable-live-goal-publication \
  --operator-confirmation OPERATOR_PRESENT_AND_ROBOTS_CLEAR

echo "LIVE_EPISODE_FINISHED: $episode_dir/episode_report.json"
