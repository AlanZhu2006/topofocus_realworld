#!/usr/bin/env bash
# Start two fresh, calibration-bound mapping daemons. No robot command path.
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
hub_dir="$workspace/hub"
python_bin="$hub_dir/.venv/bin/python"
session_tag=""
calibration_id=""
wsj_transform=""
yunji_transform=""
wsj_start_after=""
yunji_start_after=""
goal_category="chair"
hub_url="http://127.0.0.1:8188"
semantic_backend="segformer-ade20k"
semantic_min_hits="2"
semantic_winner_margin_hits="1"
resume_existing="false"
code_commit=""

usage() {
  cat <<'EOF'
Usage: bash hub/scripts/start_fresh_dual_maps.sh \
  --session-tag 20260723_v1 \
  --calibration-id shared-board-odin1-20260723-v1 \
  --wsj-transform wsj-tinynav-depth-20260723-session-v1 \
  --yunji-transform yunji-odin1-board-20260723-v1 \
  --wsj-start-after N --yunji-start-after N [--goal-category chair] \
  [--semantic-backend rednet|segformer-ade20k] \
  [--code-commit GIT_SHA] \
  [--resume-existing] \
  [--hub-url http://127.0.0.1:8188]

Creates new runtime map directories and a new tmux session. It never replaces
an old directory, changes Hub policy, starts a receiver, or contacts a robot.
--resume-existing is only for restarting the same calibration/session after a
Hub-computer reboot; existing artifacts must match the exact session contract.
The default real-camera semantic adapter emits pixel masks and requires two
keyframe votes. Real YOLOv10 runs at the source conf=0.2 threshold and is
persisted as separate Stage-1 Perception-VLM evidence; its boxes are not
projected into the BEV by this mapping-only launcher.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session-tag) session_tag="$2"; shift 2 ;;
    --calibration-id) calibration_id="$2"; shift 2 ;;
    --wsj-transform) wsj_transform="$2"; shift 2 ;;
    --yunji-transform) yunji_transform="$2"; shift 2 ;;
    --wsj-start-after) wsj_start_after="$2"; shift 2 ;;
    --yunji-start-after) yunji_start_after="$2"; shift 2 ;;
    --goal-category) goal_category="$2"; shift 2 ;;
    --semantic-backend) semantic_backend="$2"; shift 2 ;;
    --code-commit) code_commit="$2"; shift 2 ;;
    --resume-existing) resume_existing="true"; shift ;;
    --hub-url) hub_url="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$session_tag" || ! "$session_tag" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "A filesystem-safe --session-tag is required." >&2
  exit 2
fi
code_commit="${code_commit:-$(git -C "$workspace" rev-parse HEAD)}"
[[ "$code_commit" =~ ^[0-9a-f]{40,64}$ ]] || {
  echo "--code-commit must be a full Git object ID." >&2
  exit 2
}
for value in calibration_id wsj_transform yunji_transform wsj_start_after yunji_start_after; do
  [[ -n "${!value}" ]] || { echo "Missing --${value//_/-}." >&2; exit 2; }
done
[[ "$wsj_start_after" =~ ^[0-9]+$ ]] || { echo "Invalid WSJ sequence." >&2; exit 2; }
[[ "$yunji_start_after" =~ ^[0-9]+$ ]] || { echo "Invalid Yunji sequence." >&2; exit 2; }
if [[ "$calibration_id" == *20260722* || "$wsj_transform" == *20260722* || "$yunji_transform" == *20260722* ]]; then
  echo "Refusing to reuse a pre-reboot July 22 session identity." >&2
  exit 2
fi
if [[ ! "$hub_url" =~ ^http://127\.0\.0\.1:[0-9]+$ ]]; then
  echo "--hub-url must remain loopback-only." >&2
  exit 2
fi
if [[ "$semantic_backend" != "rednet" && "$semantic_backend" != "segformer-ade20k" ]]; then
  echo "Invalid --semantic-backend: $semantic_backend" >&2
  exit 2
fi

tmux_session="shared_maps_${session_tag}"
tmux has-session -t "$tmux_session" 2>/dev/null && {
  echo "Refusing to replace tmux session: $tmux_session" >&2
  exit 1
}
health_json="$(curl -fsS --max-time 5 "$hub_url/healthz")"
FOCUS_PREFLIGHT_HEALTH_JSON="$health_json" "$python_bin" -c '
import json, os
enabled = json.loads(os.environ["FOCUS_PREFLIGHT_HEALTH_JSON"]).get("goal_output_enabled", {})
if enabled.get("robot-0") is not False or enabled.get("robot-1") is not False:
    raise SystemExit("refusing map launch while Hub GOAL output is enabled")
'

wsj_out="$hub_dir/runtime/map_out_wsj_${session_tag}"
yunji_out="$hub_dir/runtime/map_out_yunji_${session_tag}"
wsj_map_relative="hub/runtime/map_out_wsj_${session_tag}"
yunji_map_relative="hub/runtime/map_out_yunji_${session_tag}"
if [[ "$resume_existing" != "true" ]]; then
  for path in "$wsj_out" "$yunji_out"; do
    [[ ! -e "$path" ]] || {
      echo "Refusing to reuse existing output: $path" >&2
      exit 1
    }
  done
fi
"$python_bin" - "$resume_existing" "$session_tag" "$code_commit" \
  "$calibration_id" "$goal_category" "$semantic_backend" \
  "$wsj_out" "$wsj_map_relative" "$wsj_transform" "$wsj_start_after" \
  "$yunji_out" "$yunji_map_relative" "$yunji_transform" \
  "$yunji_start_after" <<'PY'
import json
import os
from pathlib import Path
import sys

resume = sys.argv[1] == "true"
session_id = sys.argv[2]
code_commit = sys.argv[3]
calibration_id = sys.argv[4]
goal_category = sys.argv[5]
semantic_backend = sys.argv[6]

for directory, map_relative, robot_id, transform, boundary in (
    (Path(sys.argv[7]), sys.argv[8], "robot-0", sys.argv[9], int(sys.argv[10])),
    (Path(sys.argv[11]), sys.argv[12], "robot-1", sys.argv[13], int(sys.argv[14])),
):
    expected = {
        "schema_version": "focus-realworld-map-session-contract-v1",
        "session_id": session_id,
        "code_git_commit": code_commit,
        "robot_id": robot_id,
        "map_dir": map_relative,
        "start_after_sequence": boundary,
        "transform_version": transform,
        "shared_frame_calibration_id": calibration_id,
        "goal_category": goal_category,
        "semantic_backend": semantic_backend,
        "semantic_yolo": {
            "enabled": True,
            "confidence": 0.2,
            "evidence_only": True,
        },
    }
    path = directory / "map_session_contract.json"
    if resume:
        observed = json.loads(path.read_text(encoding="utf-8"))
        if observed != expected:
            raise SystemExit(f"{robot_id} existing map session contract mismatch")
        continue
    directory.mkdir(parents=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(expected, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)
PY

common="--spool runtime/spool --hub-url '$hub_url' --admin-token-file runtime/admin_token --no-cascade --semantic-backend '$semantic_backend' --semantic-fusion-mode multi_view --semantic-min-hits '$semantic_min_hits' --semantic-winner-margin-hits '$semantic_winner_margin_hits' --semantic-yolo --semantic-yolo-evidence-only --semantic-yolo-confidence 0.2 --goal-category '$goal_category' --decision-interval 86400 --snapshot-interval-s 3.0 --shared-frame-calibration-id '$calibration_id' --ground-drift-consecutive-frames 3 --obstacle-band-high-m 0.75 --obstacle-min-hits 2 --startup-stable-frames 3 --startup-max-pose-delta-m 0.05 --startup-max-rotation-delta-deg 5"
# WSJ publishes observation keyframes sparsely while stationary.  Preserve the
# three-frame stability gate but allow those verified-near-identical samples to
# span one minute; Yunji's 1 Hz stream keeps the stricter ten-second interval.
wsj_command="cd '$hub_dir' && FOCUS_SESSION_CODE_COMMIT='$code_commit' YOLO_CONFIG_DIR=runtime/ultralytics '$python_bin' -u tools/hub_pipeline_daemon.py $common --startup-max-interval-s 60 --robot-id robot-0 --start-after-sequence '$wsj_start_after' --expected-transform-version '$wsj_transform' --obstacle-band-low-m 0.25 --log '$wsj_out/daemon.log' --out-dir '$wsj_out'"
yunji_command="cd '$hub_dir' && FOCUS_SESSION_CODE_COMMIT='$code_commit' YOLO_CONFIG_DIR=runtime/ultralytics '$python_bin' -u tools/hub_pipeline_daemon.py $common --startup-max-interval-s 10 --robot-id robot-1 --start-after-sequence '$yunji_start_after' --expected-transform-version '$yunji_transform' --obstacle-band-low-m 0.15 --log '$yunji_out/daemon.log' --out-dir '$yunji_out'"

launch_complete="false"
cleanup_on_error() {
  if [[ "$launch_complete" != "true" ]]; then
    tmux kill-session -t "$tmux_session" >/dev/null 2>&1 || true
  fi
}
trap cleanup_on_error EXIT
tmux new-session -d -s "$tmux_session" -n wsj "bash -lc \"$wsj_command\""
tmux set-window-option -t "$tmux_session:wsj" remain-on-exit on >/dev/null
tmux new-window -d -t "$tmux_session" -n yunji "bash -lc \"$yunji_command\""
tmux set-window-option -t "$tmux_session:yunji" remain-on-exit on >/dev/null
sleep 2
for window in wsj yunji; do
  dead="$(tmux display-message -p -t "$tmux_session:$window" '#{pane_dead}')"
  [[ "$dead" == 0 ]] || {
    echo "$window daemon exited during startup:" >&2
    tmux capture-pane -pt "$tmux_session:$window" -S -80 >&2 || true
    exit 1
  }
done
launch_complete="true"
trap - EXIT

echo "Fresh mapping daemons started: $tmux_session"
echo "  WSJ:   $wsj_out"
echo "  Yunji: $yunji_out"
echo "Wait for both live_status.json files to report mapping_blocked_reason=null."
echo "No cascade, GOAL receiver, planner or robot command was started."
