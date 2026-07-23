#!/usr/bin/env bash
# Fail-closed entry point for a continuous source-derived VLM shadow scene.
# It has no GOAL option and never starts a robot-side receiver or planner.
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
python_bin="$workspace/hub/.venv/bin/python"
tool="$workspace/hub/tools/live_vlm_scene.py"
spool="$workspace/hub/runtime/spool"
hub_url="http://127.0.0.1:8088"
glm_url="http://127.0.0.1:31511/v1"
wsj_map=""
yunji_map=""
calibration_id=""
scene_id=""
goal_category="chair"
max_rounds="21"
max_idle_s="300"

usage() {
  cat <<'EOF'
Usage:
  bash hub/scripts/run_live_vlm_scene.sh \
    --wsj-map <fresh-wsj-map-dir> \
    --yunji-map <fresh-yunji-map-dir> \
    --calibration-id <new-session-calibration-id> \
    --scene-id <unique-scene-id> [options]

Options:
  --goal-category NAME   HPC target: chair/bed/plant/toilet/tv/sofa
  --hub-url URL          Loopback Hub API (default: http://127.0.0.1:8088)
  --glm-url URL          Loopback GLM API (default: http://127.0.0.1:31511/v1)
  --max-rounds N         At most 21 HPC-derived decision rounds
  --max-idle-s SECONDS   Abort if no fresh synchronized pair arrives
  -h, --help

The wrapper publishes expiring HOLD and display-only Foxglove targets.  It
contains no stale/block override and no robot command/GOAL mode.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --wsj-map) wsj_map="$2"; shift 2 ;;
    --yunji-map) yunji_map="$2"; shift 2 ;;
    --calibration-id) calibration_id="$2"; shift 2 ;;
    --scene-id) scene_id="$2"; shift 2 ;;
    --goal-category) goal_category="$2"; shift 2 ;;
    --hub-url) hub_url="$2"; shift 2 ;;
    --glm-url) glm_url="$2"; shift 2 ;;
    --max-rounds) max_rounds="$2"; shift 2 ;;
    --max-idle-s) max_idle_s="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$goal_category" in
  chair|bed|plant|toilet|tv|sofa) ;;
  *) echo "Unsupported HPC HM3D ObjectNav target: $goal_category" >&2; exit 2 ;;
esac

for value in wsj_map yunji_map calibration_id scene_id; do
  [[ -n "${!value}" ]] || { echo "Missing --${value//_/-}." >&2; exit 2; }
done
[[ -d "$wsj_map" && -d "$yunji_map" ]] || {
  echo "Both fresh map directories must exist." >&2
  exit 2
}
if [[ "$calibration_id" == *20260722* || "$scene_id" == *20260722* ]]; then
  echo "Refusing a pre-reboot July 22 calibration/scene identity." >&2
  exit 2
fi
[[ "$max_rounds" =~ ^[0-9]+$ ]] && (( max_rounds >= 1 && max_rounds <= 21 )) || {
  echo "--max-rounds must be an integer in [1, 21]." >&2
  exit 2
}
if [[ ! "$hub_url" =~ ^http://127\.0\.0\.1:[0-9]+$ ]]; then
  echo "--hub-url must remain loopback-only." >&2
  exit 2
fi
if [[ ! "$glm_url" =~ ^http://127\.0\.0\.1:[0-9]+/v1$ ]]; then
  echo "--glm-url must remain loopback-only and end in /v1." >&2
  exit 2
fi

health_json="$(curl -fsS --max-time 5 "$hub_url/healthz")"
FOCUS_SCENE_HEALTH_JSON="$health_json" "$python_bin" -c '
import json, os
enabled = json.loads(os.environ["FOCUS_SCENE_HEALTH_JSON"]).get("goal_output_enabled", {})
if enabled.get("robot-0") is not False or enabled.get("robot-1") is not False:
    raise SystemExit("refusing scene while Hub GOAL output is enabled")
'
curl -fsS --max-time 10 "$glm_url/models" >/dev/null

stamp="$(date +%Y%m%d_%H%M%S)"
output="$workspace/hub/runtime/vlm_scene_${scene_id}_${stamp}"
PYTHONPATH="$workspace/hub/src" exec "$python_bin" -u "$tool" \
  --robot "robot-0:wsj:$wsj_map" \
  --robot "robot-1:yunji:$yunji_map" \
  --spool "$spool" \
  --output "$output" \
  --scene-id "$scene_id" \
  --goal-category "$goal_category" \
  --calibration-id "$calibration_id" \
  --hub-url "$hub_url" \
  --glm-url "$glm_url" \
  --admin-token-file "$workspace/hub/runtime/admin_token" \
  --registry-state "$workspace/hub/runtime/state/registry_state.json" \
  --max-rounds "$max_rounds" \
  --max-idle-s "$max_idle_s" \
  --publish-hold \
  --write-foxglove-targets
