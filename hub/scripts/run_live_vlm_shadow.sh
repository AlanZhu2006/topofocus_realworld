#!/usr/bin/env bash
# One-command, fail-closed real-sensor VLM shadow experiment.
#
# This script never starts a robot process and never publishes GOAL. It first
# proves that Hub GOAL output is disabled, GLM is reachable, both maps are
# fresh/unblocked/synchronized, and their explicit calibration ID matches the
# current physical session. Only then does it run the real VLM cascade. The
# optional Hub decisions are HOLD and Foxglove targets are display-only.
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
python_bin="$workspace/hub/.venv/bin/python"
tool="$workspace/hub/tools/live_vlm_shadow.py"
spool="$workspace/hub/runtime/spool"
hub_url="http://127.0.0.1:8088"
glm_url="http://127.0.0.1:31511/v1"
goal_category="chair"
output_root="$workspace/hub/runtime"
wsj_map=""
yunji_map=""
calibration_id=""
preflight_only="false"

usage() {
  cat <<'EOF'
Usage:
  bash hub/scripts/run_live_vlm_shadow.sh \
    --wsj-map <fresh-wsj-map-dir> \
    --yunji-map <fresh-yunji-map-dir> \
    --calibration-id <new-session-calibration-id> [options]

Options:
  --goal-category NAME   HPC target: chair/bed/plant/toilet/tv/sofa
  --hub-url URL          Loopback Hub API (default: http://127.0.0.1:8088)
  --glm-url URL          Loopback GLM API (default: http://127.0.0.1:31511/v1)
  --output-root DIR      Runtime output parent
  --preflight-only       Validate/freeze inputs without calling GLM
  -h, --help

The wrapper intentionally has no blocked/stale override and no GOAL mode.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --wsj-map) wsj_map="$2"; shift 2 ;;
    --yunji-map) yunji_map="$2"; shift 2 ;;
    --calibration-id) calibration_id="$2"; shift 2 ;;
    --goal-category) goal_category="$2"; shift 2 ;;
    --hub-url) hub_url="$2"; shift 2 ;;
    --glm-url) glm_url="$2"; shift 2 ;;
    --output-root) output_root="$2"; shift 2 ;;
    --preflight-only) preflight_only="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$goal_category" in
  chair|bed|plant|toilet|tv|sofa) ;;
  *) echo "Unsupported HPC HM3D ObjectNav target: $goal_category" >&2; exit 2 ;;
esac

if [[ -z "$wsj_map" || -z "$yunji_map" || -z "$calibration_id" ]]; then
  echo "Fresh map directories and the new session calibration ID are required." >&2
  usage >&2
  exit 2
fi
if [[ ! -x "$python_bin" ]]; then
  echo "Missing Hub Python environment: $python_bin" >&2
  exit 2
fi
if [[ ! "$hub_url" =~ ^http://127\.0\.0\.1:[0-9]+$ ]]; then
  echo "--hub-url must remain loopback-only." >&2
  exit 2
fi
if [[ ! "$glm_url" =~ ^http://127\.0\.0\.1:[0-9]+/v1$ ]]; then
  echo "--glm-url must remain loopback-only and end in /v1." >&2
  exit 2
fi

health_json="$(curl -fsS --max-time 5 "$hub_url/healthz")"
FOCUS_PREFLIGHT_HEALTH_JSON="$health_json" "$python_bin" -c '
import json, os
health = json.loads(os.environ["FOCUS_PREFLIGHT_HEALTH_JSON"])
enabled = health.get("goal_output_enabled", {})
if enabled.get("robot-0") is not False or enabled.get("robot-1") is not False:
    raise SystemExit("refusing shadow run: Hub GOAL output is not disabled for both robots")
'
curl -fsS --max-time 10 "$glm_url/models" >/dev/null

stamp="$(date +%Y%m%d_%H%M%S)"
preflight_dir="$output_root/vlm_preflight_$stamp"
common=(
  --robot "robot-0:wsj:$wsj_map"
  --robot "robot-1:yunji:$yunji_map"
  --spool "$spool"
  --goal-category "$goal_category"
  --trusted-category chair
  --trusted-category sofa
  --trusted-category plant
  --trusted-category bed
  --trusted-category toilet
  --trusted-category tv
  --trusted-category bathtub
  --trusted-category shower
  --trusted-category fireplace
  --trusted-category appliances
  --trusted-category towel
  --trusted-category sink
  --trusted-category chest_of_drawers
  --trusted-category table
  --trusted-category stairs
  --expected-shared-frame-calibration-id "$calibration_id"
  --max-input-age-s 60
  --max-sync-skew-s 5
)

PYTHONPATH="$workspace/hub/src" "$python_bin" -u "$tool" \
  "${common[@]}" --output "$preflight_dir" --preflight-only

if [[ "$preflight_only" == "true" ]]; then
  echo "Fresh-input preflight passed: $preflight_dir/shadow_manifest.json"
  exit 0
fi

shadow_dir="$output_root/vlm_shadow_$stamp"
PYTHONPATH="$workspace/hub/src" "$python_bin" -u "$tool" \
  "${common[@]}" --output "$shadow_dir" --glm-url "$glm_url" \
  --publish-hold --write-foxglove-targets --display-expiry-s 600

echo "Real VLM shadow run passed: $shadow_dir/shadow_manifest.json"
echo "Safety: HOLD only; Foxglove display targets expire in 600s; no GOAL path."
