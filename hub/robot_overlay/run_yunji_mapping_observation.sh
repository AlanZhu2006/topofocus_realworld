#!/usr/bin/env bash
# Run the Odin1 sender in fresh-session observation mode.
#
# This entry point deliberately removes any shared-frame transform inherited
# from the persistent deployment environment. Even its explicit
# command-capable metadata mode starts no planner, WATER move request, command
# receiver, or velocity process.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
env_file="/home/nyu/focus_sender_odin1/focus-odin1.env"
calibration_file="$SCRIPT_DIR/../config/calibration/odin1_O1-P070100205_factory_20260722.json"
transform_version=""
shared_transform_file=""
metrics_out="/home/nyu/.local/state/topofocus/odin1-mapping-observation-metrics.json"
command_capable="false"
base_camera_calibration_file=""

usage() {
  cat <<'EOF'
Usage: run_yunji_mapping_observation.sh \
  --transform-version UNIQUE_CURRENT_SESSION_ID [--env FILE] [--metrics-out FILE] \
  [--shared-frame-transform-file FILE] \
  [--command-capable --base-camera-calibration-file FILE]

Starts only the Odin1 observation sender. The default is mapping-only.
--command-capable adds measured base_T_camera metadata and sets mapping_only
false, but still starts no receiver, planner, WATER request or motion endpoint.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --transform-version) transform_version="$2"; shift 2 ;;
    --shared-frame-transform-file) shared_transform_file="$2"; shift 2 ;;
    --env) env_file="$2"; shift 2 ;;
    --metrics-out) metrics_out="$2"; shift 2 ;;
    --command-capable) command_capable="true"; shift ;;
    --base-camera-calibration-file)
      base_camera_calibration_file="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$transform_version" || "$transform_version" == *20260722* ]]; then
  echo "A unique current-session transform version is required." >&2
  exit 2
fi
[[ -r "$env_file" ]] || { echo "Missing environment file: $env_file" >&2; exit 2; }
[[ -r "$calibration_file" ]] || {
  echo "Missing Odin factory calibration: $calibration_file" >&2
  exit 2
}
if [[ -n "$shared_transform_file" && ! -r "$shared_transform_file" ]]; then
  echo "Missing shared-frame transform: $shared_transform_file" >&2
  exit 2
fi
if [[ "$command_capable" == "true" ]]; then
  [[ -n "$shared_transform_file" ]] || {
    echo "--command-capable requires --shared-frame-transform-file." >&2
    exit 2
  }
  [[ -r "$base_camera_calibration_file" ]] || {
    echo "Missing measured base-camera calibration: $base_camera_calibration_file" >&2
    exit 2
  }
elif [[ -n "$base_camera_calibration_file" ]]; then
  echo "--base-camera-calibration-file requires --command-capable." >&2
  exit 2
fi

if pgrep -af 'keyboard.*teleop|joy_control' >/dev/null 2>&1; then
  echo "Refusing observation launch while a Yunji manual command process exists." >&2
  exit 1
fi
if [[ "$command_capable" != "true" ]] \
   && pgrep -af 'v2_yunji_receiver' >/dev/null 2>&1; then
  echo "Refusing mapping-only launch while a v2 receiver is running." >&2
  exit 1
fi

source_setup() {
  local had_nounset=0
  case $- in *u*) had_nounset=1; set +u ;; esac
  unset COLCON_CURRENT_PREFIX AMENT_CURRENT_PREFIX
  source /opt/ros/humble/setup.bash
  unset COLCON_CURRENT_PREFIX
  source /home/nyu/odin_ws/install/setup.bash
  [[ "$had_nounset" == 1 ]] && set -u
}

source_setup
set -a
source "$env_file"
set +a

# A powered-on robot has a new Odin odometry origin. Reusing an inherited shared
# transform would silently mix transform epochs, so only the explicit CLI file
# may be applied.
unset FOCUS_ODIN1_SHARED_TRANSFORM_FILE
export FOCUS_ODIN1_TRANSFORM_VERSION="$transform_version"
export PYTHONPATH="$SCRIPT_DIR/../src${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$(dirname "$metrics_out")"

sender_args=(
  --calibration-file "$calibration_file"
  --transform-version "$transform_version"
  --rate-hz 1
  --metrics-out "$metrics_out"
)
if [[ -n "$shared_transform_file" ]]; then
  sender_args+=(--shared-frame-transform-file "$shared_transform_file")
fi
if [[ "$command_capable" == "true" ]]; then
  sender_args+=(
    --enable-command-capable-observations
    --activation-confirmation COMMAND_CAPABLE_OBSERVATION_ONLY
    --base-camera-calibration-file "$base_camera_calibration_file"
    --heartbeat-hz 0
  )
fi

exec python3 -u "$SCRIPT_DIR/odin1_sender.py" "${sender_args[@]}"
