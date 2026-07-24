#!/usr/bin/env bash
# Start Yunji with Odin + online TinyNav + guarded WATER velocity output.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RELEASE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ENV_FILE="${FOCUS_ODIN_ENV_FILE:-/home/nyu/focus_sender_odin1/focus-odin1.env}"
CALIBRATION_FILE="${FOCUS_YUNJI_SHARED_CALIBRATION_FILE:-}"
BASE_CAMERA_CALIBRATION="${FOCUS_YUNJI_BASE_CAMERA_CALIBRATION:-/home/nyu/.local/state/topofocus/calibration/yunji_odin1_base_camera_20260723_operator.json}"
FACTORY_CALIBRATION="${FOCUS_ODIN_FACTORY_CALIBRATION:-$SCRIPT_DIR/../config/calibration/odin1_O1-P070100205_factory_20260722.json}"
TRANSFORM_VERSION="${FOCUS_YUNJI_TRANSFORM_VERSION:-}"
CALIBRATION_ID="${FOCUS_SHARED_CALIBRATION_ID:-}"
HUB_URL="${FOCUS_HUB_BASE_URL:-http://127.0.0.1:18089}"
TINYNAV_RUNTIME="${FOCUS_YUNJI_TINYNAV_RUNTIME:-/home/nyu/.local/share/topofocus/tinynav-runtime}"
mode="debug"
confirmation=""
startup_complete="false"

fail_closed_on_error() {
  local rc=$?
  if [[ "$rc" -ne 0 && "$mode" == live \
        && "$startup_complete" != true ]]; then
    sudo -n systemctl stop \
      focus-yunji-v2-live-v3.service \
      focus-yunji-water-bridge-live-v1.service \
      >/dev/null 2>&1 || true
  fi
  return "$rc"
}
trap fail_closed_on_error EXIT

usage() {
  echo "Usage: $0 --mode debug|live [--operator-confirmation OPERATOR_PRESENT_AND_YUNJI_CLEAR]"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) mode="$2"; shift 2 ;;
    --operator-confirmation) confirmation="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done
[[ "$mode" == debug || "$mode" == live ]] || {
  echo "--mode must be debug or live." >&2
  exit 2
}
if [[ "$mode" == live && "$confirmation" != OPERATOR_PRESENT_AND_YUNJI_CLEAR ]]; then
  echo "Live Yunji mode requires OPERATOR_PRESENT_AND_YUNJI_CLEAR." >&2
  exit 2
fi
[[ "$TRANSFORM_VERSION" =~ ^[A-Za-z0-9_.-]+$ ]] || {
  echo "FOCUS_YUNJI_TRANSFORM_VERSION must be explicit and filesystem-safe." >&2
  exit 2
}
[[ "$CALIBRATION_ID" =~ ^[A-Za-z0-9_.-]+$ ]] || {
  echo "FOCUS_SHARED_CALIBRATION_ID must be explicit and filesystem-safe." >&2
  exit 2
}
[[ "$CALIBRATION_FILE" = /* ]] || {
  echo "FOCUS_YUNJI_SHARED_CALIBRATION_FILE must be an explicit absolute path." >&2
  exit 2
}
[[ "$HUB_URL" =~ ^http://127\.0\.0\.1:[0-9]+$ ]] || {
  echo "FOCUS_HUB_BASE_URL must remain loopback-only." >&2
  exit 2
}
for required in \
  "$SCRIPT_DIR/install_yunji_tinynav_runtime.sh" \
  "$SCRIPT_DIR/run_yunji_tinynav_component.sh" \
  "$SCRIPT_DIR/run_yunji_mapping_observation.sh" \
  "$SCRIPT_DIR/odin1_tinynav_adapter.py" \
  "$SCRIPT_DIR/verify_tinynav_data_plane.py" \
  "$SCRIPT_DIR/water_cmd_vel_bridge.py" \
  "$SCRIPT_DIR/v2_wsj_receiver.py" \
  "$ENV_FILE" \
  "$CALIBRATION_FILE" \
  "$BASE_CAMERA_CALIBRATION" \
  "$FACTORY_CALIBRATION"; do
  [[ -r "$required" ]] || {
    echo "Missing required file: $required" >&2
    exit 1
  }
done
systemctl is-active --quiet focus-yunji-odin1-driver.service || {
  echo "Odin driver is not active." >&2
  exit 1
}
if pgrep -af 'keyboard.*teleop|yunji_wasd_teleop' >/dev/null 2>&1; then
  echo "Refusing startup while a Yunji manual command process exists." >&2
  exit 1
fi

FOCUS_YUNJI_TINYNAV_RUNTIME="$TINYNAV_RUNTIME" \
  bash "$SCRIPT_DIR/install_yunji_tinynav_runtime.sh"

stop_unit() {
  local unit="$1"
  sudo -n systemctl stop "$unit" >/dev/null 2>&1 || true
  sudo -n systemctl reset-failed "$unit" >/dev/null 2>&1 || true
}

start_unit() {
  local unit="$1"
  shift
  stop_unit "$unit"
  sudo -n systemd-run \
    --unit="${unit%.service}" \
    --property=Type=exec \
    --property=KillMode=control-group \
    --uid=nyu --gid=nyu \
    --working-directory="$RELEASE_ROOT" \
    --setenv="FOCUS_YUNJI_TINYNAV_RUNTIME=$TINYNAV_RUNTIME" \
    "$@" >/dev/null
}

# Remove every previous direct-/api/move receiver before creating the new
# online TinyNav command path.
for unit in \
  focus-yunji-v2-readonly-v4.service \
  focus-yunji-v2-runtime.service \
  focus-yunji-v2-debug-v2.service \
  focus-yunji-v2-live-v2.service \
  focus-yunji-v2-debug-v3.service \
  focus-yunji-v2-live-v3.service \
  focus-yunji-water-bridge-debug-v1.service \
  focus-yunji-water-bridge-live-v1.service; do
  stop_unit "$unit"
done

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
state_root="/home/nyu/.local/state/topofocus"
map_output="$state_root/yunji-tinynav-online-map-$stamp"
mkdir -p "$map_output"

SENDER_UNIT="focus-yunji-command-observation-v2.service"
if ! systemctl is-active --quiet "$SENDER_UNIT"; then
  metrics="$state_root/yunji-command-observation-$stamp.json"
  start_unit "$SENDER_UNIT" \
    /bin/bash "$SCRIPT_DIR/run_yunji_mapping_observation.sh" \
      --transform-version "$TRANSFORM_VERSION" \
      --shared-frame-transform-file "$CALIBRATION_FILE" \
      --base-camera-calibration-file "$BASE_CAMERA_CALIBRATION" \
      --command-capable \
      --env "$ENV_FILE" \
      --metrics-out "$metrics"
fi

start_unit focus-yunji-tinynav-adapter-v1.service \
  /bin/bash "$SCRIPT_DIR/run_yunji_tinynav_component.sh" adapter \
    --calibration-file "$FACTORY_CALIBRATION"

start_unit focus-yunji-tinynav-occupancy-v1.service \
  /bin/bash "$SCRIPT_DIR/run_yunji_tinynav_component.sh" occupancy \
    --ros-args \
    -p topics.pointcloud_input:=/focus/odin1/cloud_world \
    -p topics.camera_pose:=/focus/odin1/camera_pose_world \
    -p frames.target_frame:=world \
    -p output.directory:="$map_output" \
    -p output.save_on_shutdown:=true \
    -p bev.publish_rate_hz:=2.0

start_unit focus-yunji-tinynav-planner-v1.service \
  /bin/bash "$SCRIPT_DIR/run_yunji_tinynav_component.sh" planner \
    --body-radius-m 0.283 \
    --camera-forward-m 0.23 \
    --safety-margin-m 0.05

start_unit focus-yunji-tinynav-router-v1.service \
  /bin/bash "$SCRIPT_DIR/run_yunji_tinynav_component.sh" router \
    --frame-id world \
    --robot-id robot-1 \
    --base-camera-frame odin1_camera_optical_frame \
    --occupancy-topic /semantic_mapping/occupancy_bev \
    --base-camera-calibration-file "$BASE_CAMERA_CALIBRATION" \
    --clearance-m 0.34 \
    --start-footprint-override-m 0.34 \
    --max-cached-map-motion-m 0.25

start_unit focus-yunji-tinynav-controller-v1.service \
  /bin/bash "$SCRIPT_DIR/run_yunji_tinynav_component.sh" controller

bridge_args=(
  /bin/bash "$SCRIPT_DIR/run_yunji_tinynav_component.sh" bridge
  --input-topic /focus_guarded_cmd_vel
  --status-topic /focus/water/cmd_bridge_status
  --max-linear-mps 0.15
  --max-angular-radps 0.40
)
if [[ "$mode" == live ]]; then
  bridge_args+=(
    --enable-live-water-output
    --operator-confirmation OPERATOR_PRESENT_AND_YUNJI_CLEAR
  )
fi
BRIDGE_UNIT="focus-yunji-water-bridge-${mode}-v1.service"
start_unit "$BRIDGE_UNIT" "${bridge_args[@]}"

alignment="$state_root/yunji-v2-tinynav-$mode-$stamp.json"
log="$state_root/yunji-v2-tinynav-$mode-$stamp.jsonl"
receiver_args=(
  /bin/bash "$SCRIPT_DIR/run_yunji_tinynav_component.sh" receiver
  --base-url "$HUB_URL"
  --robot-id robot-1
  --calibration-file "$CALIBRATION_FILE"
  --base-camera-calibration-file "$BASE_CAMERA_CALIBRATION"
  --base-camera-frame odin1_camera_optical_frame
  --transform-version "$TRANSFORM_VERSION"
  --shared-frame-calibration-id "$CALIBRATION_ID"
  --online-buildmap-world
  --tracking-frame world
  --tinynav-map-frame world
  --local-map-frame yunji/world
  --occupancy-topic /semantic_mapping/occupancy_bev
  --external-odometry-health
  --platform-health-topic /focus/water/cmd_bridge_status
  --reachability-clearance-m 0.34
  --start-footprint-override-m 0.34
  --alignment-output "$alignment"
  --log "$log"
)
if [[ "$mode" == live ]]; then
  receiver_args+=(
    --enable-live-tinynav-motion
    --operator-confirmation OPERATOR_PRESENT_AND_YUNJI_CLEAR
  )
fi
RECEIVER_UNIT="focus-yunji-v2-${mode}-v3.service"
start_unit "$RECEIVER_UNIT" \
  /bin/bash -lc \
  "set -a; source '$ENV_FILE'; set +a; exec $(printf '%q ' "${receiver_args[@]}")"

deadline=$((SECONDS + 50))
until [[ -s "$alignment" ]]; do
  for unit in \
    focus-yunji-tinynav-adapter-v1.service \
    focus-yunji-tinynav-occupancy-v1.service \
    focus-yunji-tinynav-planner-v1.service \
    focus-yunji-tinynav-router-v1.service \
    focus-yunji-tinynav-controller-v1.service \
    "$BRIDGE_UNIT" \
    "$RECEIVER_UNIT"; do
    systemctl is-active --quiet "$unit" || {
      journalctl -u "$unit" -n 80 --no-pager >&2
      exit 1
    }
  done
  (( SECONDS < deadline )) || {
    echo "Timed out waiting for Yunji online TinyNav alignment." >&2
    exit 1
  }
  sleep 1
done

bash "$SCRIPT_DIR/run_yunji_tinynav_component.sh" verify \
  --robot-id robot-1 \
  --mode "$mode" \
  --frame-id world \
  --camera-frame odin1_camera_optical_frame \
  --fresh-image-topic /slam/depth \
  --platform-status-topic /focus/water/cmd_bridge_status \
  --timeout-s 35

startup_complete="true"
trap - EXIT
echo "Yunji online TinyNav stack ready: mode=$mode"
echo "  alignment: $alignment"
echo "  online map: $map_output"
echo "  planner: pinned TinyNav A*/local planner/controller"
echo "  chassis: guarded /focus_guarded_cmd_vel -> WATER /api/joy_control"
if [[ "$mode" == debug ]]; then
  echo "Safety: WATER bridge is dry-run; physical motion is impossible through this stack."
fi
