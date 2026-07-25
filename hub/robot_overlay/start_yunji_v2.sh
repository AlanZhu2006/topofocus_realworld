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
# The router uses a square cell-clearance test, while TinyNav's unchanged local
# planner remains the final footprint/depth authority.  On the 2026-07-25 live
# Yunji grid, 0.34 m rounded up to seven 5 cm cells and no 15x15 known-free
# start seed existed within the bounded one-metre escape search.  Six cells
# (0.30 m; a 0.424 m corner radius) produced a genuinely known-free seed at
# 0.962 m.  Use that observed router clearance only for graph admission; the
# local planner still enforces the measured 0.283 m body radius plus its
# original 0.05 m safety margin before any velocity reaches WATER.
REACHABILITY_CLEARANCE_M="${FOCUS_YUNJI_REACHABILITY_CLEARANCE_M:-0.30}"
START_SNAP_RADIUS_M="${FOCUS_YUNJI_START_SNAP_RADIUS_M:-1.0}"
START_FOOTPRINT_OVERRIDE_M="${FOCUS_YUNJI_START_FOOTPRINT_OVERRIDE_M:-0.34}"
# Follow the recovered start-to-seed route closely.  A one-metre rolling
# waypoint can skip the entire bounded escape path on Yunji's forward-cropped
# map and ask the local planner to cut a corner toward a wall.
LOOKAHEAD_M="${FOCUS_YUNJI_LOOKAHEAD_M:-0.35}"
# Observed during the first two-robot live episode on 2026-07-24: Odin normally
# publishes /slam/odometry at about 3.5 Hz (0.26--0.35 s intervals), but one
# processing transient exceeded the router's old 1.0 s default and aborted the
# whole coordinated episode.  Match the receiver's existing 2.0 s localization
# freshness bound.  This does not extend physical velocity authority: the
# WATER bridge independently zeros a stale guarded command after 0.30 s.
ODOMETRY_INPUT_TIMEOUT_S="${FOCUS_YUNJI_ODOMETRY_INPUT_TIMEOUT_S:-2.0}"
MAP_TIMEOUT_S="${FOCUS_YUNJI_MAP_TIMEOUT_S:-12.0}"
NO_PROGRESS_TIMEOUT_S="${FOCUS_YUNJI_NO_PROGRESS_TIMEOUT_S:-20.0}"
MINIMUM_GOAL_PROGRESS_M="${FOCUS_YUNJI_MINIMUM_GOAL_PROGRESS_M:-0.05}"
# A reverse local lookahead is recovered without reverse translation.  The
# controller holds one turn direction, WATER retains the lower 0.40 rad/s
# actuator cap, and the receiver still rejects after this bounded interval.
REVERSE_ROTATE_MAX_ANGULAR_RADPS="${FOCUS_YUNJI_REVERSE_ROTATE_MAX_ANGULAR_RADPS:-0.35}"
REVERSE_ROTATE_TIMEOUT_S="${FOCUS_YUNJI_REVERSE_ROTATE_TIMEOUT_S:-12.0}"
# Keep the transported/source-derived semantic approach mask unchanged, but
# align the physical terminal check with TinyNav's observed short-path limit.
# In the 2026-07-25 chair run the planner stopped emitting a multi-pose path at
# 0.32 m from the selected approach point; 0.50 m records that supervised demo
# terminal tolerance explicitly. Formal SR/SPL still requires the independent
# surveyed goal-region check.
SEMANTIC_ARRIVAL_RADIUS_M="${FOCUS_YUNJI_SEMANTIC_ARRIVAL_RADIUS_M:-0.50}"
# Odin's full 800 px, radius-1 projection performs nine indexed depth
# reductions per cloud.  After a cold Odin boot this was observed to pin one
# CPU core for minutes before the first /slam/depth sample, while the same
# calibrated stream at 400 px with no splat published steadily at about
# 10 Hz.  This depth is used only by TinyNav's robot-local planner; the Hub
# observation sender keeps the full 800 px RGB-D stream for VLM/semantics.
LOCAL_DEPTH_WIDTH="${FOCUS_YUNJI_LOCAL_DEPTH_WIDTH:-400}"
LOCAL_DEPTH_SPLAT_RADIUS="${FOCUS_YUNJI_LOCAL_DEPTH_SPLAT_RADIUS:-0}"
mode="debug"
confirmation=""
reuse_verified_debug_core="false"
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
  echo "Usage: $0 --mode debug|live [--operator-confirmation OPERATOR_PRESENT_AND_YUNJI_CLEAR] [--reuse-verified-debug-core]"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) mode="$2"; shift 2 ;;
    --operator-confirmation) confirmation="$2"; shift 2 ;;
    --reuse-verified-debug-core)
      reuse_verified_debug_core="true"
      shift
      ;;
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
if [[ "$reuse_verified_debug_core" == true && "$mode" != live ]]; then
  echo "--reuse-verified-debug-core is valid only for live mode." >&2
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

if [[ "$reuse_verified_debug_core" != true ]]; then
  FOCUS_YUNJI_TINYNAV_RUNTIME="$TINYNAV_RUNTIME" \
    bash "$SCRIPT_DIR/install_yunji_tinynav_runtime.sh"
fi

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
    --setenv="OPENBLAS_NUM_THREADS=1" \
    --setenv="OMP_NUM_THREADS=1" \
    --setenv="MKL_NUM_THREADS=1" \
    --setenv="NUMEXPR_NUM_THREADS=1" \
    "$@" >/dev/null
}

start_router() {
  start_unit focus-yunji-tinynav-router-v1.service \
    /bin/bash "$SCRIPT_DIR/run_yunji_tinynav_component.sh" router \
      --frame-id world \
      --robot-id robot-1 \
      --base-camera-frame odin1_camera_optical_frame \
      --occupancy-topic /semantic_mapping/occupancy_bev \
      --base-camera-calibration-file "$BASE_CAMERA_CALIBRATION" \
      --lookahead-m "$LOOKAHEAD_M" \
      --clearance-m "$REACHABILITY_CLEARANCE_M" \
      --start-snap-radius-m "$START_SNAP_RADIUS_M" \
      --start-footprint-override-m "$START_FOOTPRINT_OVERRIDE_M" \
      --input-timeout-s "$ODOMETRY_INPUT_TIMEOUT_S" \
      --map-timeout-s "$MAP_TIMEOUT_S" \
      --max-cached-map-motion-m 0.25
}

start_controller() {
  start_unit focus-yunji-tinynav-controller-v1.service \
    /bin/bash "$SCRIPT_DIR/run_yunji_tinynav_component.sh" controller \
      --rotate-first-on-reverse \
      --rotate-first-max-angular-radps \
        "$REVERSE_ROTATE_MAX_ANGULAR_RADPS" \
      --rotate-first-timeout-s "$REVERSE_ROTATE_TIMEOUT_S"
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

CORE_UNITS=(
  focus-yunji-tinynav-adapter-v1.service
  focus-yunji-tinynav-occupancy-v1.service
  focus-yunji-tinynav-planner-v1.service
  focus-yunji-tinynav-router-v1.service
  focus-yunji-tinynav-controller-v1.service
)
if [[ "$reuse_verified_debug_core" == true ]]; then
  # realworld_oneclick.sh permits this only immediately after the same
  # invocation started and data-plane-verified the debug stack from the
  # byte-identical release.  Keep Odin -> /slam/depth and the online map alive;
  # mode switching replaces the receiver and WATER bridge.  Reload the
  # non-actuating goal router as well so a newly deployed overlay cannot leave
  # an old Python process resident across a fast live reuse.
  for unit in "${CORE_UNITS[@]}"; do
    systemctl is-active --quiet "$unit" || {
      echo "Verified Yunji debug core is not active: $unit" >&2
      exit 1
    }
  done
  old_router_pid="$(
    systemctl show --property MainPID --value \
      focus-yunji-tinynav-router-v1.service
  )"
  old_controller_pid="$(
    systemctl show --property MainPID --value \
      focus-yunji-tinynav-controller-v1.service
  )"
  start_router
  start_controller
  new_router_pid="$(
    systemctl show --property MainPID --value \
      focus-yunji-tinynav-router-v1.service
  )"
  new_controller_pid="$(
    systemctl show --property MainPID --value \
      focus-yunji-tinynav-controller-v1.service
  )"
  [[ "$new_router_pid" =~ ^[1-9][0-9]*$ \
      && "$new_router_pid" != "$old_router_pid" ]] || {
    echo "Yunji goal router did not reload into a new process." >&2
    exit 1
  }
  [[ "$new_controller_pid" =~ ^[1-9][0-9]*$ \
      && "$new_controller_pid" != "$old_controller_pid" ]] || {
    echo "Yunji velocity controller did not reload into a new process." >&2
    exit 1
  }
  echo "Reusing the verified Yunji perception/planning core without interrupting /slam/depth."
  echo "Yunji goal router reloaded from the current deployment: $new_router_pid"
  echo "Yunji rotate-first controller reloaded from the current deployment: $new_controller_pid"
else
  start_unit focus-yunji-tinynav-adapter-v1.service \
    /bin/bash "$SCRIPT_DIR/run_yunji_tinynav_component.sh" adapter \
      --calibration-file "$FACTORY_CALIBRATION" \
      --output-width "$LOCAL_DEPTH_WIDTH" \
      --splat-radius "$LOCAL_DEPTH_SPLAT_RADIUS"

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

  start_router
  start_controller
fi

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
  --reject-reverse-trajectory
  --reachability-clearance-m "$REACHABILITY_CLEARANCE_M"
  --start-snap-radius-m "$START_SNAP_RADIUS_M"
  --start-footprint-override-m "$START_FOOTPRINT_OVERRIDE_M"
  --semantic-arrival-radius-m "$SEMANTIC_ARRIVAL_RADIUS_M"
  --no-progress-timeout-s "$NO_PROGRESS_TIMEOUT_S"
  --minimum-goal-progress-m "$MINIMUM_GOAL_PROGRESS_M"
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
  --geometry-image-topic /slam/depth \
  --camera-info-topic /slam/camera_info \
  --max-occupancy-age-s 12 \
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
