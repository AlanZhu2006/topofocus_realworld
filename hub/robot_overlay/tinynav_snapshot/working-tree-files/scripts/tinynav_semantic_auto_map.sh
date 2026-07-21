#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
SESSION_NAME="${TINYNAV_SEMANTIC_CAMERA_SESSION:-tinynav_semantic_usb3_camera}"

timestamp="$(date +%Y%m%d_%H%M%S)"
bag_dir="${XDG_DATA_HOME:-$HOME/.local/share}/tinynav/rosbags/semantic_map_record_${timestamp}"
map_dir="$ROOT_DIR/output/semantic_map_record_${timestamp}"
play_rate="1.0"
keep_camera="false"
from_bag=""
clean_temp="true"
precomputed_mask_directory=""
precomputed_manifest="manifest.yaml"
semantic_classes_file=""
semantic_engine=""
semantic_model_config="$HOME/.cache/tinynav/semantic_models/segformer_b0_ade20k/config.json"
semantic_preprocessor_config="$HOME/.cache/tinynav/semantic_models/segformer_b0_ade20k/preprocessor_config.json"
semantic_label_mapping="$ROOT_DIR/semantic_mapping/config/ade20k_navigation_mapping.yaml"
semantic_min_confidence="0.35"

source_setup() {
  local had_nounset=0
  case $- in
    *u*) had_nounset=1; set +u ;;
  esac
  source "$SETUP_FILE"
  if [[ "$had_nounset" == "1" ]]; then
    set -u
  fi
}

usage() {
  cat <<EOF
Usage: $0 [--bag-dir DIR] [--map-dir DIR] [--play-rate RATE] [--keep-camera] [--keep-temp]
       $0 --from-bag DIR [--map-dir DIR] [--play-rate RATE] [--keep-temp]
          [--semantic-masks DIR] [--semantic-manifest FILE]
          [--semantic-tensorrt ENGINE] [--semantic-min-confidence VALUE]
          [--semantic-classes FILE]

Records a RealSense mapping bag, then builds a TinyNav map from it.
Stop recording with Ctrl-C once you have walked the mapping trajectory.
With --from-bag, skips recording and only builds a map from an existing bag.

Outputs:
  bag:      $bag_dir
  map:      $map_dir
  symlink:  $ROOT_DIR/output/latest_semantic_map

By default, successful map builds remove TinyNav runtime temp DBs:
  $ROOT_DIR/tinynav_temp
  $ROOT_DIR/tinynav_temp_gpu_current
  $ROOT_DIR/tinynav_temp_semantic_nav_auto
Use --keep-temp to keep them for debugging.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag-dir) bag_dir="$2"; shift 2 ;;
    --from-bag) from_bag="$2"; bag_dir="$2"; shift 2 ;;
    --map-dir) map_dir="$2"; shift 2 ;;
    --play-rate) play_rate="$2"; shift 2 ;;
    --keep-camera) keep_camera="true"; shift ;;
    --keep-temp) clean_temp="false"; shift ;;
    --semantic-masks) precomputed_mask_directory="$2"; shift 2 ;;
    --semantic-manifest) precomputed_manifest="$2"; shift 2 ;;
    --semantic-tensorrt) semantic_engine="$2"; shift 2 ;;
    --semantic-model-config) semantic_model_config="$2"; shift 2 ;;
    --semantic-preprocessor) semantic_preprocessor_config="$2"; shift 2 ;;
    --semantic-label-mapping) semantic_label_mapping="$2"; shift 2 ;;
    --semantic-min-confidence) semantic_min_confidence="$2"; shift 2 ;;
    --semantic-classes) semantic_classes_file="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 1 ;;
  esac
done

if [[ -n "$precomputed_mask_directory" && -n "$semantic_engine" ]]; then
  echo "Choose either --semantic-masks or --semantic-tensorrt, not both." >&2
  exit 1
fi
if [[ -n "$precomputed_mask_directory" && ! -d "$precomputed_mask_directory" ]]; then
  echo "Precomputed semantic mask directory does not exist: $precomputed_mask_directory" >&2
  exit 1
fi
if [[ -n "$semantic_engine" ]]; then
  for required_file in \
    "$semantic_engine" \
    "$semantic_model_config" \
    "$semantic_preprocessor_config" \
    "$semantic_label_mapping"; do
    if [[ ! -f "$required_file" ]]; then
      echo "SegFormer TensorRT file does not exist: $required_file" >&2
      exit 1
    fi
  done
fi
if [[ -n "$precomputed_mask_directory" && ! -f "$precomputed_mask_directory/$precomputed_manifest" ]]; then
  echo "Precomputed semantic manifest does not exist: $precomputed_mask_directory/$precomputed_manifest" >&2
  exit 1
fi
if [[ -n "$semantic_classes_file" && ! -f "$semantic_classes_file" ]]; then
  echo "Semantic class file does not exist: $semantic_classes_file" >&2
  exit 1
fi

source_setup
cd "$ROOT_DIR"

camera_started_here="false"
perception_pid=""
semantic_pid=""

camera_running() {
  ros2 node list 2>/dev/null | grep -qx "/camera/camera"
}

start_camera() {
  echo "Starting USB3 RealSense in tmux session: $SESSION_NAME"
  tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
  tmux new-session -d -s "$SESSION_NAME" \
    "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && bash scripts/run_realsense_semantic_sensor.sh'"
  camera_started_here="true"
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="${2:-30}"
  local start
  start="$(date +%s)"
  until ros2 topic list 2>/dev/null | grep -qx "$topic"; do
    if (( "$(date +%s)" - start >= timeout_s )); then
      echo "Timed out waiting for $topic" >&2
      return 1
    fi
    sleep 1
  done
}

wait_for_message() {
  local topic="$1"
  local timeout_s="${2:-8}"
  timeout "$timeout_s" ros2 topic echo --once "$topic" >/dev/null 2>&1
}

camera_streams_ready() {
  local timeout_s="${1:-8}"
  local topic
  local required_topics=(
    "/camera/camera/infra1/image_rect_raw"
    "/camera/camera/infra2/image_rect_raw"
    "/camera/camera/depth/image_rect_raw"
    "/camera/camera/aligned_depth_to_color/image_raw"
    "/camera/camera/aligned_depth_to_color/camera_info"
    "/camera/camera/color/image_raw"
    "/camera/camera/infra1/camera_info"
    "/camera/camera/infra2/camera_info"
    "/camera/camera/color/camera_info"
    "/camera/camera/imu"
  )

  for topic in "${required_topics[@]}"; do
    echo "Checking stream: $topic"
    if ! wait_for_message "$topic" "$timeout_s"; then
      echo "No message received from $topic within ${timeout_s}s" >&2
      return 1
    fi
  done
}

stop_known_camera_sessions() {
  tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
  tmux kill-session -t tinynav_semantic_usb3_camera >/dev/null 2>&1 || true
  tmux kill-session -t tinynav_usb3_camera >/dev/null 2>&1 || true
  tmux kill-session -t tinynav_auto_map_camera >/dev/null 2>&1 || true
}

realsense_process_running() {
  pgrep -f 'realsense2_camera|rs_launch.py|run_realsense_.*sensor.sh' >/dev/null 2>&1
}

wait_for_camera_node_gone() {
  local timeout_s="${1:-12}"
  local start
  start="$(date +%s)"

  until ! camera_running; do
    if (( "$(date +%s)" - start >= timeout_s )); then
      if realsense_process_running; then
        return 1
      fi

      echo "ROS still lists /camera/camera, but no RealSense process is running; refreshing ROS daemon discovery."
      ros2 daemon stop >/dev/null 2>&1 || true
      sleep 2
      ros2 daemon start >/dev/null 2>&1 || true
      sleep 2
      ! camera_running
      return
    fi
    sleep 1
  done
}

ensure_camera_streams() {
  if camera_running; then
    echo "Using existing RealSense node: /camera/camera"
    if camera_streams_ready 8; then
      return 0
    fi

    echo "Existing RealSense node is not publishing all required streams; restarting managed camera session."
    stop_known_camera_sessions
    if ! wait_for_camera_node_gone 15; then
      echo "A RealSense node is still running outside the managed tmux sessions." >&2
      echo "Stop that node, then rerun this script." >&2
      return 1
    fi
  fi

  start_camera
  wait_for_topic "/camera/camera/infra1/image_rect_raw" 60
  wait_for_topic "/camera/camera/infra2/image_rect_raw" 60
  wait_for_topic "/camera/camera/imu" 60
  camera_streams_ready 15
}

stop_camera_if_owned() {
  if [[ "$camera_started_here" == "true" && "$keep_camera" != "true" ]]; then
    tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
  fi
}

stop_perception_if_running() {
  if [[ -n "$perception_pid" ]]; then
    kill -INT -- "-$perception_pid" >/dev/null 2>&1 || true
    wait "$perception_pid" >/dev/null 2>&1 || true
    perception_pid=""
  fi
}

stop_semantic_if_running() {
  if [[ -n "$semantic_pid" ]]; then
    # ros2 launch propagates SIGINT so the occupancy node can checkpoint first.
    kill -INT -- "-$semantic_pid" >/dev/null 2>&1 || true
    wait "$semantic_pid" >/dev/null 2>&1 || true
    semantic_pid=""
  fi
}

cleanup() {
  stop_semantic_if_running
  stop_perception_if_running
  stop_camera_if_owned
}

trap cleanup EXIT

cleanup_runtime_temp() {
  if [[ "$clean_temp" != "true" ]]; then
    echo "Keeping TinyNav runtime temp DBs because --keep-temp was set."
    return 0
  fi

  local temp_dirs=(
    "$ROOT_DIR/tinynav_temp"
    "$ROOT_DIR/tinynav_temp_gpu_current"
    "$ROOT_DIR/tinynav_temp_semantic_nav_auto"
  )
  local dir

  echo
  echo "Cleaning TinyNav runtime temp DBs..."
  for dir in "${temp_dirs[@]}"; do
    if [[ -e "$dir" ]]; then
      du -sh "$dir" 2>/dev/null || true
      rm -rf --one-file-system "$dir"
      echo "  removed: $dir"
    fi
  done
}

mkdir -p "$(dirname "$bag_dir")" "$(dirname "$map_dir")" "$ROOT_DIR/output"

if [[ -z "$from_bag" ]]; then
  echo "Checking RealSense streams..."
  ensure_camera_streams

  echo
  echo "Recording mapping bag:"
  echo "  $bag_dir"
  echo "Move through the mapping area now. Press Ctrl-C once to stop recording and start map building."
  echo

  set +e
  bash "$ROOT_DIR/scripts/run_semantic_rosbag_record.sh" --output "$bag_dir"
  record_status=$?
  set -e

  if [[ "$record_status" -ne 0 && "$record_status" -ne 130 && "$record_status" -ne 143 ]]; then
    echo "rosbag recording failed with status $record_status" >&2
    exit "$record_status"
  fi
else
  if [[ ! -d "$bag_dir" ]]; then
    echo "Bag directory does not exist: $bag_dir" >&2
    exit 1
  fi
  echo "Building from existing bag:"
  echo "  $bag_dir"
fi

echo
echo "Recording stopped. Bag info:"
ros2 bag info "$bag_dir" || true

echo
python3 "$ROOT_DIR/tool/validate_tinynav_bag.py" \
  --bag "$bag_dir" \
  --require-semantic-inputs

if [[ "$keep_camera" != "true" ]]; then
  echo "Stopping RealSense before offline map build to avoid duplicate /camera publishers."
  stop_known_camera_sessions
  camera_started_here="false"
  if ! wait_for_camera_node_gone 15; then
    echo "A /camera/camera node is still running outside the managed tmux sessions." >&2
    echo "Stop that camera node before offline map building; duplicate /camera publishers break timestamp sync." >&2
    exit 1
  fi
else
  echo "Keeping RealSense running; make sure no offline bag is being played into the same /camera topics."
fi

echo
echo "Building TinyNav map:"
echo "  $map_dir"
rm -rf "$map_dir"

echo "Starting perception_node for offline odometry/keyframes..."
(
  source_setup
  cd "$ROOT_DIR"
  exec setsid uv run python /tinynav/tinynav/core/perception_node.py
) &
perception_pid=$!
sleep 5

mkdir -p "$ROOT_DIR/logs"
semantic_log="$ROOT_DIR/logs/semantic_occupancy_offline_${timestamp}.log"
semantic_output="$map_dir/semantic_mapping"
semantic_phase3_args=()
if [[ -n "$precomputed_mask_directory" ]]; then
  semantic_phase3_args+=(
    --semantic-masks "$precomputed_mask_directory"
    --semantic-manifest "$precomputed_manifest"
  )
elif [[ -n "$semantic_engine" ]]; then
  semantic_phase3_args+=(
    --semantic-tensorrt "$semantic_engine"
    --semantic-model-config "$semantic_model_config"
    --semantic-preprocessor "$semantic_preprocessor_config"
    --semantic-label-mapping "$semantic_label_mapping"
    --semantic-min-confidence "$semantic_min_confidence"
  )
fi
if [[ -n "$semantic_classes_file" ]]; then
  semantic_phase3_args+=(--semantic-classes "$semantic_classes_file")
fi
echo "Starting timestamped RGB-D point cloud and occupancy mapper in world frame..."
(
  cd "$ROOT_DIR"
  exec setsid bash scripts/run_semantic_pointcloud.sh \
    --offline \
    --target-frame world \
    --output-dir "$semantic_output" \
    "${semantic_phase3_args[@]}"
) >"$semantic_log" 2>&1 &
semantic_pid=$!
sleep 2

uv run python /tinynav/tinynav/core/build_map_node.py \
  --bag_file "$bag_dir" \
  --map_save_path "$map_dir" \
  --play_rate "$play_rate"
stop_perception_if_running
stop_semantic_if_running

for required in poses.npy intrinsics.npy occupancy_grid.npy occupancy_meta.npy sdf_map.npy; do
  if [[ ! -f "$map_dir/$required" ]]; then
    echo "Map build did not produce $required; latest_semantic_map was not updated." >&2
    exit 1
  fi
done

cleanup_runtime_temp

if [[ ! -f "$semantic_output/metadata.yaml" ]]; then
  echo "Warning: Phase-2 occupancy output was not saved; inspect $semantic_log" >&2
fi
if [[ -n "$precomputed_mask_directory" || -n "$semantic_engine" ]]; then
  if [[ ! -f "$semantic_output/semantic_metadata.yaml" || ! -f "$semantic_output/semantic_voxels.npz" || ! -f "$semantic_output/semantic_bev_tensor.npz" ]]; then
    echo "Warning: Phase-4/5 semantic voxel or BEV output was not saved; inspect $semantic_log" >&2
  else
    echo "Reprojecting semantic BEV onto final occupancy grid..."
    python3 "$ROOT_DIR/scripts/export_semantic_bev.py" "$map_dir"
  fi
fi

ln -sfn "$(realpath "$map_dir")" "$ROOT_DIR/output/latest_semantic_map"

echo
echo "Map build finished."
echo "  map:     $map_dir"
echo "  latest:  $ROOT_DIR/output/latest_semantic_map"
echo "  occupancy: $semantic_output"
if [[ -n "$precomputed_mask_directory" ]]; then
  echo "  semantic masks: $precomputed_mask_directory"
elif [[ -n "$semantic_engine" ]]; then
  echo "  semantic backend: TensorRT ($semantic_engine)"
fi
if [[ -f "$semantic_output/semantic_voxels.npz" ]]; then
  echo "  semantic voxels: $semantic_output/semantic_voxels.npz"
fi
if [[ -f "$semantic_output/semantic_bev_tensor.npz" ]]; then
  echo "  semantic BEV tensor: $semantic_output/semantic_bev_tensor.npz"
fi
echo "  semantic geometry log: $semantic_log"
echo
echo "Start navigation with:"
echo "  bash $ROOT_DIR/scripts/tinynav_semantic_auto_nav.sh --map $ROOT_DIR/output/latest_semantic_map"
