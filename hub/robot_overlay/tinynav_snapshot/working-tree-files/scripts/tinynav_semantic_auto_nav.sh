#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
SESSION_NAME="${TINYNAV_SEMANTIC_NAV_SESSION:-tinynav_semantic_nav_auto}"
map_path="$ROOT_DIR/output/latest_semantic_map"
nav_db_path="$ROOT_DIR/tinynav_temp_semantic_nav_auto"
start_rviz="true"
start_go2="true"
display="${TINYNAV_RVIZ_DISPLAY:-:1}"
default_xauthority="/home/nvidia/.Xauthority"
if [[ "$display" == ":0" && -f "/run/user/$(id -u)/gdm/Xauthority" ]]; then
  default_xauthority="/run/user/$(id -u)/gdm/Xauthority"
fi
rviz_xauthority="${TINYNAV_RVIZ_XAUTHORITY:-$default_xauthority}"
rviz_config="${TINYNAV_RVIZ_CONFIG:-$ROOT_DIR/semantic_mapping/rviz/tinynav_semantic_nav.rviz}"
go2_net_if="${UNITREE_NET_IF:-eth0}"
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
Usage: $0 [--map DIR] [--session NAME] [--no-rviz] [--no-go2] [--go2-net-if IFACE]
          [--semantic-masks DIR] [--semantic-manifest FILE]
          [--semantic-tensorrt ENGINE] [--semantic-min-confidence VALUE]
          [--semantic-classes FILE]

Starts the minimal TinyNav navigation stack:
  RealSense if needed
  perception_node
  planning_node
  map_node
  cmd_vel_control
  Go2 cmd_vel bridge
  rviz_goal_to_poi
  static_occupancy_grid_publisher
  RGB-D point cloud + sparse occupancy voxels + occupancy BEV
  optional 2D perception + 3D semantic fusion with semantic backend flags
  RViz

Default map:
  $map_path
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) map_path="$2"; shift 2 ;;
    --session) SESSION_NAME="$2"; shift 2 ;;
    --no-rviz) start_rviz="false"; shift ;;
    --no-go2) start_go2="false"; shift ;;
    --go2-net-if) go2_net_if="$2"; shift 2 ;;
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

if [[ ! -d "$map_path" ]]; then
  echo "Map directory does not exist: $map_path" >&2
  echo "Build one first with: bash $ROOT_DIR/scripts/tinynav_semantic_auto_map.sh" >&2
  exit 1
fi

for required in poses.npy intrinsics.npy occupancy_grid.npy occupancy_meta.npy sdf_map.npy; do
  if [[ ! -f "$map_path/$required" ]]; then
    echo "Map is missing $required: $map_path" >&2
    exit 1
  fi
done

occupancy_load_args=""
occupancy_map_path="$map_path/semantic_mapping"
if [[ -f "$occupancy_map_path/metadata.yaml" && -f "$occupancy_map_path/voxels.npz" ]]; then
  printf -v quoted_occupancy_map '%q' "$occupancy_map_path"
  occupancy_load_args="--input-dir $quoted_occupancy_map --allow-frame-override"
fi

semantic_perception_args=""
if [[ -n "$precomputed_mask_directory" ]]; then
  printf -v quoted_semantic_masks '%q' "$precomputed_mask_directory"
  printf -v quoted_semantic_manifest '%q' "$precomputed_manifest"
  semantic_perception_args="--semantic-masks $quoted_semantic_masks --semantic-manifest $quoted_semantic_manifest"
elif [[ -n "$semantic_engine" ]]; then
  printf -v quoted_semantic_engine '%q' "$semantic_engine"
  printf -v quoted_semantic_model_config '%q' "$semantic_model_config"
  printf -v quoted_semantic_preprocessor '%q' "$semantic_preprocessor_config"
  printf -v quoted_semantic_mapping '%q' "$semantic_label_mapping"
  printf -v quoted_semantic_confidence '%q' "$semantic_min_confidence"
  semantic_perception_args="--semantic-tensorrt $quoted_semantic_engine --semantic-model-config $quoted_semantic_model_config --semantic-preprocessor $quoted_semantic_preprocessor --semantic-label-mapping $quoted_semantic_mapping --semantic-min-confidence $quoted_semantic_confidence"
fi
if [[ -n "$semantic_classes_file" ]]; then
  printf -v quoted_semantic_classes '%q' "$semantic_classes_file"
  semantic_perception_args="$semantic_perception_args --semantic-classes $quoted_semantic_classes"
fi

camera_running() {
  ros2 node list 2>/dev/null | grep -qx "/camera/camera"
}

wait_for_camera_node_gone() {
  local timeout_s="${1:-15}"
  local start
  start="$(date +%s)"
  while camera_running; do
    if (( "$(date +%s)" - start >= timeout_s )); then
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
  tmux kill-session -t tinynav_usb3_camera >/dev/null 2>&1 || true
  tmux kill-session -t tinynav_semantic_usb3_camera >/dev/null 2>&1 || true
  tmux kill-session -t tinynav_auto_map_camera >/dev/null 2>&1 || true
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="${2:-45}"
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

ensure_go2_network() {
  if ip -o -4 addr show dev "$go2_net_if" | grep -q "192\\.168\\.123\\."; then
    return 0
  fi

  echo "Adding temporary Go2 address 192.168.123.100/24 to $go2_net_if"
  sudo ip addr add 192.168.123.100/24 dev "$go2_net_if" 2>/dev/null || true
  sudo ip link set "$go2_net_if" up
}

vnc_display_ready() {
  DISPLAY="$display" XAUTHORITY="$rviz_xauthority" xrandr >/dev/null 2>&1
}

ensure_vnc_display() {
  if [[ "$display" != ":1" ]]; then
    return 0
  fi

  if vnc_display_ready; then
    echo "Using existing VNC display $display"
    return 0
  fi

  if systemctl --user cat tinynav-vnc.service >/dev/null 2>&1; then
    if systemctl --user is-active --quiet tinynav-vnc.service 2>/dev/null; then
      echo "Restarting tinynav-vnc.service for RViz display $display"
      systemctl --user restart tinynav-vnc.service
    else
      echo "Starting tinynav-vnc.service for RViz display $display"
      systemctl --user start tinynav-vnc.service
    fi
    sleep 3
    if vnc_display_ready; then
      return 0
    fi
  fi

  if [[ -n "${TINYNAV_VNC_PASSWORD:-}" ]]; then
    bash "$ROOT_DIR/scripts/run_visible_vnc.sh"
    sleep 2
    vnc_display_ready
    return
  fi

  echo "VNC display $display is not ready." >&2
  echo "Start it with: systemctl --user start tinynav-vnc.service" >&2
  echo "Or set TINYNAV_VNC_PASSWORD and run scripts/run_visible_vnc.sh." >&2
  return 1
}

wait_for_rviz() {
  local timeout_s="${1:-25}"
  local start
  start="$(date +%s)"
  until pgrep -u "$(id -u)" -f "rviz2.*${rviz_config}" >/dev/null 2>&1; do
    if (( "$(date +%s)" - start >= timeout_s )); then
      echo "RViz did not start within ${timeout_s}s." >&2
      echo "Check: $ROOT_DIR/logs/rviz.log" >&2
      echo "Attach: tmux attach -t $SESSION_NAME" >&2
      return 1
    fi
    sleep 1
  done
}

tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true
if [[ "$start_go2" == "true" ]]; then
  tmux kill-session -t go2_cmd_bridge >/dev/null 2>&1 || true
  ensure_go2_network
  if ! ping -I "$go2_net_if" -c 1 -W 1 192.168.123.161 >/dev/null 2>&1; then
    echo "Go2 is not reachable at 192.168.123.161 through $go2_net_if." >&2
    echo "Use --no-go2 to start navigation without sending /cmd_vel to the base." >&2
    exit 1
  fi
fi

if [[ "$start_rviz" == "true" ]]; then
  ensure_vnc_display
fi

camera_mode="start"
if camera_running; then
  echo "Found existing RealSense node: /camera/camera"
  if camera_streams_ready 8; then
    camera_mode="existing"
  else
    echo "Existing RealSense node is not publishing all required streams; restarting managed camera session."
    stop_known_camera_sessions
    if ! wait_for_camera_node_gone 15; then
      echo "A RealSense node is still running outside the managed tmux sessions." >&2
      echo "Stop that node, then rerun this script." >&2
      exit 1
    fi
  fi
fi

if [[ "$camera_mode" == "existing" ]]; then
  tmux new-session -d -s "$SESSION_NAME" -n camera \
    "bash -lc 'echo \"Using existing RealSense node /camera/camera\"; echo \"Do not start another RealSense launch while this is running.\"; sleep infinity'"
else
  tmux new-session -d -s "$SESSION_NAME" -n camera \
    "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && bash scripts/run_realsense_semantic_sensor.sh'"
fi

echo "Waiting for camera topics..."
wait_for_topic "/camera/camera/infra1/image_rect_raw" 60
wait_for_topic "/camera/camera/infra2/image_rect_raw" 60
wait_for_topic "/camera/camera/imu" 60
camera_streams_ready 15

tmux new-window -t "$SESSION_NAME" -n perception \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tinynav/core/perception_node.py'"

tmux new-window -t "$SESSION_NAME" -n planning \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tinynav/core/planning_node.py'"

tmux new-window -t "$SESSION_NAME" -n map \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tinynav/core/map_node.py --tinynav_db_path \"$nav_db_path\" --tinynav_map_path \"$map_path\"'"

tmux new-window -t "$SESSION_NAME" -n occupancy-map \
  "bash -lc 'cd \"$ROOT_DIR\" && bash scripts/run_semantic_pointcloud.sh --online --target-frame map --output-dir \"$occupancy_map_path\" $occupancy_load_args $semantic_perception_args'"

tmux new-window -t "$SESSION_NAME" -n control \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tinynav/platforms/cmd_vel_control.py'"

if [[ "$start_go2" == "true" ]]; then
  tmux new-window -t "$SESSION_NAME" -n go2-bridge \
    "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && export UNITREE_NET_IF=\"$go2_net_if\" GO2_CMD_TOPIC=/cmd_vel GO2_MAX_VX=0.30 GO2_MAX_VY=0.00 GO2_MAX_WZ=0.70 GO2_MIN_CMD_V=0.10 GO2_MIN_CMD_W=0.20 GO2_REMOTE_PRIORITY=true GO2_REMOTE_TOPIC=rt/lowstate GO2_REMOTE_DEADBAND=0.12 GO2_REMOTE_HOLD_SEC=0.8 GO2_LOG_COMMANDS=true && bash scripts/run_go2_cmd_bridge.sh'"
fi

tmux new-window -t "$SESSION_NAME" -n rviz-goal \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tool/rviz_goal_to_poi.py --tinynav_map_path \"$map_path\" --z 0.0 --marker-z-offset 1.5'"

tmux new-window -t "$SESSION_NAME" -n static-map \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tool/static_occupancy_grid_publisher.py --tinynav-map-path \"$map_path\" --topic /mapping/static_occupancy_grid --frame-id map --z 0.0'"

tmux new-window -t "$SESSION_NAME" -n map-keyframes \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tool/map_keyframe_publisher.py --tinynav-map-path \"$map_path\" --frame-id map'"

tmux new-window -t "$SESSION_NAME" -n pose-marker \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && uv run python /tinynav/tool/current_pose_marker.py --pose-topic /mapping/current_pose_in_map --marker-topic /mapping/current_pose_marker --frame-id map'"

if [[ "$start_rviz" == "true" ]]; then
  tmux new-window -t "$SESSION_NAME" -n rviz \
    "bash -lc 'source \"$SETUP_FILE\" && cd \"$ROOT_DIR\" && TINYNAV_RVIZ_DISPLAY=\"$display\" TINYNAV_RVIZ_XAUTHORITY=\"$rviz_xauthority\" TINYNAV_RVIZ_CONFIG=\"$rviz_config\" bash scripts/run_rviz_vnc.sh'"
  wait_for_rviz 30
fi

tmux new-window -t "$SESSION_NAME" -n monitor \
  "bash -lc 'source \"$SETUP_FILE\" && watch -n 1 \"ros2 topic hz /cmd_vel --window 10 2>/dev/null | tail -5; echo; ros2 topic hz /semantic_mapping/semantic_pointcloud --window 10 2>/dev/null | tail -5; echo; ros2 topic hz /semantic_mapping/semantic_voxels --window 10 2>/dev/null | tail -5; echo; ros2 topic echo --once /mapping/nav_progress 2>/dev/null || true\"'"

tmux select-window -t "$SESSION_NAME":map >/dev/null 2>&1 || true

echo
echo "TinyNav navigation started."
echo "  session: $SESSION_NAME"
echo "  map:     $map_path"
echo "  go2:     $start_go2"
echo "  rviz:    $rviz_config"
echo "  semantic point cloud: /semantic_mapping/semantic_pointcloud"
echo "  occupied voxels:      /semantic_mapping/occupied_voxels"
echo "  occupancy BEV:        /semantic_mapping/occupancy_bev"
echo "  explored BEV:         /semantic_mapping/explored_bev"
if [[ -n "$precomputed_mask_directory" || -n "$semantic_engine" ]]; then
  echo "  semantic label:       /semantic_mapping/semantic_label_image"
  echo "  semantic confidence:  /semantic_mapping/semantic_confidence_image"
  echo "  semantic overlay:     /semantic_mapping/semantic_visualization"
  echo "  semantic voxels:      /semantic_mapping/semantic_voxels"
  echo "  semantic markers:     /semantic_mapping/semantic_voxel_markers"
  echo "  semantic BEV:         /semantic_mapping/semantic_bev"
  echo "  semantic BEV view:    /semantic_mapping/semantic_bev_visualization"
fi
if [[ -n "$occupancy_load_args" ]]; then
  echo "  loaded occupancy:     $occupancy_map_path"
else
  echo "  loaded occupancy:     none (starting an empty online voxel map)"
fi
if [[ "$start_go2" == "true" ]]; then
  echo "  go2 net: $go2_net_if -> 192.168.123.161"
fi
echo
echo "Attach:"
echo "  tmux attach -d -t $SESSION_NAME"
echo
echo "Stop:"
echo "  bash $ROOT_DIR/scripts/stop_tinynav_semantic_nav.sh --session $SESSION_NAME"
