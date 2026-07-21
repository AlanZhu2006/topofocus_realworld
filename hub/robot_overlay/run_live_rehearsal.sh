#!/usr/bin/env bash
# One-click live-topic rehearsal for the Focus hub sender, styled after
# TinyNav's own tinynav_semantic_auto_nav.sh (same tmux/health-check/usage
# conventions) but built entirely from TinyNav's UNMODIFIED node scripts plus
# our own overlay files in this directory — nothing here edits the TinyNav
# repo.
#
# Safety: this starts perception_node, map_node and semantic_pointcloud_node
# ONLY (SLAM + relocalization + pose/cloud publishing). It never starts
# planning_node, cmd_vel_control, go2_cmd_bridge or any other actuation path,
# so it cannot move the robot under any circumstance, live camera or not.
#
# Default source is a recorded ROS 2 bag (--from-bag), not the live camera:
# this validates the exact live-topic wire path (message_filters sync across
# rgb/depth/camera_info/camera_pose, real SLAM, real relocalization) without
# needing the camera or the robot powered up. Pass --live to use the real
# camera instead (starts it exactly like tinynav_semantic_auto_nav.sh does);
# only do that with the operator physically present.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TINYNAV_ROOT="${TINYNAV_ROOT:-/home/nvidia/twork/tinynav}"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
SESSION_NAME="${FOCUS_REHEARSAL_SESSION:-focus_live_rehearsal}"

from_bag="${HOME}/.local/share/tinynav/rosbags/semantic_map_record_20260717_102052"
map_dir="$TINYNAV_ROOT/output/semantic_map_record_20260717_102052"
live_camera="false"
bag_rate="1.0"
bag_loop="false"
base_url="http://127.0.0.1:18089"
robot_id="robot-0"
# Empty means generate a unique session/frame epoch after argument parsing.
# Reusing a label across perception restarts can silently mix two odometry
# origins in one Hub map, even though each individual message is valid.
transform_version=""
rate_hz="2.0"
max_frames="0"
capture_time_source=""   # auto: wall for --from-bag, header for --live

usage() {
  cat <<EOF
Usage: $0 [--from-bag DIR] [--live] [--map-dir DIR] [--bag-rate RATE] [--bag-loop]
          [--base-url URL] [--robot-id ID] [--transform-version V]
          [--rate-hz HZ] [--max-frames N] [--session NAME]

Starts (in tmux session '$SESSION_NAME'):
  source            ros2 bag play (default) OR the live RealSense camera (--live)
  perception        TinyNav perception_node (SLAM)
  maploc            TinyNav map_node, relocalizing against --map-dir (read-only)
  pointcloud        TinyNav semantic_pointcloud_node (online, throwaway output dir)
  sender            focus_ros_sender.py -> hub, mapping_only, --base-url

Never starts planning/control/go2-bridge: this cannot move the robot.

Unless --transform-version is supplied, a unique test version is generated
for every launch so a fresh TinyNav odometry origin cannot enter an old map.

Stop:
  bash $SCRIPT_DIR/stop_live_rehearsal.sh --session $SESSION_NAME
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --from-bag) from_bag="$2"; shift 2 ;;
    --live) live_camera="true"; shift ;;
    --map-dir) map_dir="$2"; shift 2 ;;
    --bag-rate) bag_rate="$2"; shift 2 ;;
    --bag-loop) bag_loop="true"; shift ;;
    --base-url) base_url="$2"; shift 2 ;;
    --robot-id) robot_id="$2"; shift 2 ;;
    --transform-version) transform_version="$2"; shift 2 ;;
    --rate-hz) rate_hz="$2"; shift 2 ;;
    --max-frames) max_frames="$2"; shift 2 ;;
    --session) SESSION_NAME="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 1 ;;
  esac
done

if [[ -z "$transform_version" ]]; then
  session_tag="${SESSION_NAME//[^a-zA-Z0-9_.-]/_}"
  transform_version="wsj-${session_tag:0:40}-$(date -u +%Y%m%dT%H%M%SZ)-$$"
fi

if [[ "$bag_loop" == "true" ]]; then
  echo "WARNING: --bag-loop is known to wedge perception_node/map_node's TF" >&2
  echo "buffer at the loop boundary (timestamps jump backward), which stops" >&2
  echo "new /semantic_mapping/camera_pose messages after the first pass." >&2
  echo "This is a bag-replay-only artifact (a live camera's time is always" >&2
  echo "monotonic) — see audit/LIVE_ROS2_SENDER.md. For an extended run," >&2
  echo "prefer repeated single-pass launches over one looped launch." >&2
fi
if [[ "$live_camera" == "false" && ! -d "$from_bag" ]]; then
  echo "Bag directory does not exist: $from_bag" >&2
  exit 1
fi
if [[ ! -f "$map_dir/poses.npy" ]]; then
  echo "Map directory is missing poses.npy (not a built TinyNav map): $map_dir" >&2
  exit 1
fi
if [[ -z "$capture_time_source" ]]; then
  if [[ "$live_camera" == "true" ]]; then capture_time_source="header"; else capture_time_source="wall"; fi
fi
if [[ ! -f "$SCRIPT_DIR/.token" ]]; then
  echo "Missing $SCRIPT_DIR/.token (robot auth token for the hub); deploy it first." >&2
  exit 1
fi

source_setup() {
  local had_nounset=0
  case $- in *u*) had_nounset=1; set +u ;; esac
  source "$SETUP_FILE"
  if [[ "$had_nounset" == "1" ]]; then set -u; fi
}

wait_for_topic() {
  local topic="$1" timeout_s="${2:-45}" start
  start="$(date +%s)"
  until ros2 topic list 2>/dev/null | grep -qx "$topic"; do
    if (( $(date +%s) - start >= timeout_s )); then
      echo "Timed out waiting for topic $topic" >&2
      return 1
    fi
    sleep 1
  done
}

wait_for_message() {
  local topic="$1" timeout_s="${2:-8}"
  timeout "$timeout_s" ros2 topic echo --once "$topic" >/dev/null 2>&1
}

source_setup
cd "$TINYNAV_ROOT"

tinynav_temp_db="$SCRIPT_DIR/rehearsal_tinynav_db_$$"
rehearsal_output_dir="$SCRIPT_DIR/rehearsal_semantic_output_$$"
mkdir -p "$rehearsal_output_dir"

tmux kill-session -t "$SESSION_NAME" >/dev/null 2>&1 || true

if [[ "$live_camera" == "true" ]]; then
  echo "Starting the LIVE RealSense camera (operator must be present)."
  tmux new-session -d -s "$SESSION_NAME" -n source \
    "bash -lc 'source \"$SETUP_FILE\" && cd \"$TINYNAV_ROOT\" && bash scripts/run_realsense_semantic_sensor.sh'"
else
  bag_args=(play "$from_bag" --rate "$bag_rate")
  [[ "$bag_loop" == "true" ]] && bag_args+=(--loop)
  echo "Replaying recorded bag (no camera/robot hardware involved): $from_bag"
  tmux new-session -d -s "$SESSION_NAME" -n source \
    "bash -lc 'source \"$SETUP_FILE\" && ros2 bag ${bag_args[*]}'"
fi

# Keep panes around after their command exits/crashes, so a failure can be
# inspected with `tmux capture-pane` instead of silently vanishing. Must run
# after the session/server exists (tmux set-option -g has no implicit
# server-start the way most other tmux commands do).
tmux set-option -g remain-on-exit on

echo "Waiting for camera/color topic..."
wait_for_topic "/camera/camera/color/image_raw" 60
wait_for_message "/camera/camera/color/image_raw" 15

tmux new-window -t "$SESSION_NAME" -n perception \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$TINYNAV_ROOT\" && uv run python /tinynav/tinynav/core/perception_node.py'"

echo "Waiting for SLAM odometry..."
wait_for_topic "/slam/odometry_visual" 45
wait_for_message "/slam/odometry_visual" 15

tmux new-window -t "$SESSION_NAME" -n maploc \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$TINYNAV_ROOT\" && uv run python /tinynav/tinynav/core/map_node.py --tinynav_db_path \"$tinynav_temp_db\" --tinynav_map_path \"$map_dir\"'"

tmux new-window -t "$SESSION_NAME" -n pointcloud \
  "bash -lc 'cd \"$TINYNAV_ROOT\" && bash scripts/run_semantic_pointcloud.sh --online --target-frame map --output-dir \"$rehearsal_output_dir\"'"

echo "Waiting for /slam/keyframe_odom (perception_node's own live SLAM estimate)..."
# Was /semantic_mapping/camera_pose (needs map_node to successfully relocalize
# against a pre-built map) until 2026-07-20. That's stale now: the sender's
# pose source was switched to /slam/keyframe_odom on 2026-07-19 precisely
# because it doesn't need relocalization (see the module docstring's
# HPC-fidelity pivot section) -- but this readiness gate kept waiting on the
# old topic anyway, so the sender never started at all if relocalization
# happened not to converge, even though its actual pose source was already
# publishing fine. Confirmed live (2026-07-20): /slam/keyframe_odom updates
# in real time from the very start of the "perception" window, independent
# of whatever map_node is doing.
wait_for_topic "/slam/keyframe_odom" 45
wait_for_message "/slam/keyframe_odom" 15

tmux new-window -t "$SESSION_NAME" -n sender \
  "bash -lc 'source \"$SETUP_FILE\" && cd \"$SCRIPT_DIR\" && FOCUS_ROBOT_TOKEN=\$(cat .token) python3 focus_ros_sender.py \
     --base-url \"$base_url\" --robot-id \"$robot_id\" --transform-version \"$transform_version\" \
     --capture-time-source \"$capture_time_source\" --rate-hz \"$rate_hz\" --max-frames \"$max_frames\" \
     --metrics-out \"$SCRIPT_DIR/focus_ros_sender_metrics.json\" 2>&1 | tee \"$SCRIPT_DIR/focus_ros_sender_window.log\"'"

tmux new-window -t "$SESSION_NAME" -n monitor \
  "bash -lc 'source \"$SETUP_FILE\" && watch -n 2 \"ros2 topic hz /camera/camera/color/image_raw --window 10 2>/dev/null | tail -3; echo; ros2 topic hz /semantic_mapping/camera_pose --window 10 2>/dev/null | tail -3\"'"

tmux select-window -t "$SESSION_NAME":sender >/dev/null 2>&1 || true

echo
echo "Live rehearsal running (session: $SESSION_NAME)."
echo "  source:            $([[ "$live_camera" == "true" ]] && echo "LIVE camera" || echo "bag replay: $from_bag")"
echo "  map (relocalize):  $map_dir"
echo "  hub:               $base_url (robot_id=$robot_id, transform_version=$transform_version)"
echo "  capture_time_source: $capture_time_source"
echo "  rehearsal temp db:  $tinynav_temp_db"
echo "  rehearsal output:   $rehearsal_output_dir"
echo
echo "Attach:  tmux attach -d -t $SESSION_NAME"
echo "Stop:    bash $SCRIPT_DIR/stop_live_rehearsal.sh --session $SESSION_NAME"
