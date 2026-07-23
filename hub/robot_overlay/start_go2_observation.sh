#!/usr/bin/env bash
# Start only the verified D435i + TinyNav perception path. This intentionally
# has no map planner, cmd_vel controller, Unitree bridge, or Hub GOAL receiver.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
env_file=""
session="${FOCUS_REHEARSAL_SESSION:-focus_live_rehearsal}"
session_from_cli=false
TINYNAV_ROOT="${TINYNAV_ROOT:-/home/nvidia/twork/tinynav}"
PATCHED_ROOT="${TINYNAV_PATCHED_ROOT:-/home/nvidia/twork/tinynav-topofocus}"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
PYTHON_BIN="${TINYNAV_PYTHON:-$TINYNAV_ROOT/.venv/bin/python}"
STATE_DIR="${FOCUS_ROBOT_STATE_DIR:-$HOME/.local/state/topofocus}"
POWER_SERVICE="${FOCUS_REALSENSE_POWER_SERVICE:-focus-realsense-power.service}"

usage() {
  cat <<EOF
Usage: $0 [--env FILE] [--session NAME]

Starts camera and perception only. The operator must be physically present.
It cannot move the robot because no planning/control/Unitree process is started.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --env) env_file="$2"; shift 2 ;;
    --session) session="$2"; session_from_cli=true; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -n "$env_file" ]]; then
  [[ -f "$env_file" ]] || { echo "Missing env file: $env_file" >&2; exit 1; }
  set -a
  source "$env_file"
  set +a
  TINYNAV_ROOT="${TINYNAV_ROOT:-/home/nvidia/twork/tinynav}"
  PATCHED_ROOT="${TINYNAV_PATCHED_ROOT:-/home/nvidia/twork/tinynav-topofocus}"
  SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
  PYTHON_BIN="${TINYNAV_PYTHON:-$TINYNAV_ROOT/.venv/bin/python}"
  STATE_DIR="${FOCUS_ROBOT_STATE_DIR:-$HOME/.local/state/topofocus}"
  POWER_SERVICE="${FOCUS_REALSENSE_POWER_SERVICE:-focus-realsense-power.service}"
  if [[ "$session_from_cli" != true ]]; then
    session="${FOCUS_REHEARSAL_SESSION:-$session}"
  fi
fi

[[ -f "$SETUP_FILE" ]] || { echo "Missing setup file: $SETUP_FILE" >&2; exit 1; }
[[ -f "$PATCHED_ROOT/tinynav/core/perception_node.py" ]] || { echo "Missing patched perception tree: $PATCHED_ROOT" >&2; exit 1; }
[[ -x "$PYTHON_BIN" ]] || { echo "Missing TinyNav Python: $PYTHON_BIN" >&2; exit 1; }
tmux has-session -t "$session" 2>/dev/null && {
  echo "Refusing to replace existing tmux session: $session" >&2
  exit 1
}

source_setup() {
  local had_nounset=0
  case $- in *u*) had_nounset=1; set +u ;; esac
  source "$SETUP_FILE"
  [[ "$had_nounset" == 1 ]] && set -u
}
source_setup

if pgrep -af 'go2_cmd_bridge|cmd_vel_control|planning_node.py|nav2_controller' >/dev/null 2>&1; then
  echo "Refusing observation launch while a known actuation/planner process is running." >&2
  exit 1
fi

mkdir -p "$STATE_DIR"
stamp="$(date -u +%Y%m%dT%H%M%SZ)"
camera_log="$STATE_DIR/realsense-$stamp.log"
perception_log="$STATE_DIR/perception-$stamp.log"
launch_complete=false
cleanup_on_error() {
  if [[ "$launch_complete" != true ]]; then
    tmux kill-session -t "$session" >/dev/null 2>&1 || true
  fi
}
trap cleanup_on_error EXIT

camera_command="source '$SETUP_FILE'; set -o pipefail; ros2 launch realsense2_camera rs_launch.py initial_reset:=false publish_tf:=true tf_publish_rate:=1.0 enable_depth:=false enable_color:=true enable_infra1:=true enable_infra2:=true enable_gyro:=true enable_accel:=true enable_sync:=false align_depth.enable:=false depth_module.infra_profile:=848x480x30 rgb_camera.color_profile:=848x480x30 unite_imu_method:=2 2>&1 | tee '$camera_log'"
tmux new-session -d -s "$session" -n camera "bash -lc \"$camera_command\""
tmux set-window-option -t "$session" remain-on-exit on >/dev/null

deadline=$((SECONDS + 60))
until ros2 topic list 2>/dev/null | grep -qx '/camera/camera/color/image_raw'; do
  (( SECONDS < deadline )) || { echo "Timed out waiting for RealSense color topic" >&2; exit 1; }
  sleep 1
done
timeout 15 ros2 topic echo --once /camera/camera/color/image_raw >/dev/null

# The RealSense driver can restore power/control=auto after bind. Reapply the
# policy after the camera node exists, then fail closed if it did not stick.
if systemctl cat "$POWER_SERVICE" >/dev/null 2>&1; then
  sudo -n systemctl restart "$POWER_SERVICE" >/dev/null 2>&1 || true
fi
power_bad=0
camera_seen=0
for device in /sys/bus/usb/devices/*; do
  [[ -r "$device/idVendor" && -r "$device/idProduct" ]] || continue
  pair="$(<"$device/idVendor"):$(<"$device/idProduct")"
  case "$pair" in
    8086:0b3a) camera_seen=1; [[ "$(<"$device/power/control")" == on ]] || power_bad=1 ;;
    05e3:0625) [[ "$(<"$device/power/control")" == on ]] || power_bad=1 ;;
  esac
done
if [[ "$camera_seen" -ne 1 || "$power_bad" -ne 0 ]]; then
  echo "RealSense USB power policy is not stable after driver bind." >&2
  echo "Install/restart $POWER_SERVICE, then retry." >&2
  exit 1
fi

perception_command="source '$SETUP_FILE'; export PYTHONPATH='$PATCHED_ROOT':\${PYTHONPATH:-}; cd '$PATCHED_ROOT'; set -o pipefail; '$PYTHON_BIN' -u tinynav/core/perception_node.py 2>&1 | tee '$perception_log'"
tmux new-window -d -t "$session" -n perception "bash -lc \"$perception_command\""

deadline=$((SECONDS + 60))
until ros2 topic list 2>/dev/null | grep -qx '/slam/odometry_visual'; do
  (( SECONDS < deadline )) || { echo "Timed out waiting for TinyNav visual odometry" >&2; exit 1; }
  sleep 1
done
# A stationary post-reboot robot can legitimately take longer to select its
# first keyframe. Continuous visual odometry is the actual perception-health
# signal; downstream BuildMap startup separately waits for keyframe products.
timeout 45 ros2 topic echo --once /slam/odometry_visual >/dev/null

launch_complete=true
trap - EXIT
echo "Observation-only stack is healthy in tmux session: $session"
echo "  camera log:     $camera_log"
echo "  perception log: $perception_log"
echo "No planner, command bridge, or actuator was started."
