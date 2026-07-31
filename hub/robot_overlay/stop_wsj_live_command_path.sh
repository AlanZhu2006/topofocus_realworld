#!/usr/bin/env bash
# Fail-closed Robot 0 command-path stop. The read-only mapping core is kept.
set -euo pipefail

SESSION="${FOCUS_WSJ_NAV_SESSION:-tinynav_semantic_nav_auto}"
ENV_FILE="${FOCUS_WSJ_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/topofocus/robot-0.env}"

if [[ -r "$ENV_FILE" ]]; then
  set -a
  # bootstrap_robot0_cleanroom.sh emits shell-quoted assignments only.
  # shellcheck disable=SC1090
  source "$ENV_FILE"
  set +a
fi
SETUP_FILE="${TINYNAV_SETUP:-$HOME/twork/tinynav_setup.bash}"
[[ -r "$SETUP_FILE" ]] || {
  echo "Missing Robot 0 setup file: $SETUP_FILE" >&2
  exit 1
}

had_nounset=0
case $- in
  *u*) had_nounset=1; set +u ;;
esac
# shellcheck disable=SC1090
source "$SETUP_FILE"
[[ "$had_nounset" == 1 ]] && set -u

# Only fail-closed messages are published. There is no positive velocity or
# high-level target, and the local Go2 bridge retains final stop authority.
timeout 5 ros2 topic pub --once \
  /nav/paused std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 || true
if timeout 5 ros2 topic list 2>/dev/null \
    | grep -qx /focus_guarded_cmd_vel; then
  timeout 5 ros2 topic pub --once \
    /focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}' \
    >/dev/null 2>&1 || true
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  for window in go2-bridge v2-receiver calibration-sender; do
    if tmux list-windows -t "$SESSION" -F '#{window_name}' \
        | grep -qx "$window"; then
      tmux kill-window -t "$SESSION:$window"
    fi
  done
fi

echo "ROBOT0_COMMAND_PATH_STOPPED=true"
echo "No positive velocity or high-level target was sent."
