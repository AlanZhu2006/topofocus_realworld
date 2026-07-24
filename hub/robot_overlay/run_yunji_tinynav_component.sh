#!/usr/bin/env bash
# Internal systemd entry point for one Yunji TinyNav component.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TINYNAV_RUNTIME="${FOCUS_YUNJI_TINYNAV_RUNTIME:-/home/nyu/.local/share/topofocus/tinynav-runtime}"
TINYNAV_SOURCE="$TINYNAV_RUNTIME/source"
PYTHON_BIN="$TINYNAV_RUNTIME/venv/bin/python"
SEMANTIC_ROOT="$SCRIPT_DIR/tinynav_snapshot/working-tree-files/semantic_mapping"
component="${1:-}"
shift || true

[[ -x "$PYTHON_BIN" ]] || {
  echo "Pinned TinyNav Python is unavailable: $PYTHON_BIN" >&2
  exit 1
}
[[ -d "$TINYNAV_SOURCE/tinynav" ]] || {
  echo "Pinned TinyNav source is unavailable: $TINYNAV_SOURCE" >&2
  exit 1
}

had_nounset=0
case $- in *u*) had_nounset=1; set +u ;; esac
unset COLCON_CURRENT_PREFIX AMENT_CURRENT_PREFIX
source /opt/ros/humble/setup.bash
unset COLCON_CURRENT_PREFIX
source /home/nyu/odin_ws/install/setup.bash
[[ "$had_nounset" == 1 ]] && set -u

export PYTHONPATH="$TINYNAV_SOURCE:$SEMANTIC_ROOT:$SCRIPT_DIR/../src${PYTHONPATH:+:$PYTHONPATH}"

case "$component" in
  adapter)
    exec "$PYTHON_BIN" -u "$SCRIPT_DIR/odin1_tinynav_adapter.py" "$@"
    ;;
  occupancy)
    exec "$PYTHON_BIN" -u \
      "$SEMANTIC_ROOT/semantic_mapping/occupancy_mapper_node.py" "$@"
    ;;
  planner)
    exec "$PYTHON_BIN" -u "$SCRIPT_DIR/run_yunji_tinynav_planner.py" "$@"
    ;;
  controller)
    exec "$PYTHON_BIN" -u \
      "$TINYNAV_SOURCE/tinynav/platforms/cmd_vel_control.py" "$@"
    ;;
  router)
    exec "$PYTHON_BIN" -u "$SCRIPT_DIR/tinynav_buildmap_goal_router.py" "$@"
    ;;
  bridge)
    exec "$PYTHON_BIN" -u "$SCRIPT_DIR/water_cmd_vel_bridge.py" "$@"
    ;;
  receiver)
    exec "$PYTHON_BIN" -u "$SCRIPT_DIR/v2_wsj_receiver.py" "$@"
    ;;
  *)
    echo "Unknown Yunji TinyNav component: $component" >&2
    exit 2
    ;;
esac
