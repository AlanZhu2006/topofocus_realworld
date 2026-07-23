#!/usr/bin/env bash
# Start the minimal Yunji Odin observation + v2 WATER receiver stack.
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
mode="debug"
confirmation=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) mode="$2"; shift 2 ;;
    --operator-confirmation) confirmation="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: $0 --mode debug|live [--operator-confirmation OPERATOR_PRESENT_AND_YUNJI_CLEAR]"
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
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
  "$SCRIPT_DIR/run_yunji_mapping_observation.sh" \
  "$SCRIPT_DIR/v2_yunji_receiver.py" \
  "$ENV_FILE" \
  "$CALIBRATION_FILE" \
  "$BASE_CAMERA_CALIBRATION" \
  "$FACTORY_CALIBRATION"; do
  [[ -r "$required" ]] || { echo "Missing required file: $required" >&2; exit 1; }
done
systemctl is-active --quiet focus-yunji-odin1-driver.service || {
  echo "Odin driver is not active." >&2
  exit 1
}

SENDER_UNIT="focus-yunji-command-observation-v2.service"
RECEIVER_UNIT="focus-yunji-v2-${mode}-v2.service"
if systemctl is-active --quiet "$SENDER_UNIT" \
   && systemctl is-active --quiet "$RECEIVER_UNIT"; then
  echo "Yunji v2 stack is already ready: mode=$mode"
  [[ "$mode" == debug ]] \
    && echo "Safety: receiver is read-only; no WATER move/cancel is emitted."
  exit 0
fi

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
metrics="/home/nyu/.local/state/topofocus/yunji-command-observation-${stamp}.json"
if ! systemctl is-active --quiet "$SENDER_UNIT"; then
  for unit in \
    focus-yunji-calibration-observation-v1.service \
    focus-yunji-odin1-calibrated-v2.service \
    focus-yunji-command-observation.service \
    "$SENDER_UNIT"; do
    sudo -n systemctl stop "$unit" >/dev/null 2>&1 || true
    sudo -n systemctl reset-failed "$unit" >/dev/null 2>&1 || true
  done
  sudo -n systemd-run \
    --unit="${SENDER_UNIT%.service}" \
    --property=Type=exec \
    --uid=nyu --gid=nyu \
    --working-directory="$RELEASE_ROOT" \
    /bin/bash "$SCRIPT_DIR/run_yunji_mapping_observation.sh" \
      --transform-version "$TRANSFORM_VERSION" \
      --shared-frame-transform-file "$CALIBRATION_FILE" \
      --base-camera-calibration-file "$BASE_CAMERA_CALIBRATION" \
      --command-capable \
      --env "$ENV_FILE" \
      --metrics-out "$metrics" >/dev/null
  sleep 2
  systemctl is-active --quiet "$SENDER_UNIT" || {
    journalctl -u "$SENDER_UNIT" -n 60 --no-pager >&2
    exit 1
  }
fi

for unit in \
  focus-yunji-v2-readonly-v4.service \
  focus-yunji-v2-runtime.service \
  focus-yunji-v2-debug-v2.service \
  focus-yunji-v2-live-v2.service; do
  [[ "$unit" == "$RECEIVER_UNIT" ]] && continue
  sudo -n systemctl stop "$unit" >/dev/null 2>&1 || true
  sudo -n systemctl reset-failed "$unit" >/dev/null 2>&1 || true
done
# A stopped systemd-run transient unit can remain loaded as "failed".  The
# active/ready case returned above, so it is safe and necessary to clear the
# selected non-active unit before recreating it with the same stable name.
sudo -n systemctl stop "$RECEIVER_UNIT" >/dev/null 2>&1 || true
sudo -n systemctl reset-failed "$RECEIVER_UNIT" >/dev/null 2>&1 || true

alignment="/home/nyu/.local/state/topofocus/yunji-v2-${mode}-${stamp}.json"
log="/home/nyu/.local/state/topofocus/yunji-v2-${mode}-${stamp}.jsonl"
receiver=(
  /usr/bin/python3 -u "$SCRIPT_DIR/v2_yunji_receiver.py"
  --base-url "$HUB_URL"
  --calibration-file "$CALIBRATION_FILE"
  --base-camera-calibration-file "$BASE_CAMERA_CALIBRATION"
  --odin-factory-calibration-file "$FACTORY_CALIBRATION"
  --transform-version "$TRANSFORM_VERSION"
  --shared-frame-calibration-id "$CALIBRATION_ID"
  --alignment-output "$alignment"
  --log "$log"
)
if [[ "$mode" == live ]]; then
  receiver+=(
    --enable-live-water-motion
    --operator-confirmation OPERATOR_PRESENT_AND_YUNJI_CLEAR
  )
fi
printf -v receiver_text '%q ' "${receiver[@]}"
sudo -n systemd-run \
  --unit="${RECEIVER_UNIT%.service}" \
  --property=Type=exec \
  --uid=nyu --gid=nyu \
  --working-directory="$RELEASE_ROOT" \
  /bin/bash -c \
  "set -a; source '$ENV_FILE'; set +a; unset COLCON_CURRENT_PREFIX AMENT_CURRENT_PREFIX; source /opt/ros/humble/setup.bash; unset COLCON_CURRENT_PREFIX; source /home/nyu/odin_ws/install/setup.bash; export PYTHONPATH='$SCRIPT_DIR/../src':\${PYTHONPATH:-}; exec $receiver_text" \
  >/dev/null

deadline=$((SECONDS + 35))
until [[ -s "$alignment" ]]; do
  systemctl is-active --quiet "$RECEIVER_UNIT" || {
    journalctl -u "$RECEIVER_UNIT" -n 80 --no-pager >&2
    exit 1
  }
  (( SECONDS < deadline )) || {
    echo "Timed out waiting for Yunji v2 alignment." >&2
    exit 1
  }
  sleep 1
done

echo "Yunji v2 stack ready: mode=$mode alignment=$alignment"
if [[ "$mode" == debug ]]; then
  echo "Safety: receiver is read-only; no WATER move/cancel is emitted."
fi
