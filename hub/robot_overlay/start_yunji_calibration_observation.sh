#!/usr/bin/env bash
# Start a fresh, mapping-only Odin observation epoch for board calibration.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RELEASE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ENV_FILE="${FOCUS_ODIN_ENV_FILE:-/home/nyu/focus_sender_odin1/focus-odin1.env}"
HUB_URL="${FOCUS_HUB_BASE_URL:-http://127.0.0.1:18089}"
READY_TIMEOUT_S="${FOCUS_YUNJI_CALIBRATION_READY_TIMEOUT_S:-30}"
TRANSFORM_VERSION=""
CONFIRMATION=""
UNIT="focus-yunji-calibration-observation-v1.service"

usage() {
  cat <<'EOF'
Usage: start_yunji_calibration_observation.sh \
  --transform-version UNIQUE_RAW_TRANSFORM \
  --operator-confirmation OPERATOR_PRESENT_AND_BOARD_ONLY [--env FILE]

This stops v2 receivers and command-capable senders, then starts only the raw
Odin mapping sender. It never starts a WATER move endpoint.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --transform-version) TRANSFORM_VERSION="$2"; shift 2 ;;
    --operator-confirmation) CONFIRMATION="$2"; shift 2 ;;
    --env) ENV_FILE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$TRANSFORM_VERSION" =~ ^[A-Za-z0-9_.-]+$ ]] || {
  echo "A filesystem-safe raw --transform-version is required." >&2
  exit 2
}
[[ "$CONFIRMATION" == OPERATOR_PRESENT_AND_BOARD_ONLY ]] || {
  echo "Calibration observation requires OPERATOR_PRESENT_AND_BOARD_ONLY." >&2
  exit 2
}
for required in \
  "$ENV_FILE" \
  "$SCRIPT_DIR/ensure_yunji_water_link.sh" \
  "$SCRIPT_DIR/prepare_yunji_odin1_calibration_driver.sh" \
  "$SCRIPT_DIR/run_yunji_mapping_observation.sh"; do
  [[ -r "$required" ]] || {
    echo "Missing calibration-observation input: $required" >&2
    exit 1
  }
done
[[ "$HUB_URL" =~ ^http://127\.0\.0\.1:[0-9]+$ ]] || {
  echo "FOCUS_HUB_BASE_URL must remain loopback-only." >&2
  exit 2
}
[[ "$READY_TIMEOUT_S" =~ ^[1-9][0-9]*$ ]] || {
  echo "FOCUS_YUNJI_CALIBRATION_READY_TIMEOUT_S must be positive." >&2
  exit 2
}

bash "$SCRIPT_DIR/ensure_yunji_water_link.sh"
bash "$SCRIPT_DIR/prepare_yunji_odin1_calibration_driver.sh"

# Stopping a live receiver may issue only its fail-closed WATER cancel. No new
# move target is created by this calibration path.
for unit in \
  focus-yunji-v2-readonly-v4.service \
  focus-yunji-v2-runtime.service \
  focus-yunji-v2-debug-v2.service \
  focus-yunji-v2-live-v2.service \
  focus-yunji-command-observation-v2.service \
  focus-yunji-command-observation.service \
  focus-yunji-odin1-calibrated-v2.service \
  "$UNIT"; do
  sudo -n systemctl stop "$unit" >/dev/null 2>&1 || true
  sudo -n systemctl reset-failed "$unit" >/dev/null 2>&1 || true
done
if pgrep -af 'v2_yunji_receiver\.py' >/dev/null 2>&1; then
  echo "An untracked Yunji v2 receiver remains active." >&2
  exit 1
fi
if pgrep -af 'keyboard.*teleop|joy_control' >/dev/null 2>&1; then
  echo "A Yunji manual command process is active." >&2
  exit 1
fi

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
metrics="/home/nyu/.local/state/topofocus/yunji-calibration-$stamp.json"
hub_latest_sequence() {
  local token payload
  token="$(
    set +u
    # shellcheck disable=SC1090
    source "$ENV_FILE"
    printf '%s' "${FOCUS_ROBOT_TOKEN:-}"
  )"
  [[ -n "$token" ]] || {
    echo "Yunji environment has no FOCUS_ROBOT_TOKEN." >&2
    return 1
  }
  payload="$(
    curl -fsS --max-time 5 -H "X-Robot-Token: $token" \
      "$HUB_URL/v1/robots/robot-1/observations/latest"
  )"
  unset token
  FOCUS_SEQUENCE_JSON="$payload" python3 -c \
    'import json,os; print(int(json.loads(os.environ["FOCUS_SEQUENCE_JSON"])["last_sequence"]))'
}

initial_sequence="$(hub_latest_sequence)"
sudo -n systemd-run \
  --unit="${UNIT%.service}" \
  --property=Type=exec \
  --uid=nyu --gid=nyu \
  --working-directory="$RELEASE_ROOT" \
  /bin/bash "$SCRIPT_DIR/run_yunji_mapping_observation.sh" \
    --transform-version "$TRANSFORM_VERSION" \
    --env "$ENV_FILE" \
    --metrics-out "$metrics" >/dev/null

deadline=$((SECONDS + READY_TIMEOUT_S))
latest_sequence="$initial_sequence"
while (( SECONDS < deadline )); do
  systemctl is-active --quiet "$UNIT" || {
    journalctl -u "$UNIT" -n 80 --no-pager >&2
    exit 1
  }
  latest_sequence="$(hub_latest_sequence 2>/dev/null || true)"
  if [[ "$latest_sequence" =~ ^-?[0-9]+$ ]] \
     && (( latest_sequence > initial_sequence )); then
    break
  fi
  sleep 1
done
[[ "$latest_sequence" =~ ^-?[0-9]+$ ]] \
  && (( latest_sequence > initial_sequence )) || {
    echo "Yunji calibration observation did not advance the Hub sequence from $initial_sequence." >&2
    journalctl -u "$UNIT" -n 80 --no-pager >&2
    sudo -n systemctl stop "$UNIT" >/dev/null 2>&1 || true
    exit 1
  }
echo "Yunji calibration observation ready: transform=$TRANSFORM_VERSION sequence=$initial_sequence->$latest_sequence"
echo "Safety: no v2 receiver or WATER move path is running."
