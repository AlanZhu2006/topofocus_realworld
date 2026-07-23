#!/usr/bin/env bash
# Read-only Go2/Jetson preflight. --hardware adds checks for an attached D435i.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_COMMIT="576c082e69580f618a5ff313a3e74f3672abb69f"
LIVE_IMU_RECOVERY_COMMIT="29f26bc058886ff450f02cdc0d6e9977e1c57010"
LIVE_IMU_PERCEPTION_SHA256="3a695d5210d60ea1f721549ca7458ba89e7bf32db5178cd1c312c633aef1c3b3"
TINYNAV_ROOT="${TINYNAV_PATCHED_ROOT:-/home/nvidia/twork/tinynav-topofocus}"
TINYNAV_SETUP="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
hardware=false
run_tests=false
failures=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tinynav-root) TINYNAV_ROOT="$2"; shift 2 ;;
    --setup) TINYNAV_SETUP="$2"; shift 2 ;;
    --hardware) hardware=true; shift ;;
    --tests) run_tests=true; shift ;;
    -h|--help)
      echo "Usage: $0 [--tinynav-root DIR] [--setup FILE] [--hardware] [--tests]"
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

pass() { echo "PASS  $*"; }
warn() { echo "WARN  $*"; }
fail() { echo "FAIL  $*" >&2; failures=$((failures + 1)); }

[[ "$(uname -m)" == "aarch64" ]] && pass "architecture aarch64" || warn "architecture is $(uname -m), expected aarch64 on Go2 Jetson"
grep -q 'Ubuntu 22.04' /etc/os-release 2>/dev/null && pass "Ubuntu 22.04" || warn "verified baseline is Ubuntu 22.04"
[[ -f "$TINYNAV_SETUP" ]] && pass "setup file $TINYNAV_SETUP" || fail "missing setup file $TINYNAV_SETUP"

if git -C "$TINYNAV_ROOT" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  if git -C "$TINYNAV_ROOT" merge-base --is-ancestor "$BASE_COMMIT" HEAD 2>/dev/null; then
    pass "TinyNav contains pinned base $BASE_COMMIT"
  else
    fail "TinyNav does not contain pinned base $BASE_COMMIT"
  fi
  if git -C "$TINYNAV_ROOT" apply --reverse --check "$SCRIPT_DIR/tinynav_snapshot/tinynav-required.patch" 2>/dev/null; then
    pass "verified WSJ patch is applied"
  elif [[ "$(git -C "$TINYNAV_ROOT" rev-parse HEAD 2>/dev/null)" == "$LIVE_IMU_RECOVERY_COMMIT" ]] \
      && [[ "$(sha256sum "$TINYNAV_ROOT/tinynav/core/perception_node.py" | awk '{print $1}')" == "$LIVE_IMU_PERCEPTION_SHA256" ]] \
      && [[ -z "$(git -C "$TINYNAV_ROOT" status --porcelain)" ]]; then
    pass "verified live-tested IMU recovery commit and perception hash"
  else
    fail "neither the repository patch nor the live-tested IMU recovery tree matches"
  fi
  git -C "$TINYNAV_ROOT" diff --check >/dev/null && pass "TinyNav diff whitespace check" || fail "TinyNav diff check"
else
  fail "missing TinyNav Git checkout $TINYNAV_ROOT"
fi

python_bin="${TINYNAV_PYTHON:-$TINYNAV_ROOT/.venv/bin/python}"
if [[ -x "$python_bin" ]]; then
  python_version="$($python_bin -c 'import sys; print(".".join(map(str, sys.version_info[:2])))' 2>/dev/null)"
  [[ "$python_version" == "3.10" ]] && pass "TinyNav Python 3.10" || fail "TinyNav requires Python 3.10, found $python_version"
else
  fail "missing TinyNav Python $python_bin"
fi

[[ -r /sys/module/usbcore/parameters/usbfs_memory_mb ]] && {
  usbfs_mb="$(</sys/module/usbcore/parameters/usbfs_memory_mb)"
  (( usbfs_mb >= 1000 )) && pass "usbfs_memory_mb=$usbfs_mb" || fail "usbfs_memory_mb=$usbfs_mb; expected >=1000"
}

if pgrep -af 'go2_cmd_bridge|cmd_vel_control|planning_node.py|nav2_controller' >/dev/null 2>&1; then
  fail "an actuation/planning process is already running"
else
  pass "no known TopoFocus actuation/planning process"
fi

if [[ "$hardware" == true ]]; then
  camera_seen=false
  for device in /sys/bus/usb/devices/*; do
    [[ -r "$device/idVendor" && -r "$device/idProduct" ]] || continue
    pair="$(<"$device/idVendor"):$(<"$device/idProduct")"
    case "$pair" in
      8086:0b3a)
        camera_seen=true
        [[ "$(<"$device/power/control")" == "on" ]] && pass "$pair power/control=on" || fail "$pair power/control is not on"
        ;;
      05e3:0625)
        [[ "$(<"$device/power/control")" == "on" ]] && pass "$pair power/control=on" || fail "$pair power/control is not on"
        ;;
    esac
  done
  [[ "$camera_seen" == true ]] || fail "D435i USB id 8086:0b3a not found"
fi

if [[ "$run_tests" == true && -x "$python_bin" ]]; then
  "$python_bin" -m pytest "$TINYNAV_ROOT/tests/test_perception_health.py" -q \
    && pass "perception health tests" || fail "perception health tests"
fi

if (( failures > 0 )); then
  echo "$failures required check(s) failed." >&2
  exit 1
fi
echo "All required checks passed. No physical command was sent."
