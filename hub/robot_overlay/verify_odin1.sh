#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ODIN_WS="${ODIN_WS:-/home/nyu/odin_ws}"
DRIVER_ROOT="${ODIN_DRIVER_ROOT:-${ODIN_WS}/src/odin_ros_driver}"
EXPECTED_COMMIT="13aa528b1da581e2168ac858f8b144f0b4438a7a"
PATCH_FILE="${SCRIPT_DIR}/odin1_snapshot/odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch"
CALIBRATION_FILE="${ODIN_CALIBRATION_FILE:-${ODIN_WS}/calibration/O1-P070100205.calib.yaml}"
HARDWARE=0

if [[ "${1:-}" == "--hardware" ]]; then
  HARDWARE=1
elif [[ $# -ne 0 ]]; then
  echo "usage: $0 [--hardware]" >&2
  exit 2
fi

require_file() {
  if [[ ! -r "$1" ]]; then
    echo "missing required file: $1" >&2
    exit 1
  fi
}

require_hash() {
  local path="$1"
  local expected="$2"
  local actual
  actual="$(sha256sum "${path}" | cut -d' ' -f1)"
  if [[ "${actual}" != "${expected}" ]]; then
    echo "SHA-256 mismatch: ${path}" >&2
    echo "expected ${expected}" >&2
    echo "actual   ${actual}" >&2
    exit 1
  fi
}

require_file "${PATCH_FILE}"
require_file "${DRIVER_ROOT}/config/control_command.yaml"
require_file "${DRIVER_ROOT}/src/host_sdk_sample.cpp"
require_file "${DRIVER_ROOT}/src/yaml_parser.cpp"
require_file "${CALIBRATION_FILE}"
require_file "${ODIN_WS}/install/setup.bash"

actual_commit="$(git -C "${DRIVER_ROOT}" rev-parse HEAD)"
if [[ "${actual_commit}" != "${EXPECTED_COMMIT}" ]]; then
  echo "Odin driver commit mismatch: ${actual_commit}" >&2
  exit 1
fi
if ! git -C "${DRIVER_ROOT}" apply --reverse --check "${PATCH_FILE}"; then
  echo "the tracked firmware-0.13.1 compatibility patch is not exactly applied" >&2
  exit 1
fi

require_hash "${PATCH_FILE}" "2a73aa48d163e2a362670b7b9b778edf8328aba7323e1cc04dd6b8fb28ba5806"
require_hash "${CALIBRATION_FILE}" "c8cbd48bd8f8b08b8f174f557faf48649ee1101a3dfe0daf82ceae3832d7c23d"
require_hash "${DRIVER_ROOT}/config/control_command.yaml" "c9a0c3466d8526cc290ddd24a31dd8670bb988b8e8a9e1356c625da0dc8ac5ef"
require_hash "${DRIVER_ROOT}/src/host_sdk_sample.cpp" "edddec679c13f0e7af3940238faf227aa6282a8e14797f4f0d2899f00110ac85"
require_hash "${DRIVER_ROOT}/src/yaml_parser.cpp" "826594ab4397e223b6ed0b05e0a585538bea19155902f2a609741ce349f08024"

if [[ ${HARDWARE} -eq 1 ]]; then
  if ! lsusb -d 2207:0019 >/dev/null; then
    echo "Odin1 USB device 2207:0019 is not connected" >&2
    exit 1
  fi
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1090
  source "${ODIN_WS}/install/setup.bash"
  set -u
  for topic in /odin1/image /odin1/cloud_slam /odin1/odometry; do
    if ! timeout 8 ros2 topic echo --once "${topic}" >/dev/null; then
      echo "no live message on ${topic}" >&2
      exit 1
    fi
  done
fi

echo "Odin1 verification passed (hardware=${HARDWARE}, commit=${EXPECTED_COMMIT})."
echo "This check is read-only and issued no robot command."
