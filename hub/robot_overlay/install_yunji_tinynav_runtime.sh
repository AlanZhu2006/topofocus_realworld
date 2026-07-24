#!/usr/bin/env bash
# Install the minimal pinned TinyNav planner/controller runtime on Yunji.
set -euo pipefail

REPOSITORY_URL="${FOCUS_YUNJI_TINYNAV_REPOSITORY:-git@github.com:AlanZhu2006/go2_tinynav.git}"
PINNED_COMMIT="${FOCUS_YUNJI_TINYNAV_COMMIT:-5705bb61dafb407594970ab2bc85c63fc71e0a24}"
RUNTIME_ROOT="${FOCUS_YUNJI_TINYNAV_RUNTIME:-/home/nyu/.local/share/topofocus/tinynav-runtime}"
SOURCE_ROOT="$RUNTIME_ROOT/source"
VENV_ROOT="$RUNTIME_ROOT/venv"
PROVENANCE="$RUNTIME_ROOT/provenance.json"

[[ "$RUNTIME_ROOT" == /home/nyu/.local/share/topofocus/* ]] || {
  echo "Runtime root must stay under /home/nyu/.local/share/topofocus." >&2
  exit 2
}
[[ "$PINNED_COMMIT" =~ ^[0-9a-f]{40}$ ]] || {
  echo "TinyNav commit must be a full SHA-1." >&2
  exit 2
}

mkdir -p "$RUNTIME_ROOT"
if [[ ! -d "$SOURCE_ROOT/.git" ]]; then
  temporary="$(mktemp -d "$RUNTIME_ROOT/.install.XXXXXX")"
  cleanup() {
    [[ -n "${temporary:-}" && -d "$temporary" ]] && rm -rf -- "$temporary"
  }
  trap cleanup EXIT
  git clone --filter=blob:none --no-checkout "$REPOSITORY_URL" "$temporary/source"
  git -C "$temporary/source" checkout --detach "$PINNED_COMMIT"
  mv "$temporary/source" "$SOURCE_ROOT"
  rmdir "$temporary"
  temporary=""
  trap - EXIT
fi

actual_commit="$(git -C "$SOURCE_ROOT" rev-parse HEAD)"
[[ "$actual_commit" == "$PINNED_COMMIT" ]] || {
  echo "Existing TinyNav runtime commit mismatch: $actual_commit" >&2
  exit 1
}
[[ -z "$(git -C "$SOURCE_ROOT" status --porcelain)" ]] || {
  echo "Existing TinyNav runtime source is dirty." >&2
  exit 1
}

declare -A expected_hashes=(
  ["tinynav/core/planning_node.py"]="1d78d6204508a3cec880eb6899980fc77850fc5b262bf1266f0e15ba43c7dc0e"
  ["tinynav/platforms/cmd_vel_control.py"]="40519ebb1c9845e0a112f55f0a1ef5790280153ebaf198ff5122103e1372c50b"
  ["tinynav/core/math_utils.py"]="067bcc799b35d68850c4c90d54d579935fe9b7fffe84ea29865b33a9d825c787"
)
for relative_path in "${!expected_hashes[@]}"; do
  actual_hash="$(sha256sum "$SOURCE_ROOT/$relative_path" | awk '{print $1}')"
  [[ "$actual_hash" == "${expected_hashes[$relative_path]}" ]] || {
    echo "TinyNav source checksum mismatch: $relative_path $actual_hash" >&2
    exit 1
  }
done

if [[ ! -x "$VENV_ROOT/bin/python" ]]; then
  python3 -m venv --system-site-packages "$VENV_ROOT"
fi
"$VENV_ROOT/bin/python" -m pip install \
  --disable-pip-version-check \
  --no-input \
  "numpy==1.26.1" \
  "scipy==1.15.3" \
  "numba==0.61.2" \
  "codetiming==1.4.0" \
  "fufpy==0.1.1"

# ROS Python packages live under the Humble prefix rather than the normal
# distro site-packages directory. Source that prefix before validating the
# system-site-packages venv, exactly as the component runner does.
had_nounset=0
case $- in *u*) had_nounset=1; set +u ;; esac
unset COLCON_CURRENT_PREFIX AMENT_CURRENT_PREFIX
source /opt/ros/humble/setup.bash
[[ "$had_nounset" == 1 ]] && set -u
PYTHONPATH="$SOURCE_ROOT${PYTHONPATH:+:$PYTHONPATH}" \
  "$VENV_ROOT/bin/python" -c \
    "import cv_bridge, message_filters, numba, scipy; from tinynav.core import planning_node; from tinynav.platforms import cmd_vel_control"

python3 - "$PROVENANCE" "$REPOSITORY_URL" "$PINNED_COMMIT" "$SOURCE_ROOT" <<'PY'
import hashlib
import json
from pathlib import Path
import sys
import time

output, repository, commit, source_root = sys.argv[1:]
root = Path(source_root)
files = []
for relative in (
    "tinynav/core/planning_node.py",
    "tinynav/platforms/cmd_vel_control.py",
    "tinynav/core/math_utils.py",
):
    path = root / relative
    payload = path.read_bytes()
    files.append(
        {
            "source_path": str(path),
            "relative_path": relative,
            "size_bytes": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(),
            "status": "observed_on_install",
        }
    )
artifact = {
    "schema_version": "focus-yunji-tinynav-runtime-v1",
    "created_at_ns": time.time_ns(),
    "repository": repository,
    "commit": commit,
    "files": files,
    "scope": "planner_controller_only_no_models_no_simulator_data",
}
Path(output).write_text(
    json.dumps(artifact, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
PY

echo "Yunji TinyNav runtime ready: $SOURCE_ROOT@$PINNED_COMMIT"
echo "Provenance: $PROVENANCE"
