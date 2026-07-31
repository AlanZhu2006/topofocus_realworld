#!/usr/bin/env bash
# Create the pinned RTX 4090 Hub environment from a clean Ubuntu 22.04 host.
# Default mode prints a plan. --apply installs software and validates models;
# it never starts the Hub, publishes a target, or contacts a robot.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HUB_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$(cd "$HUB_DIR/.." && pwd)"
GPU_PROJECT="$HUB_DIR/gpu_runtime"
ENV_DIR="${FOCUS_HUB_ENV_DIR:-$HUB_DIR/.venv}"
TOOLS_ROOT="${FOCUS_CLEANROOM_TOOLS_ROOT:-${XDG_DATA_HOME:-$HOME/.local/share}/topofocus/tools}"
STATE_ROOT="${FOCUS_HUB_STATE_ROOT:-${XDG_STATE_HOME:-$HOME/.local/state}/topofocus}"
UV_VERSION=0.11.2
PYTHON_VERSION=3.10.20
apply=false
fetch_models=false
accept_model_licenses=false

usage() {
  cat <<'EOF'
Usage:
  bash hub/scripts/bootstrap_gpu_hub_cleanroom.sh [options]

Options:
  --apply                  create the environment and run all gates
  --env-dir DIR            environment path (default: hub/.venv)
  --state-root DIR          provenance directory
  --fetch-models           fetch checksum-pinned real-world model artifacts
  --accept-model-licenses  confirm upstream model licenses were reviewed

Without --apply this is a read-only plan. The script never starts the Hub,
publishes a target, opens a robot connection, or downloads simulator data.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --apply) apply=true; shift ;;
    --env-dir) ENV_DIR="$2"; shift 2 ;;
    --state-root) STATE_ROOT="$2"; shift 2 ;;
    --fetch-models) fetch_models=true; shift ;;
    --accept-model-licenses) accept_model_licenses=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ "$ENV_DIR" == /* && "$ENV_DIR" != / ]] || {
  echo "--env-dir must be an absolute, non-root path." >&2
  exit 2
}
[[ "$STATE_ROOT" == /* && "$STATE_ROOT" != / ]] || {
  echo "--state-root must be an absolute, non-root path." >&2
  exit 2
}
[[ -f "$GPU_PROJECT/uv.lock" ]] || {
  echo "Missing GPU runtime lock: $GPU_PROJECT/uv.lock" >&2
  exit 1
}

cat <<EOF
GPU Hub clean-room plan
  workspace:          $WORKSPACE
  environment:        $ENV_DIR
  dependency project: $GPU_PROJECT
  uv:                 $UV_VERSION
  Python:             $PYTHON_VERSION
  model fetch:        $fetch_models
  Detectron2 build:   pinned
  provenance:         $STATE_ROOT

Phases:
  1. validate Ubuntu 22.04, x86_64, NVIDIA driver and RTX 4090
  2. install build prerequisites
  3. create the hash-locked Python $PYTHON_VERSION/CUDA 12.8 runtime
  4. install this repository's Hub package without changing source/
  5. optionally fetch only checksum-pinned model artifacts
  6. fully verify all external models once
  7. build pinned Detectron2 and run G0/G1/source-semantic gates
  8. write repository, host, GPU and environment provenance

No Hub service, robot connection, target, or physical command is started.
EOF

if [[ "$apply" != true ]]; then
  "$ENV_DIR/bin/python" "$HUB_DIR/tools/fetch_cleanroom_models.py" \
    --workspace "$WORKSPACE" 2>/dev/null || \
    python3 "$HUB_DIR/tools/fetch_cleanroom_models.py" \
      --workspace "$WORKSPACE"
  echo "PLAN_ONLY=true"
  exit 0
fi

[[ "$(uname -m)" == x86_64 ]] || {
  echo "The reference GPU Hub requires x86_64." >&2
  exit 1
}
grep -q 'Ubuntu 22.04' /etc/os-release || {
  echo "The reference GPU Hub requires Ubuntu 22.04." >&2
  exit 1
}
command -v nvidia-smi >/dev/null || {
  echo "nvidia-smi is missing; install the NVIDIA driver first." >&2
  exit 1
}
gpu_name="$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)"
[[ "$gpu_name" == *"RTX 4090"* ]] || {
  echo "Expected an RTX 4090; found $gpu_name." >&2
  exit 1
}

if pgrep -af \
  'realworld_oneclick|v2_source_episode|v2_wsj_receiver|v2_yunji_receiver' \
  >/dev/null 2>&1; then
  echo "Refusing environment installation while a real-world episode exists." >&2
  exit 1
fi
[[ -z "$(git -C "$WORKSPACE" status --porcelain)" ]] || {
  echo "Clean-room installation requires a clean repository checkout." >&2
  exit 1
}

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential ca-certificates curl git libgl1 libglib2.0-0 \
  ninja-build python3.10 python3.10-dev python3.10-venv xz-utils

uv_env="$TOOLS_ROOT/uv-$UV_VERSION"
if [[ ! -x "$uv_env/bin/uv" ]]; then
  mkdir -p "$TOOLS_ROOT"
  python3.10 -m venv "$uv_env"
  "$uv_env/bin/python" -m pip install "uv==$UV_VERSION"
fi
uv_bin="$uv_env/bin/uv"
[[ "$("$uv_bin" --version)" == "uv $UV_VERSION" ]] || {
  echo "uv version mismatch: $("$uv_bin" --version)" >&2
  exit 1
}
python_install_root="$TOOLS_ROOT/python"
UV_PYTHON_INSTALL_DIR="$python_install_root" \
  "$uv_bin" python install "$PYTHON_VERSION"

if [[ -e "$ENV_DIR" ]]; then
  echo "Refusing to replace existing environment: $ENV_DIR" >&2
  exit 1
fi
UV_PROJECT_ENVIRONMENT="$ENV_DIR" \
UV_PYTHON_INSTALL_DIR="$python_install_root" \
  "$uv_bin" sync --project "$GPU_PROJECT" --locked \
    --python "$PYTHON_VERSION" --managed-python
[[ "$("$ENV_DIR/bin/python" -c 'import platform; print(platform.python_version())')" == "$PYTHON_VERSION" ]] || {
  echo "Hub Python version mismatch after sync." >&2
  exit 1
}
"$uv_bin" pip install --python "$ENV_DIR/bin/python" --no-deps -e "$HUB_DIR"

mkdir -p "$STATE_ROOT"
model_provenance="$STATE_ROOT/gpu-cleanroom-models.json"
if [[ "$fetch_models" == true ]]; then
  [[ "$accept_model_licenses" == true ]] || {
    echo "--fetch-models requires --accept-model-licenses." >&2
    exit 2
  }
  "$ENV_DIR/bin/python" "$HUB_DIR/tools/fetch_cleanroom_models.py" \
    --workspace "$WORKSPACE" --apply --accept-model-licenses \
    --write-provenance "$model_provenance"
else
  "$ENV_DIR/bin/python" "$HUB_DIR/tools/fetch_cleanroom_models.py" \
    --workspace "$WORKSPACE" --verify \
    --write-provenance "$model_provenance"
fi

FOCUS_HUB_PYTHON="$ENV_DIR/bin/python" \
FOCUS_UV_BIN="$uv_bin" \
  bash "$HUB_DIR/scripts/install_source_semantic_stack.sh"

"$ENV_DIR/bin/python" "$HUB_DIR/tools/g0_audit.py" \
  --workspace "$WORKSPACE"
"$ENV_DIR/bin/python" "$HUB_DIR/tools/g1_preflight.py" \
  --workspace "$WORKSPACE"
"$ENV_DIR/bin/python" "$HUB_DIR/tools/verify_source_semantic_stack.py" \
  --workspace "$WORKSPACE" \
  --output "$STATE_ROOT/gpu-source-semantic.json"
"$ENV_DIR/bin/python" -m pytest "$HUB_DIR/tests" -q
"$ENV_DIR/bin/python" "$HUB_DIR/tools/verify_gpu_cleanroom.py" \
  --workspace "$WORKSPACE" \
  --env-dir "$ENV_DIR" \
  --uv "$uv_bin" \
  --model-provenance "$model_provenance" \
  --write-provenance "$STATE_ROOT/gpu-cleanroom-provenance.json"

echo "GPU_HUB_CLEANROOM_INSTALL_COMPLETE=true"
echo "Provenance: $STATE_ROOT/gpu-cleanroom-provenance.json"
echo "No Hub service or physical command was started."
