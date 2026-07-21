#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HUB_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ENV_DIR="$HUB_DIR/.venv"
BASE_PYTHON="${FOCUS_TORCH_PYTHON:-/home/asus/miniconda3/envs/memnav/bin/python}"
UV_BIN="${FOCUS_UV_BIN:-/home/asus/miniconda3/bin/uv}"

if [[ -e "$ENV_DIR" ]]; then
  echo "Refusing to replace existing environment: $ENV_DIR" >&2
  exit 1
fi
if [[ ! -x "$BASE_PYTHON" ]]; then
  echo "Kernel-tested base Python is missing: $BASE_PYTHON" >&2
  exit 1
fi
if [[ ! -x "$UV_BIN" ]]; then
  echo "uv is missing: $UV_BIN" >&2
  exit 1
fi

"$UV_BIN" venv --python "$BASE_PYTHON" --system-site-packages "$ENV_DIR"

# --no-deps is intentional: the inherited memnav environment owns the tested
# torch/CUDA stack.  Letting a generic resolver run here can replace it with a
# much larger and driver-incompatible CUDA build.
"$UV_BIN" pip install --python "$ENV_DIR/bin/python" --no-deps \
  -e "$HUB_DIR" -r "$HUB_DIR/requirements-g1.txt" \
  pytest pluggy iniconfig pygments tomli \
  lazy-loader sniffio distro jiter typing-extensions

echo "Created $ENV_DIR"
echo "Run: $ENV_DIR/bin/python $HUB_DIR/tools/g1_preflight.py --workspace $(cd "$HUB_DIR/.." && pwd)"
