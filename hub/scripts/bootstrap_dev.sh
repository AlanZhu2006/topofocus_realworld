#!/usr/bin/env bash
# Build the lightweight transport/mapping test environment from hub/uv.lock.
# Model weights and the inherited CUDA/G1 environment are separate gates.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HUB_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ENV_DIR="${FOCUS_DEV_ENV:-$HUB_DIR/.venv}"
PYTHON_BIN="${FOCUS_DEV_PYTHON:-python3.10}"
run_tests=true

while [[ $# -gt 0 ]]; do
  case "$1" in
    --env-dir) ENV_DIR="$2"; shift 2 ;;
    --python) PYTHON_BIN="$2"; shift 2 ;;
    --skip-tests) run_tests=false; shift ;;
    -h|--help)
      echo "Usage: $0 [--env-dir DIR] [--python PYTHON] [--skip-tests]"
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

command -v uv >/dev/null || {
  echo "uv is required: https://docs.astral.sh/uv/" >&2
  exit 1
}
command -v "$PYTHON_BIN" >/dev/null || {
  echo "Python interpreter not found: $PYTHON_BIN" >&2
  exit 1
}
[[ -f "$HUB_DIR/uv.lock" ]] || {
  echo "Missing lockfile: $HUB_DIR/uv.lock" >&2
  exit 1
}
if [[ -e "$ENV_DIR" ]]; then
  echo "Refusing to replace existing environment: $ENV_DIR" >&2
  exit 1
fi

UV_PROJECT_ENVIRONMENT="$ENV_DIR" uv sync \
  --project "$HUB_DIR" \
  --python "$PYTHON_BIN" \
  --extra test \
  --locked

if [[ "$run_tests" == true ]]; then
  "$ENV_DIR/bin/python" -m pytest "$HUB_DIR/tests" -q
fi

echo "Created reproducible lightweight environment: $ENV_DIR"
echo "For CUDA/model gate G1, follow docs/REPRODUCE.md; it uses a separate environment."
