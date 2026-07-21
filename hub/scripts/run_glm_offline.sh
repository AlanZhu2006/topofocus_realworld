#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HUB_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WS="$(cd "$HUB_DIR/.." && pwd)"
PYTHON_BIN="${FOCUS_HUB_PYTHON:-$HUB_DIR/.venv/bin/python}"
PORT="${FOCUS_GLM_PORT:-31511}"
DEMO_DIR="$WS/source/Focus_realworld/CogVLM2/basic_demo"
RUNTIME_DIR="$HUB_DIR/runtime/glm"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Python environment missing: $PYTHON_BIN" >&2
  exit 1
fi

export HF_HOME="$WS/artifacts/models/hf_cache"
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export PYTHONPATH="$WS/source/Focus_realworld:$WS/dependencies:$WS/dependencies/habitat-lab${PYTHONPATH:+:$PYTHONPATH}"

# The recorded upstream tree is deliberately read-only.  The demo configures a
# relative server_debug.log, so run it from the ignored writable runtime area.
mkdir -p "$RUNTIME_DIR"
cd "$RUNTIME_DIR"
# The upstream entrypoint hard-codes 127.0.0.1 and accepts only --port.
exec "$PYTHON_BIN" "$DEMO_DIR/glm4_openai_api_demo_1gpu.py" --port "$PORT"
