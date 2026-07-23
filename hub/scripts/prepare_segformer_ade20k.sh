#!/usr/bin/env bash
# Download and verify the pinned real-camera semantic adapter. No datasets.
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
python_bin="$workspace/hub/.venv/bin/python"
output_dir="$workspace/artifacts/vision/segformer_b0_ade20k_hf"
revision="489d5cd81a0b59fab9b7ea758d3548ebe99677da"

"$python_bin" - "$output_dir" "$revision" <<'PY'
from pathlib import Path
import sys

from huggingface_hub import snapshot_download

output = Path(sys.argv[1])
revision = sys.argv[2]
snapshot_download(
    repo_id="nvidia/segformer-b0-finetuned-ade-512-512",
    revision=revision,
    local_dir=output,
    allow_patterns=[
        "config.json",
        "preprocessor_config.json",
        "model.safetensors",
    ],
)
PY

PYTHONPATH="$workspace/hub/src" "$python_bin" - "$output_dir" <<'PY'
from pathlib import Path
import json
import sys

from focus_hub.segformer_ade20k import verify_model_directory

print(json.dumps(verify_model_directory(Path(sys.argv[1])), indent=2))
PY

echo "Pinned SegFormer model ready: $output_dir"
