#!/usr/bin/env bash
set -euo pipefail

MODEL_REPO="Xenova/segformer-b0-finetuned-ade-512-512"
MODEL_REVISION="d3e5499fa8701ff0453ca940a8dfeae39b2f1504"
MODEL_SHA256="3e5c18a4be395f16646438d54c42377ddc202edfa33d5eced0c9506de75c44c2"
CONFIG_SHA256="435799652b2b64c3e422dea20fed4c59651dae9f0e291fd885e9e067fee0ce2a"
PREPROCESSOR_SHA256="dbabd93c735c8a5c39ef207c6c4459bf2d261a5dcc55e1ba1c1b982e5947f518"

cache_dir="${TINYNAV_SEMANTIC_MODEL_DIR:-$HOME/.cache/tinynav/semantic_models/segformer_b0_ade20k}"
trtexec_path="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
force_engine="false"
download_only="false"

usage() {
  cat <<EOF
Usage: $0 [--cache-dir DIR] [--trtexec PATH] [--force-engine] [--download-only]

Downloads a pinned SegFormer-B0 ADE20K ONNX export, verifies SHA256 hashes,
and builds a local TensorRT FP16 engine for 1x3x512x512 input.

Default cache:
  $cache_dir
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cache-dir) cache_dir="$2"; shift 2 ;;
    --trtexec) trtexec_path="$2"; shift 2 ;;
    --force-engine) force_engine="true"; shift ;;
    --download-only) download_only="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 1 ;;
  esac
done

mkdir -p "$cache_dir"

download_verified() {
  local relative_path="$1"
  local output_path="$2"
  local expected_sha="$3"
  local current_sha=""
  local temporary_path="${output_path}.part"
  local url="https://huggingface.co/${MODEL_REPO}/resolve/${MODEL_REVISION}/${relative_path}"

  if [[ -f "$output_path" ]]; then
    current_sha="$(sha256sum "$output_path" | awk '{print $1}')"
  fi
  if [[ "$current_sha" == "$expected_sha" ]]; then
    echo "Verified existing: $output_path"
    return 0
  fi

  echo "Downloading: $url"
  curl -L --fail --retry 3 -o "$temporary_path" "$url"
  current_sha="$(sha256sum "$temporary_path" | awk '{print $1}')"
  if [[ "$current_sha" != "$expected_sha" ]]; then
    rm -f "$temporary_path"
    echo "SHA256 mismatch for $relative_path" >&2
    echo "  expected: $expected_sha" >&2
    echo "  received: $current_sha" >&2
    exit 1
  fi
  mv "$temporary_path" "$output_path"
}

onnx_path="$cache_dir/model.onnx"
config_path="$cache_dir/config.json"
preprocessor_path="$cache_dir/preprocessor_config.json"
engine_path="$cache_dir/model_fp16.engine"

download_verified "onnx/model.onnx" "$onnx_path" "$MODEL_SHA256"
download_verified "config.json" "$config_path" "$CONFIG_SHA256"
download_verified "preprocessor_config.json" "$preprocessor_path" "$PREPROCESSOR_SHA256"

if [[ "$download_only" == "true" ]]; then
  echo "Download and verification complete: $cache_dir"
  exit 0
fi
if [[ ! -x "$trtexec_path" ]]; then
  echo "TensorRT trtexec is not executable: $trtexec_path" >&2
  exit 1
fi
if [[ -f "$engine_path" && "$force_engine" != "true" ]]; then
  echo "Using existing TensorRT engine: $engine_path"
  echo "Use --force-engine after changing TensorRT, CUDA, or Jetson hardware."
  exit 0
fi

echo "Building TensorRT FP16 engine. Initial tactic selection can take several minutes."
"$trtexec_path" \
  --onnx="$onnx_path" \
  --saveEngine="$engine_path" \
  --fp16 \
  --minShapes=pixel_values:1x3x512x512 \
  --optShapes=pixel_values:1x3x512x512 \
  --maxShapes=pixel_values:1x3x512x512 \
  --skipInference

echo "TensorRT engine ready: $engine_path"
