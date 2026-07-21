#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"
mode="online"
target_frame=""
publish_once="false"
enable_occupancy="true"
output_directory=""
input_directory=""
allow_input_frame_override="false"
enable_semantic_perception="false"
enable_semantic_fusion="false"
precomputed_mask_directory=""
precomputed_manifest="manifest.yaml"
semantic_classes_file=""
semantic_engine=""
semantic_model_config="$HOME/.cache/tinynav/semantic_models/segformer_b0_ade20k/config.json"
semantic_preprocessor_config="$HOME/.cache/tinynav/semantic_models/segformer_b0_ade20k/preprocessor_config.json"
semantic_label_mapping="$ROOT_DIR/semantic_mapping/config/ade20k_navigation_mapping.yaml"
semantic_min_confidence="0.35"
semantic_backend="precomputed"

usage() {
  cat <<EOF
Usage: $0 [--online|--offline] [--target-frame FRAME] [--once]
          [--pointcloud-only] [--output-dir DIR] [--input-dir DIR]
          [--allow-frame-override] [--semantic-masks DIR]
          [--semantic-manifest FILE] [--semantic-tensorrt ENGINE]
          [--semantic-model-config FILE] [--semantic-preprocessor FILE]
          [--semantic-label-mapping FILE] [--semantic-min-confidence VALUE]
          [--semantic-classes FILE]

Starts aligned RGB-D geometry, Phase-2 occupancy, optional Phase-3 perception,
Phase-4 semantic voxel fusion, and Phase-5 semantic BEV projection. Build first:
  cd $ROOT_DIR
  colcon build --packages-select semantic_mapping --symlink-install
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --online) mode="online"; shift ;;
    --offline) mode="offline"; shift ;;
    --target-frame) target_frame="$2"; shift 2 ;;
    --once) publish_once="true"; shift ;;
    --pointcloud-only) enable_occupancy="false"; shift ;;
    --output-dir) output_directory="$2"; shift 2 ;;
    --input-dir) input_directory="$2"; shift 2 ;;
    --allow-frame-override) allow_input_frame_override="true"; shift ;;
    --semantic-masks)
      precomputed_mask_directory="$2"
      shift 2
      ;;
    --semantic-manifest) precomputed_manifest="$2"; shift 2 ;;
    --semantic-tensorrt) semantic_engine="$2"; shift 2 ;;
    --semantic-model-config) semantic_model_config="$2"; shift 2 ;;
    --semantic-preprocessor) semantic_preprocessor_config="$2"; shift 2 ;;
    --semantic-label-mapping) semantic_label_mapping="$2"; shift 2 ;;
    --semantic-min-confidence) semantic_min_confidence="$2"; shift 2 ;;
    --semantic-classes) semantic_classes_file="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 1 ;;
  esac
done

if [[ -n "$precomputed_mask_directory" && -n "$semantic_engine" ]]; then
  echo "Choose either --semantic-masks or --semantic-tensorrt, not both." >&2
  exit 1
fi
if [[ -n "$precomputed_mask_directory" ]]; then
  enable_semantic_perception="true"
  enable_semantic_fusion="true"
  semantic_backend="precomputed"
fi
if [[ -n "$semantic_engine" ]]; then
  enable_semantic_perception="true"
  enable_semantic_fusion="true"
  semantic_backend="segformer_tensorrt"
fi
if [[ "$semantic_backend" == "precomputed" && "$enable_semantic_perception" == "true" && ! -d "$precomputed_mask_directory" ]]; then
  echo "Precomputed semantic mask directory does not exist: $precomputed_mask_directory" >&2
  exit 1
fi
if [[ "$semantic_backend" == "precomputed" && "$enable_semantic_perception" == "true" && ! -f "$precomputed_mask_directory/$precomputed_manifest" ]]; then
  echo "Precomputed semantic manifest does not exist: $precomputed_mask_directory/$precomputed_manifest" >&2
  exit 1
fi
if [[ "$semantic_backend" == "segformer_tensorrt" ]]; then
  for required_file in \
    "$semantic_engine" \
    "$semantic_model_config" \
    "$semantic_preprocessor_config" \
    "$semantic_label_mapping"; do
    if [[ ! -f "$required_file" ]]; then
      echo "SegFormer TensorRT file does not exist: $required_file" >&2
      exit 1
    fi
  done
fi
if [[ -n "$semantic_classes_file" && ! -f "$semantic_classes_file" ]]; then
  echo "Semantic class file does not exist: $semantic_classes_file" >&2
  exit 1
fi

if [[ -f "$SETUP_FILE" ]]; then
  set +u
  source "$SETUP_FILE"
  set -u
fi
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  set +u
  source "$ROOT_DIR/install/setup.bash"
  set -u
fi

if ! ros2 pkg prefix semantic_mapping >/dev/null 2>&1; then
  echo "semantic_mapping is not built in the sourced workspace." >&2
  usage >&2
  exit 1
fi

launch_file="semantic_mapping_${mode}.launch.py"
arguments=(
  "publish_once:=$publish_once"
  "enable_occupancy:=$enable_occupancy"
  "allow_input_frame_override:=$allow_input_frame_override"
  "enable_semantic_perception:=$enable_semantic_perception"
  "enable_semantic_fusion:=$enable_semantic_fusion"
)
if [[ "$enable_semantic_perception" == "true" ]]; then
  arguments+=("semantic_backend:=$semantic_backend")
  if [[ "$semantic_backend" == "precomputed" ]]; then
    arguments+=(
      "precomputed_mask_directory:=$precomputed_mask_directory"
      "precomputed_manifest:=$precomputed_manifest"
    )
  else
    arguments+=(
      "semantic_engine:=$semantic_engine"
      "semantic_model_config:=$semantic_model_config"
      "semantic_preprocessor_config:=$semantic_preprocessor_config"
      "semantic_label_mapping:=$semantic_label_mapping"
      "semantic_min_confidence:=$semantic_min_confidence"
    )
  fi
fi
if [[ -n "$semantic_classes_file" ]]; then
  arguments+=("semantic_classes_file:=$semantic_classes_file")
fi
if [[ -n "$output_directory" ]]; then
  arguments+=("output_directory:=$output_directory")
fi
if [[ -n "$input_directory" ]]; then
  arguments+=("input_directory:=$input_directory")
fi
if [[ -n "$target_frame" ]]; then
  arguments+=("target_frame:=$target_frame")
fi
exec ros2 launch semantic_mapping "$launch_file" "${arguments[@]}"
