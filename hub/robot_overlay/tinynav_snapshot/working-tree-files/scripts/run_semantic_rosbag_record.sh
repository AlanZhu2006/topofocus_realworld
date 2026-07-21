#!/bin/bash
set -euo pipefail

# Usage: run_semantic_rosbag_record.sh [--output DIR] [--minimal]
# --minimal records only the posed aligned RGB-D inputs needed by phase 1.

SETUP_FILE="${TINYNAV_SETUP:-/home/nvidia/twork/tinynav_setup.bash}"

source_setup() {
    local had_nounset=0
    case $- in
        *u*) had_nounset=1; set +u ;;
    esac
    source "$SETUP_FILE"
    if [[ "$had_nounset" == "1" ]]; then
        set -u
    fi
}

if [[ -f "$SETUP_FILE" ]]; then
    source_setup
fi

output_dir=""
minimal="false"

usage() {
    cat <<EOF
Usage: $0 [--output DIR] [--minimal]

Without --minimal, records raw TinyNav mapping sensors plus aligned RGB-D and
any live TinyNav pose/TF topics. With --minimal, records only aligned RGB-D,
CameraInfo, pose/TF, and static TF for deterministic phase-1 replay.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output|-o) output_dir="$2"; shift 2 ;;
        --minimal) minimal="true"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 1 ;;
    esac
done

if [ -z "$output_dir" ]; then
    xdg_data_home="${XDG_DATA_HOME:-$HOME/.local/share}"
    record_root="${xdg_data_home}/tinynav/rosbags"
    timestamp="$(date +%Y%m%d_%H%M%S)"
    output_dir="${record_root}/semantic_map_record_${timestamp}"
    mkdir -p "${record_root}"
else
    mkdir -p "$(dirname "$output_dir")"
fi

minimal_topics=(
    /camera/camera/color/image_raw
    /camera/camera/color/camera_info
    /camera/camera/aligned_depth_to_color/image_raw
    /camera/camera/aligned_depth_to_color/camera_info
    /slam/odometry_visual
    /slam/odometry
    /slam/keyframe_odom
    /tf
    /tf_static
)

if [[ "$minimal" == "true" ]]; then
    topics=("${minimal_topics[@]}")
    max_cache_size=536870912
else
    topics=(
        /camera/camera/infra1/camera_info
        /camera/camera/infra1/image_rect_raw
        /camera/camera/infra1/metadata
        /camera/camera/infra2/camera_info
        /camera/camera/infra2/image_rect_raw
        /camera/camera/infra2/metadata
        /camera/camera/depth/image_rect_raw
        /camera/camera/extrinsics/depth_to_infra1
        /camera/camera/extrinsics/depth_to_infra2
        /camera/camera/imu
        /camera/camera/color/image_rect_raw/compressed
        /insight/vio_20hz
        "${minimal_topics[@]}"
    )
    max_cache_size=2147483648
fi

ros2 bag record \
    --output "$output_dir" \
    --max-cache-size "$max_cache_size" \
    "${topics[@]}"
