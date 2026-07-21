#!/bin/bash
set -euo pipefail

MIN_FW_VERSION="5.17"

if ! command -v rs-enumerate-devices >/dev/null 2>&1; then
	echo "rs-enumerate-devices is not installed; cannot check RealSense firmware."
	exit 1
fi

RS_ENUM_OUTPUT="$(rs-enumerate-devices 2>&1)" || {
	echo "Could not enumerate RealSense devices."
	echo "$RS_ENUM_OUTPUT"
	exit 1
}

FW_VERSION="$(awk -F: '/Firmware Version/ {gsub(/^[ \t]+|[ \t]+$/, "", $2); print $2; exit}' <<<"$RS_ENUM_OUTPUT")"
if [[ -z "$FW_VERSION" ]]; then
	echo "Could not detect RealSense firmware version."
	echo "$RS_ENUM_OUTPUT"
	exit 1
fi

if [[ "$(printf '%s\n%s\n' "$MIN_FW_VERSION" "$FW_VERSION" | sort -V | head -n1)" != "$MIN_FW_VERSION" ]]; then
	echo "RealSense firmware $FW_VERSION is too old; expected at least $MIN_FW_VERSION."
	exit 1
fi

echo "RealSense firmware $FW_VERSION detected."

ros2 launch realsense2_camera rs_launch.py \
	initial_reset:=true \
	tf_publish_rate:=1.0 \
	publish_tf:=true \
	enable_depth:=true \
	enable_color:=true \
	enable_infra1:=true \
	enable_infra2:=true \
	enable_gyro:=true \
	enable_accel:=true \
	enable_sync:=true \
	align_depth.enable:=true \
	depth_module.depth_profile:=848x480x30 \
	depth_module.infra_profile:=848x480x30 \
	rgb_camera.color_profile:=848x480x30 \
	unite_imu_method:=2
