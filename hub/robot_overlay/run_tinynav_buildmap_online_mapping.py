#!/usr/bin/env python3
"""Launch TinyNav stereo geometry/occupancy directly in the fresh ``world`` frame.

The installed ``semantic_mapping`` package normally targets a relocalized
saved-map frame and RealSense aligned depth. This deployment launch keeps the
package unchanged but overrides it to consume TinyNav's timestamp-matched
stereo products:

``/slam/keyframe_image + /slam/keyframe_depth + /slam/camera_info``.

No semantic inference or actuator process is started here.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-frame", default="world")
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--max-rate-hz", type=float, default=3.0)
    parser.add_argument("--depth-stride", type=int, default=3)
    args = parser.parse_args()
    if not args.target_frame:
        parser.error("--target-frame is required")
    if args.max_rate_hz <= 0:
        parser.error("--max-rate-hz must be positive")
    if args.depth_stride <= 0:
        parser.error("--depth-stride must be positive")

    from ament_index_python.packages import get_package_share_directory
    from launch.actions import ExecuteProcess
    from launch import LaunchDescription, LaunchService
    from launch_ros.actions import Node

    share = Path(get_package_share_directory("semantic_mapping"))
    default_config = share / "config" / "semantic_mapping.yaml"
    if not default_config.is_file():
        parser.error(f"semantic mapping config is missing: {default_config}")
    output = args.output_directory.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    alias_script = Path(__file__).resolve().with_name("ros_image_frame_alias.py")
    if not alias_script.is_file():
        parser.error(f"RGB frame-alias bridge is missing: {alias_script}")
    normalized_rgb_topic = "/focus/slam/keyframe_image_camera_alias"

    geometry_overrides = {
        "topics.rgb": normalized_rgb_topic,
        "topics.depth": "/slam/keyframe_depth",
        "topics.camera_info": "/slam/camera_info",
        "topics.pointcloud": "/semantic_mapping/semantic_pointcloud",
        "topics.camera_pose": "/semantic_mapping/camera_pose",
        "frames.target_frame": args.target_frame,
        "frames.odom_frame": args.target_frame,
        "frames.pose_camera_frame": "camera",
        "frames.tracking_camera_frame": "camera",
        "frames.camera_frame": "camera",
        "sync.queue_size": 30,
        "sync.max_slop_sec": 0.005,
        "pose.allow_latest_map_alignment": False,
        "pose.wait_for_target_alignment": False,
        "processing.max_rate_hz": args.max_rate_hz,
        "depth.stride": args.depth_stride,
        "validation.require_frame_ids": True,
        "use_sim_time": False,
    }
    occupancy_overrides = {
        "frames.target_frame": args.target_frame,
        "topics.pointcloud_input": "/semantic_mapping/semantic_pointcloud",
        "topics.camera_pose": "/semantic_mapping/camera_pose",
        "topics.occupancy_bev": "/semantic_mapping/occupancy_bev",
        "output.directory": str(output),
        "input.directory": "",
        "input.allow_frame_id_override": False,
        "use_sim_time": False,
    }
    description = LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    "-u",
                    str(alias_script),
                    "--input-topic",
                    "/slam/keyframe_image",
                    "--output-topic",
                    normalized_rgb_topic,
                    "--source-frame",
                    "camera_infra1_optical_frame",
                    "--target-frame",
                    "camera",
                    "--width",
                    "848",
                    "--height",
                    "480",
                    "--encoding",
                    "mono8",
                ],
                name="focus_image_frame_alias",
                output="screen",
            ),
            Node(
                package="semantic_mapping",
                executable="semantic_pointcloud_node",
                name="semantic_pointcloud_node",
                output="screen",
                parameters=[str(default_config), geometry_overrides],
            ),
            Node(
                package="semantic_mapping",
                executable="occupancy_mapper_node",
                name="occupancy_mapper_node",
                output="screen",
                parameters=[str(default_config), occupancy_overrides],
            ),
        ]
    )
    service = LaunchService()
    service.include_launch_description(description)
    return int(service.run())


if __name__ == "__main__":
    raise SystemExit(main())
