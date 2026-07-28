#!/usr/bin/env python3
"""Launch continuous TinyNav geometry/occupancy in the fresh ``world`` frame.

The installed ``semantic_mapping`` package normally targets a relocalized
saved-map frame and RealSense aligned depth. This deployment launch keeps the
package unchanged but overrides it to consume TinyNav's continuously
published, timestamp-matched geometry products:

``/slam/depth + /slam/camera_info + world->camera TF``.

The point-cloud node requires an RGB field although occupancy uses only XYZ.
A read-only adapter therefore supplies a strictly stamped black RGB companion
for each validated depth frame. No semantic inference or actuator process is
started here.
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
    geometry_rgb_script = Path(__file__).resolve().with_name(
        "ros_continuous_depth_geometry_rgb.py"
    )
    if not geometry_rgb_script.is_file():
        parser.error(
            f"continuous geometry RGB adapter is missing: "
            f"{geometry_rgb_script}"
        )
    geometry_rgb_topic = "/focus/slam/continuous_depth_geometry_rgb"

    geometry_overrides = {
        "topics.rgb": geometry_rgb_topic,
        "topics.depth": "/slam/depth",
        "topics.camera_info": "/slam/camera_info",
        "topics.pointcloud": "/semantic_mapping/semantic_pointcloud",
        "topics.camera_pose": "/semantic_mapping/camera_pose",
        "frames.target_frame": args.target_frame,
        "frames.odom_frame": args.target_frame,
        "frames.pose_camera_frame": "camera",
        "frames.tracking_camera_frame": "camera",
        "frames.camera_frame": "camera",
        "sync.queue_size": 30,
        # This is the same continuous depth/CameraInfo/pose tuple already
        # validated by the persistent WSJ observation sender.  Its measured
        # pose skew is 0 ms; retain the sender's 50 ms fail-closed sync bound.
        "sync.max_slop_sec": 0.05,
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
        # Observed during supervised Go2 motion on 2026-07-25: the
        # low-rate synchronized keyframes legitimately differed by
        # 22.24--23.24 degrees while the local controller commanded a turn.
        # The source 20-degree discontinuity threshold therefore suppressed
        # every fresh grid until the router stopped the robot.  Keep genuine
        # larger relocalization discontinuities fail-closed while admitting
        # the measured physical motion.
        "keyframe.pose_jump_rotation_deg": 35.0,
        "use_sim_time": False,
    }
    description = LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    "-u",
                    str(geometry_rgb_script),
                    "--input-topic",
                    "/slam/depth",
                    "--output-topic",
                    geometry_rgb_topic,
                    "--camera-frame",
                    "camera",
                    "--approved-size",
                    "848x480",
                    "--approved-size",
                    "640x480",
                    "--max-capture-age-s",
                    "2.0",
                ],
                name="focus_continuous_depth_geometry_rgb",
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
