#!/usr/bin/env python3
"""Publish Odin1 geometry under the ROS contract consumed by TinyNav.

Odin remains the only localization/depth source.  This adapter does not
estimate another pose and never emits a motion command.  It synchronizes the
driver's RGB, SLAM cloud and odometry using the existing calibrated Odin
projector, then publishes:

* calibrated pinhole depth and camera information for TinyNav's local planner;
* camera odometry in the online ``world`` frame;
* the original SLAM cloud relabelled as ``world`` plus its synchronized camera
  pose for the online occupancy mapper.

The deployed Odin ``odom`` frame is intentionally treated as the fresh
session-local TinyNav ``world`` frame.  Cross-robot alignment remains a
separate measured Hub calibration.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
from typing import Any

import numpy as np

from odin1_sender import (
    DEFAULT_SYNC_SLOP_S,
    OdinProjector,
    OdinRos2Source,
    load_odin_calibration,
)


CAMERA_FRAME = "odin1_camera_optical_frame"
WORLD_FRAME = "world"


def matrix_to_quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    """Return a normalized ROS-order quaternion for a rigid rotation matrix."""

    value = np.asarray(matrix, dtype=np.float64)
    if value.shape == (4, 4):
        value = value[:3, :3]
    if value.shape != (3, 3) or not np.all(np.isfinite(value)):
        raise ValueError("rotation must be a finite 3x3 or 4x4 matrix")
    if not np.allclose(value.T @ value, np.eye(3), atol=2e-5):
        raise ValueError("rotation matrix is not orthonormal")
    if not np.isclose(np.linalg.det(value), 1.0, atol=2e-5):
        raise ValueError("rotation matrix determinant is not +1")

    trace = float(np.trace(value))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (value[2, 1] - value[1, 2]) / scale
        qy = (value[0, 2] - value[2, 0]) / scale
        qz = (value[1, 0] - value[0, 1]) / scale
    elif value[0, 0] > value[1, 1] and value[0, 0] > value[2, 2]:
        scale = math.sqrt(1.0 + value[0, 0] - value[1, 1] - value[2, 2]) * 2.0
        qw = (value[2, 1] - value[1, 2]) / scale
        qx = 0.25 * scale
        qy = (value[0, 1] + value[1, 0]) / scale
        qz = (value[0, 2] + value[2, 0]) / scale
    elif value[1, 1] > value[2, 2]:
        scale = math.sqrt(1.0 + value[1, 1] - value[0, 0] - value[2, 2]) * 2.0
        qw = (value[0, 2] - value[2, 0]) / scale
        qx = (value[0, 1] + value[1, 0]) / scale
        qy = 0.25 * scale
        qz = (value[1, 2] + value[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + value[2, 2] - value[0, 0] - value[1, 1]) * 2.0
        qw = (value[1, 0] - value[0, 1]) / scale
        qx = (value[0, 2] + value[2, 0]) / scale
        qy = (value[1, 2] + value[2, 1]) / scale
        qz = 0.25 * scale
    quaternion = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("rotation produced an invalid quaternion")
    quaternion /= norm
    return tuple(float(component) for component in quaternion)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration-file", type=Path, required=True)
    parser.add_argument("--expected-serial", default="O1-P070100205")
    parser.add_argument("--rgb-topic", default="/odin1/image")
    parser.add_argument("--cloud-topic", default="/odin1/cloud_slam")
    parser.add_argument("--odom-topic", default="/odin1/odometry")
    parser.add_argument("--output-width", type=int, default=800)
    parser.add_argument("--splat-radius", type=int, default=1)
    parser.add_argument("--depth-min-m", type=float, default=0.20)
    parser.add_argument("--depth-max-m", type=float, default=8.0)
    parser.add_argument("--sync-slop-s", type=float, default=DEFAULT_SYNC_SLOP_S)
    parser.add_argument("--read-timeout-s", type=float, default=2.0)
    parser.add_argument("--world-frame", default=WORLD_FRAME)
    parser.add_argument("--camera-frame", default=CAMERA_FRAME)
    parser.add_argument("--depth-topic", default="/slam/depth")
    parser.add_argument("--keyframe-depth-topic", default="/slam/keyframe_depth")
    parser.add_argument("--image-topic", default="/slam/keyframe_image")
    parser.add_argument("--camera-info-topic", default="/slam/camera_info")
    parser.add_argument(
        "--planner-camera-info-topic",
        default="/camera/camera/color/camera_info",
    )
    parser.add_argument("--slam-odom-topic", default="/slam/odometry")
    parser.add_argument("--visual-odom-topic", default="/slam/odometry_visual")
    parser.add_argument("--relocalization-topic", default="/map/relocalization")
    parser.add_argument("--world-cloud-topic", default="/focus/odin1/cloud_world")
    parser.add_argument(
        "--camera-pose-topic", default="/focus/odin1/camera_pose_world"
    )
    parser.add_argument(
        "--status-topic", default="/focus/odin1/tinynav_adapter_status"
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.world_frame != WORLD_FRAME:
        raise SystemExit(
            "the approved online contract requires --world-frame world"
        )
    if not args.camera_frame:
        raise SystemExit("--camera-frame must not be empty")
    calibration = load_odin_calibration(
        str(args.calibration_file), args.expected_serial
    )
    if calibration.odometry_frame != "odom":
        raise SystemExit(
            "Odin factory artifact no longer declares the observed odom frame"
        )
    projector = OdinProjector(
        calibration,
        output_width=args.output_width,
        splat_radius=args.splat_radius,
        depth_min_m=args.depth_min_m,
        depth_max_m=args.depth_max_m,
    )
    # OdinRos2Source uses this explicit name so it can coexist with the Hub
    # observation sender during a full-stack run.
    args.source_node_name = "focus_odin1_tinynav_source"

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from std_msgs.msg import String

    rclpy.init()
    node = Node("focus_odin1_tinynav_adapter")
    stream_qos = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    status_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    depth_publisher = node.create_publisher(Image, args.depth_topic, stream_qos)
    keyframe_depth_publisher = node.create_publisher(
        Image, args.keyframe_depth_topic, stream_qos
    )
    image_publisher = node.create_publisher(
        Image, args.image_topic, stream_qos
    )
    camera_info_publisher = node.create_publisher(
        CameraInfo, args.camera_info_topic, stream_qos
    )
    planner_camera_info_publisher = node.create_publisher(
        CameraInfo, args.planner_camera_info_topic, stream_qos
    )
    odom_publisher = node.create_publisher(
        Odometry, args.slam_odom_topic, stream_qos
    )
    visual_odom_publisher = node.create_publisher(
        Odometry, args.visual_odom_topic, stream_qos
    )
    relocalization_publisher = node.create_publisher(
        Odometry, args.relocalization_topic, stream_qos
    )
    cloud_publisher = node.create_publisher(
        PointCloud2, args.world_cloud_topic, stream_qos
    )
    pose_publisher = node.create_publisher(
        PoseStamped, args.camera_pose_topic, stream_qos
    )
    status_publisher = node.create_publisher(
        String, args.status_topic, status_qos
    )

    source = OdinRos2Source(args, projector)
    frames = 0
    last_status_monotonic = 0.0
    exit_code = 0
    try:
        while rclpy.ok():
            try:
                frame = source.read()
            except TimeoutError as exc:
                node.get_logger().warning(
                    str(exc), throttle_duration_sec=2.0
                )
                continue

            raw_cloud = frame.cloud_message
            stamp = raw_cloud.header.stamp

            depth = Image()
            depth.header.stamp = stamp
            depth.header.frame_id = args.camera_frame
            depth.height = projector.height
            depth.width = projector.width
            depth.encoding = "32FC1"
            depth.is_bigendian = False
            depth.step = projector.width * 4
            depth.data = np.ascontiguousarray(
                frame.depth_m, dtype=np.float32
            ).tobytes()

            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = args.camera_frame
            image.height = projector.height
            image.width = projector.width
            image.encoding = "bgr8"
            image.is_bigendian = False
            image.step = projector.width * 3
            image.data = np.ascontiguousarray(frame.bgr).tobytes()

            camera_info = CameraInfo()
            camera_info.header.stamp = stamp
            camera_info.header.frame_id = args.camera_frame
            camera_info.width = projector.width
            camera_info.height = projector.height
            camera_info.distortion_model = "plumb_bob"
            camera_info.d = [0.0] * 5
            camera_info.k = [
                projector.fx,
                0.0,
                projector.cx,
                0.0,
                projector.fy,
                projector.cy,
                0.0,
                0.0,
                1.0,
            ]
            camera_info.r = [
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ]
            camera_info.p = [
                projector.fx,
                0.0,
                projector.cx,
                0.0,
                0.0,
                projector.fy,
                projector.cy,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ]

            qx, qy, qz, qw = matrix_to_quaternion(frame.T_odom_camera)
            odometry = Odometry()
            odometry.header.stamp = stamp
            odometry.header.frame_id = args.world_frame
            odometry.child_frame_id = args.camera_frame
            odometry.pose.pose.position.x = float(frame.T_odom_camera[0, 3])
            odometry.pose.pose.position.y = float(frame.T_odom_camera[1, 3])
            odometry.pose.pose.position.z = float(frame.T_odom_camera[2, 3])
            odometry.pose.pose.orientation.x = qx
            odometry.pose.pose.orientation.y = qy
            odometry.pose.pose.orientation.z = qz
            odometry.pose.pose.orientation.w = qw
            odometry.pose.covariance = frame.covariance_6x6

            camera_pose = PoseStamped()
            camera_pose.header = odometry.header
            camera_pose.pose = odometry.pose.pose

            cloud = PointCloud2()
            cloud.header.stamp = stamp
            cloud.header.frame_id = args.world_frame
            cloud.height = raw_cloud.height
            cloud.width = raw_cloud.width
            cloud.fields = raw_cloud.fields
            cloud.is_bigendian = raw_cloud.is_bigendian
            cloud.point_step = raw_cloud.point_step
            cloud.row_step = raw_cloud.row_step
            cloud.data = raw_cloud.data
            cloud.is_dense = raw_cloud.is_dense

            depth_publisher.publish(depth)
            keyframe_depth_publisher.publish(depth)
            image_publisher.publish(image)
            camera_info_publisher.publish(camera_info)
            planner_camera_info_publisher.publish(camera_info)
            odom_publisher.publish(odometry)
            visual_odom_publisher.publish(odometry)
            relocalization_publisher.publish(odometry)
            cloud_publisher.publish(cloud)
            pose_publisher.publish(camera_pose)
            frames += 1

            now = time.monotonic()
            if now - last_status_monotonic >= 1.0:
                status = String()
                status.data = json.dumps(
                    {
                        "schema_version": "focus-odin1-tinynav-adapter-v1",
                        "ready": True,
                        "frames_published": frames,
                        "world_frame": args.world_frame,
                        "camera_frame": args.camera_frame,
                        "device_stamp_ns": (
                            int(stamp.sec) * 1_000_000_000
                            + int(stamp.nanosec)
                        ),
                        "depth_valid_fraction": frame.diagnostics.get(
                            "depth_valid_fraction"
                        ),
                        "provenance": {
                            "factory_calibration": str(
                                args.calibration_file.resolve()
                            ),
                            "odin_cloud_frame_observed": "odom",
                            "world_frame_derivation": (
                                "source_derived_session_local_identity"
                            ),
                        },
                    },
                    separators=(",", ":"),
                )
                status_publisher.publish(status)
                last_status_monotonic = now
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 - fail closed via stale ROS inputs
        node.get_logger().error(str(exc))
        exit_code = 3
    finally:
        source.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
