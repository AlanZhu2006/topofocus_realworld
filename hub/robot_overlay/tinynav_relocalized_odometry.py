#!/usr/bin/env python3
"""Publish continuous TinyNav odometry in a validated saved-map frame.

Upstream TinyNav ``MapNode`` estimates ``world_T_map`` from exact-stamped
keyframe relocalizations, but it leaves ``/slam/odometry*`` in the drifting
``world`` frame.  This non-actuating deployment node:

* pairs ``/slam/keyframe_odom`` with ``/map/relocalization`` by exact stamp;
* requires multiple mutually consistent correction estimates;
* checks TinyNav's live TF against that independent pair consensus;
* publishes continuous map-frame odometry only while the contract is ready.

No fallback raw pose is ever relabelled as ``map``.  Missing/stale/inconsistent
relocalization simply stops the output topics so every downstream freshness
gate remains in HOLD.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
from typing import Any


OVERLAY = Path(__file__).resolve().parent
HUB_SRC = OVERLAY.parent / "src"
if HUB_SRC.is_dir():
    sys.path.insert(0, str(HUB_SRC))

from focus_hub.geometry import invert_rigid  # noqa: E402
from focus_hub.relocalized_odometry import (  # noqa: E402
    ExactStampPosePairs,
    RelocalizationConsensus,
    map_pose,
    pose_matrix,
    quaternion_xyzw,
    rotate_pose_covariance,
)
from focus_hub.tinynav_map_contract import (  # noqa: E402
    sha256_file,
    validate_saved_map_manifest,
)


def stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def odometry_matrix(message: Any) -> tuple[float, ...]:
    pose = message.pose.pose
    return pose_matrix(
        (pose.position.x, pose.position.y, pose.position.z),
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def transform_matrix(message: Any) -> tuple[float, ...]:
    transform = message.transform
    return pose_matrix(
        (
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ),
        (
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ),
    )


def write_pose(message: Any, matrix: tuple[float, ...]) -> None:
    quaternion = quaternion_xyzw(matrix)
    message.pose.pose.position.x = matrix[3]
    message.pose.pose.position.y = matrix[7]
    message.pose.pose.position.z = matrix[11]
    message.pose.pose.orientation.x = quaternion[0]
    message.pose.pose.orientation.y = quaternion[1]
    message.pose.pose.orientation.z = quaternion[2]
    message.pose.pose.orientation.w = quaternion[3]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-manifest", type=Path, required=True)
    parser.add_argument("--map-directory", type=Path)
    parser.add_argument("--verify-map-hashes", action="store_true")
    parser.add_argument("--tracking-frame", default="world")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--camera-frame", default="camera")
    parser.add_argument("--tracking-odom-topic", default="/slam/odometry")
    parser.add_argument(
        "--tracking-visual-odom-topic", default="/slam/odometry_visual"
    )
    parser.add_argument(
        "--tracking-keyframe-odom-topic", default="/slam/keyframe_odom"
    )
    parser.add_argument(
        "--relocalization-topic", default="/map/relocalization"
    )
    parser.add_argument(
        "--source-relocalization-frame",
        default="world",
        help=(
            "literal header emitted by pinned TinyNav MapNode; its payload is "
            "semantically map_T_camera despite this upstream wire label"
        ),
    )
    parser.add_argument(
        "--output-odom-topic", default="/focus/maploc/odometry"
    )
    parser.add_argument(
        "--output-visual-odom-topic",
        default="/focus/maploc/odometry_visual",
    )
    parser.add_argument(
        "--status-topic", default="/focus/maploc/status"
    )
    parser.add_argument("--minimum-support", type=int, default=2)
    parser.add_argument("--candidate-window-s", type=float, default=60.0)
    parser.add_argument(
        "--maximum-supported-age-s", type=float, default=30.0
    )
    parser.add_argument(
        "--maximum-cluster-translation-m", type=float, default=0.25
    )
    parser.add_argument(
        "--maximum-cluster-rotation-deg", type=float, default=7.0
    )
    parser.add_argument(
        "--maximum-source-translation-m", type=float, default=0.30
    )
    parser.add_argument(
        "--maximum-source-rotation-deg", type=float, default=8.0
    )
    parser.add_argument(
        "--maximum-alignment-tilt-deg", type=float, default=15.0
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not args.tracking_frame or not args.map_frame or not args.camera_frame:
        raise SystemExit("tracking, map and camera frames must be non-empty")
    if args.tracking_frame == args.map_frame:
        raise SystemExit("tracking and map frames must be distinct")
    topic_values = (
        args.tracking_odom_topic,
        args.tracking_visual_odom_topic,
        args.tracking_keyframe_odom_topic,
        args.relocalization_topic,
        args.output_odom_topic,
        args.output_visual_odom_topic,
        args.status_topic,
    )
    if any(not topic.startswith("/") for topic in topic_values):
        raise SystemExit("all ROS topics must be absolute")
    if args.output_odom_topic in {
        args.tracking_odom_topic,
        args.tracking_visual_odom_topic,
        args.tracking_keyframe_odom_topic,
    } or args.output_visual_odom_topic in {
        args.tracking_odom_topic,
        args.tracking_visual_odom_topic,
        args.tracking_keyframe_odom_topic,
    }:
        raise SystemExit("relocalized outputs must not overwrite raw inputs")
    manifest_path = args.map_manifest.expanduser().resolve()
    map_directory = (
        manifest_path.parent
        if args.map_directory is None
        else args.map_directory.expanduser().resolve()
    )
    try:
        map_contract = validate_saved_map_manifest(
            manifest_path,
            map_directory=map_directory,
            verify_hashes=args.verify_map_hashes,
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    manifest_identity = {
        "source_path": str(manifest_path),
        "size_bytes": manifest_path.stat().st_size,
        "sha256": sha256_file(manifest_path),
        "status": "observed_validated_saved_map_manifest",
    }
    gate = RelocalizationConsensus(
        minimum_support=args.minimum_support,
        candidate_window_s=args.candidate_window_s,
        maximum_supported_age_s=args.maximum_supported_age_s,
        maximum_cluster_translation_m=args.maximum_cluster_translation_m,
        maximum_cluster_rotation_deg=args.maximum_cluster_rotation_deg,
        maximum_source_translation_m=args.maximum_source_translation_m,
        maximum_source_rotation_deg=args.maximum_source_rotation_deg,
        maximum_alignment_tilt_deg=args.maximum_alignment_tilt_deg,
    )
    pairs = ExactStampPosePairs(maximum_entries=60)

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()

    class RelocalizedOdometryNode(Node):
        def __init__(self) -> None:
            super().__init__("focus_tinynav_relocalized_odometry")
            self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
            stream_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=30,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            status_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.odom_publisher = self.create_publisher(
                Odometry, args.output_odom_topic, stream_qos
            )
            self.visual_publisher = self.create_publisher(
                Odometry, args.output_visual_odom_topic, stream_qos
            )
            self.status_publisher = self.create_publisher(
                String, args.status_topic, status_qos
            )
            self.create_subscription(
                Odometry,
                args.tracking_keyframe_odom_topic,
                self.on_keyframe,
                stream_qos,
            )
            self.create_subscription(
                Odometry,
                args.relocalization_topic,
                self.on_relocalization,
                stream_qos,
            )
            self.create_subscription(
                Odometry,
                args.tracking_odom_topic,
                lambda message: self.on_continuous(
                    message, self.odom_publisher, "odometry"
                ),
                stream_qos,
            )
            self.create_subscription(
                Odometry,
                args.tracking_visual_odom_topic,
                lambda message: self.on_continuous(
                    message, self.visual_publisher, "odometry_visual"
                ),
                stream_qos,
            )
            self.last_decision = gate.evaluate(
                source_tracking_T_map=None,
                now_ns=time.monotonic_ns(),
            )
            self.last_pair_stamp_ns = 0
            self.pairs_accepted = 0
            self.pairs_rejected = 0
            self.outputs = {"odometry": 0, "odometry_visual": 0}
            self.input_rejections = 0
            self.last_source_tf: tuple[float, ...] | None = None
            self.last_status_signature = ""
            self.create_timer(0.5, self.publish_status)

        def validate_raw_odom(self, message: Odometry) -> None:
            if str(message.header.frame_id) != args.tracking_frame:
                raise ValueError(
                    f"raw odometry frame {message.header.frame_id!r} is not "
                    f"{args.tracking_frame!r}"
                )
            if str(message.child_frame_id) != args.camera_frame:
                raise ValueError(
                    f"raw odometry child {message.child_frame_id!r} is not "
                    f"{args.camera_frame!r}"
                )
            if stamp_ns(message) <= 0:
                raise ValueError("raw odometry stamp is zero")

        def accept_pair(
            self, pair: tuple[tuple[float, ...], tuple[float, ...]] | None,
            *, pair_stamp_ns: int,
        ) -> None:
            if pair is None:
                return
            try:
                gate.add_pair(
                    tracking_T_camera=pair[0],
                    map_T_camera=pair[1],
                    stamp_ns=pair_stamp_ns,
                    observed_ns=time.monotonic_ns(),
                )
            except ValueError as exc:
                self.pairs_rejected += 1
                self.get_logger().warning(
                    f"relocalization pair rejected: {exc}",
                    throttle_duration_sec=2.0,
                )
                return
            self.pairs_accepted += 1
            self.last_pair_stamp_ns = pair_stamp_ns

        def on_keyframe(self, message: Odometry) -> None:
            try:
                self.validate_raw_odom(message)
                key = stamp_ns(message)
                pair = pairs.add_tracking(
                    stamp_ns=key,
                    tracking_T_camera=odometry_matrix(message),
                )
                self.accept_pair(pair, pair_stamp_ns=key)
            except ValueError as exc:
                self.input_rejections += 1
                self.get_logger().warning(
                    f"keyframe odometry rejected: {exc}",
                    throttle_duration_sec=2.0,
                )

        def on_relocalization(self, message: Odometry) -> None:
            try:
                if (
                    str(message.header.frame_id)
                    != args.source_relocalization_frame
                ):
                    raise ValueError(
                        "pinned MapNode relocalization frame "
                        f"{message.header.frame_id!r} is not "
                        f"{args.source_relocalization_frame!r}"
                    )
                if str(message.child_frame_id) != args.camera_frame:
                    raise ValueError(
                        "relocalization child frame "
                        f"{message.child_frame_id!r} is not "
                        f"{args.camera_frame!r}"
                    )
                key = stamp_ns(message)
                if key <= 0:
                    raise ValueError("relocalization stamp is zero")
                pair = pairs.add_map(
                    stamp_ns=key,
                    map_T_camera=odometry_matrix(message),
                )
                self.accept_pair(pair, pair_stamp_ns=key)
            except ValueError as exc:
                self.input_rejections += 1
                self.get_logger().warning(
                    f"relocalization rejected: {exc}",
                    throttle_duration_sec=2.0,
                )

        def lookup_tracking_T_map(self) -> tuple[float, ...] | None:
            try:
                message = self.tf_buffer.lookup_transform(
                    args.tracking_frame,
                    args.map_frame,
                    Time(),
                )
            except TransformException:
                return None
            try:
                return transform_matrix(message)
            except ValueError:
                return None

        def on_continuous(
            self,
            message: Odometry,
            publisher: Any,
            stream_name: str,
        ) -> None:
            try:
                self.validate_raw_odom(message)
                tracking_camera = odometry_matrix(message)
            except ValueError as exc:
                self.input_rejections += 1
                self.get_logger().warning(
                    f"{stream_name} rejected: {exc}",
                    throttle_duration_sec=2.0,
                )
                return
            source_tf = self.lookup_tracking_T_map()
            self.last_source_tf = source_tf
            self.last_decision = gate.evaluate(
                source_tracking_T_map=source_tf,
                now_ns=time.monotonic_ns(),
            )
            if not self.last_decision.ready or source_tf is None:
                return
            try:
                corrected = map_pose(
                    tracking_T_map=source_tf,
                    tracking_T_camera=tracking_camera,
                )
                map_T_tracking = invert_rigid(source_tf)
                covariance = rotate_pose_covariance(
                    message.pose.covariance,
                    map_T_tracking=map_T_tracking,
                )
            except ValueError as exc:
                self.input_rejections += 1
                self.get_logger().warning(
                    f"{stream_name} transform rejected: {exc}",
                    throttle_duration_sec=2.0,
                )
                return
            output = Odometry()
            output.header.stamp = message.header.stamp
            output.header.frame_id = args.map_frame
            output.child_frame_id = args.camera_frame
            write_pose(output, corrected)
            output.pose.covariance = list(covariance)
            # Odometry twist is expressed in child_frame_id. Changing only
            # the pose parent frame therefore leaves twist and its covariance
            # unchanged.
            output.twist = message.twist
            publisher.publish(output)
            self.outputs[stream_name] += 1

        def publish_status(self) -> None:
            decision = gate.evaluate(
                source_tracking_T_map=self.lookup_tracking_T_map(),
                now_ns=time.monotonic_ns(),
            )
            self.last_decision = decision
            age_s = (
                None
                if decision.latest_supported_observed_ns <= 0
                else max(
                    0.0,
                    (
                        time.monotonic_ns()
                        - decision.latest_supported_observed_ns
                    )
                    * 1e-9,
                )
            )
            payload = {
                "schema_version": "focus-tinynav-relocalized-odometry-v1",
                "ready": decision.ready,
                "reason": decision.reason,
                "tracking_frame": args.tracking_frame,
                "map_frame": args.map_frame,
                "camera_frame": args.camera_frame,
                "support": decision.support,
                "minimum_support": gate.minimum_support,
                "latest_supported_age_s": age_s,
                "last_pair_stamp_ns": self.last_pair_stamp_ns,
                "pairs_accepted": self.pairs_accepted,
                "pairs_rejected": self.pairs_rejected,
                "input_rejections": self.input_rejections,
                "outputs": self.outputs,
                "source_translation_error_m": (
                    decision.source_translation_error_m
                ),
                "source_rotation_error_deg": (
                    decision.source_rotation_error_deg
                ),
                "map": {
                    "map_id": map_contract["map_id"],
                    "map_snapshot_sha256": map_contract[
                        "map_snapshot_sha256"
                    ],
                    "result_status": map_contract["result_status"],
                    "manifest": manifest_identity,
                },
                "pose_contract": (
                    "map_T_camera=inverse(tracking_T_map)"
                    "@tracking_T_camera"
                ),
                "raw_pose_fallback_enabled": False,
                "robot_commands_issued": False,
                "published_at_ns": time.time_ns(),
            }
            encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
            # Always publish the 2 Hz transient-local heartbeat, while logging
            # only state changes.
            self.status_publisher.publish(String(data=encoded))
            signature = json.dumps(
                {
                    "ready": decision.ready,
                    "reason": decision.reason,
                    "support": decision.support,
                },
                sort_keys=True,
            )
            if signature != self.last_status_signature:
                self.get_logger().info(signature)
                self.last_status_signature = signature

    node = RelocalizedOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
