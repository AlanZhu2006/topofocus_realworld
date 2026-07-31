#!/usr/bin/env python3
"""Read-only preflight for TinyNav saved-map relocalized odometry."""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import time
from typing import Any


OVERLAY = Path(__file__).resolve().parent
HUB_SRC = OVERLAY.parent / "src"
if HUB_SRC.is_dir():
    sys.path.insert(0, str(HUB_SRC))

from focus_hub.relocalized_odometry import pose_matrix  # noqa: E402
from focus_hub.tinynav_map_contract import (  # noqa: E402
    validate_saved_map_manifest,
)


def stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def validate_status(
    message: Any,
    *,
    expected_map_id: str,
    maximum_age_s: float,
) -> dict[str, Any]:
    try:
        payload = json.loads(str(message.data))
    except json.JSONDecodeError as exc:
        raise ValueError("maploc status is not JSON") from exc
    if not isinstance(payload, dict):
        raise ValueError("maploc status must be an object")
    if payload.get("schema_version") != (
        "focus-tinynav-relocalized-odometry-v1"
    ):
        raise ValueError("maploc status schema mismatch")
    if payload.get("ready") is not True or payload.get("reason") != "READY":
        raise ValueError(
            f"maploc is not ready: {payload.get('reason')}"
        )
    if payload.get("raw_pose_fallback_enabled") is not False:
        raise ValueError("maploc raw-pose fallback must be disabled")
    if payload.get("robot_commands_issued") is not False:
        raise ValueError("maploc status claims a robot command")
    if payload.get("map_frame") != "map" or payload.get("tracking_frame") != "world":
        raise ValueError("maploc coordinate contract mismatch")
    map_record = payload.get("map")
    if (
        not isinstance(map_record, dict)
        or map_record.get("map_id") != expected_map_id
    ):
        raise ValueError("maploc map identity mismatch")
    age_s = payload.get("latest_supported_age_s")
    if (
        not isinstance(age_s, (float, int))
        or not math.isfinite(float(age_s))
        or not 0.0 <= float(age_s) <= maximum_age_s
    ):
        raise ValueError("maploc relocalization support is stale")
    support = payload.get("support")
    minimum = payload.get("minimum_support")
    if (
        not isinstance(support, int)
        or not isinstance(minimum, int)
        or support < minimum
        or minimum < 2
    ):
        raise ValueError("maploc support count is invalid")
    return payload


def validate_odometry(
    message: Any,
    *,
    expected_frame: str,
    expected_camera_frame: str,
) -> int:
    if str(message.header.frame_id) != expected_frame:
        raise ValueError("relocalized odometry parent frame mismatch")
    if str(message.child_frame_id) != expected_camera_frame:
        raise ValueError("relocalized odometry child frame mismatch")
    stamp = stamp_ns(message)
    if stamp <= 0:
        raise ValueError("relocalized odometry stamp is zero")
    pose = message.pose.pose
    pose_matrix(
        (pose.position.x, pose.position.y, pose.position.z),
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )
    covariance = tuple(float(value) for value in message.pose.covariance)
    if len(covariance) != 36 or not all(
        math.isfinite(value) for value in covariance
    ):
        raise ValueError("relocalized covariance is invalid")
    return stamp


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-manifest", type=Path, required=True)
    parser.add_argument("--map-directory", type=Path)
    parser.add_argument("--status-topic", default="/focus/maploc/status")
    parser.add_argument(
        "--raw-visual-odom-topic", default="/slam/odometry_visual"
    )
    parser.add_argument(
        "--relocalized-visual-odom-topic",
        default="/focus/maploc/odometry_visual",
    )
    parser.add_argument("--timeout-s", type=float, default=90.0)
    parser.add_argument("--minimum-samples", type=int, default=3)
    parser.add_argument("--maximum-support-age-s", type=float, default=30.0)
    args = parser.parse_args()
    if (
        args.timeout_s <= 0.0
        or args.minimum_samples <= 0
        or args.maximum_support_age_s <= 0.0
    ):
        parser.error("timeouts, ages and sample count must be positive")
    manifest_path = args.map_manifest.expanduser().resolve()
    directory = (
        manifest_path.parent
        if args.map_directory is None
        else args.map_directory.expanduser().resolve()
    )
    try:
        map_contract = validate_saved_map_manifest(
            manifest_path,
            map_directory=directory,
            verify_hashes=False,
        )
    except ValueError as exc:
        parser.error(str(exc))

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import String

    rclpy.init()
    node = Node("focus_tinynav_relocalization_verifier")
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
    raw_stamps: deque[int] = deque(maxlen=100)
    corrected_stamps: deque[int] = deque(maxlen=100)
    latest_status: Any | None = None
    errors: list[str] = []

    def on_raw(message: Odometry) -> None:
        raw_stamps.append(stamp_ns(message))

    def on_corrected(message: Odometry) -> None:
        try:
            corrected_stamps.append(
                validate_odometry(
                    message,
                    expected_frame="map",
                    expected_camera_frame="camera",
                )
            )
        except ValueError as exc:
            errors.append(str(exc))

    def on_status(message: String) -> None:
        nonlocal latest_status
        latest_status = message

    subscriptions = (
        node.create_subscription(
            Odometry, args.raw_visual_odom_topic, on_raw, stream_qos
        ),
        node.create_subscription(
            Odometry,
            args.relocalized_visual_odom_topic,
            on_corrected,
            stream_qos,
        ),
        node.create_subscription(
            String, args.status_topic, on_status, status_qos
        ),
    )
    deadline = time.monotonic() + args.timeout_s
    status_payload: dict[str, Any] | None = None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if errors:
                raise ValueError(errors[-1])
            if latest_status is None:
                continue
            try:
                status_payload = validate_status(
                    latest_status,
                    expected_map_id=map_contract["map_id"],
                    maximum_age_s=args.maximum_support_age_s,
                )
            except ValueError:
                continue
            exact = set(raw_stamps).intersection(corrected_stamps)
            if len(exact) >= args.minimum_samples:
                result = {
                    "schema_version": (
                        "focus-tinynav-relocalization-preflight-v1"
                    ),
                    "passed": True,
                    "map_id": map_contract["map_id"],
                    "map_snapshot_sha256": map_contract[
                        "map_snapshot_sha256"
                    ],
                    "status_reason": status_payload["reason"],
                    "support": status_payload["support"],
                    "exact_stamp_samples": len(exact),
                    "latest_stamp_ns": max(exact),
                    "classification": (
                        "observed_read_only_saved_map_relocalization"
                    ),
                    "robot_commands_issued": False,
                }
                print(json.dumps(result, sort_keys=True))
                return 0
        reason = (
            "no_status"
            if latest_status is None
            else (
                "status_not_ready"
                if status_payload is None
                else "insufficient_exact_stamp_output"
            )
        )
        raise TimeoutError(f"relocalization preflight timed out: {reason}")
    finally:
        _ = subscriptions
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
