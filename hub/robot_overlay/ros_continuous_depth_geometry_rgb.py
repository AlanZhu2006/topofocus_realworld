#!/usr/bin/env python3
"""Provide an inert RGB companion for TinyNav's continuous depth geometry.

WSJ's local occupancy mapper needs geometry, not image semantics, but its
immutable point-cloud node synchronizes an RGB image with depth, CameraInfo
and image-time pose.  TinyNav's keyframe RGB/depth pair can pause for many
seconds even while the independently verified continuous tuple

``/slam/depth + /slam/camera_info + /slam/odometry_visual``

remains healthy.  This read-only adapter publishes a black ``rgb8`` image for
each strictly validated, fresh, monotonically stamped continuous depth frame.
The pixels are used only as unused point-cloud colour fields; depth,
intrinsics and TinyNav pose remain the sole geometry inputs.

The adapter has no actuator subscription or publisher.
"""
from __future__ import annotations

import argparse
import json
import math
from typing import Sequence


DEPTH_ENCODING = "32FC1"
DEPTH_BYTES_PER_PIXEL = 4


def parse_dimensions(raw: str) -> tuple[int, int]:
    """Parse an explicitly approved ``WIDTHxHEIGHT`` image size."""

    try:
        width_text, height_text = raw.lower().split("x", maxsplit=1)
        width = int(width_text)
        height = int(height_text)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError(
            "image size must use WIDTHxHEIGHT"
        ) from exc
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError("image dimensions must be positive")
    return width, height


def validate_continuous_depth_contract(
    *,
    frame_id: str,
    width: int,
    height: int,
    encoding: str,
    is_bigendian: bool,
    step: int,
    data_length: int,
    stamp_ns: int,
    now_ns: int,
    last_published_stamp_ns: int,
    expected_frame: str,
    approved_dimensions: Sequence[tuple[int, int]],
    max_capture_age_s: float,
    max_future_skew_s: float,
) -> str | None:
    """Return a fail-closed rejection reason, or ``None`` when valid."""

    if not expected_frame:
        raise ValueError("expected_frame must not be empty")
    if not approved_dimensions:
        raise ValueError("at least one approved dimension is required")
    if not math.isfinite(max_capture_age_s) or max_capture_age_s <= 0.0:
        raise ValueError("max_capture_age_s must be finite and positive")
    if not math.isfinite(max_future_skew_s) or max_future_skew_s < 0.0:
        raise ValueError("max_future_skew_s must be finite and non-negative")
    if frame_id != expected_frame:
        return f"frame_id={frame_id!r}, expected {expected_frame!r}"
    if (width, height) not in approved_dimensions:
        approved = ", ".join(
            f"{approved_width}x{approved_height}"
            for approved_width, approved_height in approved_dimensions
        )
        return f"dimensions={width}x{height}, expected one of {approved}"
    if encoding.upper() != DEPTH_ENCODING:
        return f"encoding={encoding!r}, expected {DEPTH_ENCODING!r}"
    if is_bigendian:
        return "big-endian continuous depth is not approved"
    expected_step = width * DEPTH_BYTES_PER_PIXEL
    if step != expected_step:
        return f"step={step}, expected tightly packed {expected_step}"
    expected_data_length = step * height
    if data_length != expected_data_length:
        return (
            f"data_length={data_length}, expected {expected_data_length}"
        )
    if stamp_ns <= 0:
        return "capture stamp is missing"
    if (
        last_published_stamp_ns > 0
        and stamp_ns <= last_published_stamp_ns
    ):
        return (
            f"capture stamp {stamp_ns} is not newer than "
            f"{last_published_stamp_ns}"
        )
    maximum_age_ns = int(max_capture_age_s * 1e9)
    maximum_future_ns = int(max_future_skew_s * 1e9)
    if now_ns - stamp_ns > maximum_age_ns:
        return (
            f"capture age {(now_ns - stamp_ns) / 1e9:.3f}s exceeds "
            f"{max_capture_age_s:.3f}s"
        )
    if stamp_ns - now_ns > maximum_future_ns:
        return (
            f"capture stamp is {(stamp_ns - now_ns) / 1e9:.3f}s in the "
            "future"
        )
    return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default="/slam/depth")
    parser.add_argument(
        "--output-topic",
        default="/focus/slam/continuous_depth_geometry_rgb",
    )
    parser.add_argument("--camera-frame", default="camera")
    parser.add_argument(
        "--approved-size",
        action="append",
        type=parse_dimensions,
        required=True,
    )
    parser.add_argument("--max-capture-age-s", type=float, default=2.0)
    parser.add_argument("--max-future-skew-s", type=float, default=0.25)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.input_topic.startswith("/"):
        raise SystemExit("--input-topic must be absolute")
    if not args.output_topic.startswith("/"):
        raise SystemExit("--output-topic must be absolute")
    if args.input_topic == args.output_topic:
        raise SystemExit("input and output topics must differ")
    if not args.camera_frame:
        raise SystemExit("--camera-frame must not be empty")
    approved_dimensions = tuple(dict.fromkeys(args.approved_size))

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header

    class ContinuousDepthGeometryRgbNode(Node):
        def __init__(self) -> None:
            super().__init__("focus_continuous_depth_geometry_rgb")
            self.received = 0
            self.published = 0
            self.dropped = 0
            self.last_published_stamp_ns = 0
            self.black_pixels: dict[tuple[int, int], bytes] = {}
            self.publisher = self.create_publisher(
                Image,
                args.output_topic,
                qos_profile_sensor_data,
            )
            self.subscription = self.create_subscription(
                Image,
                args.input_topic,
                self._callback,
                qos_profile_sensor_data,
            )
            self.timer = self.create_timer(5.0, self._diagnostics)
            self.get_logger().info(
                "Continuous depth geometry RGB ready: "
                f"{args.input_topic} -> {args.output_topic}, "
                f"frame={args.camera_frame}, "
                f"approved_dimensions={approved_dimensions}, "
                f"max_capture_age={args.max_capture_age_s:.3f}s"
            )
            print(
                json.dumps(
                    {
                        "schema_version": (
                            "focus-continuous-depth-geometry-rgb-v1"
                        ),
                        "classification": (
                            "source_derived_geometry_only_adapter"
                        ),
                        "input_topic": args.input_topic,
                        "output_topic": args.output_topic,
                        "camera_frame": args.camera_frame,
                        "approved_dimensions": approved_dimensions,
                        "depth_geometry_unchanged": True,
                        "robot_commands_issued": False,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )

        def _callback(self, message: Image) -> None:
            self.received += 1
            stamp_ns = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            reason = validate_continuous_depth_contract(
                frame_id=str(message.header.frame_id),
                width=int(message.width),
                height=int(message.height),
                encoding=str(message.encoding),
                is_bigendian=bool(message.is_bigendian),
                step=int(message.step),
                data_length=len(message.data),
                stamp_ns=stamp_ns,
                now_ns=int(self.get_clock().now().nanoseconds),
                last_published_stamp_ns=self.last_published_stamp_ns,
                expected_frame=args.camera_frame,
                approved_dimensions=approved_dimensions,
                max_capture_age_s=args.max_capture_age_s,
                max_future_skew_s=args.max_future_skew_s,
            )
            if reason is not None:
                self.dropped += 1
                self.get_logger().error(
                    "Rejected continuous depth outside geometry contract: "
                    + reason,
                    throttle_duration_sec=2.0,
                )
                return
            dimensions = (int(message.width), int(message.height))
            black_pixels = self.black_pixels.get(dimensions)
            if black_pixels is None:
                black_pixels = bytes(dimensions[0] * dimensions[1] * 3)
                self.black_pixels[dimensions] = black_pixels
            output = Image(
                header=Header(
                    stamp=message.header.stamp,
                    frame_id=args.camera_frame,
                ),
                height=message.height,
                width=message.width,
                encoding="rgb8",
                is_bigendian=False,
                step=int(message.width) * 3,
                data=black_pixels,
            )
            self.publisher.publish(output)
            self.last_published_stamp_ns = stamp_ns
            self.published += 1

        def _diagnostics(self) -> None:
            self.get_logger().info(
                "Continuous depth geometry RGB diagnostics: "
                f"received={self.received}, published={self.published}, "
                f"dropped={self.dropped}, "
                f"last_stamp_ns={self.last_published_stamp_ns}"
            )

    rclpy.init(args=None)
    node = ContinuousDepthGeometryRgbNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
