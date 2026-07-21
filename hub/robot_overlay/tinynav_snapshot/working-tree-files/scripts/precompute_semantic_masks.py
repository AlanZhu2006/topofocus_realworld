#!/usr/bin/env python3
"""Generate timestamped Phase-3 semantic masks directly from a ROS 2 bag."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
from pathlib import Path
import shutil
import sys
import time

from cv_bridge import CvBridge
import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml


ROOT_DIR = Path(__file__).resolve().parents[1]
PACKAGE_DIR = ROOT_DIR / "semantic_mapping"
DEFAULT_MODEL_DIR = (
    Path.home() / ".cache/tinynav/semantic_models/segformer_b0_ade20k"
)
DEFAULT_RGB_TOPIC = "/camera/camera/color/image_raw"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--rgb-topic", default=DEFAULT_RGB_TOPIC)
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument("--start-offset-sec", type=float, default=0.0)
    parser.add_argument("--duration-sec", type=float)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--min-confidence", type=float, default=0.35)
    parser.add_argument(
        "--confidence-dtype", choices=("float16", "float32"), default="float16"
    )
    parser.add_argument("--engine", type=Path, default=DEFAULT_MODEL_DIR / "model_fp16.engine")
    parser.add_argument("--model-config", type=Path, default=DEFAULT_MODEL_DIR / "config.json")
    parser.add_argument(
        "--preprocessor-config",
        type=Path,
        default=DEFAULT_MODEL_DIR / "preprocessor_config.json",
    )
    parser.add_argument(
        "--label-mapping",
        type=Path,
        default=PACKAGE_DIR / "config/ade20k_navigation_mapping.yaml",
    )
    parser.add_argument(
        "--semantic-classes",
        type=Path,
        default=PACKAGE_DIR / "config/semantic_classes.yaml",
    )
    return parser.parse_args()


def stamp_to_ns(message: object) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_write_yaml(path: Path, document: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
    )
    temporary.replace(path)


def validate_args(args: argparse.Namespace) -> None:
    if not args.bag.is_dir():
        raise ValueError(f"Bag directory does not exist: {args.bag}")
    required_files = (
        args.engine,
        args.model_config,
        args.preprocessor_config,
        args.label_mapping,
        args.semantic_classes,
    )
    missing = [str(path) for path in required_files if not path.expanduser().is_file()]
    if missing:
        raise ValueError("Missing required files: " + ", ".join(missing))
    if args.rate_hz <= 0.0:
        raise ValueError("--rate-hz must be positive")
    if args.start_offset_sec < 0.0:
        raise ValueError("--start-offset-sec must be non-negative")
    if args.duration_sec is not None and args.duration_sec <= 0.0:
        raise ValueError("--duration-sec must be positive")
    if args.max_frames is not None and args.max_frames <= 0:
        raise ValueError("--max-frames must be positive")
    if not 0.0 <= args.min_confidence <= 1.0:
        raise ValueError("--min-confidence must be in [0, 1]")
    if args.output_dir.exists():
        if not args.output_dir.is_dir():
            raise ValueError(f"Output path is not a directory: {args.output_dir}")
        if any(args.output_dir.iterdir()):
            raise ValueError(f"Output directory must be empty: {args.output_dir}")


def normalize_paths(args: argparse.Namespace) -> None:
    for name in (
        "bag",
        "output_dir",
        "engine",
        "model_config",
        "preprocessor_config",
        "label_mapping",
        "semantic_classes",
    ):
        setattr(args, name, getattr(args, name).expanduser().resolve())


def open_reader(bag_path: Path) -> tuple[rosbag2_py.SequentialReader, dict[str, str]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    return reader, topic_types


def manifest_document(
    args: argparse.Namespace,
    frames: list[dict],
    schema_version: int,
    engine_hash: str,
) -> dict:
    return {
        "version": 1,
        "semantic_classes_version": schema_version,
        "source": {"bag": str(args.bag.resolve()), "rgb_topic": args.rgb_topic},
        "generator": {
            "backend": "segformer_tensorrt",
            "engine_sha256": engine_hash,
            "rate_hz": args.rate_hz,
            "min_confidence": args.min_confidence,
            "confidence_dtype": args.confidence_dtype,
        },
        "frames": frames,
    }


def main() -> int:
    args = parse_args()
    normalize_paths(args)
    validate_args(args)
    sys.path.insert(0, str(PACKAGE_DIR))
    from semantic_mapping.segformer_tensorrt_backend import (
        SegformerTensorRtBackend,
    )
    from semantic_mapping.semantic_schema import SemanticClassSchema
    from semantic_mapping.semantic_visualizer import blend_semantic_overlay

    output_dir = args.output_dir
    labels_dir = output_dir / "labels"
    confidence_dir = output_dir / "confidence"
    color_labels_dir = output_dir / "color_labels"
    visualization_dir = output_dir / "visualization"
    for directory in (
        labels_dir,
        confidence_dir,
        color_labels_dir,
        visualization_dir,
    ):
        directory.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(args.semantic_classes, output_dir / "semantic_classes.yaml")
    shutil.copyfile(args.label_mapping, output_dir / "label_mapping.yaml")

    schema = SemanticClassSchema.from_yaml(args.semantic_classes)
    engine_hash = sha256(args.engine.expanduser())
    backend = SegformerTensorRtBackend(
        args.engine,
        args.model_config,
        args.preprocessor_config,
        args.label_mapping,
        schema,
        min_confidence=args.min_confidence,
    )
    reader, topic_types = open_reader(args.bag)
    if args.rgb_topic not in topic_types:
        backend.close()
        raise ValueError(f"RGB topic is absent from bag: {args.rgb_topic}")
    message_type = get_message(topic_types[args.rgb_topic])
    bridge = CvBridge()
    period_ns = int(round(1e9 / args.rate_hz))
    duration_ns = (
        None if args.duration_sec is None else int(round(args.duration_sec * 1e9))
    )
    start_offset_ns = int(round(args.start_offset_sec * 1e9))
    first_stamp_ns: int | None = None
    sampling_start_ns: int | None = None
    next_sample_ns: int | None = None
    frames: list[dict] = []
    target_pixel_counts: Counter[int] = Counter()
    rgb_messages = 0
    total_processing_sec = 0.0
    started = time.monotonic()

    try:
        while reader.has_next():
            topic, serialized, _receive_stamp = reader.read_next()
            if topic != args.rgb_topic:
                continue
            rgb_messages += 1
            message = deserialize_message(serialized, message_type)
            timestamp_ns = stamp_to_ns(message)
            if first_stamp_ns is None:
                first_stamp_ns = timestamp_ns
                sampling_start_ns = first_stamp_ns + start_offset_ns
                next_sample_ns = sampling_start_ns
            if sampling_start_ns is None or next_sample_ns is None:
                raise RuntimeError("RGB sampling clock was not initialized")
            if duration_ns is not None and timestamp_ns > sampling_start_ns + duration_ns:
                break
            if timestamp_ns < next_sample_ns:
                continue
            while next_sample_ns <= timestamp_ns:
                next_sample_ns += period_ns

            rgb = bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
            rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
            inference_started = time.monotonic()
            frame = backend.infer(rgb, timestamp_ns)
            processing_sec = time.monotonic() - inference_started
            total_processing_sec += processing_sec

            filename = f"{timestamp_ns}.npy"
            label_relative = Path("labels") / filename
            confidence_relative = Path("confidence") / filename
            np.save(output_dir / label_relative, frame.label_image, allow_pickle=False)
            confidence_dtype = np.dtype(args.confidence_dtype)
            np.save(
                output_dir / confidence_relative,
                frame.confidence_image.astype(confidence_dtype),
                allow_pickle=False,
            )
            overlay = blend_semantic_overlay(
                rgb,
                frame.label_image,
                frame.confidence_image,
                schema,
            )
            color_relative = Path("color_labels") / f"{timestamp_ns}.png"
            if not cv2.imwrite(
                str(output_dir / color_relative),
                cv2.cvtColor(schema.colorize(frame.label_image), cv2.COLOR_RGB2BGR),
            ):
                raise RuntimeError("Failed to write semantic color label")
            visualization_relative = Path("visualization") / f"{timestamp_ns}.png"
            if not cv2.imwrite(
                str(output_dir / visualization_relative),
                cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR),
            ):
                raise RuntimeError("Failed to write semantic visualization")

            unique_ids, counts = np.unique(frame.label_image, return_counts=True)
            target_pixel_counts.update(
                {int(class_id): int(count) for class_id, count in zip(unique_ids, counts)}
            )
            frames.append(
                {
                    "timestamp_ns": timestamp_ns,
                    "label": str(label_relative),
                    "confidence": str(confidence_relative),
                    "color_visualization": str(color_relative),
                    "visualization": str(visualization_relative),
                    "processing_ms": round(processing_sec * 1000.0, 3),
                }
            )
            atomic_write_yaml(
                output_dir / "manifest.yaml",
                manifest_document(args, frames, schema.version, engine_hash),
            )
            print(
                f"frame={len(frames)} stamp={timestamp_ns} "
                f"processing_ms={processing_sec * 1000.0:.1f}",
                flush=True,
            )
            if args.max_frames is not None and len(frames) >= args.max_frames:
                break
    finally:
        backend.close()

    if not frames:
        raise RuntimeError("No RGB frames matched the requested sampling interval")
    elapsed_sec = time.monotonic() - started
    class_names = schema.class_names
    summary = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "source_rgb_messages_read": rgb_messages,
        "processed_frames": len(frames),
        "wall_time_sec": round(elapsed_sec, 3),
        "mean_processing_ms": round(total_processing_sec * 1000.0 / len(frames), 3),
        "target_pixel_counts": {
            class_names[class_id]: count
            for class_id, count in sorted(target_pixel_counts.items())
        },
    }
    atomic_write_yaml(output_dir / "summary.yaml", summary)
    print(yaml.safe_dump(summary, sort_keys=False), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
