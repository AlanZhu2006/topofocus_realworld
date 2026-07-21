#!/usr/bin/env python3
"""RedNet real-camera domain-gap diagnostic.

Runs the exact MP3D-40 model/preprocessing used by the central mapper over
either a preserved TinyNav recording or a bounded range from the live Hub
spool.  It reports both the raw argmax and the production thresholded output,
including confidence statistics, and writes representative visualizations.

There is no semantic ground truth for these recordings, so this is a
statistical/qualitative diagnostic rather than an accuracy or IoU benchmark.
Mapping only: nothing here talks to a robot or issues commands.
"""
from __future__ import annotations

import argparse
import hashlib
from itertools import islice
import json
from pathlib import Path
import sys

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))
sys.path.insert(0, str(WORKSPACE / "dependencies"))
sys.path.insert(0, str(WORKSPACE / "dependencies" / "RedNet"))
sys.path.insert(0, str(WORKSPACE / "source" / "Focus_realworld"))

from focus_hub.central_mapping import (  # noqa: E402
    DEPTH_MAX_M,
    DEPTH_MIN_M,
    HM3D_CATEGORY_NAMES,
    MP_CATEGORIES_MAPPING,
    RedNetSegmenter,
)
from focus_hub.pipeline import iter_spooled_observations  # noqa: E402
from focus_hub.tinynav_replay import TinyNavReplayReader  # noqa: E402

NUM_MP3D_CLASSES = 40
PRODUCTION_CONFIDENCE_THRESHOLD = 0.8


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def colorize(class_map: np.ndarray) -> np.ndarray:
    """HSV colormap over raw 1..40 class ids (0 = black)."""
    import cv2

    hue = (class_map.astype(np.float32) * (179.0 / NUM_MP3D_CLASSES)).astype(np.uint8)
    saturation = np.where(class_map > 0, 255, 0).astype(np.uint8)
    value = np.where(class_map > 0, 255, 0).astype(np.uint8)
    hsv = np.stack((hue, saturation, value), axis=-1)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def predict_with_confidence(
    segmenter: RedNetSegmenter, rgb_bgr: np.ndarray, depth_m: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return production-thresholded IDs, raw argmax IDs and confidence.

    This is source-derived from the immutable ``RedNetResizeWrapper.forward``
    implementation.  It exposes the values immediately before that wrapper's
    fixed 0.8 confidence replacement so a near-empty semantic map can be
    distinguished from a model that never proposes target classes at all.
    """
    torch = segmenter._torch
    functional = torch.nn.functional
    wrapper = segmenter.model
    rgb = np.ascontiguousarray(rgb_bgr[:, :, ::-1], dtype=np.float32)
    depth_norm = (depth_m - DEPTH_MIN_M) / (DEPTH_MAX_M - DEPTH_MIN_M)
    depth_norm = np.clip(depth_norm, 0.0, 1.0).astype(np.float32)
    depth_norm[depth_m <= 0.0] = 1.0
    output_shape = rgb.shape[:2]
    with torch.no_grad():
        rgb_tensor = torch.from_numpy(rgb).to(segmenter.device).unsqueeze(0)
        depth_tensor = torch.from_numpy(depth_norm[..., None]).to(
            segmenter.device
        ).unsqueeze(0)
        rgb_tensor = rgb_tensor.permute(0, 3, 1, 2).float() / 255.0
        depth_tensor = depth_tensor.permute(0, 3, 1, 2)
        rgb_tensor = functional.interpolate(
            rgb_tensor, wrapper.pretrained_size, mode="bilinear"
        )
        depth_tensor = functional.interpolate(
            depth_tensor, wrapper.pretrained_size, mode="nearest"
        )
        logits = wrapper.rednet(
            wrapper.semmap_rgb_norm(rgb_tensor),
            wrapper.semmap_depth_norm(depth_tensor),
        )
        confidence, zero_based = torch.softmax(logits, dim=1).max(dim=1)
        raw = functional.interpolate(
            (zero_based + 1).float().unsqueeze(1),
            output_shape,
            mode="nearest",
        ).squeeze(1)
        confidence = functional.interpolate(
            confidence.unsqueeze(1), output_shape, mode="nearest"
        ).squeeze(1)
    raw_ids = raw.squeeze(0).cpu().numpy().astype(np.int16)
    confidence_np = confidence.squeeze(0).cpu().numpy().astype(np.float32)
    thresholded = raw_ids.copy()
    thresholded[confidence_np < PRODUCTION_CONFIDENCE_THRESHOLD] = 1
    return thresholded, raw_ids, confidence_np


def _top_classes(counts: np.ndarray, total_pixels: int) -> list[dict]:
    ranked = sorted(
        ((class_id, int(counts[class_id])) for class_id in range(1, NUM_MP3D_CLASSES + 1)),
        key=lambda item: -item[1],
    )[:15]
    return [
        {
            "mp3d_class_id": class_id,
            "total_pixels": pixels,
            "area_fraction_of_all_frames": round(pixels / total_pixels, 8),
        }
        for class_id, pixels in ranked
    ]


def _relevant_stats(
    pixels: np.ndarray, frames: np.ndarray, frame_area: int, num_frames: int
) -> dict:
    return {
        name: {
            "mp3d_class_id": class_id,
            "frames_present": int(frames[class_id]),
            "frame_fraction": round(float(frames[class_id]) / num_frames, 6),
            "total_area_fraction": round(
                float(pixels[class_id]) / (frame_area * num_frames), 8
            ),
            "mean_area_fraction_when_present": (
                round(
                    float(pixels[class_id])
                    / (frame_area * frames[class_id]),
                    8,
                )
                if frames[class_id] > 0
                else 0.0
            ),
        }
        for name, class_id in zip(HM3D_CATEGORY_NAMES, MP_CATEGORIES_MAPPING)
    }


def _histogram_percentile(histogram: np.ndarray, percentile: float) -> float:
    target = histogram.sum() * percentile / 100.0
    index = int(np.searchsorted(np.cumsum(histogram), target, side="left"))
    return round(min(index, len(histogram) - 1) / (len(histogram) - 1), 4)


def _spool_source_record(spool: Path, robot_id: str, observation) -> dict:
    directory = spool / robot_id / f"{observation.sequence:020d}"
    metadata_path = directory / "metadata.json"
    rgb_name = "rgb.jpg" if observation.metadata.rgb_encoding == "jpeg" else "rgb.png"
    rgb_path = directory / rgb_name
    depth_path = directory / "depth.png"
    return {
        "sequence": observation.sequence,
        "source_dir": str(directory.resolve()),
        "metadata_size_bytes": metadata_path.stat().st_size,
        "metadata_sha256": sha256_file(metadata_path),
        "rgb_path": str(rgb_path.resolve()),
        "rgb_size_bytes": rgb_path.stat().st_size,
        "rgb_sha256": observation.metadata.rgb_sha256,
        "depth_path": str(depth_path.resolve()),
        "depth_size_bytes": depth_path.stat().st_size,
        "depth_sha256": observation.metadata.depth_sha256,
        "payload_checksum_source": "verified_wire_metadata",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path)
    parser.add_argument("--extracted", type=Path)
    parser.add_argument("--spool", type=Path)
    parser.add_argument("--robot-id")
    parser.add_argument("--start-after-sequence", type=int, default=-1)
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="0 means all TinyNav frames; live spool mode requires a positive bound",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument(
        "--rednet-checkpoint",
        type=Path,
        default=WORKSPACE / "artifacts" / "checkpoints" / "rednet_semmap_mp3d_40.pth",
    )
    parser.add_argument("--num-sample-frames", type=int, default=8)
    args = parser.parse_args()

    tinynav_mode = args.record is not None or args.extracted is not None
    spool_mode = args.spool is not None or args.robot_id is not None
    if tinynav_mode == spool_mode:
        parser.error("choose exactly one input mode: --record/--extracted or --spool/--robot-id")
    if tinynav_mode and (args.record is None or args.extracted is None):
        parser.error("TinyNav mode requires both --record and --extracted")
    if spool_mode and (args.spool is None or not args.robot_id):
        parser.error("spool mode requires both --spool and --robot-id")
    if spool_mode and args.max_frames <= 0:
        parser.error("live spool mode requires a positive --max-frames bound")
    if args.max_frames < 0 or args.num_sample_frames <= 0:
        parser.error("frame limits must be non-negative and sample count positive")
    if args.output.exists():
        print(f"refusing to overwrite existing output: {args.output}", file=sys.stderr)
        return 2

    input_provenance: dict
    source_records: list[dict] = []
    if tinynav_mode:
        reader = TinyNavReplayReader(args.record, args.extracted)
        frame_budget = min(len(reader), args.max_frames) if args.max_frames else len(reader)
        frames = islice(reader.frames(), frame_budget)
        input_provenance = {
            "mode": "tinynav_record",
            "record": str(args.record.resolve()),
            "extracted": str(args.extracted.resolve()),
            "frame_budget": frame_budget,
        }
        for root_name, root in (("record", args.record), ("extracted", args.extracted)):
            manifest_path = root / "manifest.json"
            if manifest_path.is_file():
                input_provenance[f"{root_name}_manifest"] = {
                    "path": str(manifest_path.resolve()),
                    "size_bytes": manifest_path.stat().st_size,
                    "sha256": sha256_file(manifest_path),
                }
    else:
        frame_budget = args.max_frames
        observations = islice(
            iter_spooled_observations(
                args.spool, args.robot_id, after_sequence=args.start_after_sequence
            ),
            frame_budget,
        )

        def spool_frames():
            for observation in observations:
                source_records.append(
                    _spool_source_record(args.spool, args.robot_id, observation)
                )
                yield observation

        frames = spool_frames()
        input_provenance = {
            "mode": "hub_live_spool",
            "spool": str(args.spool.resolve()),
            "robot_id": args.robot_id,
            "start_after_sequence": args.start_after_sequence,
            "frame_budget": frame_budget,
        }

    args.output.mkdir(parents=True)
    samples_dir = args.output / "samples"
    samples_dir.mkdir()
    checkpoint = args.rednet_checkpoint.resolve()
    segmenter = RedNetSegmenter(checkpoint, device=args.device)

    threshold_pixels = np.zeros(NUM_MP3D_CLASSES + 1, dtype=np.int64)
    threshold_frames = np.zeros(NUM_MP3D_CLASSES + 1, dtype=np.int64)
    raw_pixels = np.zeros(NUM_MP3D_CLASSES + 1, dtype=np.int64)
    raw_frames = np.zeros(NUM_MP3D_CLASSES + 1, dtype=np.int64)
    confidence_histogram = np.zeros(1001, dtype=np.int64)
    confidence_above_threshold = 0
    frame_area = 0
    frame_shape: tuple[int, int] | None = None
    num_frames = 0
    valid_depth_pixels = 0
    sample_indices = set(
        np.linspace(
            0,
            max(0, frame_budget - 1),
            num=min(args.num_sample_frames, frame_budget),
            dtype=int,
        ).tolist()
    )
    saved_sample_indices = []

    import cv2

    for index, frame in enumerate(frames):
        thresholded, raw, confidence = predict_with_confidence(
            segmenter, frame.rgb_bgr, frame.depth_m
        )
        valid_depth_pixels += int(
            np.count_nonzero(
                (frame.depth_m >= 0.3) & (frame.depth_m <= DEPTH_MAX_M)
            )
        )
        if frame_shape is None:
            frame_shape = (int(raw.shape[0]), int(raw.shape[1]))
            frame_area = int(raw.size)
        threshold_count = np.bincount(
            thresholded.ravel(), minlength=NUM_MP3D_CLASSES + 1
        )
        raw_count = np.bincount(raw.ravel(), minlength=NUM_MP3D_CLASSES + 1)
        threshold_pixels += threshold_count
        threshold_frames += (threshold_count > 0).astype(np.int64)
        raw_pixels += raw_count
        raw_frames += (raw_count > 0).astype(np.int64)
        bins = np.minimum((confidence * 1000.0).astype(np.int32), 1000)
        confidence_histogram += np.bincount(bins.ravel(), minlength=1001)
        confidence_above_threshold += int(
            np.count_nonzero(confidence >= PRODUCTION_CONFIDENCE_THRESHOLD)
        )
        num_frames += 1

        if index in sample_indices:
            prefix = samples_dir / f"frame{index:04d}"
            cv2.imwrite(str(prefix.with_name(prefix.name + "_rgb.png")), frame.rgb_bgr)
            cv2.imwrite(
                str(prefix.with_name(prefix.name + "_raw_argmax.png")), colorize(raw)
            )
            cv2.imwrite(
                str(prefix.with_name(prefix.name + "_thresholded.png")),
                colorize(thresholded),
            )
            cv2.imwrite(
                str(prefix.with_name(prefix.name + "_confidence.png")),
                np.clip(confidence * 255.0, 0, 255).astype(np.uint8),
            )
            saved_sample_indices.append(index)

    if num_frames == 0 or frame_shape is None:
        print("no frames found in the requested input range", file=sys.stderr)
        return 1

    total_pixels = frame_area * num_frames
    if source_records:
        manifest = json.dumps(
            source_records, sort_keys=True, separators=(",", ":")
        ).encode("utf-8")
        input_provenance["source_manifest_sha256"] = hashlib.sha256(manifest).hexdigest()
        input_provenance["source_observations"] = source_records
    input_provenance["first_sequence"] = (
        source_records[0]["sequence"] if source_records else None
    )
    input_provenance["last_sequence"] = (
        source_records[-1]["sequence"] if source_records else None
    )

    selected_ids = set(MP_CATEGORIES_MAPPING)
    raw_selected_pixels = sum(int(raw_pixels[class_id]) for class_id in selected_ids)
    threshold_selected_pixels = sum(
        int(threshold_pixels[class_id]) for class_id in selected_ids
    )
    summary = {
        "schema_version": 2,
        "result_status": "observed_model_output_without_semantic_ground_truth",
        "input": input_provenance,
        "checkpoint": {
            "path": str(checkpoint),
            "size_bytes": checkpoint.stat().st_size,
            "sha256": sha256_file(checkpoint),
        },
        "num_frames": num_frames,
        "frame_hw": list(frame_shape),
        "production_confidence_threshold": PRODUCTION_CONFIDENCE_THRESHOLD,
        "geometry_input": {
            "valid_depth_range_m": [0.3, DEPTH_MAX_M],
            "valid_depth_fraction": round(valid_depth_pixels / total_pixels, 8),
        },
        "confidence": {
            "p50_approx": _histogram_percentile(confidence_histogram, 50),
            "p90_approx": _histogram_percentile(confidence_histogram, 90),
            "p99_approx": _histogram_percentile(confidence_histogram, 99),
            "fraction_at_or_above_threshold": round(
                confidence_above_threshold / total_pixels, 8
            ),
        },
        "target_15_area_fraction": {
            "raw_argmax": round(raw_selected_pixels / total_pixels, 8),
            "production_thresholded": round(
                threshold_selected_pixels / total_pixels, 8
            ),
        },
        "relevant_15_categories_raw_argmax": _relevant_stats(
            raw_pixels, raw_frames, frame_area, num_frames
        ),
        "relevant_15_categories_production_thresholded": _relevant_stats(
            threshold_pixels, threshold_frames, frame_area, num_frames
        ),
        "top15_raw_argmax_classes_by_total_pixels": _top_classes(
            raw_pixels, total_pixels
        ),
        "top15_production_thresholded_classes_by_total_pixels": _top_classes(
            threshold_pixels, total_pixels
        ),
        "sample_frame_indices": saved_sample_indices,
        "sample_artifacts": [
            {
                "path": str(path.resolve()),
                "size_bytes": path.stat().st_size,
                "sha256": sha256_file(path),
                "status": "observed_model_visualization",
            }
            for path in sorted(samples_dir.iterdir())
            if path.is_file()
        ],
        "limitations": [
            "No per-pixel semantic ground truth is available, so accuracy/IoU is unverified.",
            "Raw argmax classes below 0.8 confidence are not safe to treat as detections.",
            "Lowering the production threshold requires a labelled validation set.",
        ],
    }
    summary_path = args.output / "domain_gap_summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=False) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "output": str(summary_path),
        "num_frames": num_frames,
        "confidence": summary["confidence"],
        "geometry_input": summary["geometry_input"],
        "target_15_area_fraction": summary["target_15_area_fraction"],
        "sample_artifacts": len(summary["sample_artifacts"]),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
