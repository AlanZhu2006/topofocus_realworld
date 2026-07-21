#!/usr/bin/env python3
"""RedNet domain-gap diagnostic: how well does the MP3D-40-trained model
hold up on a real (non-Matterport) recording?

There is no ground-truth semantic annotation for real robot recordings in
this workspace, so this cannot compute IoU/accuracy against a label. What it
CAN do, honestly: run the exact same segmenter used in production over every
keyframe of a real session, report per-class pixel-area statistics across
the whole run (a class that never fires, or one that swallows the frame,
is a real signal even without ground truth), and save RGB + colorized
semantic overlays for a handful of representative frames so a human can
visually judge plausibility. Read this as a qualitative/statistical
diagnostic, not a benchmark number.

Mapping only: nothing here talks to a robot or issues commands.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))
sys.path.insert(0, str(WORKSPACE / "dependencies"))
sys.path.insert(0, str(WORKSPACE / "source" / "Focus_realworld"))

from focus_hub.central_mapping import (  # noqa: E402
    HM3D_CATEGORY_NAMES,
    MP_CATEGORIES_MAPPING,
    RedNetSegmenter,
)
from focus_hub.tinynav_replay import TinyNavReplayReader  # noqa: E402

NUM_MP3D_CLASSES = 40


def colorize(class_map: np.ndarray) -> np.ndarray:
    """HSV colormap over raw 1..40 class ids (0 = nothing / clipped to black)."""
    import cv2

    hue = (class_map.astype(np.float32) * (179.0 / NUM_MP3D_CLASSES)).astype(np.uint8)
    sat = np.where(class_map > 0, 255, 0).astype(np.uint8)
    val = np.where(class_map > 0, 255, 0).astype(np.uint8)
    hsv = np.stack([hue, sat, val], axis=-1)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--extracted", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--rednet-checkpoint", type=Path,
                        default=WORKSPACE / "artifacts" / "checkpoints" / "rednet_semmap_mp3d_40.pth")
    parser.add_argument("--num-sample-frames", type=int, default=8,
                        help="how many evenly-spaced frames to save RGB+overlay images for")
    args = parser.parse_args()

    if args.output.exists():
        print(f"refusing to overwrite existing output: {args.output}", file=sys.stderr)
        return 2
    args.output.mkdir(parents=True)

    reader = TinyNavReplayReader(args.record, args.extracted)
    segmenter = RedNetSegmenter(args.rednet_checkpoint, device=args.device)

    # class id 0 means "no valid depth pixels contributed"; ids 1..40 are real.
    pixels_per_class = np.zeros(NUM_MP3D_CLASSES + 1, dtype=np.int64)
    frames_with_class = np.zeros(NUM_MP3D_CLASSES + 1, dtype=np.int64)
    frame_area = None
    frame_hw: tuple[int, int] | None = None
    num_frames = 0

    sample_indices = set(
        np.linspace(0, len(reader) - 1, num=min(args.num_sample_frames, len(reader)), dtype=int).tolist()
    )
    samples_dir = args.output / "samples"
    samples_dir.mkdir()

    import cv2

    for idx, frame in enumerate(reader.frames()):
        pred = segmenter.segment(frame.rgb_bgr, frame.depth_m)
        if frame_area is None:
            frame_hw = (int(pred.shape[0]), int(pred.shape[1]))
            frame_area = pred.shape[0] * pred.shape[1]
        counts = np.bincount(pred.ravel(), minlength=NUM_MP3D_CLASSES + 1)
        pixels_per_class += counts
        frames_with_class += (counts > 0).astype(np.int64)
        num_frames += 1

        if idx in sample_indices:
            cv2.imwrite(str(samples_dir / f"frame{idx:04d}_rgb.png"), frame.rgb_bgr)
            cv2.imwrite(str(samples_dir / f"frame{idx:04d}_semantic.png"), colorize(pred))

    assert frame_area is not None

    relevant_ids = list(MP_CATEGORIES_MAPPING)
    relevant_stats = {
        name: {
            "mp3d_class_id": class_id,
            "frames_present": int(frames_with_class[class_id]),
            "frame_fraction": round(float(frames_with_class[class_id]) / num_frames, 4),
            "mean_area_fraction_when_present": (
                round(float(pixels_per_class[class_id]) / (frame_area * frames_with_class[class_id]), 5)
                if frames_with_class[class_id] > 0 else 0.0
            ),
        }
        for name, class_id in zip(HM3D_CATEGORY_NAMES, relevant_ids)
    }

    top_classes_by_pixels = sorted(
        ((int(cid), int(pixels_per_class[cid])) for cid in range(1, NUM_MP3D_CLASSES + 1)),
        key=lambda kv: -kv[1],
    )[:15]

    summary = {
        "num_frames": num_frames,
        "frame_hw": list(frame_hw) if frame_hw else None,
        "class0_no_valid_depth_area_fraction": round(
            float(pixels_per_class[0]) / (frame_area * num_frames), 5),
        "relevant_15_categories": relevant_stats,
        "top15_raw_mp3d_classes_by_total_pixels": [
            {"mp3d_class_id": cid, "total_pixels": px,
             "area_fraction_of_all_frames": round(px / (frame_area * num_frames), 5)}
            for cid, px in top_classes_by_pixels
        ],
        "sample_frame_indices": sorted(sample_indices),
    }
    (args.output / "domain_gap_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=False) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
