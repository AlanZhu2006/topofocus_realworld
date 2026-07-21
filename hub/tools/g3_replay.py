#!/usr/bin/env python3
"""G3 gate: replay one recorded robot session through central RedNet mapping.

Runs the whole pipeline (extracted TinyNav record -> RedNet segmentation ->
world-frame semantic BEV map) ``--runs`` times and verifies the fused map is
bit-identical across runs.  Writes, under --output:

  run1/central_map.npz     fused grid + metadata arrays
  run1/obstacle.png, explored.png, semantic.png   previews
  g3_run_manifest.json     input hashes, config, per-run map hashes, timings

Mapping only: nothing here talks to a robot or issues commands.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))
sys.path.insert(0, str(WORKSPACE / "dependencies"))
# RedNet's model file imports the upstream `utils` package.
sys.path.insert(0, str(WORKSPACE / "source" / "Focus_realworld"))

from focus_hub.central_mapping import (  # noqa: E402
    HM3D_CATEGORY_NAMES,
    CentralMapper,
    MapperConfig,
    RedNetSegmenter,
    estimate_floor_z,
)
from focus_hub.tinynav_replay import TinyNavReplayReader  # noqa: E402


def map_digest(grid: np.ndarray, meta: dict) -> str:
    h = hashlib.sha256()
    h.update(grid.tobytes())
    h.update(json.dumps(meta, sort_keys=True).encode("utf-8"))
    return h.hexdigest()


def save_previews(out_dir: Path, grid: np.ndarray) -> None:
    import cv2

    def to_u8(a: np.ndarray) -> np.ndarray:
        return np.flipud((np.clip(a, 0.0, 1.0) * 255).astype(np.uint8))

    cv2.imwrite(str(out_dir / "obstacle.png"), to_u8(grid[0]))
    cv2.imwrite(str(out_dir / "explored.png"), to_u8(grid[1]))
    sem = grid[2:]
    argmax = sem.argmax(axis=0).astype(np.uint8)
    strength = sem.max(axis=0)
    hue = (argmax.astype(np.float32) * (179.0 / max(1, sem.shape[0]))).astype(np.uint8)
    hsv = np.stack(
        (hue, np.full_like(hue, 255), (np.clip(strength, 0, 1) * 255).astype(np.uint8)),
        axis=-1,
    )
    cv2.imwrite(str(out_dir / "semantic.png"), np.flipud(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)))


def run_once(
    reader: TinyNavReplayReader,
    segmenter: RedNetSegmenter,
    config: MapperConfig,
    origin_xy: tuple[float, float],
    floor_z: float,
) -> tuple[np.ndarray, dict]:
    mapper = CentralMapper(
        config=config,
        K_infra1=reader.calibration.K_infra1,
        K_rgb=reader.calibration.K_rgb,
        T_rgb_to_infra1=reader.calibration.T_rgb_to_infra1,
        origin_xy_m=origin_xy,
        floor_z_m=floor_z,
    )
    t0 = time.perf_counter()
    for frame in reader.frames():
        pred = segmenter.segment(frame.rgb_bgr, frame.depth_m)
        mapper.integrate(frame, pred)
    elapsed = time.perf_counter() - t0
    stats = {
        "frames_fused": mapper.map.frames_fused,
        "elapsed_s": round(elapsed, 3),
        "obstacle_cells": int((mapper.map.grid[0] > 0.5).sum()),
        "explored_cells": int((mapper.map.grid[1] > 0.5).sum()),
        "semantic_cells_per_category": {
            name: int((mapper.map.grid[2 + i] > 0.5).sum())
            for i, name in enumerate(HM3D_CATEGORY_NAMES)
        },
    }
    return mapper.map.grid, stats


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--extracted", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--runs", type=int, default=2)
    parser.add_argument("--rednet-checkpoint", type=Path,
                        default=WORKSPACE / "artifacts" / "checkpoints" / "rednet_semmap_mp3d_40.pth")
    args = parser.parse_args()

    if args.output.exists():
        print(f"refusing to overwrite existing output: {args.output}", file=sys.stderr)
        return 2
    args.output.mkdir(parents=True)

    reader = TinyNavReplayReader(args.record, args.extracted)
    config = MapperConfig()

    # Deterministic map extent: trajectory bounding box plus sensor range.
    translations = np.array([reader.poses[t][:3, 3] for t in reader.timestamps])
    margin = config.max_range_m + 1.0
    min_xy = translations[:, :2].min(axis=0) - margin
    max_xy = translations[:, :2].max(axis=0) + margin
    span = float(max(max_xy - min_xy))
    config = MapperConfig(map_size_m=float(np.ceil(span)))
    origin_xy = (float(min_xy[0]), float(min_xy[1]))

    floor_z = estimate_floor_z(reader.frames(), reader.calibration.K_infra1, config)
    print(f"frames={len(reader)} map_size={config.map_size_m} m "
          f"origin={origin_xy} floor_z={floor_z:.3f} m")

    segmenter = RedNetSegmenter(args.rednet_checkpoint, device=args.device)

    meta = {
        "record_dir": str(args.record.resolve()),
        "extracted_manifest_sha256": hashlib.sha256(
            (args.extracted / "manifest.json").read_bytes()
        ).hexdigest(),
        "config": {k: getattr(config, k) for k in config.__dataclass_fields__},
        "origin_xy_m": list(origin_xy),
        "floor_z_m": floor_z,
        "category_names": list(HM3D_CATEGORY_NAMES),
        "num_keyframes": len(reader),
    }

    digests: list[str] = []
    all_stats: list[dict] = []
    for run in range(1, args.runs + 1):
        grid, stats = run_once(reader, segmenter, config, origin_xy, floor_z)
        digest = map_digest(grid, meta)
        digests.append(digest)
        all_stats.append(stats)
        print(f"run{run}: {stats['frames_fused']} frames in {stats['elapsed_s']}s, "
              f"map sha256 {digest[:16]}…")
        run_dir = args.output / f"run{run}"
        run_dir.mkdir()
        np.savez_compressed(
            run_dir / "central_map.npz",
            grid=grid,
            origin_xy_m=np.array(origin_xy),
            floor_z_m=np.array(floor_z),
            resolution_m=np.array(config.resolution_m),
        )
        save_previews(run_dir, grid)

    deterministic = len(set(digests)) == 1
    manifest = {
        "meta": meta,
        "runs": all_stats,
        "map_sha256": digests,
        "deterministic": deterministic,
    }
    with (args.output / "g3_run_manifest.json").open("w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)

    print(f"deterministic={deterministic}")
    return 0 if deterministic else 1


if __name__ == "__main__":
    raise SystemExit(main())
