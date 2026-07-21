#!/usr/bin/env python3
"""Pseudo-dual G4 machinery rehearsal (NOT a G4 pass).

Splits one real recorded session into two halves and treats them as two
virtual robots that share the TinyNav world of the same session, so the
shared frame is trivially identity — explicitly labelled TEST.  What this
exercises is the full G4 *machinery* on real data:

  two per-robot RedNet maps on one common grid
    -> element-wise max fusion (source-derived)
    -> frontier extraction on the fused map
    -> sequential GLM-4V allocation: robot-0 chooses, candidate removed,
       robot-1 chooses from the remainder (distinct targets guaranteed)
    -> two dry-run decision payloads written to disk.

G4 itself still requires a second physical robot and an independently
verified per-robot ``T_shared_world_robot_map``.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))
sys.path.insert(0, str(WORKSPACE / "dependencies"))
sys.path.insert(0, str(WORKSPACE / "source" / "Focus_realworld"))

from focus_hub.central_mapping import (  # noqa: E402
    CentralMapper, MapperConfig, RedNetSegmenter, estimate_floor_z,
)
from focus_hub.frontiers import extract_frontiers, render_annotated_bev  # noqa: E402
from focus_hub.fusion import allocate_frontiers_sequential, fuse_grids  # noqa: E402
from focus_hub.models import Decision  # noqa: E402
from focus_hub.tinynav_replay import TinyNavReplayReader  # noqa: E402
from focus_hub.vlm_decision import choose_frontier_fallback, choose_frontier_glm  # noqa: E402

TRANSFORM_VERSION = "g4-pseudo-test-v1"   # explicit test label, never production


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--extracted", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--glm-url", default=None, help="omit to use the recorded fallback")
    parser.add_argument("--goal-category", default="chair")
    args = parser.parse_args()

    if args.output.exists():
        print(f"refusing to overwrite existing output: {args.output}", file=sys.stderr)
        return 2
    args.output.mkdir(parents=True)

    reader = TinyNavReplayReader(args.record, args.extracted)
    half = len(reader.timestamps) // 2
    split = {
        "robot-0": set(reader.timestamps[:half]),
        "robot-1": set(reader.timestamps[half:]),
    }

    # One common grid covering the whole session (shared frame = identity TEST).
    translations = np.array([reader.poses[t][:3, 3] for t in reader.timestamps])
    margin = MapperConfig().max_range_m + 1.0
    min_xy = translations[:, :2].min(axis=0) - margin
    max_xy = translations[:, :2].max(axis=0) + margin
    config = MapperConfig(map_size_m=float(np.ceil(max(max_xy - min_xy))))
    origin = (float(min_xy[0]), float(min_xy[1]))
    floor_z = estimate_floor_z(reader.frames(), reader.calibration.K_infra1, config)

    segmenter = RedNetSegmenter(WORKSPACE / "artifacts" / "checkpoints" / "rednet_semmap_mp3d_40.pth")
    mappers = {
        robot_id: CentralMapper(
            config=config,
            K_infra1=reader.calibration.K_infra1,
            K_rgb=reader.calibration.K_rgb,
            T_rgb_to_infra1=reader.calibration.T_rgb_to_infra1,
            origin_xy_m=origin,
            floor_z_m=floor_z,
        )
        for robot_id in split
    }
    last_xy: dict[str, tuple[float, float]] = {}
    for frame in reader.frames():
        robot_id = "robot-0" if frame.timestamp_ns in split["robot-0"] else "robot-1"
        pred = segmenter.segment(frame.rgb_bgr, frame.depth_m)
        mappers[robot_id].integrate(frame, pred)
        last_xy[robot_id] = (float(frame.T_world_infra1[0, 3]), float(frame.T_world_infra1[1, 3]))

    fused = fuse_grids([mappers["robot-0"].map.grid, mappers["robot-1"].map.grid])
    per_robot_cells = {
        robot_id: {
            "explored": int((mapper.map.grid[1] > 0.5).sum()),
            "obstacle": int((mapper.map.grid[0] > 0.5).sum()),
        }
        for robot_id, mapper in mappers.items()
    }
    fused_cells = {
        "explored": int((fused[1] > 0.5).sum()),
        "obstacle": int((fused[0] > 0.5).sum()),
    }
    # Fusion sanity: max-fusion can only grow coverage.
    assert fused_cells["explored"] >= max(c["explored"] for c in per_robot_cells.values())

    frontiers = extract_frontiers(fused, origin, config.resolution_m)
    if len(frontiers) < 2:
        print("fewer than two frontier candidates; cannot allocate distinct targets",
              file=sys.stderr)
        return 1

    import cv2

    reference_mapper = mappers["robot-0"]

    def choose(robot_id: str, remaining):
        rc = None
        if robot_id in last_xy:
            row, col = reference_mapper.map.world_to_cell(
                np.array([last_xy[robot_id][0]]), np.array([last_xy[robot_id][1]]))
            rc = (int(row[0]), int(col[0]))
        bev = render_annotated_bev(fused, remaining, rc)
        cv2.imwrite(str(args.output / f"bev_{robot_id}.png"), bev)
        if args.glm_url:
            return choose_frontier_glm(bev, remaining, base_url=args.glm_url,
                                       goal_category=args.goal_category)
        return choose_frontier_fallback(remaining)

    allocations = allocate_frontiers_sequential(["robot-0", "robot-1"], frontiers, choose)
    if len(allocations) < 2:
        print("allocation did not produce two targets", file=sys.stderr)
        return 1
    assert allocations[0].frontier.frontier_id != allocations[1].frontier.frontier_id

    now_ns = time.time_ns()
    decisions = {}
    for allocation in allocations:
        decision = Decision(
            robot_id=allocation.robot_id,
            decision_id=f"g4-pseudo-{allocation.robot_id}-{now_ns}",
            mode="HOLD",   # dry-run: GOAL stays policy-blocked pre-G5
            map_version=1,
            transform_version=TRANSFORM_VERSION,
            issued_at_ns=now_ns,
            expires_at_ns=now_ns + 30_000_000_000,
            frontier_id=allocation.frontier.frontier_id,
            reason=(f"pseudo-dual rehearsal: {allocation.source} allocated frontier "
                    f"{allocation.frontier.frontier_id} at "
                    f"({allocation.frontier.x_m:.2f}, {allocation.frontier.y_m:.2f})"),
        )
        decisions[allocation.robot_id] = json.loads(decision.model_dump_json())

    np.savez_compressed(args.output / "fused_map.npz", grid=fused,
                        origin_xy_m=np.array(origin), resolution_m=np.array(config.resolution_m))
    manifest = {
        "transform_version": TRANSFORM_VERSION,
        "note": "shared frame is identity because both halves come from one session; "
                "this is machinery rehearsal only, not G4 calibration evidence",
        "frames_per_robot": {k: len(v) for k, v in split.items()},
        "per_robot_cells": per_robot_cells,
        "fused_cells": fused_cells,
        "frontiers": [
            {"id": f.frontier_id, "x_m": round(f.x_m, 3), "y_m": round(f.y_m, 3),
             "size_cells": f.size_cells} for f in frontiers
        ],
        "allocations": [
            {"robot_id": a.robot_id, "frontier_id": a.frontier.frontier_id,
             "source": a.source, "probabilities": a.probabilities}
            for a in allocations
        ],
        "decisions": decisions,
    }
    (args.output / "g4_pseudo_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "per_robot_cells": per_robot_cells,
        "fused_cells": fused_cells,
        "allocations": [(a.robot_id, a.frontier.frontier_id, a.source) for a in allocations],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
