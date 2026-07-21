#!/usr/bin/env python3
"""Live, one-shot validation of the full Perception->Judgment->gate->Decision
VLM cascade against a real running GLM-4V server and real recorded data.

Builds a real central map from the G3 wsj recording (same data used
throughout this project's earlier validation), extracts real frontiers,
runs real YOLO detection on the last real RGB frame, and drives the full
cascade exactly as `hub_pipeline_daemon.py` would. Prints every
intermediate value (Perception_PR, Judgment_PR, gate decision, Decision
VLM output) so a human can sanity-check the whole chain actually works with
genuine GPU inference, not mocks.

Mapping/decision only: nothing here talks to a robot or issues commands.
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
sys.path.insert(0, str(WORKSPACE / "hub" / "tools"))

from focus_hub.central_mapping import (  # noqa: E402
    HM3D_CATEGORY_NAMES, CentralMapper, MapperConfig, RedNetSegmenter, estimate_floor_z,
)
from focus_hub.directional_memory import DirectionalMemory  # noqa: E402
from focus_hub.frontiers import extract_frontiers, render_semantic_decision_map  # noqa: E402
from focus_hub.tinynav_replay import TinyNavReplayReader  # noqa: E402
from focus_hub.vlm_decision import run_decision_cascade  # noqa: E402
from focus_hub.vlm_prompts import extract_scene_objects, format_scene_objects_for_prompt  # noqa: E402
from focus_hub.yolo_detector import YoloDetector  # noqa: E402
from hub_pipeline_daemon import heading_deg_from_pose  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path,
                        default=WORKSPACE / "data/robot_replays/wsj_semantic_map_record_20260717_102052")
    parser.add_argument("--extracted", type=Path,
                        default=WORKSPACE / "data/robot_replays/wsj_semantic_map_record_20260717_102052_extracted")
    parser.add_argument("--glm-url", required=True)
    parser.add_argument("--goal-category", default="chair")
    parser.add_argument("--yolo-weights", type=Path, default=WORKSPACE / "artifacts/vision/yolov10m.pt")
    parser.add_argument("--num-frames", type=int, default=60, help="how many keyframes to fuse into the map")
    args = parser.parse_args()

    reader = TinyNavReplayReader(args.record, args.extracted)
    config = MapperConfig()
    segmenter = RedNetSegmenter(WORKSPACE / "artifacts/checkpoints/rednet_semmap_mp3d_40.pth")

    frames = list(reader.frames())[:args.num_frames]
    translations = np.array([f.T_world_infra1[:3, 3] for f in frames])
    margin = config.max_range_m + 1.0
    min_xy = translations[:, :2].min(axis=0) - margin
    max_xy = translations[:, :2].max(axis=0) + margin
    config = MapperConfig(map_size_m=float(np.ceil(max(max_xy - min_xy))))
    origin = (float(min_xy[0]), float(min_xy[1]))
    floor_z = estimate_floor_z(iter(frames), reader.calibration.K_infra1, config)

    mapper = CentralMapper(
        config=config, K_infra1=reader.calibration.K_infra1, K_rgb=reader.calibration.K_rgb,
        T_rgb_to_infra1=reader.calibration.T_rgb_to_infra1, origin_xy_m=origin, floor_z_m=floor_z,
    )
    last_rgb = None
    last_T = None
    for frame in frames:
        pred = segmenter.segment(frame.rgb_bgr, frame.depth_m)
        mapper.integrate(frame, pred)
        last_rgb = frame.rgb_bgr
        last_T = frame.T_world_rgb

    grid = mapper.map.grid
    print(f"map built: {frames.__len__()} frames, explored={int((grid[1] > 0.5).sum())} cells, "
          f"obstacle={int((grid[0] > 0.5).sum())} cells")

    frontiers = extract_frontiers(grid, mapper.map.origin_xy_m, config.resolution_m)
    print(f"frontiers: {[(f.frontier_id, f.row, f.col, f.size_cells) for f in frontiers]}")
    if not frontiers:
        print("no frontiers found in this partial map — try --num-frames larger", file=sys.stderr)
        return 1

    row, col = mapper.map.world_to_cell(np.array([last_T[0, 3]]), np.array([last_T[1, 3]]))
    robot_rc = (int(row[0]), int(col[0]))
    heading = heading_deg_from_pose(last_T)
    print(f"robot_rc={robot_rc} heading_deg={heading:.1f}")

    print("loading YOLO...")
    yolo = YoloDetector(args.yolo_weights)
    detections = yolo.detect(last_rgb)
    print(f"YOLO detections on the latest real frame: {detections}")

    scene_objects_dict = extract_scene_objects(grid[2:2 + len(HM3D_CATEGORY_NAMES)], HM3D_CATEGORY_NAMES)
    scene_objects_str = format_scene_objects_for_prompt(scene_objects_dict)
    print(f"scene objects extracted from the map: {list(scene_objects_dict.keys())}")

    judgment_map = render_semantic_decision_map(
        grid, HM3D_CATEGORY_NAMES, frontiers, robot_rc, heading, history_nodes=[])
    decision_map = render_semantic_decision_map(grid, HM3D_CATEGORY_NAMES, frontiers, robot_rc, heading)

    memory = DirectionalMemory()
    print(f"\nrunning full cascade against {args.glm_url} ...")
    result = run_decision_cascade(
        rgb_bgr=last_rgb, judgment_map_bgr=judgment_map, decision_map_bgr=decision_map,
        frontiers=frontiers, target=args.goal_category, detections=detections,
        scene_objects=scene_objects_str, cur_location_rc=robot_rc, heading_deg=heading,
        pre_goal_point=None, step=1, early_episode_step_threshold=125, memory=memory,
        base_url=args.glm_url,
    )
    summary = {
        "perception_pr": result.perception_pr,
        "judgment_pr": result.judgment_pr,
        "gate_passed": result.gate_passed,
        "gate_reason": result.gate_reason,
        "frontier_chosen": result.frontier_choice.frontier.frontier_id if result.frontier_choice else None,
        "decision_probabilities": result.frontier_choice.probabilities if result.frontier_choice else None,
        "decision_raw_content": result.frontier_choice.raw_content if result.frontier_choice else None,
        "errors": result.errors,
    }
    print("\n=== CASCADE RESULT ===")
    print(json.dumps(summary, indent=2))
    return 0 if not result.errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
