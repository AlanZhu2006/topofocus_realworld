#!/usr/bin/env python3
"""Run the pinned TinyNav local planner as a forward-only deployment.

The pinned planner always appends one fixed ``-0.20 m/s`` reverse trajectory.
When any dilated obstacle falls within its front-clearance gate, that reverse
vocabulary is hard-selected even if a collision-free in-place turn exists.
Both deployed chassis paths reject reverse motion, so converting that source
trajectory into rotate-first recovery can otherwise make the two layers fight
forever.

Keep the pinned source immutable.  This wrapper removes only that one reverse
vocabulary before constructing ``PlanningNode``.  The source forward lattice
still contains zero-linear, in-place turns; its full footprint/ESDF scoring,
depth map, stale-input behavior and no-path stop remain final local authority.
It also applies Yunji's measured geometry when requested and preserves the
source Go2 geometry for WSJ.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path


ROBOT_PROFILES = ("yunji-water", "source-default")


def forward_only_predefined_trajectory_vocabularies(
    duration: float = 3.0,
    dt: float = 0.1,
    init_p=None,
    init_q=None,
):
    """Return a shape-compatible empty replacement for the reverse vocabulary."""

    import numpy as np

    values = (duration, dt)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("trajectory timing must be finite")
    if duration <= 0.0 or dt <= 0.0:
        raise ValueError("trajectory timing must be positive")
    num_steps = int(duration / dt) + 1
    return (
        np.empty((0, num_steps, 7), dtype=np.float64),
        np.empty((0, 2), dtype=np.float64),
    )


def planner_source_provenance(source_path: str | Path) -> dict[str, object]:
    """Describe the immutable planner file used by this deployment."""

    path = Path(source_path).resolve()
    payload = path.read_bytes()
    return {
        "classification": "observed_pinned_source",
        "source_path": str(path),
        "size_bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-profile",
        choices=ROBOT_PROFILES,
        default="yunji-water",
        help=(
            "apply measured Yunji geometry or retain the pinned source robot "
            "configuration"
        ),
    )
    parser.add_argument(
        "--body-radius-m",
        type=float,
        default=0.283,
        help=(
            "Yunji circumscribed body radius; 0.283 m is preserved from the "
            "existing robot-local reachability deployment"
        ),
    )
    parser.add_argument(
        "--camera-forward-m",
        type=float,
        default=0.23,
        help="operator-measured base-to-Odin forward offset",
    )
    parser.add_argument(
        "--safety-margin-m",
        type=float,
        default=0.05,
        help="additional planner ESDF margin outside the body",
    )
    return parser


def main() -> int:
    args, ros_args = build_parser().parse_known_args()
    if args.robot_profile == "yunji-water":
        values = (
            args.body_radius_m,
            args.camera_forward_m,
            args.safety_margin_m,
        )
        if not all(math.isfinite(value) for value in values):
            raise SystemExit("Yunji geometry must contain only finite values")
        if (
            not 0.15 <= args.body_radius_m <= 0.60
            or not 0.0 <= args.camera_forward_m <= args.body_radius_m
            or not 0.02 <= args.safety_margin_m <= 0.30
        ):
            raise SystemExit(
                "Yunji planner geometry is outside deployment bounds"
            )

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from tinynav.core import planning_node

    if args.robot_profile == "yunji-water":
        planning_node.GO2_CONFIG = planning_node.RobotConfig(
            name="yunji-water",
            shape="circle",
            radius=args.body_radius_m,
            camera_x=args.camera_forward_m,
            camera_y=0.0,
            control_x=0.0,
            control_y=0.0,
            safety_radius=args.safety_margin_m,
        )
    planning_node.generate_predefined_trajectory_vocabularies = (
        forward_only_predefined_trajectory_vocabularies
    )
    provenance = planner_source_provenance(planning_node.__file__)
    provenance.update(
        {
            "schema_version": "focus-forward-only-tinynav-planner-v1",
            "robot_profile": args.robot_profile,
            "adaptation": "source_reverse_vocabulary_removed",
            "forward_lattice_and_esdf_unchanged": True,
        }
    )
    print(json.dumps(provenance, sort_keys=True), flush=True)

    # Pass the remaining list explicitly; ``None`` would make rclpy re-read
    # this wrapper's already-consumed geometry flags from sys.argv.
    rclpy.init(args=ros_args)
    node = planning_node.PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
