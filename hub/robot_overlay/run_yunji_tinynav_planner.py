#!/usr/bin/env python3
"""Run the pinned TinyNav local planner as a progress-capable deployment.

The pinned planner always appends one fixed ``-0.20 m/s`` reverse trajectory.
When any dilated obstacle falls within its front-clearance gate, that reverse
vocabulary is hard-selected even if a collision-free in-place turn exists.
Both deployed chassis paths reject reverse motion, so converting that source
trajectory into rotate-first recovery can otherwise make the two layers fight
forever.

Keep the pinned source immutable.  This wrapper removes only that one reverse
vocabulary and the single exact ``(v=0, omega=0)`` lattice candidate before
constructing ``PlanningNode``.  Zero-linear, nonzero-angular in-place turns
remain available; the source footprint/ESDF scoring, depth map, stale-input
behavior and all-candidates-in-collision stop remain final local authority.
It also applies Yunji's measured geometry when requested and preserves the
source Go2 geometry for WSJ.
"""
from __future__ import annotations

import argparse
from collections.abc import Callable
import hashlib
import json
import math
from pathlib import Path
import sys


OVERLAY = Path(__file__).resolve().parent
if str(OVERLAY) not in sys.path:
    sys.path.insert(0, str(OVERLAY))

from tinynav_source_contract import (  # noqa: E402
    ROBOT_PROFILES,
    verify_tinynav_source,
)


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


def remove_stationary_trajectory_candidate(
    trajectories,
    parameters,
    *,
    zero_tolerance: float = 1e-12,
):
    """Remove only lattice rows with zero linear and angular velocity."""

    import numpy as np

    trajectories = np.asarray(trajectories)
    parameters = np.asarray(parameters)
    if (
        trajectories.ndim != 3
        or trajectories.shape[2] != 7
        or parameters.ndim != 2
        or parameters.shape[1] != 2
        or trajectories.shape[0] != parameters.shape[0]
    ):
        raise ValueError("source trajectory lattice has incompatible shapes")
    if (
        not math.isfinite(zero_tolerance)
        or zero_tolerance < 0.0
    ):
        raise ValueError("zero tolerance must be finite and non-negative")
    moving_or_turning = np.logical_or(
        np.abs(parameters[:, 0]) > zero_tolerance,
        np.abs(parameters[:, 1]) > zero_tolerance,
    )
    if not np.any(moving_or_turning):
        raise ValueError("source trajectory lattice contains no actionable row")
    return trajectories[moving_or_turning], parameters[moving_or_turning]


def progress_capable_trajectory_library(
    source_generator: Callable[..., tuple[object, object]],
    *args,
    **kwargs,
):
    """Generate the pinned lattice, then drop its exact no-action row."""

    trajectories, parameters = source_generator(*args, **kwargs)
    return remove_stationary_trajectory_candidate(
        trajectories,
        parameters,
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
        required=True,
        help=(
            "apply measured Yunji geometry or retain the pinned source robot "
            "configuration; also selects the exact immutable source contract"
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
    source_trajectory_generator = (
        planning_node.generate_trajectory_library_3d
    )

    def generate_progress_capable_trajectory_library(*args, **kwargs):
        return progress_capable_trajectory_library(
            source_trajectory_generator,
            *args,
            **kwargs,
        )

    planning_node.generate_trajectory_library_3d = (
        generate_progress_capable_trajectory_library
    )
    provenance = verify_tinynav_source(
        planning_node.__file__,
        robot_profile=args.robot_profile,
        component="planner",
    )
    provenance.update(
        {
            "schema_version": "focus-progress-capable-tinynav-planner-v3",
            "adaptation": (
                "source_reverse_and_exact_stationary_vocabularies_removed"
            ),
            "in_place_turns_preserved": True,
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
