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
constructing ``PlanningNode``.  It also adds stopped copies of each actionable
trajectory at short horizons.  The original three-second candidates stay
first and win in open space; a short prefix is available when the full
three-second arc eventually intersects an obstacle.  Every published prefix
is still scored in full by the source footprint/ESDF function, and a scene
with no collision-free prefix still stops.

Zero-linear, nonzero-angular in-place turns remain available.  The source
depth map, stale-input behavior and all-candidates-in-collision stop remain
final local authority.  The wrapper also applies Yunji's measured geometry
when requested and preserves the source Go2 geometry for WSJ.
"""
from __future__ import annotations

import argparse
from collections.abc import Callable
import hashlib
import json
import math
from pathlib import Path
import sys
import time


OVERLAY = Path(__file__).resolve().parent
if str(OVERLAY) not in sys.path:
    sys.path.insert(0, str(OVERLAY))

from tinynav_source_contract import (  # noqa: E402
    ROBOT_PROFILES,
    verify_tinynav_source,
)


SOURCE_DEFAULT_TRAJECTORY_DT_S = 0.1
STOPPED_PREFIX_HORIZONS_S = (0.5, 1.0, 2.0)
SCORE_OBSERVABILITY_INTERVAL_S = 30.0


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


def source_trajectory_dt_s(
    args: tuple[object, ...],
    kwargs: dict[str, object],
) -> float:
    """Resolve ``dt`` from the pinned generator's unchanged call contract."""

    value = (
        kwargs["dt"]
        if "dt" in kwargs
        else (
            args[2]
            if len(args) >= 3
            else SOURCE_DEFAULT_TRAJECTORY_DT_S
        )
    )
    try:
        dt_s = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("source trajectory dt must be numeric") from exc
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("source trajectory dt must be finite and positive")
    return dt_s


def append_stopped_prefix_trajectories(
    trajectories,
    parameters,
    *,
    dt_s: float,
    horizons_s: tuple[float, ...] = STOPPED_PREFIX_HORIZONS_S,
):
    """Append collision-scorable paths that stop at shorter safe horizons.

    A prefix retains the source trajectory through its horizon and repeats its
    final pose for the rest of the source-shaped array.  The immutable planner
    therefore evaluates the complete stopped path with its normal footprint
    and ESDF scorer, while its existing 10-step publication cadence remains
    shape-compatible.
    """

    import numpy as np

    trajectories = np.asarray(trajectories)
    parameters = np.asarray(parameters)
    if (
        trajectories.ndim != 3
        or trajectories.shape[2] != 7
        or trajectories.shape[1] < 2
        or parameters.ndim != 2
        or parameters.shape[1] != 2
        or trajectories.shape[0] != parameters.shape[0]
    ):
        raise ValueError("source trajectory lattice has incompatible shapes")
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("trajectory dt must be finite and positive")

    prefix_last_indices: list[int] = []
    for horizon_s in horizons_s:
        if not math.isfinite(horizon_s) or horizon_s <= 0.0:
            raise ValueError(
                "stopped-prefix horizons must be finite and positive"
            )
        last_index = max(1, int(round(horizon_s / dt_s)))
        if (
            last_index < trajectories.shape[1] - 1
            and last_index not in prefix_last_indices
        ):
            prefix_last_indices.append(last_index)

    if not prefix_last_indices:
        return trajectories, parameters

    trajectory_groups = [trajectories]
    parameter_groups = [parameters]
    for last_index in prefix_last_indices:
        stopped = trajectories.copy()
        stopped[:, last_index + 1 :, :] = stopped[
            :, last_index : last_index + 1, :
        ]
        trajectory_groups.append(stopped)
        parameter_groups.append(parameters.copy())
    return (
        np.concatenate(trajectory_groups, axis=0),
        np.concatenate(parameter_groups, axis=0),
    )


def progress_capable_trajectory_library(
    source_generator: Callable[..., tuple[object, object]],
    *args,
    stopped_prefix_horizons_s: tuple[float, ...] = (
        STOPPED_PREFIX_HORIZONS_S
    ),
    **kwargs,
):
    """Generate the pinned lattice and add safe stopped-prefix candidates."""

    trajectories, parameters = source_generator(*args, **kwargs)
    trajectories, parameters = remove_stationary_trajectory_candidate(
        trajectories,
        parameters,
    )
    return append_stopped_prefix_trajectories(
        trajectories,
        parameters,
        dt_s=source_trajectory_dt_s(args, kwargs),
        horizons_s=stopped_prefix_horizons_s,
    )


def trajectory_score_summary(
    scores,
    parameters,
) -> dict[str, object]:
    """Summarize source ESDF results without changing planner selection."""

    import numpy as np

    score_array = np.asarray(scores, dtype=np.float64)
    parameter_array = np.asarray(parameters, dtype=np.float64)
    if (
        score_array.ndim != 1
        or parameter_array.ndim != 2
        or parameter_array.shape[1] != 2
        or len(score_array) != len(parameter_array)
        or len(score_array) == 0
    ):
        raise ValueError("source trajectory scores have incompatible shapes")
    finite = np.isfinite(score_array)
    in_place = np.abs(parameter_array[:, 0]) <= 1e-12
    return {
        "event": "focus_planner_candidate_scores",
        "classification": "observed_source_esdf_scores",
        "candidate_count": int(len(score_array)),
        "finite_candidate_count": int(np.count_nonzero(finite)),
        "finite_in_place_candidate_count": int(
            np.count_nonzero(finite & in_place)
        ),
        "all_candidates_in_collision": bool(not np.any(finite)),
    }


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
    source_trajectory_scorer = planning_node.score_trajectories_by_ESDF
    latest_parameters: list[object | None] = [None]
    last_score_state: tuple[bool, bool] | None = None
    last_score_log_monotonic = 0.0

    def score_with_observability(*args, **kwargs):
        nonlocal last_score_state, last_score_log_monotonic
        scores, occupied_points = source_trajectory_scorer(*args, **kwargs)
        # The immutable scorer receives trajectories first; parameters are
        # unavailable there.  Preserve behavior and report the collision
        # state using the wrapper's most recently generated lattice.
        parameters = latest_parameters[0]
        if parameters is None:
            raise RuntimeError(
                "source scorer ran before trajectory generation"
            )
        summary = trajectory_score_summary(scores, parameters)
        state = (
            bool(summary["all_candidates_in_collision"]),
            int(summary["finite_in_place_candidate_count"]) > 0,
        )
        now = time.monotonic()
        if (
            state != last_score_state
            or now - last_score_log_monotonic
            >= SCORE_OBSERVABILITY_INTERVAL_S
        ):
            print(json.dumps(summary, sort_keys=True), flush=True)
            last_score_state = state
            last_score_log_monotonic = now
        return scores, occupied_points

    def generate_observable_trajectory_library(*args, **kwargs):
        trajectories, parameters = progress_capable_trajectory_library(
            source_trajectory_generator,
            *args,
            **kwargs,
        )
        latest_parameters[0] = parameters
        return trajectories, parameters

    planning_node.generate_trajectory_library_3d = (
        generate_observable_trajectory_library
    )
    planning_node.score_trajectories_by_ESDF = score_with_observability
    provenance = verify_tinynav_source(
        planning_node.__file__,
        robot_profile=args.robot_profile,
        component="planner",
    )
    provenance.update(
        {
            "schema_version": "focus-progress-capable-tinynav-planner-v4",
            "adaptation": (
                "source_reverse_and_exact_stationary_vocabularies_removed_"
                "with_source_scored_stopped_prefixes"
            ),
            "in_place_turns_preserved": True,
            "original_trajectory_lattice_preserved_first": True,
            "stopped_prefix_horizons_s": STOPPED_PREFIX_HORIZONS_S,
            "source_footprint_and_esdf_scorer_unchanged": True,
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
