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
final local authority.  The wrapper also applies Yunji's measured circular
geometry when requested.  The pinned scorer ignores ``RobotConfig.shape`` and
tests four square corners even for a circle, inflating Yunji's 0.283 m body to
a 0.400 m corner radius before obstacle dilation.  Yunji therefore uses the
equivalent exact circle-vs-ESDF test at each source trajectory center.  WSJ's
rectangular Go2 footprint continues to use the pinned source scorer unchanged.
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
    *,
    classification: str = "observed_source_esdf_scores",
) -> dict[str, object]:
    """Summarize the selected ESDF scorer's results."""

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
        "classification": classification,
        "candidate_count": int(len(score_array)),
        "finite_candidate_count": int(np.count_nonzero(finite)),
        "finite_in_place_candidate_count": int(
            np.count_nonzero(finite & in_place)
        ),
        "all_candidates_in_collision": bool(not np.any(finite)),
    }


def score_circular_trajectories_by_esdf(
    trajectories,
    esdf_map,
    origin,
    resolution,
    safety_radius=0.1,
    front_len=0.35,
    rear_len=0.35,
    half_w=0.35,
):
    """Score a measured circular footprint against the source ESDF.

    Distance to the closest occupied cell is already encoded by the ESDF.
    For a circle, subtracting its measured radius from the center clearance is
    exact and orientation-independent.  This preserves the source trajectory
    lattice, stopped prefixes, collision threshold, safety-margin cost and
    closest-step decay while avoiding the source scorer's square-corner
    approximation.  Invalid or out-of-map geometry fails closed.
    """

    import numpy as np

    trajectory_array = np.asarray(trajectories, dtype=np.float64)
    esdf_array = np.asarray(esdf_map, dtype=np.float64)
    origin_array = np.asarray(origin, dtype=np.float64)
    geometry = np.asarray(
        [resolution, safety_radius, front_len, rear_len, half_w],
        dtype=np.float64,
    )
    if (
        trajectory_array.ndim != 3
        or trajectory_array.shape[0] == 0
        or trajectory_array.shape[1] == 0
        or trajectory_array.shape[2] < 2
        or esdf_array.ndim != 2
        or esdf_array.size == 0
        or origin_array.ndim != 1
        or len(origin_array) < 2
    ):
        raise ValueError("circular trajectory scorer inputs are incompatible")
    if (
        not np.all(np.isfinite(geometry))
        or not np.all(np.isfinite(origin_array[:2]))
        or resolution <= 0.0
        or safety_radius < 0.0
        or min(front_len, rear_len, half_w) <= 0.0
    ):
        raise ValueError("circular trajectory scorer geometry is invalid")
    body_radius = float(front_len)
    if not (
        math.isclose(body_radius, float(rear_len), abs_tol=1e-9)
        and math.isclose(body_radius, float(half_w), abs_tol=1e-9)
    ):
        raise ValueError(
            "circular trajectory scorer requires one measured body radius"
        )

    rows, columns = esdf_array.shape
    scores: list[float] = []
    occupied_points: list[int] = []
    for trajectory in trajectory_array:
        minimum_body_clearance = float("inf")
        closest_step = -1
        invalid_or_outside = False
        for step_index, pose in enumerate(trajectory):
            x_world = float(pose[0])
            y_world = float(pose[1])
            if not math.isfinite(x_world) or not math.isfinite(y_world):
                invalid_or_outside = True
                closest_step = step_index
                break
            x_grid = (x_world - float(origin_array[0])) / resolution
            y_grid = (y_world - float(origin_array[1])) / resolution
            if (
                x_grid < 0.0
                or y_grid < 0.0
                or x_grid >= rows
                or y_grid >= columns
            ):
                invalid_or_outside = True
                closest_step = step_index
                break
            center_clearance = float(
                esdf_array[int(x_grid), int(y_grid)]
            )
            if not math.isfinite(center_clearance) or center_clearance < 0.0:
                invalid_or_outside = True
                closest_step = step_index
                break
            body_clearance = center_clearance - body_radius
            if body_clearance < minimum_body_clearance:
                minimum_body_clearance = body_clearance
                closest_step = step_index

        if (
            invalid_or_outside
            or closest_step < 0
            or minimum_body_clearance <= 0.0
        ):
            scores.append(float("inf"))
        elif minimum_body_clearance > safety_radius:
            scores.append(0.0)
        else:
            decay_factor = (
                len(trajectory) - closest_step
            ) / len(trajectory)
            scores.append(
                decay_factor / (minimum_body_clearance + 1e-3)
            )
        occupied_points.append(closest_step)
    return scores, occupied_points


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

    def score_with_observability(*score_args, **score_kwargs):
        nonlocal last_score_state, last_score_log_monotonic
        if args.robot_profile == "yunji-water":
            scores, occupied_points = (
                score_circular_trajectories_by_esdf(
                    *score_args, **score_kwargs
                )
            )
            score_classification = (
                "deployment_measured_circle_esdf_scores"
            )
        else:
            scores, occupied_points = source_trajectory_scorer(
                *score_args, **score_kwargs
            )
            score_classification = "observed_source_esdf_scores"
        # The immutable scorer receives trajectories first; parameters are
        # unavailable there.  Preserve behavior and report the collision
        # state using the wrapper's most recently generated lattice.
        parameters = latest_parameters[0]
        if parameters is None:
            raise RuntimeError(
                "source scorer ran before trajectory generation"
            )
        summary = trajectory_score_summary(
            scores,
            parameters,
            classification=score_classification,
        )
        summary["robot_profile"] = args.robot_profile
        if args.robot_profile == "yunji-water":
            summary["measured_body_radius_m"] = args.body_radius_m
            summary["safety_margin_m"] = args.safety_margin_m
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
            "schema_version": "focus-progress-capable-tinynav-planner-v5",
            "adaptation": (
                "source_reverse_and_exact_stationary_vocabularies_removed_"
                "with_collision_scored_stopped_prefixes_and_measured_"
                "circular_yunji_esdf"
            ),
            "in_place_turns_preserved": True,
            "original_trajectory_lattice_preserved_first": True,
            "stopped_prefix_horizons_s": STOPPED_PREFIX_HORIZONS_S,
            "trajectory_scorer": (
                "measured_circle_center_clearance_against_source_esdf"
                if args.robot_profile == "yunji-water"
                else "pinned_source_rectangle_footprint_esdf"
            ),
            "source_esdf_unchanged": True,
            "source_rectangle_scorer_unchanged_for_wsj": True,
            "yunji_body_radius_m": (
                args.body_radius_m
                if args.robot_profile == "yunji-water"
                else None
            ),
            "yunji_safety_margin_m": (
                args.safety_margin_m
                if args.robot_profile == "yunji-water"
                else None
            ),
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
