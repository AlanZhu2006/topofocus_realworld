#!/usr/bin/env python3
"""Deployment wrapper for TinyNav's pinned velocity controller.

The pinned TinyNav controller exposes ``linear_engage_threshold`` but compares
the requested speed against ``min_effective_linear_speed`` instead.  A valid
short trajectory segment can therefore request, for example, 0.078 m/s: above
the intended 0.04 m/s engage threshold but below the 0.10 m/s static-friction
floor.  Its timer then publishes zero forever.

The pinned controller already forbids negative velocity and turns in place
when its lookahead lies behind the robot.  The Focus deployment normally keeps
the stricter reverse-path rejection at 2 cm.  A measured robot deployment can
explicitly opt into a bounded rotate-first recovery: publish zero linear
velocity, retain one turn direction, and let the pinned pose/path/depth guards
govern the yaw command.  It can also explicitly stabilize a large in-place
turn from the current base pose toward a non-local path point.  That prevents a
jittering first path segment from changing the turn sign on every replan.  If
either recovery does not resolve before the bounded deadline, the existing
receiver rejection remains fail-closed.

Rotation-only paths can also contain a minute negative base translation from
the measured camera lever arm.  When that specific geometry coincides with a
non-local heading error, the same opt-in stabilizer performs a bounded,
zero-linear heading alignment instead of repeatedly suppressing both velocity
components.

Keep the source controller immutable.  The two pinned source histories do not
have identical guards, so this shared subclass adds the common denominator:
exact source verification, stale-pose/path stops and a bounded pose-jump
freeze.  Source-specific depth, turn, acceleration and arrival guards remain
authoritative where present.  Rotate-first remains an explicit launcher
option.
"""
from __future__ import annotations

import argparse
import json
import logging
import math
from pathlib import Path
import sys
import time


OVERLAY = Path(__file__).resolve().parent
if str(OVERLAY) not in sys.path:
    sys.path.insert(0, str(OVERLAY))
HUB_SRC = OVERLAY.parent / "src"
if str(HUB_SRC) not in sys.path:
    sys.path.insert(0, str(HUB_SRC))

from focus_hub.base_camera_calibration import (  # noqa: E402
    load_base_camera_calibration,
)
from focus_hub.shadow_coordination import (  # noqa: E402
    shared_base_pose_from_camera,
)
from tinynav_source_contract import (  # noqa: E402
    ROBOT_PROFILES,
    verify_tinynav_source,
)


MEANINGFUL_REVERSE_SEGMENT_M = 0.02
MINIMUM_DISTINCT_PATH_POSE_M = 0.01
MINIMUM_DISTINCT_PATH_POSE_ROTATION_RAD = math.radians(1.0)
DEFAULT_LINEAR_COMMAND_FLOOR_MPS = 0.18
MAX_DEPLOYMENT_LINEAR_COMMAND_FLOOR_MPS = 0.20
DEFAULT_ROTATE_FIRST_MAX_ANGULAR_RADPS = 0.35
DEFAULT_ROTATE_FIRST_MIN_ANGULAR_RADPS = 0.10
DEFAULT_ROTATE_FIRST_TIMEOUT_S = 12.0
DEFAULT_STABLE_TURN_LOOKAHEAD_M = 0.35
# A collision-scored trajectory shorter than the robot-scale lookahead does
# not carry a reliable route direction: the measured Scene 03 wall-front
# failure repeatedly exposed 5--20 cm trajectory prefixes that crossed the
# body axis between replans.  Keep the full local path authoritative once it
# reaches 0.30 m, and otherwise fall back to the fixed router waypoint.
DEFAULT_STABLE_PATH_MIN_TARGET_M = 0.30
DEFAULT_ROUTER_TURN_MIN_TARGET_M = 0.10
DEFAULT_STABLE_TURN_ENTER_RAD = math.radians(75.0)
DEFAULT_STABLE_TURN_EXIT_RAD = math.radians(35.0)
# TinyNav also emits pure-yaw commands for ordinary path alignment.  Those
# turns need the same direction latch as a large rotate-first recovery, but
# only after a real non-zero angular request.  The 15/8-degree hysteresis
# matches the source controller's yaw deadband and prevents successive local
# trajectory samples from alternating the turn sign.
DEFAULT_PATH_TURN_ENTER_RAD = math.radians(15.0)
DEFAULT_PATH_TURN_EXIT_RAD = math.radians(8.0)
DEFAULT_TINY_REVERSE_ALIGNMENT_ENTER_RAD = math.radians(15.0)
DEFAULT_TINY_REVERSE_ALIGNMENT_EXIT_RAD = math.radians(8.0)
DEFAULT_TINY_REVERSE_ALIGNMENT_GAIN = 0.5
DEFAULT_SOURCE_ARRIVAL_FRESHNESS_S = 1.0
DEFAULT_ROUTER_TARGET_TIMEOUT_S = 2.0
DEFAULT_CONTROLLER_POSE_TIMEOUT_S = 0.8
DEFAULT_CONTROLLER_PATH_TIMEOUT_S = 1.0
DEFAULT_CONTROLLER_POSE_JUMP_M = 0.40
DEFAULT_CONTROLLER_POSE_JUMP_FREEZE_S = 0.60
EXPECTED_CONTROLLER_PATH_FRAME = "world"
DEFAULT_CONTROLLER_PAUSE_SERVICE = "/focus/set_navigation_paused"


class DegenerateControllerPathError(ValueError):
    """A well-formed path contains no meaningful translation or rotation."""


def trajectory_contract_hold_reason(error: ValueError) -> str:
    """Classify invalid geometry without falsely claiming reverse motion."""

    if isinstance(error, DegenerateControllerPathError):
        return "trajectory_degenerate_hold"
    return f"trajectory_contract_invalid:{error}"


def validate_controller_path_message(
    message: object,
    *,
    expected_frame: str = EXPECTED_CONTROLLER_PATH_FRAME,
) -> int:
    """Validate every pose before the immutable controller consumes a path."""

    if message is None:
        raise ValueError("trajectory message is missing")
    poses = getattr(message, "poses", None)
    if poses is None:
        raise ValueError("trajectory has no poses field")
    frame_id = str(getattr(getattr(message, "header", None), "frame_id", ""))
    if frame_id != expected_frame:
        raise ValueError(
            f"trajectory frame {frame_id!r} != {expected_frame!r}"
        )
    if len(poses) < 2:
        raise ValueError("trajectory has fewer than two poses")
    for index, pose_stamped in enumerate(poses):
        try:
            pose = pose_stamped.pose
            position = pose.position
            orientation = pose.orientation
            values = (
                float(position.x),
                float(position.y),
                float(position.z),
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            )
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError(
                f"trajectory pose {index} is malformed"
            ) from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                f"trajectory pose {index} contains a non-finite value"
            )
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        if quaternion_norm < 1e-9:
            raise ValueError(
                f"trajectory pose {index} has a zero quaternion"
            )
    return len(poses)


def distinct_controller_path_pose_indices(
    message: object,
    *,
    minimum_translation_m: float = MINIMUM_DISTINCT_PATH_POSE_M,
    minimum_rotation_rad: float = (
        MINIMUM_DISTINCT_PATH_POSE_ROTATION_RAD
    ),
) -> tuple[int, ...]:
    """Drop consecutive duplicate transforms before source control.

    TinyNav can repeat its measured start pose before the first traversed grid
    cell. Passing that duplicate pair to the pinned controller makes its first
    control segment look like a tiny negative translation even when the
    remaining collision-scored path is forward. Removing only poses whose
    translation *and* rotation are both negligible preserves legitimate
    rotate-in-place trajectories while ensuring the immutable controller sees
    the first meaningful segment.
    """

    if (
        not math.isfinite(minimum_translation_m)
        or minimum_translation_m <= 0.0
    ):
        raise ValueError(
            "minimum path-pose translation must be positive"
        )
    if (
        not math.isfinite(minimum_rotation_rad)
        or minimum_rotation_rad <= 0.0
    ):
        raise ValueError("minimum path-pose rotation must be positive")
    poses = getattr(message, "poses", None)
    if poses is None or len(poses) < 2:
        raise ValueError("trajectory must contain at least two poses")

    def transform_values(pose_stamped: object) -> tuple[
        tuple[float, float, float],
        tuple[float, float, float, float],
    ]:
        pose = pose_stamped.pose
        position = pose.position
        orientation = pose.orientation
        translation = (
            float(position.x),
            float(position.y),
            float(position.z),
        )
        quaternion = (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        norm = math.sqrt(sum(value * value for value in quaternion))
        return translation, tuple(value / norm for value in quaternion)

    selected = [0]
    previous_translation, previous_quaternion = transform_values(poses[0])
    for index, pose_stamped in enumerate(poses[1:], start=1):
        translation, quaternion = transform_values(pose_stamped)
        translation_distance = math.sqrt(
            sum(
                (current - previous) ** 2
                for current, previous in zip(
                    translation, previous_translation
                )
            )
        )
        quaternion_dot = min(
            1.0,
            abs(
                sum(
                    current * previous
                    for current, previous in zip(
                        quaternion, previous_quaternion
                    )
                )
            ),
        )
        rotation_distance = 2.0 * math.acos(quaternion_dot)
        if (
            translation_distance < minimum_translation_m
            and rotation_distance < minimum_rotation_rad
        ):
            continue
        selected.append(index)
        previous_translation = translation
        previous_quaternion = quaternion
    if len(selected) < 2:
        raise DegenerateControllerPathError(
            "trajectory has fewer than two geometrically distinct poses"
        )
    return tuple(selected)


def command_components_finite(message: object) -> bool:
    """Return whether all six Twist components are finite."""

    try:
        values = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.linear.z),
            float(message.angular.x),
            float(message.angular.y),
            float(message.angular.z),
        )
    except (AttributeError, TypeError, ValueError):
        return False
    return all(math.isfinite(value) for value in values)


def path_segment_forward_component(
    first_xy: tuple[float, float],
    second_xy: tuple[float, float],
    *,
    robot_heading_rad: float,
) -> float:
    """Project the first path segment onto the predicted robot forward axis."""

    values = (*first_xy, *second_xy, robot_heading_rad)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("trajectory segment values must be finite")
    dx = second_xy[0] - first_xy[0]
    dy = second_xy[1] - first_xy[1]
    return dx * math.cos(robot_heading_rad) + dy * math.sin(
        robot_heading_rad
    )


def apply_linear_engagement_floor(
    requested_mps: float,
    *,
    engage_threshold_mps: float,
    minimum_effective_mps: float,
) -> float:
    """Raise an intentional small forward command to the executable floor."""

    values = (
        requested_mps,
        engage_threshold_mps,
        minimum_effective_mps,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("velocity thresholds must be finite")
    if (
        engage_threshold_mps < 0.0
        or minimum_effective_mps <= 0.0
        or engage_threshold_mps > minimum_effective_mps
    ):
        raise ValueError("invalid velocity engagement thresholds")
    if engage_threshold_mps <= requested_mps < minimum_effective_mps:
        return minimum_effective_mps
    return requested_mps


def classify_forward_component(
    forward_m: float | None,
    *,
    meaningful_reverse_m: float = MEANINGFUL_REVERSE_SEGMENT_M,
) -> str:
    """Classify a segment before TinyNav quantizes reverse to -0.2 m/s."""

    if not math.isfinite(meaningful_reverse_m) or meaningful_reverse_m <= 0.0:
        raise ValueError("meaningful_reverse_m must be finite and positive")
    if forward_m is None:
        return "unknown"
    if not math.isfinite(forward_m):
        raise ValueError("forward_m must be finite")
    if forward_m < -meaningful_reverse_m:
        return "reject_reverse"
    if forward_m < 0.0:
        return "zero_tiny_reverse"
    return "allow"


def bounded_rotate_first_angular(
    requested_radps: float,
    *,
    latched_direction: int,
    minimum_radps: float = DEFAULT_ROTATE_FIRST_MIN_ANGULAR_RADPS,
    maximum_radps: float = DEFAULT_ROTATE_FIRST_MAX_ANGULAR_RADPS,
) -> float:
    """Return a non-reversing yaw command with a stable turn direction.

    ``requested_radps`` comes from the pinned controller after all of its
    trajectory geometry and arrival handling.  A zero request remains zero so
    this wrapper cannot invent rotation after the pinned controller has
    stopped.  Once recovery begins, ``latched_direction`` prevents a
    near-180-degree heading error from switching sign on successive replans.
    """

    values = (requested_radps, minimum_radps, maximum_radps)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("rotate-first angular values must be finite")
    if minimum_radps <= 0.0 or maximum_radps < minimum_radps:
        raise ValueError("invalid rotate-first angular bounds")
    if latched_direction not in {-1, 0, 1}:
        raise ValueError("latched_direction must be -1, 0 or 1")
    if requested_radps == 0.0:
        return 0.0
    direction = (
        latched_direction
        if latched_direction
        else (1 if requested_radps > 0.0 else -1)
    )
    magnitude = min(
        maximum_radps,
        max(minimum_radps, abs(requested_radps)),
    )
    return float(direction * magnitude)


def stable_path_heading_error(
    robot_xy: tuple[float, float],
    *,
    robot_heading_rad: float,
    path_xy: list[tuple[float, float]],
    lookahead_m: float = DEFAULT_STABLE_TURN_LOOKAHEAD_M,
    minimum_target_m: float = DEFAULT_STABLE_PATH_MIN_TARGET_M,
) -> float | None:
    """Return a stable base-frame heading error to a non-local path point.

    TinyNav's pinned controller derives its large-turn sign from the first
    short path segment.  A replanner can legitimately move that segment across
    the base axis on successive callbacks.  Selecting the first point at least
    ``lookahead_m`` from the *current base pose* keeps the route's local shape
    while filtering that near-pose jitter.  A shorter path uses its farthest
    point only when it reaches the robot-scale reliability horizon; otherwise
    the caller can use the fixed router waypoint.
    """

    values = (*robot_xy, robot_heading_rad, lookahead_m, minimum_target_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("stable-turn geometry values must be finite")
    if lookahead_m <= 0.0 or minimum_target_m <= 0.0:
        raise ValueError("stable-turn distances must be positive")
    if minimum_target_m > lookahead_m:
        raise ValueError("minimum_target_m cannot exceed lookahead_m")

    chosen: tuple[float, float] | None = None
    farthest: tuple[float, float] | None = None
    farthest_distance = -1.0
    for point in path_xy:
        if len(point) != 2 or not all(math.isfinite(value) for value in point):
            raise ValueError("stable-turn path points must be finite XY pairs")
        dx = point[0] - robot_xy[0]
        dy = point[1] - robot_xy[1]
        distance = math.hypot(dx, dy)
        if distance > farthest_distance:
            farthest = point
            farthest_distance = distance
        if distance >= lookahead_m:
            chosen = point
            break
    if chosen is None:
        if farthest is None or farthest_distance < minimum_target_m:
            return None
        chosen = farthest

    world_bearing = math.atan2(
        chosen[1] - robot_xy[1],
        chosen[0] - robot_xy[0],
    )
    return math.atan2(
        math.sin(world_bearing - robot_heading_rad),
        math.cos(world_bearing - robot_heading_rad),
    )


def world_target_heading_error(
    robot_xy: tuple[float, float],
    target_xy: tuple[float, float],
    *,
    robot_heading_rad: float,
    minimum_target_m: float = DEFAULT_ROUTER_TURN_MIN_TARGET_M,
) -> float | None:
    """Return heading error to the router's fixed local waypoint."""

    values = (*robot_xy, *target_xy, robot_heading_rad, minimum_target_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("target-heading geometry values must be finite")
    if minimum_target_m <= 0.0:
        raise ValueError("minimum target distance must be positive")
    dx = target_xy[0] - robot_xy[0]
    dy = target_xy[1] - robot_xy[1]
    if math.hypot(dx, dy) < minimum_target_m:
        return None
    world_bearing = math.atan2(dy, dx)
    return math.atan2(
        math.sin(world_bearing - robot_heading_rad),
        math.cos(world_bearing - robot_heading_rad),
    )


def select_authoritative_heading_error(
    *,
    path_heading_error_rad: float | None,
    router_heading_error_rad: float | None,
) -> float | None:
    """Prefer the collision-scored path; use the router point as fallback."""

    values = (
        value
        for value in (path_heading_error_rad, router_heading_error_rad)
        if value is not None
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("heading errors must be finite")
    return (
        path_heading_error_rad
        if path_heading_error_rad is not None
        else router_heading_error_rad
    )


def large_turn_stabilization_required(
    heading_error_rad: float | None,
    *,
    recovery_active: bool,
    requested_linear_mps: float,
    requested_angular_radps: float,
    enter_rad: float = DEFAULT_STABLE_TURN_ENTER_RAD,
    exit_rad: float = DEFAULT_STABLE_TURN_EXIT_RAD,
) -> bool:
    """Return whether an existing pinned in-place turn needs sign latching."""

    values = (
        requested_linear_mps,
        requested_angular_radps,
        enter_rad,
        exit_rad,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("stable-turn command values must be finite")
    if heading_error_rad is not None and not math.isfinite(heading_error_rad):
        raise ValueError("stable-turn heading error must be finite")
    if not 0.0 < exit_rad < enter_rad <= math.pi:
        raise ValueError("stable-turn angular thresholds are invalid")
    if heading_error_rad is None:
        return False
    if recovery_active:
        # Do not let one intermittent zero/tiny-reverse lattice sample clear
        # an active turn while the authoritative path bearing remains
        # off-axis. The caller preserves the latched direction with the
        # bounded minimum yaw request; the existing recovery deadline still
        # rejects a turn that does not converge.
        return abs(heading_error_rad) >= exit_rad
    if (
        abs(requested_linear_mps) <= 1e-9
        and abs(requested_angular_radps) <= 1e-9
    ):
        return False
    return bool(
        abs(requested_linear_mps) <= 1e-9
        and abs(requested_angular_radps) > 1e-9
        and abs(heading_error_rad) >= enter_rad
    )


def path_turn_recovery_required(
    segment_action: str,
    heading_error_rad: float | None,
    *,
    recovery_active: bool,
    requested_linear_mps: float,
    requested_angular_radps: float,
) -> bool:
    """Stabilize a turn only on a positively traversable path segment."""

    if segment_action not in {
        "allow",
        "zero_tiny_reverse",
        "reject_reverse",
        "unknown",
    }:
        raise ValueError(f"unknown segment action: {segment_action}")
    if segment_action in {"reject_reverse", "zero_tiny_reverse"}:
        return False
    return large_turn_stabilization_required(
        heading_error_rad,
        recovery_active=recovery_active,
        requested_linear_mps=requested_linear_mps,
        requested_angular_radps=requested_angular_radps,
        enter_rad=DEFAULT_PATH_TURN_ENTER_RAD,
        exit_rad=DEFAULT_PATH_TURN_EXIT_RAD,
    )


def tiny_reverse_alignment_required(
    segment_action: str,
    heading_error_rad: float | None,
    *,
    recovery_active: bool,
    paused: bool,
    enter_rad: float = DEFAULT_TINY_REVERSE_ALIGNMENT_ENTER_RAD,
    exit_rad: float = DEFAULT_TINY_REVERSE_ALIGNMENT_EXIT_RAD,
) -> bool:
    """Break a zero-command path deadlock with bounded heading alignment.

    A rotation-only TinyNav path can acquire a minute negative translation
    from the measured camera-to-base lever arm.  The pinned controller then
    intermittently emits zero yaw, while treating the path as navigating.
    Permit zero-linear alignment only when the collision-scored path has this
    specific sub-threshold geometry and a non-local heading remains available.
    Hysteresis keeps the recovery continuous down to TinyNav's eight-degree
    yaw deadband.
    """

    if segment_action not in {
        "unknown",
        "reject_reverse",
        "zero_tiny_reverse",
        "allow",
    }:
        raise ValueError("unknown forward-component classification")
    if heading_error_rad is not None and not math.isfinite(heading_error_rad):
        raise ValueError("alignment heading error must be finite")
    if not all(math.isfinite(value) for value in (enter_rad, exit_rad)):
        raise ValueError("alignment angular thresholds must be finite")
    if not 0.0 < exit_rad < enter_rad <= math.pi:
        raise ValueError("alignment angular thresholds are invalid")
    if (
        paused
        or segment_action != "zero_tiny_reverse"
        or heading_error_rad is None
    ):
        return False
    threshold = exit_rad if recovery_active else enter_rad
    return abs(heading_error_rad) >= threshold


def bounded_heading_alignment_angular(
    heading_error_rad: float,
    *,
    gain: float = DEFAULT_TINY_REVERSE_ALIGNMENT_GAIN,
    minimum_radps: float = DEFAULT_ROTATE_FIRST_MIN_ANGULAR_RADPS,
    maximum_radps: float = DEFAULT_ROTATE_FIRST_MAX_ANGULAR_RADPS,
) -> float:
    """Return a tapered, zero-linear yaw request toward a stable heading."""

    values = (heading_error_rad, gain, minimum_radps, maximum_radps)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("heading-alignment values must be finite")
    if gain <= 0.0 or minimum_radps <= 0.0:
        raise ValueError("heading-alignment gain and minimum must be positive")
    if maximum_radps < minimum_radps:
        raise ValueError("heading-alignment angular bounds are invalid")
    if heading_error_rad == 0.0:
        return 0.0
    magnitude = min(
        maximum_radps,
        max(minimum_radps, gain * abs(heading_error_rad)),
    )
    return math.copysign(magnitude, heading_error_rad)


def latched_heading_target_crossed(
    heading_error_rad: float | None,
    *,
    latched_direction: int,
    crossing_window_rad: float = (
        DEFAULT_TINY_REVERSE_ALIGNMENT_ENTER_RAD
    ),
) -> bool:
    """Stop a latched alignment after a small, genuine zero crossing."""

    if heading_error_rad is not None and not math.isfinite(heading_error_rad):
        raise ValueError("crossing heading error must be finite")
    if latched_direction not in {-1, 0, 1}:
        raise ValueError("latched_direction must be -1, 0 or 1")
    if (
        not math.isfinite(crossing_window_rad)
        or not 0.0 < crossing_window_rad <= math.pi
    ):
        raise ValueError("crossing window is invalid")
    return bool(
        heading_error_rad is not None
        and latched_direction != 0
        and abs(heading_error_rad) < crossing_window_rad
        and heading_error_rad * latched_direction <= 0.0
    )


def source_arrival_stop_active(
    goal_distance_m: float | None,
    *,
    goal_distance_age_s: float,
    arrival_radius_m: float,
    freshness_s: float = DEFAULT_SOURCE_ARRIVAL_FRESHNESS_S,
) -> bool:
    """Preserve the pinned controller's fresh final-arrival stop."""

    values = (goal_distance_age_s, arrival_radius_m, freshness_s)
    if goal_distance_m is not None:
        values = (*values, goal_distance_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("source-arrival values must be finite")
    if goal_distance_age_s < 0.0:
        raise ValueError("source-arrival age cannot be negative")
    if arrival_radius_m <= 0.0 or freshness_s <= 0.0:
        raise ValueError("source-arrival limits must be positive")
    return bool(
        goal_distance_m is not None
        and goal_distance_age_s < freshness_s
        and goal_distance_m < arrival_radius_m
    )


def tiny_reverse_recovery_continuation_required(
    segment_action: str,
    heading_error_rad: float | None,
    *,
    recovery_active: bool,
    rotate_first_enabled: bool,
    paused: bool,
    exit_rad: float = DEFAULT_STABLE_TURN_EXIT_RAD,
) -> bool:
    """Keep an existing turn alive across the near-zero reverse deadband."""

    if segment_action not in {
        "unknown",
        "reject_reverse",
        "zero_tiny_reverse",
        "allow",
    }:
        raise ValueError("unknown forward-component classification")
    if heading_error_rad is not None and not math.isfinite(heading_error_rad):
        raise ValueError("continuation heading error must be finite")
    if not math.isfinite(exit_rad) or not 0.0 < exit_rad <= math.pi:
        raise ValueError("continuation exit angle is invalid")
    return bool(
        recovery_active
        and rotate_first_enabled
        and not paused
        and segment_action == "zero_tiny_reverse"
        and heading_error_rad is not None
        and abs(heading_error_rad) >= exit_rad
    )


def rotate_first_continuation_request(
    requested_radps: float,
    *,
    continuation_required: bool,
    latched_direction: int,
    minimum_radps: float = DEFAULT_ROTATE_FIRST_MIN_ANGULAR_RADPS,
) -> float:
    """Preserve an already-authorized turn when pinned output crosses zero."""

    if not all(
        math.isfinite(value) for value in (requested_radps, minimum_radps)
    ):
        raise ValueError("continuation angular values must be finite")
    if minimum_radps <= 0.0:
        raise ValueError("continuation minimum angular speed must be positive")
    if latched_direction not in {-1, 0, 1}:
        raise ValueError("latched_direction must be -1, 0 or 1")
    if (
        continuation_required
        and requested_radps == 0.0
        and latched_direction == 0
    ):
        raise ValueError("active continuation requires a latched direction")
    if continuation_required and requested_radps == 0.0:
        return float(latched_direction * minimum_radps)
    return requested_radps


def reverse_recovery_expired(
    *,
    started_monotonic: float,
    now_monotonic: float,
    timeout_s: float,
) -> bool:
    """Return whether one continuous rotate-first recovery exceeded its lease."""

    values = (started_monotonic, now_monotonic, timeout_s)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("reverse recovery timing values must be finite")
    if started_monotonic < 0.0 or now_monotonic < started_monotonic:
        raise ValueError("invalid reverse recovery monotonic timestamps")
    if timeout_s <= 0.0:
        raise ValueError("reverse recovery timeout must be positive")
    return now_monotonic - started_monotonic >= timeout_s


def controller_input_guard_reason(
    *,
    now_monotonic: float,
    pose_received_monotonic: float,
    path_received_monotonic: float,
    pose_jump_freeze_until_monotonic: float,
    paused: bool,
    pose_timeout_s: float = DEFAULT_CONTROLLER_POSE_TIMEOUT_S,
    path_timeout_s: float = DEFAULT_CONTROLLER_PATH_TIMEOUT_S,
) -> str | None:
    """Return the common fail-closed controller guard, if any."""

    values = (
        now_monotonic,
        pose_received_monotonic,
        path_received_monotonic,
        pose_jump_freeze_until_monotonic,
        pose_timeout_s,
        path_timeout_s,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("controller guard timing must be finite")
    if (
        now_monotonic < 0.0
        or pose_received_monotonic < 0.0
        or path_received_monotonic < 0.0
        or pose_jump_freeze_until_monotonic < 0.0
        or pose_timeout_s <= 0.0
        or path_timeout_s <= 0.0
    ):
        raise ValueError("controller guard timing is invalid")
    if paused:
        return "navigation_paused"
    if (
        pose_received_monotonic <= 0.0
        or now_monotonic - pose_received_monotonic > pose_timeout_s
    ):
        return "pose_missing_or_stale"
    if now_monotonic < pose_jump_freeze_until_monotonic:
        return "pose_jump_freeze"
    if (
        path_received_monotonic <= 0.0
        or now_monotonic - path_received_monotonic > path_timeout_s
    ):
        return "path_missing_or_stale"
    return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-profile",
        choices=ROBOT_PROFILES,
        required=True,
        help="select and verify the exact immutable TinyNav source contract",
    )
    parser.add_argument(
        "--robot-id",
        choices=("robot-0", "robot-1"),
        required=True,
        help="identity required by the measured base-camera artifact",
    )
    parser.add_argument(
        "--base-camera-frame",
        required=True,
        help="camera child frame required by the measured mount artifact",
    )
    parser.add_argument(
        "--base-camera-calibration-file",
        type=Path,
        required=True,
        help="measured base_link_T_camera artifact used for control geometry",
    )
    parser.add_argument(
        "--pause-service",
        default=DEFAULT_CONTROLLER_PAUSE_SERVICE,
        help=(
            "acknowledged local service used by the v2 receiver to change "
            "the source controller pause state"
        ),
    )
    parser.add_argument(
        "--rotate-first-on-reverse",
        action="store_true",
        help=(
            "replace a meaningful reverse lookahead with bounded in-place "
            "turning; requires an explicit measured-robot launcher opt-in"
        ),
    )
    parser.add_argument(
        "--stabilize-large-turn",
        action="store_true",
        help=(
            "latch the sign of an existing large in-place turn using the "
            "current base pose and a non-local path point; requires an "
            "explicit measured-robot launcher opt-in"
        ),
    )
    parser.add_argument(
        "--linear-command-floor-mps",
        type=float,
        default=DEFAULT_LINEAR_COMMAND_FLOOR_MPS,
        help=(
            "minimum intentional nonzero forward command after all pinned "
            "pose, path, depth and arrival guards"
        ),
    )
    parser.add_argument(
        "--rotate-first-max-angular-radps",
        type=float,
        default=DEFAULT_ROTATE_FIRST_MAX_ANGULAR_RADPS,
    )
    parser.add_argument(
        "--rotate-first-timeout-s",
        type=float,
        default=DEFAULT_ROTATE_FIRST_TIMEOUT_S,
    )
    parser.add_argument(
        "--pose-timeout-s",
        type=float,
        default=DEFAULT_CONTROLLER_POSE_TIMEOUT_S,
    )
    parser.add_argument(
        "--path-timeout-s",
        type=float,
        default=DEFAULT_CONTROLLER_PATH_TIMEOUT_S,
    )
    parser.add_argument(
        "--pose-jump-m",
        type=float,
        default=DEFAULT_CONTROLLER_POSE_JUMP_M,
    )
    parser.add_argument(
        "--pose-jump-freeze-s",
        type=float,
        default=DEFAULT_CONTROLLER_POSE_JUMP_FREEZE_S,
    )
    return parser


def main(args: list[str] | None = None) -> None:
    deployment_args, ros_args = build_parser().parse_known_args(args)
    if (
        not math.isfinite(deployment_args.linear_command_floor_mps)
        or deployment_args.linear_command_floor_mps <= 0.0
        or deployment_args.linear_command_floor_mps
        > MAX_DEPLOYMENT_LINEAR_COMMAND_FLOOR_MPS
    ):
        raise SystemExit(
            "--linear-command-floor-mps must be finite, positive and at "
            f"most {MAX_DEPLOYMENT_LINEAR_COMMAND_FLOOR_MPS:.2f}"
        )
    if (
        not math.isfinite(deployment_args.rotate_first_max_angular_radps)
        or deployment_args.rotate_first_max_angular_radps
        < DEFAULT_ROTATE_FIRST_MIN_ANGULAR_RADPS
    ):
        raise SystemExit(
            "--rotate-first-max-angular-radps must be finite and at least "
            f"{DEFAULT_ROTATE_FIRST_MIN_ANGULAR_RADPS:.2f}"
        )
    if (
        not math.isfinite(deployment_args.rotate_first_timeout_s)
        or deployment_args.rotate_first_timeout_s <= 0.0
    ):
        raise SystemExit(
            "--rotate-first-timeout-s must be finite and positive"
        )
    common_guard_values = (
        deployment_args.pose_timeout_s,
        deployment_args.path_timeout_s,
        deployment_args.pose_jump_m,
        deployment_args.pose_jump_freeze_s,
    )
    if (
        not all(math.isfinite(value) for value in common_guard_values)
        or min(common_guard_values) <= 0.0
    ):
        raise SystemExit(
            "controller pose/path guard limits must be finite and positive"
        )
    base_camera_calibration = load_base_camera_calibration(
        deployment_args.base_camera_calibration_file,
        expected_robot_id=deployment_args.robot_id,
        expected_camera_frame=deployment_args.base_camera_frame,
    )

    import numpy as np
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from scipy.spatial.transform import Rotation as R
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool
    from tinynav.platforms import cmd_vel_control as source_controller

    provenance = verify_tinynav_source(
        source_controller.__file__,
        robot_profile=deployment_args.robot_profile,
        component="controller",
    )
    provenance.update(
        {
            "schema_version": "focus-tinynav-controller-wrapper-v2",
            "adaptations": [
                "linear_engagement_floor",
                "bounded_deployment_linear_floor",
                "consecutive_duplicate_path_pose_filter",
                "reverse_segment_fail_closed",
                "stable_large_turn",
                "bounded_tiny_reverse_heading_alignment",
                "measured_base_pose_for_stable_heading",
                "common_pose_path_stale_stop",
                "common_pose_jump_freeze",
                "acknowledged_pause_service",
            ],
            "linear_command_floor_mps": (
                deployment_args.linear_command_floor_mps
            ),
            "base_camera_calibration": {
                "source_path": base_camera_calibration.source_path,
                "source_size_bytes": (
                    base_camera_calibration.source_size_bytes
                ),
                "source_sha256": base_camera_calibration.source_sha256,
                "measurement_status": (
                    base_camera_calibration.measurement_status
                ),
            },
        }
    )
    print(json.dumps(provenance, sort_keys=True), flush=True)
    CmdVelControlNode = source_controller.CmdVelControlNode

    class FocusCmdVelControlNode(CmdVelControlNode):
        def __init__(self) -> None:
            super().__init__()
            self._last_focus_speed_floor_log = 0.0
            self._last_focus_reverse_log = 0.0
            self._last_focus_rotate_first_log = 0.0
            self._last_focus_tiny_reverse_log = 0.0
            self._last_focus_input_guard_log = 0.0
            self._focus_rotation_recovery_started: float | None = None
            self._focus_rotation_turn_direction = 0
            self._focus_router_target_xy: tuple[float, float] | None = None
            self._focus_router_target_received_monotonic = 0.0
            self._focus_pose_received_monotonic = 0.0
            self._focus_path_received_monotonic = 0.0
            self._focus_pose_jump_freeze_until_monotonic = 0.0
            self._focus_last_pose_xy: tuple[float, float] | None = None
            self._focus_base_T_camera = np.asarray(
                base_camera_calibration.matrix,
                dtype=np.float64,
            ).reshape(4, 4)
            self._reverse_required_publisher = self.create_publisher(
                Bool, "/planning/reverse_required", 10
            )
            self.create_subscription(
                Odometry,
                "/control/target_pose",
                self._on_focus_router_target,
                10,
            )
            self._focus_pause_service = self.create_service(
                SetBool,
                deployment_args.pause_service,
                self._on_focus_set_paused,
            )

        def _reset_focus_rotation_recovery(self) -> None:
            self._focus_rotation_recovery_started = None
            self._focus_rotation_turn_direction = 0

        def _on_focus_router_target(self, message) -> None:
            position = message.pose.pose.position
            target = (float(position.x), float(position.y))
            if all(math.isfinite(value) for value in target):
                self._focus_router_target_xy = target
                self._focus_router_target_received_monotonic = time.monotonic()

        def _on_paused(self, message) -> None:
            super()._on_paused(message)
            if bool(message.data):
                self._reset_focus_rotation_recovery()
                # A path from the previous authority must not become live when
                # a later lease unpauses the controller.
                self._focus_path_received_monotonic = 0.0

        def _on_focus_set_paused(self, request, response):
            requested = bool(request.data)
            self._on_paused(Bool(data=requested))
            response.success = bool(self._paused) == requested
            response.message = (
                f"robot_id={deployment_args.robot_id};"
                f"paused={str(bool(self._paused)).lower()}"
            )
            return response

        def pose_callback(self, message) -> None:
            try:
                matrix = self._raw_pose_matrix(message.pose.pose)
            except (AttributeError, TypeError, ValueError):
                self._focus_pose_received_monotonic = 0.0
                self._reset_focus_rotation_recovery()
                self._publish_focus_guarded_zero("pose_geometry_invalid")
                return
            now = time.monotonic()
            current_base = shared_base_pose_from_camera(
                matrix,
                self._focus_base_T_camera,
            )
            current_xy = (
                float(current_base[0, 3]),
                float(current_base[1, 3]),
            )
            if (
                self._focus_last_pose_xy is not None
                and math.hypot(
                    current_xy[0] - self._focus_last_pose_xy[0],
                    current_xy[1] - self._focus_last_pose_xy[1],
                )
                > deployment_args.pose_jump_m
            ):
                self._focus_pose_jump_freeze_until_monotonic = max(
                    self._focus_pose_jump_freeze_until_monotonic,
                    now + deployment_args.pose_jump_freeze_s,
                )
                self._reset_focus_rotation_recovery()
            try:
                super().pose_callback(message)
            except (
                AttributeError,
                TypeError,
                ValueError,
                np.linalg.LinAlgError,
            ):
                self._focus_pose_received_monotonic = 0.0
                self._reset_focus_rotation_recovery()
                self._publish_focus_guarded_zero(
                    "source_pose_callback_failed"
                )
                return
            self._focus_last_pose_xy = current_xy
            self._focus_pose_received_monotonic = now

        def _publish_focus_guarded_zero(self, reason: str) -> None:
            self.latest_cmd = Twist()
            self.prev_cmd = Twist()
            self.cmd_pub.publish(Twist())
            now = time.monotonic()
            if now - self._last_focus_input_guard_log >= 2.0:
                self.get_logger().warning(
                    f"Focus TinyNav controller hold: {reason}"
                )
                self._last_focus_input_guard_log = now

        def cmd_timer_callback(self) -> None:
            now = time.monotonic()
            reason = controller_input_guard_reason(
                now_monotonic=now,
                pose_received_monotonic=(
                    self._focus_pose_received_monotonic
                ),
                path_received_monotonic=(
                    self._focus_path_received_monotonic
                ),
                pose_jump_freeze_until_monotonic=(
                    self._focus_pose_jump_freeze_until_monotonic
                ),
                paused=bool(self._paused),
                pose_timeout_s=deployment_args.pose_timeout_s,
                path_timeout_s=deployment_args.path_timeout_s,
            )
            if reason is not None:
                self._reset_focus_rotation_recovery()
                self._reverse_required_publisher.publish(Bool(data=False))
                self._publish_focus_guarded_zero(reason)
                return
            super().cmd_timer_callback()

        @staticmethod
        def _raw_pose_matrix(pose) -> np.ndarray:
            quaternion = pose.orientation
            values = (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
                float(quaternion.x),
                float(quaternion.y),
                float(quaternion.z),
                float(quaternion.w),
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError("pose contains a non-finite value")
            if math.sqrt(sum(value * value for value in values[3:])) < 1e-9:
                raise ValueError("pose quaternion has zero norm")
            matrix = np.eye(4, dtype=np.float64)
            matrix[:3, :3] = R.from_quat(
                list(values[3:])
            ).as_matrix()
            matrix[:3, 3] = list(values[:3])
            return matrix

        @classmethod
        def _pose_matrix(cls, pose_stamped) -> np.ndarray:
            return cls._raw_pose_matrix(pose_stamped.pose)

        def _control_segment_forward_m(self, message) -> float:
            first = message.poses[0]
            step_index = int(
                min(self.lookahead_steps, len(message.poses) - 1)
            )
            second = message.poses[step_index]
            first_robot = self._pose_matrix(first) @ self.T_robot_to_camera
            second_robot = self._pose_matrix(second) @ self.T_robot_to_camera
            # This is intentionally byte-for-byte equivalent in geometry to
            # the pinned controller's raw_vx calculation:
            # inv(T_robot_1) @ T_robot_2, translation X.  Testing only the
            # world-frame camera-point delta would miss the camera-to-base
            # lever arm and could disagree with the command actually rejected.
            relative = np.linalg.inv(first_robot) @ second_robot
            forward_m = float(relative[0, 3])
            if not math.isfinite(forward_m):
                raise ValueError(
                    "controller trajectory forward component is not finite"
                )
            return forward_m

        def _stable_path_heading_error(self, message) -> float | None:
            current_camera = self._raw_pose_matrix(self.pose.pose.pose)
            current_robot = shared_base_pose_from_camera(
                current_camera,
                self._focus_base_T_camera,
            )
            robot_xy = (
                float(current_robot[0, 3]),
                float(current_robot[1, 3]),
            )
            robot_heading_rad = math.atan2(
                float(current_robot[1, 0]),
                float(current_robot[0, 0]),
            )
            path_robot_xy = []
            for pose_stamped in message.poses:
                # TinyNav's planner initializes every trajectory at
                # camera_to_robot_center(T); path translation is therefore
                # already the robot center. Its orientation remains a camera
                # orientation and is intentionally ignored for these XY
                # lookahead bearings.
                position = pose_stamped.pose.position
                path_robot_xy.append(
                    (
                        float(position.x),
                        float(position.y),
                    )
                )
            # The collision-scored local path is authoritative.  A router
            # waypoint may intentionally sit behind the measured base while
            # the start seed is being repaired; letting that seed override a
            # valid forward path caused unnecessary full rotations.  Smooth
            # near-pose jitter with a path lookahead first, and consult the
            # fixed router waypoint only when the path is too short to define
            # a meaningful bearing.
            path_error = stable_path_heading_error(
                robot_xy,
                robot_heading_rad=robot_heading_rad,
                path_xy=path_robot_xy,
            )
            router_error = None
            if (
                self._focus_router_target_xy is not None
                and time.monotonic()
                - self._focus_router_target_received_monotonic
                <= DEFAULT_ROUTER_TARGET_TIMEOUT_S
            ):
                router_error = world_target_heading_error(
                    robot_xy,
                    self._focus_router_target_xy,
                    robot_heading_rad=robot_heading_rad,
                )
            return select_authoritative_heading_error(
                path_heading_error_rad=path_error,
                router_heading_error_rad=router_error,
            )

        def path_callback(self, message) -> None:
            try:
                validate_controller_path_message(message)
                distinct_indices = distinct_controller_path_pose_indices(
                    message
                )
            except ValueError as exc:
                self._focus_path_received_monotonic = 0.0
                self._reset_focus_rotation_recovery()
                # Invalid or stationary geometry must close velocity, but it
                # is not evidence that the path asks the robot to reverse.
                # Only a measured negative control segment below publishes
                # reverse_required=True.
                self._reverse_required_publisher.publish(Bool(data=False))
                self._publish_focus_guarded_zero(
                    trajectory_contract_hold_reason(exc)
                )
                return
            if len(distinct_indices) != len(message.poses):
                message.poses = [
                    message.poses[index] for index in distinct_indices
                ]
            if self.pose is None:
                self._focus_path_received_monotonic = 0.0
                self._reset_focus_rotation_recovery()
                self._reverse_required_publisher.publish(Bool(data=False))
                self._publish_focus_guarded_zero("pose_missing_for_path")
                return
            control_segment_forward_m = None
            stable_heading_error_rad = None
            geometry_error = False
            try:
                control_segment_forward_m = (
                    self._control_segment_forward_m(message)
                )
                stable_heading_error_rad = (
                    self._stable_path_heading_error(message)
                )
            except (
                AttributeError,
                TypeError,
                ValueError,
                np.linalg.LinAlgError,
            ):
                geometry_error = True
            if geometry_error or control_segment_forward_m is None:
                self._focus_path_received_monotonic = 0.0
                self._reset_focus_rotation_recovery()
                self._reverse_required_publisher.publish(Bool(data=False))
                self._publish_focus_guarded_zero(
                    "trajectory_geometry_invalid"
                )
                return
            try:
                super().path_callback(message)
            except (
                AttributeError,
                TypeError,
                ValueError,
                np.linalg.LinAlgError,
            ):
                self._focus_path_received_monotonic = 0.0
                self._reset_focus_rotation_recovery()
                self._reverse_required_publisher.publish(Bool(data=False))
                self._publish_focus_guarded_zero(
                    "source_path_callback_failed"
                )
                return
            if not command_components_finite(self.latest_cmd):
                self._focus_path_received_monotonic = 0.0
                self._reset_focus_rotation_recovery()
                self._reverse_required_publisher.publish(Bool(data=False))
                self._publish_focus_guarded_zero(
                    "source_controller_command_nonfinite"
                )
                return
            segment_action = classify_forward_component(
                control_segment_forward_m
            )
            requested_linear = float(self.latest_cmd.linear.x)
            requested_angular = float(self.latest_cmd.angular.z)
            now = time.monotonic()
            self._focus_path_received_monotonic = now
            source_goal_distance = getattr(self, "goal_dist", None)
            source_goal_distance_time = float(
                getattr(self, "goal_dist_time", 0.0)
            )
            source_arrival_radius = float(
                getattr(self, "arrival_radius", 0.5)
            )
            try:
                source_arrival_stop = source_arrival_stop_active(
                    (
                        None
                        if source_goal_distance is None
                        else float(source_goal_distance)
                    ),
                    goal_distance_age_s=max(
                        0.0,
                        now - source_goal_distance_time,
                    ),
                    arrival_radius_m=source_arrival_radius,
                )
            except (TypeError, ValueError):
                source_arrival_stop = True
            if source_arrival_stop:
                self._reset_focus_rotation_recovery()
                self._reverse_required_publisher.publish(Bool(data=False))
                self.latest_cmd = Twist()
                self.prev_cmd = Twist()
                self.cmd_pub.publish(Twist())
                return
            recovery_active = (
                self._focus_rotation_recovery_started is not None
            )
            reverse_recovery_requested = bool(
                segment_action == "reject_reverse"
                and deployment_args.rotate_first_on_reverse
                and not self._paused
            )
            large_turn_recovery_requested = bool(
                deployment_args.stabilize_large_turn
                and not self._paused
                and path_turn_recovery_required(
                    segment_action,
                    stable_heading_error_rad,
                    recovery_active=recovery_active,
                    requested_linear_mps=requested_linear,
                    requested_angular_radps=requested_angular,
                )
            )
            tiny_reverse_alignment_requested = bool(
                deployment_args.stabilize_large_turn
                and tiny_reverse_alignment_required(
                    segment_action,
                    stable_heading_error_rad,
                    recovery_active=recovery_active,
                    paused=bool(self._paused),
                )
            )
            alignment_target_crossed = bool(
                segment_action == "zero_tiny_reverse"
                and recovery_active
                and latched_heading_target_crossed(
                    stable_heading_error_rad,
                    latched_direction=(
                        self._focus_rotation_turn_direction
                    ),
                )
            )
            if alignment_target_crossed:
                tiny_reverse_alignment_requested = False
            tiny_reverse_recovery_requested = bool(
                not alignment_target_crossed
                and tiny_reverse_recovery_continuation_required(
                    segment_action,
                    stable_heading_error_rad,
                    recovery_active=recovery_active,
                    rotate_first_enabled=(
                        deployment_args.rotate_first_on_reverse
                    ),
                    paused=self._paused,
                )
            )
            if (
                reverse_recovery_requested
                or large_turn_recovery_requested
                or tiny_reverse_alignment_requested
                or tiny_reverse_recovery_requested
            ):
                if self._focus_rotation_recovery_started is None:
                    self._focus_rotation_recovery_started = now
                    if (
                        stable_heading_error_rad is not None
                        and abs(stable_heading_error_rad) > 1e-9
                    ):
                        self._focus_rotation_turn_direction = (
                            1 if stable_heading_error_rad > 0.0 else -1
                        )
                    elif requested_angular != 0.0:
                        self._focus_rotation_turn_direction = (
                            1 if requested_angular > 0.0 else -1
                        )
                expired = reverse_recovery_expired(
                    started_monotonic=self._focus_rotation_recovery_started,
                    now_monotonic=now,
                    timeout_s=deployment_args.rotate_first_timeout_s,
                )
                if tiny_reverse_alignment_requested:
                    continuation_request = (
                        bounded_heading_alignment_angular(
                            stable_heading_error_rad,
                            maximum_radps=(
                                deployment_args.rotate_first_max_angular_radps
                            ),
                        )
                    )
                else:
                    continuation_request = rotate_first_continuation_request(
                        requested_angular,
                        continuation_required=bool(
                            tiny_reverse_recovery_requested
                            or (
                                recovery_active
                                and large_turn_recovery_requested
                            )
                        ),
                        latched_direction=(
                            self._focus_rotation_turn_direction
                        ),
                    )
                rotate_angular = bounded_rotate_first_angular(
                    continuation_request,
                    latched_direction=self._focus_rotation_turn_direction,
                    maximum_radps=(
                        deployment_args.rotate_first_max_angular_radps
                    ),
                )
                if not expired and rotate_angular != 0.0:
                    # Do not publish directly here. The pinned timer keeps
                    # final authority for pause, stale pose/path,
                    # relocalization, depth and acceleration guards.
                    rotate_command = Twist()
                    rotate_command.angular.z = rotate_angular
                    self.latest_cmd = rotate_command
                    self.prev_cmd = Twist()
                    self._reverse_required_publisher.publish(Bool(data=False))
                    if now - self._last_focus_rotate_first_log >= 2.0:
                        elapsed = (
                            now - self._focus_rotation_recovery_started
                        )
                        context = (
                            "reverse_segment"
                            if reverse_recovery_requested
                            else (
                                "tiny_reverse_alignment"
                                if tiny_reverse_alignment_requested
                                else (
                                    "tiny_reverse_continuation"
                                    if tiny_reverse_recovery_requested
                                    else "large_turn"
                                )
                            )
                        )
                        heading_text = (
                            "unavailable"
                            if stable_heading_error_rad is None
                            else f"{math.degrees(stable_heading_error_rad):.1f}"
                        )
                        self.get_logger().warning(
                            "Focus TinyNav rotate-first recovery: "
                            f"context={context}, linear=0.000 m/s, "
                            f"angular={rotate_angular:.3f} rad/s, "
                            f"stable_heading_error_deg={heading_text}, "
                            f"forward_component="
                            f"{control_segment_forward_m:.3f} m, "
                            f"elapsed={elapsed:.2f} s"
                        )
                        self._last_focus_rotate_first_log = now
                    return
                # A recovery timeout is a controller failure even when a new
                # short path segment is nominally forward.  Reuse the existing
                # receiver-visible fail-closed rejection channel.
                self._reverse_required_publisher.publish(Bool(data=True))
                self.latest_cmd = Twist()
                self.prev_cmd = Twist()
                self.cmd_pub.publish(Twist())
                if now - self._last_focus_reverse_log >= 2.0:
                    self.get_logger().warning(
                        "Focus TinyNav rejected unresolved rotate-first "
                        "recovery: bounded timeout expired"
                    )
                    self._last_focus_reverse_log = now
                return

            if (
                segment_action == "reject_reverse"
                and deployment_args.rotate_first_on_reverse
                and self._paused
            ):
                # A path received while navigation is paused must not consume
                # the bounded recovery budget.
                self._reset_focus_rotation_recovery()
                self.latest_cmd = Twist()
                self.prev_cmd = Twist()
                self._reverse_required_publisher.publish(Bool(data=False))
                return
            if segment_action == "reject_reverse":
                # Opt-out or a zero-yaw request retains the original
                # fail-closed receiver rejection.
                self._reset_focus_rotation_recovery()
                self._reverse_required_publisher.publish(Bool(data=True))
                self.latest_cmd = Twist()
                self.prev_cmd = Twist()
                self.cmd_pub.publish(Twist())
                if now - self._last_focus_reverse_log >= 2.0:
                    self.get_logger().warning(
                        "Focus TinyNav rejected reverse control segment: "
                        f"forward_component={control_segment_forward_m:.3f} "
                        "m; rotate-first unavailable"
                    )
                    self._last_focus_reverse_log = now
                return

            self._reset_focus_rotation_recovery()
            self._reverse_required_publisher.publish(Bool(data=False))
            if segment_action == "zero_tiny_reverse":
                # A sub-threshold negative component is trajectory jitter, not
                # permission for either translation or recovery rotation.
                self.latest_cmd = Twist()
                self.prev_cmd = Twist()
                self.cmd_pub.publish(Twist())
                now = time.monotonic()
                if now - self._last_focus_tiny_reverse_log >= 2.0:
                    self.get_logger().info(
                        "Focus TinyNav suppressed tiny reverse segment: "
                        f"forward_component={control_segment_forward_m:.4f} m"
                    )
                    self._last_focus_tiny_reverse_log = now
                return
            requested = float(self.latest_cmd.linear.x)
            floored = apply_linear_engagement_floor(
                requested,
                engage_threshold_mps=float(self.linear_engage_threshold),
                minimum_effective_mps=max(
                    float(self.min_effective_linear_speed),
                    deployment_args.linear_command_floor_mps,
                ),
            )
            if floored == requested:
                return
            self.latest_cmd.linear.x = floored
            now = time.monotonic()
            if now - self._last_focus_speed_floor_log >= 2.0:
                self.get_logger().info(
                    "Focus TinyNav velocity floor: "
                    f"{requested:.3f} -> {floored:.3f} m/s"
                )
                self._last_focus_speed_floor_log = now

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(filename)s:%(lineno)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    rclpy.init(args=ros_args)
    node = FocusCmdVelControlNode()
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


if __name__ == "__main__":
    main()
