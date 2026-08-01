import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "robot_overlay"
    / "yunji_tinynav_cmd_vel_control.py"
)
SPEC = importlib.util.spec_from_file_location(
    "yunji_tinynav_cmd_vel_control",
    MODULE_PATH,
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _deployment_args(
    robot_profile: str = "source-default",
) -> list[str]:
    return [
        "--robot-profile",
        robot_profile,
        "--robot-id",
        "robot-0",
        "--base-camera-frame",
        "camera",
        "--base-camera-calibration-file",
        "/tmp/measured-base-camera.json",
    ]


def _pose_stamped(
    x: float,
    y: float,
    *,
    quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
):
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=0.0),
            orientation=SimpleNamespace(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3],
            ),
        )
    )


def _path(*poses, frame_id: str = "world"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        poses=list(poses),
    )


def _twist(*, linear_x: float = 0.0, angular_z: float = 0.0):
    return SimpleNamespace(
        linear=SimpleNamespace(x=linear_x, y=0.0, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=angular_z),
    )


def test_controller_path_contract_accepts_only_finite_world_geometry():
    valid = _path(_pose_stamped(0.0, 0.0), _pose_stamped(0.2, 0.0))
    assert MODULE.validate_controller_path_message(valid) == 2

    with pytest.raises(ValueError, match="frame"):
        MODULE.validate_controller_path_message(
            _path(
                _pose_stamped(0.0, 0.0),
                _pose_stamped(0.2, 0.0),
                frame_id="map",
            )
        )
    with pytest.raises(ValueError, match="fewer than two"):
        MODULE.validate_controller_path_message(_path(_pose_stamped(0.0, 0.0)))
    with pytest.raises(ValueError, match="non-finite"):
        MODULE.validate_controller_path_message(
            _path(
                _pose_stamped(0.0, 0.0),
                _pose_stamped(float("nan"), 0.0),
            )
        )
    with pytest.raises(ValueError, match="zero quaternion"):
        MODULE.validate_controller_path_message(
            _path(
                _pose_stamped(0.0, 0.0),
                _pose_stamped(
                    0.2,
                    0.0,
                    quaternion=(0.0, 0.0, 0.0, 0.0),
                ),
            )
        )


def test_controller_path_filter_skips_duplicate_start_poses():
    path = _path(
        _pose_stamped(0.0, 0.0),
        _pose_stamped(0.0, 0.0),
        _pose_stamped(0.05, 0.0),
        _pose_stamped(0.10, 0.0),
    )

    assert MODULE.distinct_controller_path_pose_indices(path) == (0, 2, 3)


def test_controller_path_filter_rejects_all_duplicate_geometry():
    path = _path(
        _pose_stamped(0.0, 0.0),
        _pose_stamped(0.005, 0.0),
        _pose_stamped(0.009, 0.0),
    )

    with pytest.raises(
        MODULE.DegenerateControllerPathError,
        match="geometrically distinct",
    ) as raised:
        MODULE.distinct_controller_path_pose_indices(path)
    assert (
        MODULE.trajectory_contract_hold_reason(raised.value)
        == "trajectory_degenerate_hold"
    )


def test_malformed_path_hold_does_not_claim_reverse_motion():
    error = ValueError("trajectory frame 'map' != 'world'")

    assert MODULE.trajectory_contract_hold_reason(error) == (
        "trajectory_contract_invalid:" "trajectory frame 'map' != 'world'"
    )


def test_controller_path_filter_preserves_rotate_in_place_geometry():
    yaw = math.radians(10.0)
    path = _path(
        _pose_stamped(0.0, 0.0),
        _pose_stamped(
            0.0,
            0.0,
            quaternion=(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)),
        ),
    )

    assert MODULE.distinct_controller_path_pose_indices(path) == (0, 1)


def test_controller_rejects_nonfinite_twist_components():
    assert MODULE.command_components_finite(_twist(linear_x=0.1, angular_z=-0.2))
    assert not MODULE.command_components_finite(_twist(linear_x=float("nan")))


def test_small_intentional_forward_command_reaches_static_friction_floor():
    assert MODULE.apply_linear_engagement_floor(
        0.078,
        engage_threshold_mps=0.04,
        minimum_effective_mps=0.10,
    ) == pytest.approx(0.10)


def test_deployment_linear_floor_is_bounded_to_observed_source_maximum():
    parsed = MODULE.build_parser().parse_args(_deployment_args())

    assert parsed.linear_command_floor_mps == pytest.approx(0.18)
    assert MODULE.MAX_DEPLOYMENT_LINEAR_COMMAND_FLOOR_MPS == pytest.approx(0.20)
    assert MODULE.apply_linear_engagement_floor(
        0.10,
        engage_threshold_mps=0.04,
        minimum_effective_mps=parsed.linear_command_floor_mps,
    ) == pytest.approx(0.18)


def test_reverse_path_segment_is_negative_in_robot_forward_axis():
    assert MODULE.path_segment_forward_component(
        (0.90, 0.08),
        (0.70, 0.08),
        robot_heading_rad=0.0,
    ) == pytest.approx(-0.20)
    assert MODULE.path_segment_forward_component(
        (0.90, 0.08),
        (0.90, 0.28),
        robot_heading_rad=0.5 * math.pi,
    ) == pytest.approx(0.20)


@pytest.mark.parametrize(
    ("forward_m", "expected"),
    [
        (None, "unknown"),
        (0.0, "allow"),
        (0.08, "allow"),
        (-0.001, "zero_tiny_reverse"),
        (-0.02, "zero_tiny_reverse"),
        (-0.020001, "reject_reverse"),
        (-0.2, "reject_reverse"),
    ],
)
def test_tiny_negative_segments_cannot_become_fixed_reverse(
    forward_m,
    expected,
):
    assert MODULE.classify_forward_component(forward_m) == expected


def test_reverse_classifier_rejects_invalid_values():
    with pytest.raises(ValueError):
        MODULE.classify_forward_component(float("nan"))
    with pytest.raises(ValueError):
        MODULE.classify_forward_component(0.1, meaningful_reverse_m=0.0)


@pytest.mark.parametrize(
    ("segment_action", "requested_linear_mps", "expected_action"),
    [
        ("reject_reverse", 0.0, "allow"),
        ("reject_reverse", 0.18, "allow"),
        ("zero_tiny_reverse", 0.0, "zero_tiny_reverse"),
        ("allow", 0.18, "allow"),
    ],
)
def test_verified_forward_only_planner_treats_path_geometry_as_a_turn(
    segment_action,
    requested_linear_mps,
    expected_action,
):
    effective_action, violation = MODULE.resolve_forward_only_control_contract(
        segment_action,
        requested_linear_mps,
        verified_forward_only_planner=True,
    )

    assert effective_action == expected_action
    assert violation is False


def test_actual_negative_twist_remains_a_detectable_source_reverse_request():
    effective_action, violation = MODULE.resolve_forward_only_control_contract(
        "allow",
        -0.01,
        verified_forward_only_planner=True,
    )

    assert effective_action == "reject_reverse"
    assert violation is True


@pytest.mark.parametrize(
    (
        "reverse_requested",
        "heading_deg",
        "recovery_active",
        "rotate_first",
        "stabilize",
        "paused",
        "expected",
    ),
    [
        (False, 90.0, False, True, True, False, "none"),
        (True, 90.0, False, True, True, False, "align"),
        (True, 10.0, True, True, True, False, "align"),
        (True, 10.0, False, True, True, False, "hold"),
        (True, 5.0, True, True, True, False, "hold"),
        (True, None, False, True, True, False, "reject"),
        (True, 90.0, False, False, True, False, "reject"),
        (True, 90.0, False, True, False, False, "reject"),
        (True, 90.0, False, True, True, True, "hold"),
    ],
)
def test_verified_source_reverse_is_aligned_held_or_rejected_without_reverse(
    reverse_requested,
    heading_deg,
    recovery_active,
    rotate_first,
    stabilize,
    paused,
    expected,
):
    heading = None if heading_deg is None else math.radians(heading_deg)
    assert (
        MODULE.classify_verified_reverse_command_recovery(
            reverse_requested,
            heading,
            recovery_active=recovery_active,
            rotate_first_enabled=rotate_first,
            stabilize_large_turn=stabilize,
            paused=paused,
        )
        == expected
    )


def test_verified_source_reverse_recovery_rejects_invalid_heading():
    with pytest.raises(ValueError):
        MODULE.classify_verified_reverse_command_recovery(
            True,
            float("nan"),
            recovery_active=False,
            rotate_first_enabled=True,
            stabilize_large_turn=True,
            paused=False,
        )


def test_unverified_planner_retains_legacy_geometry_rejection():
    assert MODULE.resolve_forward_only_control_contract(
        "reject_reverse",
        0.0,
        verified_forward_only_planner=False,
    ) == ("reject_reverse", False)


def test_forward_only_turn_uses_absolute_bound_not_replanning_error_clock():
    assert MODULE.controller_recovery_timeout_is_terminal(
        absolute_timeout_expired=True,
        convergence_stalled=False,
        verified_forward_only_planner=True,
    )
    assert MODULE.controller_recovery_timeout_is_terminal(
        absolute_timeout_expired=False,
        convergence_stalled=True,
        verified_forward_only_planner=False,
    )
    assert not MODULE.controller_recovery_timeout_is_terminal(
        absolute_timeout_expired=False,
        convergence_stalled=True,
        verified_forward_only_planner=True,
    )
    assert not MODULE.controller_recovery_timeout_is_terminal(
        absolute_timeout_expired=False,
        convergence_stalled=True,
        verified_forward_only_planner=True,
        source_reverse_command=True,
    )
    with pytest.raises(ValueError):
        MODULE.controller_recovery_timeout_is_terminal(
            absolute_timeout_expired=1,
            convergence_stalled=False,
            verified_forward_only_planner=True,
        )


@pytest.mark.parametrize(
    ("requested", "latched_direction", "expected"),
    [
        (0.70, 0, 0.35),
        (-0.70, 0, -0.35),
        (-0.70, 1, 0.35),
        (0.70, -1, -0.35),
        (0.03, 1, 0.10),
        (0.0, 1, 0.0),
    ],
)
def test_rotate_first_is_zero_linear_compatible_and_direction_latched(
    requested,
    latched_direction,
    expected,
):
    assert MODULE.bounded_rotate_first_angular(
        requested,
        latched_direction=latched_direction,
    ) == pytest.approx(expected)


def test_rotate_first_rejects_invalid_angular_contract():
    with pytest.raises(ValueError):
        MODULE.bounded_rotate_first_angular(
            float("nan"),
            latched_direction=0,
        )
    with pytest.raises(ValueError):
        MODULE.bounded_rotate_first_angular(
            0.3,
            latched_direction=2,
        )
    with pytest.raises(ValueError):
        MODULE.bounded_rotate_first_angular(
            0.3,
            latched_direction=1,
            minimum_radps=0.4,
            maximum_radps=0.3,
        )


def test_rotate_first_has_a_strict_bounded_timeout():
    assert not MODULE.reverse_recovery_expired(
        started_monotonic=10.0,
        now_monotonic=21.999,
        timeout_s=12.0,
    )
    assert MODULE.reverse_recovery_expired(
        started_monotonic=10.0,
        now_monotonic=22.0,
        timeout_s=12.0,
    )
    with pytest.raises(ValueError):
        MODULE.reverse_recovery_expired(
            started_monotonic=10.0,
            now_monotonic=9.0,
            timeout_s=12.0,
        )


def test_stable_path_heading_ignores_near_pose_replan_jitter():
    left = MODULE.stable_path_heading_error(
        (0.0, 0.0),
        robot_heading_rad=0.0,
        path_xy=[(0.02, 0.01), (0.03, -0.02), (0.0, 0.60)],
    )
    right = MODULE.stable_path_heading_error(
        (0.0, 0.0),
        robot_heading_rad=0.0,
        path_xy=[(0.02, -0.01), (0.03, 0.02), (0.0, 0.60)],
    )

    assert left == pytest.approx(0.5 * math.pi)
    assert right == pytest.approx(0.5 * math.pi)


def test_planner_path_heading_reference_stays_in_the_path_frame():
    first_path_pose = [
        [0.0, -1.0, 0.0, 3.0],
        [1.0, 0.0, 0.0, 4.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    base_t_camera = [
        [1.0, 0.0, 0.0, 0.23],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.40],
        [0.0, 0.0, 0.0, 1.0],
    ]

    reference_xy, heading = MODULE.planner_path_base_reference(
        first_path_pose,
        base_t_camera,
    )

    # TinyNav's path translation is already the base centre, so the mount
    # translation must not be subtracted a second time.
    assert reference_xy == pytest.approx((3.0, 4.0))
    assert heading == pytest.approx(math.pi / 2.0)


def test_turn_progress_watchdog_allows_convergence_and_stops_stale_yaw():
    best, last_progress, stalled = MODULE.heading_recovery_progress_state(
        best_abs_error_rad=math.pi,
        last_progress_monotonic=10.0,
        current_error_rad=math.pi,
        now_monotonic=12.9,
    )
    assert not stalled

    best, last_progress, stalled = MODULE.heading_recovery_progress_state(
        best_abs_error_rad=best,
        last_progress_monotonic=last_progress,
        current_error_rad=math.pi - math.radians(6.0),
        now_monotonic=13.0,
    )
    assert not stalled
    assert last_progress == pytest.approx(13.0)

    _best, _last_progress, stalled = MODULE.heading_recovery_progress_state(
        best_abs_error_rad=best,
        last_progress_monotonic=last_progress,
        current_error_rad=math.pi - math.radians(6.0),
        now_monotonic=16.0,
    )
    assert stalled


def test_turn_progress_watchdog_accepts_slow_measurable_convergence():
    best = math.radians(136.0)
    last_progress = 10.0
    for now, heading_deg in (
        (12.5, 134.8),
        (15.0, 133.6),
        (17.5, 132.4),
    ):
        best, last_progress, stalled = MODULE.heading_recovery_progress_state(
            best_abs_error_rad=best,
            last_progress_monotonic=last_progress,
            current_error_rad=math.radians(heading_deg),
            now_monotonic=now,
        )
        assert not stalled
        assert last_progress == pytest.approx(now)


def test_collision_scored_path_heading_overrides_conflicting_router_seed():
    target_error = MODULE.world_target_heading_error(
        (2.8755, -6.0557),
        (3.225, -6.175),
        robot_heading_rad=0.5597,
    )
    reverse_path_error = MODULE.stable_path_heading_error(
        (2.8755, -6.0557),
        robot_heading_rad=0.5597,
        path_xy=[
            (2.8755, -6.0557),
            (2.8755 - 0.4 * math.cos(0.5597), -6.0557 - 0.4 * math.sin(0.5597)),
        ],
    )

    assert target_error is not None
    assert math.degrees(target_error) == pytest.approx(-50.8, abs=0.3)
    assert reverse_path_error is not None
    assert abs(math.degrees(reverse_path_error)) == pytest.approx(180.0)
    assert (
        MODULE.select_authoritative_heading_error(
            path_heading_error_rad=reverse_path_error,
            router_heading_error_rad=target_error,
        )
        == reverse_path_error
    )
    assert (
        MODULE.select_authoritative_heading_error(
            path_heading_error_rad=None,
            router_heading_error_rad=target_error,
        )
        == target_error
    )


def test_short_wall_front_paths_defer_to_stable_router_heading():
    robot_xy = (0.523, -4.1567)
    robot_heading_rad = math.radians(179.0)
    router_error = MODULE.world_target_heading_error(
        robot_xy,
        (-0.775, 0.775),
        robot_heading_rad=robot_heading_rad,
    )
    left_jitter = MODULE.stable_path_heading_error(
        robot_xy,
        robot_heading_rad=robot_heading_rad,
        path_xy=[
            robot_xy,
            (0.48, -4.13),
            (0.43, -4.10),
        ],
    )
    right_jitter = MODULE.stable_path_heading_error(
        robot_xy,
        robot_heading_rad=robot_heading_rad,
        path_xy=[
            robot_xy,
            (0.49, -4.19),
            (0.42, -4.24),
        ],
    )

    assert left_jitter is None
    assert right_jitter is None
    assert router_error is not None
    assert (
        MODULE.select_authoritative_heading_error(
            path_heading_error_rad=left_jitter,
            router_heading_error_rad=router_error,
        )
        == router_error
    )
    assert (
        MODULE.select_authoritative_heading_error(
            path_heading_error_rad=right_jitter,
            router_heading_error_rad=router_error,
        )
        == router_error
    )


def test_measured_rear_left_goal_has_one_stable_positive_turn():
    heading_error = MODULE.stable_path_heading_error(
        (0.376, -0.320),
        robot_heading_rad=-2.46,
        path_xy=[(0.39, -0.31), (4.75, -3.82)],
    )

    assert heading_error is not None
    assert math.degrees(heading_error) == pytest.approx(102.2, abs=0.2)
    assert MODULE.large_turn_stabilization_required(
        heading_error,
        recovery_active=False,
        requested_linear_mps=0.0,
        requested_angular_radps=-0.7,
    )
    assert MODULE.bounded_rotate_first_angular(
        -0.7,
        latched_direction=1,
    ) == pytest.approx(0.35)


def test_large_turn_latch_uses_hysteresis_only_after_explicit_entry():
    assert MODULE.large_turn_stabilization_required(
        math.radians(76.0),
        recovery_active=False,
        requested_linear_mps=0.0,
        requested_angular_radps=0.7,
    )
    assert MODULE.large_turn_stabilization_required(
        math.radians(40.0),
        recovery_active=True,
        requested_linear_mps=0.0,
        requested_angular_radps=-0.7,
    )
    assert not MODULE.large_turn_stabilization_required(
        math.radians(34.0),
        recovery_active=True,
        requested_linear_mps=0.0,
        requested_angular_radps=0.7,
    )
    assert not MODULE.large_turn_stabilization_required(
        math.radians(90.0),
        recovery_active=False,
        requested_linear_mps=0.0,
        requested_angular_radps=0.0,
    )
    assert MODULE.large_turn_stabilization_required(
        math.radians(40.0),
        recovery_active=True,
        requested_linear_mps=0.1,
        requested_angular_radps=0.0,
    )
    # A transient zero request must not erase a turn that already entered
    # through a real pinned-controller yaw command. The bounded continuation
    # path supplies the minimum yaw request until the 35-degree exit.
    assert MODULE.large_turn_stabilization_required(
        math.radians(40.0),
        recovery_active=True,
        requested_linear_mps=0.0,
        requested_angular_radps=0.0,
    )


def test_reverse_segment_cannot_enter_generic_large_turn_recovery():
    assert not MODULE.path_turn_recovery_required(
        "reject_reverse",
        math.pi,
        recovery_active=False,
        requested_linear_mps=0.0,
        requested_angular_radps=0.7,
    )
    assert not MODULE.path_turn_recovery_required(
        "zero_tiny_reverse",
        math.radians(90.0),
        recovery_active=False,
        requested_linear_mps=0.0,
        requested_angular_radps=0.7,
    )
    assert MODULE.path_turn_recovery_required(
        "allow",
        math.radians(90.0),
        recovery_active=False,
        requested_linear_mps=0.0,
        requested_angular_radps=0.7,
    )


def test_traversable_moderate_turn_latches_direction_until_yaw_deadband():
    # Scene 03 Formal 07 showed alternating source yaw signs at ordinary
    # 15--30 degree path alignment angles.  Once a real pure-yaw command
    # enters recovery, local trajectory jitter must not flip that direction.
    assert MODULE.path_turn_recovery_required(
        "allow",
        math.radians(20.0),
        recovery_active=False,
        requested_linear_mps=0.0,
        requested_angular_radps=0.3,
    )
    assert MODULE.path_turn_recovery_required(
        "allow",
        math.radians(10.0),
        recovery_active=True,
        requested_linear_mps=0.0,
        requested_angular_radps=-0.3,
    )
    assert not MODULE.path_turn_recovery_required(
        "allow",
        math.radians(7.9),
        recovery_active=True,
        requested_linear_mps=0.0,
        requested_angular_radps=-0.3,
    )
    assert not MODULE.path_turn_recovery_required(
        "allow",
        math.radians(20.0),
        recovery_active=False,
        requested_linear_mps=0.1,
        requested_angular_radps=0.3,
    )


def test_restart14_tiny_reverse_alignment_recovers_from_zero_yaw():
    # Observed robot-1 router-heading sequence after the source controller
    # entered its exact-zero/tiny-reverse deadlock in Scene 03 Restart 14.
    heading_degrees = (68.5, 59.4, 44.4, 34.5, 22.1)
    recovery_active = False
    commands = []
    for heading_degrees_value in heading_degrees:
        heading_error = math.radians(heading_degrees_value)
        assert MODULE.tiny_reverse_alignment_required(
            "zero_tiny_reverse",
            heading_error,
            recovery_active=recovery_active,
            paused=False,
        )
        commands.append(MODULE.bounded_heading_alignment_angular(heading_error))
        recovery_active = True

    assert commands[0] == pytest.approx(0.35)
    assert commands[-1] == pytest.approx(math.radians(22.1) * 0.5)
    assert all(0.10 <= command <= 0.35 for command in commands)
    assert not MODULE.tiny_reverse_alignment_required(
        "zero_tiny_reverse",
        math.radians(7.9),
        recovery_active=True,
        paused=False,
    )


@pytest.mark.parametrize(
    ("segment_action", "heading_error", "paused"),
    [
        ("allow", math.radians(65.0), False),
        ("reject_reverse", math.radians(65.0), False),
        ("zero_tiny_reverse", None, False),
        ("zero_tiny_reverse", math.radians(65.0), True),
    ],
)
def test_tiny_reverse_alignment_remains_narrowly_gated(
    segment_action,
    heading_error,
    paused,
):
    assert not MODULE.tiny_reverse_alignment_required(
        segment_action,
        heading_error,
        recovery_active=False,
        paused=paused,
    )


def test_latched_alignment_stops_only_after_a_small_target_crossing():
    assert MODULE.latched_heading_target_crossed(
        math.radians(-5.0),
        latched_direction=1,
    )
    assert not MODULE.latched_heading_target_crossed(
        math.radians(-179.0),
        latched_direction=1,
    )
    assert not MODULE.latched_heading_target_crossed(
        math.radians(5.0),
        latched_direction=1,
    )


def test_tiny_reverse_alignment_cannot_override_fresh_source_arrival():
    assert MODULE.source_arrival_stop_active(
        0.49,
        goal_distance_age_s=0.2,
        arrival_radius_m=0.5,
    )
    assert not MODULE.source_arrival_stop_active(
        0.51,
        goal_distance_age_s=0.2,
        arrival_radius_m=0.5,
    )
    assert not MODULE.source_arrival_stop_active(
        0.49,
        goal_distance_age_s=1.0,
        arrival_radius_m=0.5,
    )
    assert not MODULE.source_arrival_stop_active(
        None,
        goal_distance_age_s=100.0,
        arrival_radius_m=0.5,
    )


def test_active_rotate_first_crosses_tiny_reverse_deadband():
    assert MODULE.tiny_reverse_recovery_continuation_required(
        "zero_tiny_reverse",
        math.radians(-85.9),
        recovery_active=True,
        rotate_first_enabled=True,
        paused=False,
    )
    request = MODULE.rotate_first_continuation_request(
        0.0,
        continuation_required=True,
        latched_direction=-1,
    )
    assert request == pytest.approx(-0.10)
    assert MODULE.bounded_rotate_first_angular(
        request,
        latched_direction=-1,
    ) == pytest.approx(-0.10)


def test_reverse_path_uses_stable_heading_when_source_yaw_is_zero():
    # The pinned controller suppresses yaw when its lookahead is behind the
    # base.  Rotate-first must therefore derive yaw from the validated
    # collision-scored path/router heading, not from that zero Twist.
    request = MODULE.controller_recovery_angular_request(
        0.0,
        math.radians(105.0),
        use_stable_heading=True,
        continuation_required=False,
        latched_direction=1,
    )

    assert request == pytest.approx(0.35)


def test_missing_stable_heading_cannot_invent_recovery_yaw():
    assert (
        MODULE.controller_recovery_angular_request(
            0.0,
            None,
            use_stable_heading=True,
            continuation_required=False,
            latched_direction=0,
        )
        == 0.0
    )


@pytest.mark.parametrize(
    ("recovery_active", "enabled", "paused", "heading_deg"),
    [
        (False, True, False, -85.9),
        (True, False, False, -85.9),
        (True, True, True, -85.9),
        (True, True, False, -34.9),
    ],
)
def test_tiny_reverse_cannot_start_or_extend_unbounded_rotation(
    recovery_active,
    enabled,
    paused,
    heading_deg,
):
    assert not MODULE.tiny_reverse_recovery_continuation_required(
        "zero_tiny_reverse",
        math.radians(heading_deg),
        recovery_active=recovery_active,
        rotate_first_enabled=enabled,
        paused=paused,
    )


def test_tiny_reverse_continuation_requires_latched_direction():
    with pytest.raises(ValueError):
        MODULE.rotate_first_continuation_request(
            0.0,
            continuation_required=True,
            latched_direction=0,
        )


def test_rotate_first_is_explicitly_opt_in():
    defaults = MODULE.build_parser().parse_args(_deployment_args("yunji-water"))
    enabled = MODULE.build_parser().parse_args(
        _deployment_args()
        + [
            "--rotate-first-on-reverse",
            "--stabilize-large-turn",
            "--verified-forward-only-planner",
            "--rotate-first-max-angular-radps",
            "0.30",
            "--rotate-first-timeout-s",
            "8.0",
        ]
    )

    assert defaults.rotate_first_on_reverse is False
    assert defaults.stabilize_large_turn is False
    assert enabled.rotate_first_on_reverse is True
    assert enabled.stabilize_large_turn is True
    assert enabled.verified_forward_only_planner is True
    assert enabled.rotate_first_max_angular_radps == pytest.approx(0.30)
    assert enabled.rotate_first_timeout_s == pytest.approx(8.0)
    assert defaults.turn_no_progress_timeout_s == pytest.approx(3.0)
    assert defaults.turn_progress_epsilon_deg == pytest.approx(1.0)


def test_controller_profile_is_required_and_common_guards_are_enabled():
    with pytest.raises(SystemExit):
        MODULE.build_parser().parse_args([])
    parsed = MODULE.build_parser().parse_args(_deployment_args())

    assert parsed.pose_timeout_s == pytest.approx(0.8)
    assert parsed.path_timeout_s == pytest.approx(1.0)
    assert parsed.pose_jump_m == pytest.approx(0.4)
    assert parsed.pose_jump_freeze_s == pytest.approx(0.6)
    assert parsed.pause_service == MODULE.DEFAULT_CONTROLLER_PAUSE_SERVICE
    assert parsed.turn_stalled_topic == MODULE.DEFAULT_CONTROLLER_TURN_STALLED_TOPIC
    assert parsed.robot_id == "robot-0"
    assert parsed.base_camera_frame == "camera"


def test_controller_exposes_acknowledged_pause_service():
    source = MODULE_PATH.read_text(encoding="utf-8")

    assert "from std_srvs.srv import SetBool" in source
    assert "self.create_service(" in source
    assert "self._on_focus_set_paused" in source
    callback = source.split("def _on_focus_set_paused", 1)[1].split(
        "def pose_callback", 1
    )[0]
    assert "self._on_paused(Bool(data=requested))" in callback
    assert "response.success = bool(self._paused) == requested" in callback


def test_common_controller_input_guard_fails_closed():
    base = {
        "now_monotonic": 10.0,
        "pose_received_monotonic": 9.7,
        "path_received_monotonic": 9.6,
        "pose_jump_freeze_until_monotonic": 0.0,
        "paused": False,
    }
    assert MODULE.controller_input_guard_reason(**base) is None
    assert (
        MODULE.controller_input_guard_reason(**{**base, "paused": True})
        == "navigation_paused"
    )
    assert (
        MODULE.controller_input_guard_reason(
            **{**base, "pose_received_monotonic": 9.19}
        )
        == "pose_missing_or_stale"
    )
    assert (
        MODULE.controller_input_guard_reason(
            **{**base, "pose_jump_freeze_until_monotonic": 10.1}
        )
        == "pose_jump_freeze"
    )
    assert (
        MODULE.controller_input_guard_reason(
            **{**base, "path_received_monotonic": 8.99}
        )
        == "path_missing_or_stale"
    )


@pytest.mark.parametrize("requested", [-0.2, 0.0, 0.039, 0.1, 0.3])
def test_commands_outside_engagement_band_are_unchanged(requested):
    assert MODULE.apply_linear_engagement_floor(
        requested,
        engage_threshold_mps=0.04,
        minimum_effective_mps=0.10,
    ) == pytest.approx(requested)


@pytest.mark.parametrize(
    ("engage", "minimum"),
    [(-0.01, 0.1), (0.1, 0.0), (0.2, 0.1)],
)
def test_invalid_engagement_thresholds_are_rejected(engage, minimum):
    with pytest.raises(ValueError):
        MODULE.apply_linear_engagement_floor(
            0.05,
            engage_threshold_mps=engage,
            minimum_effective_mps=minimum,
        )
