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
        MODULE.validate_controller_path_message(
            _path(_pose_stamped(0.0, 0.0))
        )
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

    with pytest.raises(ValueError, match="geometrically distinct"):
        MODULE.distinct_controller_path_pose_indices(path)


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
    assert MODULE.command_components_finite(
        _twist(linear_x=0.1, angular_z=-0.2)
    )
    assert not MODULE.command_components_finite(
        _twist(linear_x=float("nan"))
    )


def test_small_intentional_forward_command_reaches_static_friction_floor():
    assert MODULE.apply_linear_engagement_floor(
        0.078,
        engage_threshold_mps=0.04,
        minimum_effective_mps=0.10,
    ) == pytest.approx(0.10)


def test_deployment_linear_floor_is_bounded_to_observed_source_maximum():
    parsed = MODULE.build_parser().parse_args(_deployment_args())

    assert parsed.linear_command_floor_mps == pytest.approx(0.18)
    assert MODULE.MAX_DEPLOYMENT_LINEAR_COMMAND_FLOOR_MPS == pytest.approx(
        0.20
    )
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
            (2.8755 - 0.2 * math.cos(0.5597), -6.0557 - 0.2 * math.sin(0.5597)),
        ],
    )

    assert target_error is not None
    assert math.degrees(target_error) == pytest.approx(-50.8, abs=0.3)
    assert reverse_path_error is not None
    assert abs(math.degrees(reverse_path_error)) == pytest.approx(180.0)
    assert MODULE.select_authoritative_heading_error(
        path_heading_error_rad=reverse_path_error,
        router_heading_error_rad=target_error,
    ) == reverse_path_error
    assert MODULE.select_authoritative_heading_error(
        path_heading_error_rad=None,
        router_heading_error_rad=target_error,
    ) == target_error


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
    assert MODULE.path_turn_recovery_required(
        "allow",
        math.radians(90.0),
        recovery_active=False,
        requested_linear_mps=0.0,
        requested_angular_radps=0.7,
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
    defaults = MODULE.build_parser().parse_args(
        _deployment_args("yunji-water")
    )
    enabled = MODULE.build_parser().parse_args(
        _deployment_args()
        + [
            "--rotate-first-on-reverse",
            "--stabilize-large-turn",
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
    assert enabled.rotate_first_max_angular_radps == pytest.approx(0.30)
    assert enabled.rotate_first_timeout_s == pytest.approx(8.0)


def test_controller_profile_is_required_and_common_guards_are_enabled():
    with pytest.raises(SystemExit):
        MODULE.build_parser().parse_args([])
    parsed = MODULE.build_parser().parse_args(_deployment_args())

    assert parsed.pose_timeout_s == pytest.approx(0.8)
    assert parsed.path_timeout_s == pytest.approx(1.0)
    assert parsed.pose_jump_m == pytest.approx(0.4)
    assert parsed.pose_jump_freeze_s == pytest.approx(0.6)
    assert parsed.robot_id == "robot-0"
    assert parsed.base_camera_frame == "camera"


def test_common_controller_input_guard_fails_closed():
    base = {
        "now_monotonic": 10.0,
        "pose_received_monotonic": 9.7,
        "path_received_monotonic": 9.6,
        "pose_jump_freeze_until_monotonic": 0.0,
        "paused": False,
    }
    assert MODULE.controller_input_guard_reason(**base) is None
    assert MODULE.controller_input_guard_reason(
        **{**base, "paused": True}
    ) == "navigation_paused"
    assert MODULE.controller_input_guard_reason(
        **{**base, "pose_received_monotonic": 9.19}
    ) == "pose_missing_or_stale"
    assert MODULE.controller_input_guard_reason(
        **{**base, "pose_jump_freeze_until_monotonic": 10.1}
    ) == "pose_jump_freeze"
    assert MODULE.controller_input_guard_reason(
        **{**base, "path_received_monotonic": 8.99}
    ) == "path_missing_or_stale"


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
