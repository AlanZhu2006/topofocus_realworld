import importlib.util
import math
from pathlib import Path

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


def test_small_intentional_forward_command_reaches_static_friction_floor():
    assert MODULE.apply_linear_engagement_floor(
        0.078,
        engage_threshold_mps=0.04,
        minimum_effective_mps=0.10,
    ) == pytest.approx(0.10)


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


def test_rotate_first_is_explicitly_opt_in():
    defaults = MODULE.build_parser().parse_args([])
    enabled = MODULE.build_parser().parse_args(
        [
            "--rotate-first-on-reverse",
            "--rotate-first-max-angular-radps",
            "0.30",
            "--rotate-first-timeout-s",
            "8.0",
        ]
    )

    assert defaults.rotate_first_on_reverse is False
    assert enabled.rotate_first_on_reverse is True
    assert enabled.rotate_first_max_angular_radps == pytest.approx(0.30)
    assert enabled.rotate_first_timeout_s == pytest.approx(8.0)


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
