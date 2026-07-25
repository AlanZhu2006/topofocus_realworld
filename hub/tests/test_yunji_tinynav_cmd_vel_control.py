import importlib.util
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
