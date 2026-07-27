from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "robot_overlay"
    / "run_yunji_tinynav_planner.py"
)
SPEC = importlib.util.spec_from_file_location(
    "run_yunji_tinynav_planner",
    MODULE_PATH,
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_forward_only_vocabulary_is_empty_and_shape_compatible():
    trajectories, parameters = (
        MODULE.forward_only_predefined_trajectory_vocabularies(
            duration=3.0,
            dt=0.1,
        )
    )

    assert trajectories.shape == (0, 31, 7)
    assert parameters.shape == (0, 2)


@pytest.mark.parametrize(
    ("duration", "dt"),
    [(0.0, 0.1), (3.0, 0.0), (float("nan"), 0.1)],
)
def test_forward_only_vocabulary_rejects_invalid_timing(duration, dt):
    with pytest.raises(ValueError):
        MODULE.forward_only_predefined_trajectory_vocabularies(
            duration=duration,
            dt=dt,
        )


def test_planner_profiles_are_explicit():
    yunji = MODULE.build_parser().parse_args(
        ["--robot-profile", "yunji-water"]
    )
    source = MODULE.build_parser().parse_args(
        ["--robot-profile", "source-default"]
    )

    assert yunji.robot_profile == "yunji-water"
    assert source.robot_profile == "source-default"
    with pytest.raises(SystemExit):
        MODULE.build_parser().parse_args([])


def test_planner_source_provenance_is_observed(tmp_path):
    source = tmp_path / "planning_node.py"
    source.write_text("immutable source\n", encoding="utf-8")

    provenance = MODULE.planner_source_provenance(source)

    assert provenance["classification"] == "observed_pinned_source"
    assert provenance["source_path"] == str(source.resolve())
    assert provenance["size_bytes"] == 17
    assert len(str(provenance["sha256"])) == 64
