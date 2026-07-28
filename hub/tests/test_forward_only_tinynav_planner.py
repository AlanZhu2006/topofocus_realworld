from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
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


def test_stationary_candidate_is_removed_but_in_place_turns_are_preserved():
    trajectories = np.arange(5 * 3 * 7, dtype=np.float64).reshape(5, 3, 7)
    parameters = np.asarray(
        [
            [0.0, -0.5],
            [0.0, 0.0],
            [0.0, 0.5],
            [0.2, 0.0],
            [0.2, 0.5],
        ],
        dtype=np.float64,
    )

    filtered_trajectories, filtered_parameters = (
        MODULE.remove_stationary_trajectory_candidate(
            trajectories,
            parameters,
        )
    )

    assert filtered_trajectories.shape == (4, 3, 7)
    assert filtered_parameters.tolist() == [
        [0.0, -0.5],
        [0.0, 0.5],
        [0.2, 0.0],
        [0.2, 0.5],
    ]
    assert np.array_equal(filtered_trajectories[0], trajectories[0])
    assert np.array_equal(filtered_trajectories[1], trajectories[2])


def test_progress_capable_library_calls_source_with_unchanged_arguments():
    calls = []

    def source_generator(*args, **kwargs):
        calls.append((args, kwargs))
        return (
            np.zeros((3, 4, 7), dtype=np.float64),
            np.asarray([[0.0, -0.2], [0.0, 0.0], [0.1, 0.0]]),
        )

    trajectories, parameters = MODULE.progress_capable_trajectory_library(
        source_generator,
        "sentinel",
        init_p="pose",
    )

    assert calls == [(("sentinel",), {"init_p": "pose"})]
    assert trajectories.shape == (2, 4, 7)
    assert parameters.tolist() == [[0.0, -0.2], [0.1, 0.0]]


@pytest.mark.parametrize(
    ("trajectories", "parameters"),
    [
        (np.zeros((2, 3, 7)), np.zeros((1, 2))),
        (np.zeros((2, 3, 6)), np.zeros((2, 2))),
        (np.zeros((2, 3, 7)), np.zeros((2, 3))),
    ],
)
def test_stationary_filter_rejects_malformed_source_lattice(
    trajectories,
    parameters,
):
    with pytest.raises(ValueError, match="incompatible shapes"):
        MODULE.remove_stationary_trajectory_candidate(
            trajectories,
            parameters,
        )


def test_stationary_filter_fails_closed_if_source_has_no_actionable_row():
    with pytest.raises(ValueError, match="no actionable row"):
        MODULE.remove_stationary_trajectory_candidate(
            np.zeros((1, 3, 7)),
            np.zeros((1, 2)),
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
