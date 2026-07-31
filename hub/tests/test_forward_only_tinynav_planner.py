from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

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


def test_planner_uses_bounded_headered_approximate_sensor_sync():
    calls = []

    class Approximate:
        def __init__(
            self,
            subscribers,
            queue_size,
            slop_s,
            *,
            allow_headerless,
        ):
            calls.append(
                (
                    subscribers,
                    queue_size,
                    slop_s,
                    allow_headerless,
                )
            )

    filters = SimpleNamespace(
        ApproximateTimeSynchronizer=Approximate,
        TimeSynchronizer=object,
    )
    provenance = MODULE.install_bounded_approximate_sensor_sync(
        SimpleNamespace(message_filters=filters),
        slop_s=0.05,
    )
    filters.TimeSynchronizer(["depth", "odom"], 10)

    assert calls == [(["depth", "odom"], 10, 0.05, False)]
    assert provenance == {
        "sensor_synchronizer": "bounded_approximate_header_stamp",
        "sensor_sync_slop_s": 0.05,
        "allow_headerless": False,
    }


def test_approximate_sync_initialization_preserves_ros_global_base_lookup():
    calls = []
    filters = SimpleNamespace()

    class Exact:
        def __init__(self, subscribers, queue_size):
            calls.append(("exact", subscribers, queue_size))

    class Approximate(Exact):
        def __init__(
            self,
            subscribers,
            queue_size,
            slop_s,
            *,
            allow_headerless,
        ):
            # Match ROS 2 message_filters: this is a module-global lookup,
            # not super().__init__().
            filters.TimeSynchronizer.__init__(self, subscribers, queue_size)
            calls.append(("approximate", slop_s, allow_headerless))

    filters.TimeSynchronizer = Exact
    filters.ApproximateTimeSynchronizer = Approximate
    MODULE.install_bounded_approximate_sensor_sync(
        SimpleNamespace(message_filters=filters),
        slop_s=0.05,
    )

    synchronizer = filters.TimeSynchronizer(["depth", "odom"], 10)

    assert isinstance(synchronizer, Approximate)
    assert calls == [
        ("exact", ["depth", "odom"], 10),
        ("approximate", 0.05, False),
    ]


def test_invalid_synchronized_frame_does_not_terminate_planner():
    errors = []

    class Logger:
        def error(self, message):
            errors.append(message)

    class PlanningNode:
        def sync_callback(self, _depth, _odom):
            raise RuntimeError("bad frame")

        def get_logger(self):
            return Logger()

    module = SimpleNamespace(PlanningNode=PlanningNode)
    MODULE.install_guarded_planning_callback(module)

    assert PlanningNode().sync_callback(object(), object()) is None
    assert len(errors) == 1
    assert "planner remains alive" in errors[0]


@pytest.mark.parametrize("slop_s", [0.0, 0.21, float("nan")])
def test_sensor_sync_rejects_unbounded_skew(slop_s):
    with pytest.raises(ValueError):
        MODULE.install_bounded_approximate_sensor_sync(
            SimpleNamespace(
                message_filters=SimpleNamespace(
                    ApproximateTimeSynchronizer=object,
                )
            ),
            slop_s=slop_s,
        )


def test_forward_only_vocabulary_is_empty_and_shape_compatible():
    trajectories, parameters = MODULE.forward_only_predefined_trajectory_vocabularies(
        duration=3.0,
        dt=0.1,
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


def test_stopped_prefixes_preserve_full_paths_and_repeat_safe_end_pose():
    trajectories = np.arange(
        2 * 31 * 7,
        dtype=np.float64,
    ).reshape(2, 31, 7)
    parameters = np.asarray([[0.0, 0.5], [0.2, 0.0]])

    augmented_trajectories, augmented_parameters = (
        MODULE.append_stopped_prefix_trajectories(
            trajectories,
            parameters,
            dt_s=0.1,
        )
    )

    assert augmented_trajectories.shape == (8, 31, 7)
    assert augmented_parameters.tolist() == parameters.tolist() * 4
    assert np.array_equal(augmented_trajectories[:2], trajectories)
    for group_index, last_index in enumerate((5, 10, 20), start=1):
        group = augmented_trajectories[group_index * 2 : (group_index + 1) * 2]
        assert np.array_equal(
            group[:, : last_index + 1],
            trajectories[:, : last_index + 1],
        )
        expected_tail = np.repeat(
            trajectories[:, last_index : last_index + 1],
            31 - last_index - 1,
            axis=1,
        )
        assert np.array_equal(group[:, last_index + 1 :], expected_tail)


def test_progress_library_removes_noop_before_adding_stopped_prefixes():
    source_trajectories = np.zeros((3, 31, 7), dtype=np.float64)
    source_trajectories[0, :, 0] = np.arange(31)
    source_trajectories[1, :, 1] = np.arange(31)
    source_trajectories[2, :, 2] = np.arange(31)
    source_parameters = np.asarray(
        [[0.0, -0.5], [0.0, 0.0], [0.2, 0.0]],
        dtype=np.float64,
    )

    trajectories, parameters = MODULE.progress_capable_trajectory_library(
        lambda **_kwargs: (source_trajectories, source_parameters),
        dt=0.1,
    )

    assert trajectories.shape == (8, 31, 7)
    assert (
        parameters.tolist()
        == [
            [0.0, -0.5],
            [0.2, 0.0],
        ]
        * 4
    )
    assert not np.any(np.all(parameters == 0.0, axis=1))


@pytest.mark.parametrize(
    ("dt_s", "horizons_s"),
    [
        (0.0, (0.5,)),
        (float("nan"), (0.5,)),
        (0.1, (0.0,)),
        (0.1, (float("nan"),)),
    ],
)
def test_stopped_prefixes_reject_invalid_timing(dt_s, horizons_s):
    with pytest.raises(ValueError):
        MODULE.append_stopped_prefix_trajectories(
            np.zeros((2, 31, 7)),
            np.zeros((2, 2)),
            dt_s=dt_s,
            horizons_s=horizons_s,
        )


def test_score_summary_exposes_all_collision_and_in_place_recovery():
    all_collision = MODULE.trajectory_score_summary(
        [float("inf"), float("inf")],
        np.asarray([[0.0, -0.5], [0.2, 0.0]]),
    )
    recovered = MODULE.trajectory_score_summary(
        [0.0, float("inf")],
        np.asarray([[0.0, -0.5], [0.2, 0.0]]),
    )

    assert all_collision["all_candidates_in_collision"] is True
    assert all_collision["finite_candidate_count"] == 0
    assert recovered["all_candidates_in_collision"] is False
    assert recovered["finite_candidate_count"] == 1
    assert recovered["finite_in_place_candidate_count"] == 1


def test_current_circle_clear_removes_only_cells_inside_measured_body():
    mask = np.zeros((4, 7), dtype=bool)
    # Non-square shape makes an accidental x/y-axis swap observable.
    mask[1, 4] = True
    mask[2, 4] = True
    mask[1, 5] = True
    mask[3, 4] = True
    mask[0, 0] = True

    cleared, summary = MODULE.clear_current_circular_footprint(
        mask,
        origin=[10.0, 20.0, -1.0],
        resolution=1.0,
        center_xy=[11.5, 24.5],
        body_radius=1.1,
    )

    assert summary == {
        "current_footprint_clearing": True,
        "current_footprint_cleared_cell_count": 3,
        "current_footprint_center_xy": [11.5, 24.5],
        "current_footprint_radius_m": 1.1,
    }
    assert not cleared[1, 4]
    assert not cleared[2, 4]
    assert not cleared[1, 5]
    assert cleared[3, 4]
    assert cleared[0, 0]
    # The caller's source obstacle mask remains immutable.
    assert mask[1, 4]
    assert mask[2, 4]
    assert mask[1, 5]


@pytest.mark.parametrize(
    ("origin", "resolution", "center_xy", "body_radius"),
    [
        ([0.0, 0.0], 0.0, [0.0, 0.0], 0.2),
        ([0.0, 0.0], 0.1, [0.0, 0.0], 0.0),
        ([0.0, 0.0], 0.1, [float("nan"), 0.0], 0.2),
    ],
)
def test_current_circle_clear_rejects_invalid_geometry(
    origin,
    resolution,
    center_xy,
    body_radius,
):
    with pytest.raises(ValueError):
        MODULE.clear_current_circular_footprint(
            np.zeros((4, 7), dtype=bool),
            origin=origin,
            resolution=resolution,
            center_xy=center_xy,
            body_radius=body_radius,
        )


def test_circular_scorer_uses_measured_radius_not_square_corner_radius():
    rows, columns = np.indices((20, 20))
    # Put one obstacle exactly at the square scorer's front-left corner.
    # Its distance from the trajectory center is sqrt(2)*0.25 ~= 0.354 m:
    # outside the measured 0.283 m circle but on the source corner sample.
    esdf = np.hypot(rows - 15, columns - 15) * 0.05
    trajectories = np.zeros((1, 3, 7), dtype=np.float64)
    trajectories[0, :, :2] = [0.5, 0.5]

    scores, closest_steps = MODULE.score_circular_trajectories_by_esdf(
        trajectories,
        esdf,
        origin=[0.0, 0.0, 0.0],
        resolution=0.05,
        safety_radius=0.05,
        front_len=0.283,
        rear_len=0.283,
        half_w=0.283,
    )

    assert esdf[15, 15] == 0.0
    assert esdf[10, 10] > 0.283
    # The source's square-corner proxy samples the obstacle and rejects the
    # path. The exact circular body remains collision-free.
    assert np.isfinite(scores[0])
    assert scores[0] == 0.0
    assert closest_steps == [0]


def test_circular_scorer_rejects_true_body_overlap_and_out_of_map_path():
    esdf = np.full((20, 20), 1.0, dtype=np.float64)
    esdf[10, 10] = 0.25
    trajectories = np.zeros((2, 2, 7), dtype=np.float64)
    trajectories[0, :, :2] = [0.5, 0.5]
    trajectories[1, :, :2] = [-0.01, 0.5]

    scores, closest_steps = MODULE.score_circular_trajectories_by_esdf(
        trajectories,
        esdf,
        origin=[0.0, 0.0],
        resolution=0.05,
        safety_radius=0.05,
        front_len=0.283,
        rear_len=0.283,
        half_w=0.283,
    )

    assert scores == [float("inf"), float("inf")]
    assert closest_steps == [0, 0]


def test_circular_scorer_preserves_source_xy_order_on_non_square_esdf():
    esdf = np.full((12, 20), 1.0, dtype=np.float64)
    esdf[5, 7] = 0.10
    trajectories = np.zeros((1, 2, 7), dtype=np.float64)
    trajectories[0, :, :2] = [1.55, 2.75]

    scores, closest_steps = MODULE.score_circular_trajectories_by_esdf(
        trajectories,
        esdf,
        origin=[1.0, 2.0],
        resolution=0.1,
        safety_radius=0.05,
        front_len=0.20,
        rear_len=0.20,
        half_w=0.20,
    )

    assert scores == [float("inf")]
    assert closest_steps == [0]


def test_circular_scorer_preserves_open_space_and_closest_step_decay():
    esdf = np.full((20, 20), 1.0, dtype=np.float64)
    trajectories = np.zeros((2, 4, 7), dtype=np.float64)
    trajectories[0, :, :2] = [0.5, 0.5]
    trajectories[1, :, :2] = [0.5, 0.5]
    esdf[10, 10] = 0.32

    scores, closest_steps = MODULE.score_circular_trajectories_by_esdf(
        trajectories,
        esdf,
        origin=[0.0, 0.0],
        resolution=0.05,
        safety_radius=0.05,
        front_len=0.283,
        rear_len=0.283,
        half_w=0.283,
    )

    assert scores[0] == pytest.approx(1.0 / (0.037 + 0.001))
    assert scores[1] == pytest.approx(scores[0])
    assert closest_steps == [0, 0]

    esdf[10, 10] = 0.50
    open_scores, _ = MODULE.score_circular_trajectories_by_esdf(
        trajectories,
        esdf,
        origin=[0.0, 0.0],
        resolution=0.05,
        safety_radius=0.05,
        front_len=0.283,
        rear_len=0.283,
        half_w=0.283,
    )
    assert open_scores == [0.0, 0.0]


@pytest.mark.parametrize(
    ("front_len", "rear_len", "half_w"),
    [
        (0.283, 0.300, 0.283),
        (0.283, 0.283, 0.300),
        (0.0, 0.0, 0.0),
    ],
)
def test_circular_scorer_rejects_non_circular_or_invalid_geometry(
    front_len,
    rear_len,
    half_w,
):
    with pytest.raises(ValueError):
        MODULE.score_circular_trajectories_by_esdf(
            np.zeros((1, 2, 7)),
            np.ones((4, 4)),
            origin=[0.0, 0.0],
            resolution=0.05,
            safety_radius=0.05,
            front_len=front_len,
            rear_len=rear_len,
            half_w=half_w,
        )


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
    yunji = MODULE.build_parser().parse_args(["--robot-profile", "yunji-water"])
    source = MODULE.build_parser().parse_args(["--robot-profile", "source-default"])

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
