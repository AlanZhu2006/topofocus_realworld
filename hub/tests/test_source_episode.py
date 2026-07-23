from __future__ import annotations

import numpy as np
import pytest

from focus_hub.directional_memory import DirectionalMemory
from focus_hub.map_snapshot import MapSnapshot
from focus_hub.source_episode import (
    SOURCE_MAX_EPISODE_STEPS,
    SourceEpisodeState,
    extract_source_goal_component,
    select_history_index,
    source_decision_round_limit,
    source_decision_step,
)


def make_snapshot(grid: np.ndarray) -> MapSnapshot:
    return MapSnapshot(
        grid=grid,
        origin_xy_m=(-2.0, -3.0),
        resolution_m=0.05,
        frame_id="shared_world",
        transform_version="transform-v1",
        shared_frame_calibration_id="calibration-v1",
        map_format_version="focus-hub-central-map-v3",
    )


def test_source_decision_clock_matches_hpc_main_loop():
    assert [source_decision_step(index) for index in range(7)] == [
        0, 24, 49, 74, 99, 124, 149,
    ]
    assert source_decision_round_limit() == 21
    assert source_decision_step(20) == SOURCE_MAX_EPISODE_STEPS - 1
    with pytest.raises(ValueError):
        source_decision_step(-1)


def test_source_find_goal_uses_any_positive_cell_and_largest_component():
    grid = np.zeros((17, 20, 20), dtype=np.float32)
    # chair is the first semantic channel (grid channel 2).  The source uses
    # >0 and the largest connected component, not a new confidence rule.
    grid[2, 1, 1] = 1e-6
    grid[2, 10:13, 12:16] = 0.2
    component = extract_source_goal_component(make_snapshot(grid), "chair")

    assert component is not None
    assert component.size_cells == 12
    assert (component.row, component.col) == (11, 14)
    assert component.mask[11, 14]
    assert component.to_record()["source_find_goal"] is True


def test_source_find_goal_absent_and_unknown_category():
    grid = np.zeros((17, 5, 5), dtype=np.float32)
    snapshot = make_snapshot(grid)
    assert extract_source_goal_component(snapshot, "chair") is None
    with pytest.raises(ValueError, match="unsupported"):
        extract_source_goal_component(snapshot, "not-a-source-category")
    with pytest.raises(ValueError, match="ObjectNav"):
        extract_source_goal_component(snapshot, "table")


def test_source_tv_goal_applies_the_hpc_seven_by_seven_dilation():
    grid = np.zeros((17, 20, 20), dtype=np.float32)
    grid[7, 10, 10] = 1.0  # tv is semantic index 5 -> map channel 7
    component = extract_source_goal_component(make_snapshot(grid), "tv")

    assert component is not None
    assert component.size_cells == 49


def test_source_history_selection_is_first_argmax_over_candidate_copy():
    memory = DirectionalMemory(
        history_nodes=[(1, 1), (2, 2), (3, 3)],
        history_count=[1, 1, 1],
        history_states=[[0.0] * 360 for _ in range(3)],
        history_score=[4.0, 9.0, 9.0],
    )
    assert select_history_index(memory) == 1
    assert select_history_index(memory, [0, 2]) == 2
    assert select_history_index(memory, []) is None
    # Agent 1 may update the live shared score after agent 0 creates
    # ``history_score_copy``. Selection must still use the frozen copy.
    assert select_history_index(
        memory,
        [0, 1],
        candidate_scores={0: 1.0, 1: 10.0},
    ) == 1


def test_source_episode_state_round_trip_and_contract_lock():
    state = SourceEpisodeState(
        scene_id="scene-1",
        goal_category="chair",
        shared_frame_calibration_id="calibration-v1",
        robot_ids=("robot-0", "robot-1"),
    )
    state.memory.update((5, 6), 90.0, 2.0)
    state.previous_positions_rc["robot-0"] = (5, 6)
    state.last_source_sequences["robot-0"] = 10
    state.source_find_goal["robot-0"] = False
    state.fused_origin_xy_m = (-2.0, -3.0)
    state.resolution_m = 0.05
    state.fused_shape_hw = (20, 20)

    restored = SourceEpisodeState.from_dict(state.to_dict())

    assert restored.to_dict() == state.to_dict()
    restored.validate_contract(
        goal_category="chair",
        calibration_id="calibration-v1",
        robot_ids=("robot-0", "robot-1"),
        fused_origin_xy_m=(-2.0, -3.0),
        resolution_m=0.05,
        fused_shape_hw=(20, 20),
    )
    with pytest.raises(ValueError, match="calibration"):
        restored.validate_contract(
            goal_category="chair",
            calibration_id="calibration-v2",
            robot_ids=("robot-0", "robot-1"),
            fused_origin_xy_m=(-2.0, -3.0),
            resolution_m=0.05,
            fused_shape_hw=(20, 20),
        )
