"""Source-derived MCoCoNav episode state for real-robot shadow execution.

The authoritative behavior lives in the immutable HPC copy:

* ``source/Focus_realworld/main.py`` owns the shared multi-agent map,
  frontier/history selection and global decision cadence;
* ``source/Focus_realworld/agents/vlm_agents.py`` replaces an exploration
  goal with the largest connected target-semantic component and stops only
  when the local planner reports arrival;
* ``source/Focus_realworld/tasks/multi_objectnav_hm3d.yaml`` caps an episode
  at 500 simulator action steps.

This module ports only the state and pure decisions needed to keep those
semantics across repeated real-robot *shadow* rounds.  It never emits a robot
command and deliberately distinguishes ``Find_Goal`` from navigation
success: the former comes from model-derived semantic map evidence; source
episode termination still requires a robot-local planner STOP, and HM3D
`multi_Total_SR` additionally requires the GT agent's target evidence. The
real deployment therefore needs independent target validation as well.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np
from scipy import ndimage

from .central_mapping import HM3D_CATEGORY_NAMES
from .directional_memory import DirectionalMemory
from .map_snapshot import MapSnapshot


SOURCE_EPISODE_SCHEMA_VERSION = "focus-source-episode-state-v1"
SOURCE_NUM_LOCAL_STEPS = 25
SOURCE_EARLY_FRONTIER_STEP = 125
SOURCE_MAX_EPISODE_STEPS = 500
# ``constants.py:category_to_id`` defines the HM3D ObjectNav target set.  The
# adapter spells upstream ``tv_monitor`` as its map-channel name ``tv``.
SOURCE_HM3D_OBJECTNAV_GOALS: tuple[str, ...] = (
    "chair",
    "bed",
    "plant",
    "toilet",
    "tv",
    "sofa",
)


@dataclass(frozen=True)
class SemanticGoalComponent:
    """Largest connected target channel, matching ``find_big_connect``."""

    category: str
    row: int
    col: int
    x_m: float
    y_m: float
    size_cells: int
    mask: np.ndarray = field(repr=False, compare=False)

    def to_record(self) -> dict[str, object]:
        return {
            "kind": "semantic_goal",
            "target_id": f"target-{self.category}",
            "category": self.category,
            "row": self.row,
            "col": self.col,
            "x_m": self.x_m,
            "y_m": self.y_m,
            "size_cells": self.size_cells,
            "source_find_goal": True,
            "evidence_status": "model_inference_map_projected_unverified",
            "centroid_status": "source-derived display adapter; HPC local planner consumes the full mask",
        }


def source_decision_step(round_index: int, *, num_local_steps: int = SOURCE_NUM_LOCAL_STEPS) -> int:
    """Map a decision-round index to the exact HPC decision-step sequence.

    ``main.py`` decides at step 0, then when ``l_step % 25 == 24``:
    0, 24, 49, ..., 499.  In shadow mode this is an explicitly labelled
    source-derived logical clock, not a claim that a physical robot executed
    that many discrete Habitat actions.
    """
    if isinstance(round_index, bool) or not isinstance(round_index, int) or round_index < 0:
        raise ValueError("round_index must be a non-negative integer")
    if isinstance(num_local_steps, bool) or not isinstance(num_local_steps, int) or num_local_steps <= 0:
        raise ValueError("num_local_steps must be a positive integer")
    if round_index == 0:
        return 0
    return num_local_steps - 1 + (round_index - 1) * num_local_steps


def source_decision_round_limit(
    *,
    max_episode_steps: int = SOURCE_MAX_EPISODE_STEPS,
    num_local_steps: int = SOURCE_NUM_LOCAL_STEPS,
) -> int:
    """Number of global decisions occurring inside one source episode."""
    if max_episode_steps <= 0:
        raise ValueError("max_episode_steps must be positive")
    rounds = 0
    while source_decision_step(rounds, num_local_steps=num_local_steps) < max_episode_steps:
        rounds += 1
    return rounds


def select_history_index(
    memory: DirectionalMemory,
    candidate_indices: list[int] | None = None,
    *,
    candidate_scores: dict[int, float] | None = None,
) -> int | None:
    """Source history branch: argmax over ``history_score_copy``.

    The current HPC ``main.py`` imports a History prompt but does not call it
    in this branch.  It assigns ``Final_PR = history_score_copy`` and uses
    ``Final_PR.index(max(Final_PR))``.  First-max tie behavior is preserved.
    """
    if candidate_indices is None:
        candidate_indices = list(range(len(memory.history_score)))
    if not candidate_indices:
        return None
    for index in candidate_indices:
        if index < 0 or index >= len(memory.history_score):
            raise ValueError(f"history candidate index {index} is out of range")
        if candidate_scores is not None and index not in candidate_scores:
            raise ValueError(f"history candidate score {index} is missing")
    scores = memory.history_score if candidate_scores is None else candidate_scores
    return max(candidate_indices, key=lambda index: scores[index])


def extract_source_goal_component(
    snapshot: MapSnapshot,
    goal_category: str,
    *,
    category_names: tuple[str, ...] = HM3D_CATEGORY_NAMES,
) -> SemanticGoalComponent | None:
    """Apply the HPC ``Find_Goal``/``find_big_connect`` semantic rule.

    Any positive evidence in the requested category triggers ``Find_Goal``;
    the largest 8-connected component becomes the local planner goal mask.
    No confidence or multi-frame rule is invented here.  Real-world evidence
    remains explicitly model-derived/unverified until independent validation.
    """
    if goal_category not in SOURCE_HM3D_OBJECTNAV_GOALS:
        raise ValueError(
            f"unsupported source ObjectNav goal: {goal_category!r}; "
            f"expected one of {SOURCE_HM3D_OBJECTNAV_GOALS}"
        )
    try:
        category_index = category_names.index(goal_category)
    except ValueError as exc:
        raise ValueError(f"unsupported source goal category: {goal_category!r}") from exc
    channel_index = 2 + category_index
    if channel_index >= snapshot.grid.shape[0]:
        raise ValueError(
            f"map has {snapshot.grid.shape[0] - 2} semantic channels, "
            f"cannot read {goal_category!r} at channel {channel_index}"
        )
    binary = np.asarray(snapshot.grid[channel_index] > 0.0, dtype=bool)
    if not np.any(binary):
        return None
    if goal_category == "tv":
        # ``vlm_agents.py`` applies one 7x7 rectangular dilation when its
        # mapped target channel is ``cn == 9`` (the HM3D TV target).
        binary = ndimage.binary_dilation(
            binary,
            structure=np.ones((7, 7), dtype=bool),
            iterations=1,
        )
    labels, count = ndimage.label(binary, structure=np.ones((3, 3), dtype=bool))
    if count <= 0:
        return None
    sizes = ndimage.sum_labels(
        np.ones_like(labels, dtype=np.int64),
        labels,
        index=range(1, count + 1),
    )
    label_id = int(np.argmax(sizes)) + 1
    mask = labels == label_id
    rows, cols = np.nonzero(mask)
    # The full mask is authoritative for the source local planner.  A rounded
    # centroid is retained only so the shadow dashboard can display it.
    row = int(np.round(rows.mean()))
    col = int(np.round(cols.mean()))
    return SemanticGoalComponent(
        category=goal_category,
        row=row,
        col=col,
        x_m=snapshot.origin_xy_m[0] + (col + 0.5) * snapshot.resolution_m,
        y_m=snapshot.origin_xy_m[1] + (row + 0.5) * snapshot.resolution_m,
        size_cells=int(mask.sum()),
        mask=mask,
    )


@dataclass
class SourceEpisodeState:
    """Persistent state shared by every robot in one shadow scene."""

    scene_id: str
    goal_category: str
    shared_frame_calibration_id: str
    robot_ids: tuple[str, ...]
    round_index: int = 0
    memory: DirectionalMemory = field(default_factory=DirectionalMemory)
    previous_positions_rc: dict[str, tuple[int, int]] = field(default_factory=dict)
    last_source_sequences: dict[str, int] = field(default_factory=dict)
    source_find_goal: dict[str, bool] = field(default_factory=dict)
    fused_origin_xy_m: tuple[float, float] | None = None
    resolution_m: float | None = None
    fused_shape_hw: tuple[int, int] | None = None

    @property
    def source_step(self) -> int:
        return source_decision_step(self.round_index)

    def validate_contract(
        self,
        *,
        goal_category: str,
        calibration_id: str,
        robot_ids: tuple[str, ...],
        fused_origin_xy_m: tuple[float, float],
        resolution_m: float,
        fused_shape_hw: tuple[int, int],
    ) -> None:
        if goal_category != self.goal_category:
            raise ValueError("scene goal category changed")
        if calibration_id != self.shared_frame_calibration_id:
            raise ValueError("scene shared calibration changed")
        if robot_ids != self.robot_ids:
            raise ValueError("scene robot order/identity changed")
        if self.fused_origin_xy_m is not None and not np.allclose(
            self.fused_origin_xy_m, fused_origin_xy_m, rtol=0.0, atol=1e-12
        ):
            raise ValueError("scene fused-map origin changed; history cells are no longer valid")
        if self.resolution_m is not None and not math.isclose(
            self.resolution_m, resolution_m, rel_tol=0.0, abs_tol=1e-12
        ):
            raise ValueError("scene map resolution changed")
        if self.fused_shape_hw is not None and self.fused_shape_hw != fused_shape_hw:
            raise ValueError("scene fused-map shape changed; history cells are no longer valid")

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": SOURCE_EPISODE_SCHEMA_VERSION,
            "scene_id": self.scene_id,
            "goal_category": self.goal_category,
            "shared_frame_calibration_id": self.shared_frame_calibration_id,
            "robot_ids": list(self.robot_ids),
            "round_index": self.round_index,
            "next_source_step": self.source_step,
            "shared_directional_memory": self.memory.to_dict(),
            "previous_positions_rc": {
                robot_id: list(position)
                for robot_id, position in sorted(self.previous_positions_rc.items())
            },
            "last_source_sequences": dict(sorted(self.last_source_sequences.items())),
            "source_find_goal": dict(sorted(self.source_find_goal.items())),
            "fused_origin_xy_m": (
                None if self.fused_origin_xy_m is None else list(self.fused_origin_xy_m)
            ),
            "resolution_m": self.resolution_m,
            "fused_shape_hw": (
                None if self.fused_shape_hw is None else list(self.fused_shape_hw)
            ),
            "provenance": {
                "algorithm": "source-derived MCoCoNav episode semantics",
                "source_paths": [
                    "source/Focus_realworld/main.py",
                    "source/Focus_realworld/agents/vlm_agents.py",
                    "source/Focus_realworld/tasks/multi_objectnav_hm3d.yaml",
                ],
                "authority": "shadow_only_no_robot_command",
            },
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "SourceEpisodeState":
        if payload.get("schema_version") != SOURCE_EPISODE_SCHEMA_VERSION:
            raise ValueError("unsupported source episode state schema")
        scene_id = str(payload.get("scene_id", ""))
        goal_category = str(payload.get("goal_category", ""))
        calibration_id = str(payload.get("shared_frame_calibration_id", ""))
        robot_values = payload.get("robot_ids")
        if not scene_id or not goal_category or not calibration_id:
            raise ValueError("scene identity fields must be non-empty")
        if goal_category not in SOURCE_HM3D_OBJECTNAV_GOALS:
            raise ValueError("scene goal category is not an HPC HM3D ObjectNav target")
        if not isinstance(robot_values, list) or len(robot_values) < 2:
            raise ValueError("scene must contain at least two robot IDs")
        robot_ids = tuple(str(value) for value in robot_values)
        if any(not value for value in robot_ids) or len(set(robot_ids)) != len(robot_ids):
            raise ValueError("scene robot IDs must be unique and non-empty")
        round_index = payload.get("round_index")
        if isinstance(round_index, bool) or not isinstance(round_index, int) or round_index < 0:
            raise ValueError("scene round_index must be a non-negative integer")
        if round_index > source_decision_round_limit():
            raise ValueError("scene round_index exceeds the source episode")
        next_source_step = payload.get("next_source_step")
        if next_source_step != source_decision_step(round_index):
            raise ValueError("scene next_source_step does not match round_index")
        raw_memory = payload.get("shared_directional_memory")
        if not isinstance(raw_memory, dict):
            raise ValueError("scene shared_directional_memory must be an object")

        def parse_positions(key: str) -> dict[str, tuple[int, int]]:
            raw = payload.get(key, {})
            if not isinstance(raw, dict):
                raise ValueError(f"scene {key} must be an object")
            result: dict[str, tuple[int, int]] = {}
            for robot_id, value in raw.items():
                if robot_id not in robot_ids:
                    raise ValueError(f"scene {key} contains unknown robot {robot_id!r}")
                if (
                    not isinstance(value, list)
                    or len(value) != 2
                    or any(isinstance(item, bool) or not isinstance(item, int) for item in value)
                ):
                    raise ValueError(f"scene position for {robot_id} must contain two integers")
                result[robot_id] = (value[0], value[1])
            return result

        raw_sequences = payload.get("last_source_sequences", {})
        raw_find_goal = payload.get("source_find_goal", {})
        if not isinstance(raw_sequences, dict) or not isinstance(raw_find_goal, dict):
            raise ValueError("scene sequence/find-goal fields must be objects")
        sequences: dict[str, int] = {}
        find_goal: dict[str, bool] = {}
        for robot_id, value in raw_sequences.items():
            if robot_id not in robot_ids or isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError("scene contains an invalid source sequence")
            sequences[robot_id] = value
        for robot_id, value in raw_find_goal.items():
            if robot_id not in robot_ids or not isinstance(value, bool):
                raise ValueError("scene contains an invalid Find_Goal value")
            find_goal[robot_id] = value

        raw_origin = payload.get("fused_origin_xy_m")
        origin = None if raw_origin is None else tuple(float(value) for value in raw_origin)
        if origin is not None and (len(origin) != 2 or not all(math.isfinite(value) for value in origin)):
            raise ValueError("scene fused origin must contain two finite values")
        raw_resolution = payload.get("resolution_m")
        resolution = None if raw_resolution is None else float(raw_resolution)
        if resolution is not None and (not math.isfinite(resolution) or resolution <= 0.0):
            raise ValueError("scene resolution must be finite and positive")
        raw_shape = payload.get("fused_shape_hw")
        shape = None if raw_shape is None else tuple(int(value) for value in raw_shape)
        if shape is not None and (len(shape) != 2 or any(value <= 0 for value in shape)):
            raise ValueError("scene fused shape must contain two positive values")

        return cls(
            scene_id=scene_id,
            goal_category=goal_category,
            shared_frame_calibration_id=calibration_id,
            robot_ids=robot_ids,
            round_index=round_index,
            memory=DirectionalMemory.from_dict(raw_memory),
            previous_positions_rc=parse_positions("previous_positions_rc"),
            last_source_sequences=sequences,
            source_find_goal=find_goal,
            fused_origin_xy_m=origin,
            resolution_m=resolution,
            fused_shape_hw=shape,
        )
