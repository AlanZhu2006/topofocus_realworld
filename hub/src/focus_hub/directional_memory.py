"""Shared directional exploration memory, ported from `main.py`'s
`history_nodes` / `history_count` / `history_state` bookkeeping (the block
right after the Perception+Judgment VLM calls, upstream lines ~1331-1410).

This is a real part of upstream's decision algorithm that the hub's first
VLM port missed entirely (see `vlm_prompts.py`'s module docstring for the
full story): each step, the combined Perception+Judgment score gets smeared
across source's nominal 39-degree directional slices of a 360-bucket "how
promising was facing this direction" array, keyed to the nearest known
position (new position if none within 25 cells). ``main.py`` creates these
four lists once outside the agent loop, so every agent in an episode shares
one memory. It persists across the whole episode, not just the current step.

**Known upstream inconsistency, preserved on purpose, not "fixed":**
the first-visit and revisit branches use different wraparound arithmetic
near the 0/360 boundary. In particular, the low-angle first-visit slice uses
``360-c_angle-39`` while the revisit slice uses ``360-c_angle``; the two
branches therefore cover different, asymmetric bucket counts. This looks
like a real bug in the original source, but the executable source is
authoritative here, so it is reproduced exactly rather than normalized.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any


@dataclass
class DirectionalMemory:
    """One instance per episode, shared across agents.

    Coordinates are map cells ``(row, col)``, the same convention as
    upstream's ``start = [row_cell, col_cell]``.
    """

    history_nodes: list[tuple[int, int]] = field(default_factory=list)
    history_count: list[int] = field(default_factory=list)
    history_states: list[list[float]] = field(default_factory=list)
    history_score: list[float] = field(default_factory=list)

    NEAR_THRESHOLD_CELLS: float = 25.0
    WEDGE_HALF_WIDTH_DEG: int = 39

    def to_dict(self) -> dict[str, object]:
        """Serialize the episode-scoped memory without changing its values.

        The HPC loop keeps these four lists alive for the whole episode.  A
        real-robot shadow scene spans multiple one-shot VLM processes, so the
        deployment adapter must persist exactly this state between rounds.
        """
        return {
            "history_nodes": [list(node) for node in self.history_nodes],
            "history_count": list(self.history_count),
            "history_states": [list(state) for state in self.history_states],
            "history_score": list(self.history_score),
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "DirectionalMemory":
        """Load and strictly validate a serialized episode memory."""
        required = {
            "history_nodes",
            "history_count",
            "history_states",
            "history_score",
        }
        missing = required.difference(payload)
        if missing:
            raise ValueError(f"directional memory is missing {sorted(missing)}")

        raw_nodes = payload["history_nodes"]
        raw_counts = payload["history_count"]
        raw_states = payload["history_states"]
        raw_scores = payload["history_score"]
        if not all(isinstance(value, list) for value in (
            raw_nodes,
            raw_counts,
            raw_states,
            raw_scores,
        )):
            raise ValueError("directional memory fields must be lists")
        lengths = {len(raw_nodes), len(raw_counts), len(raw_states), len(raw_scores)}
        if len(lengths) != 1:
            raise ValueError("directional memory list lengths disagree")

        nodes: list[tuple[int, int]] = []
        counts: list[int] = []
        states: list[list[float]] = []
        scores: list[float] = []
        for index, raw_node in enumerate(raw_nodes):
            if (
                not isinstance(raw_node, list)
                or len(raw_node) != 2
                or any(isinstance(value, bool) or not isinstance(value, int) for value in raw_node)
            ):
                raise ValueError(f"history node {index} must contain two integers")
            nodes.append((raw_node[0], raw_node[1]))

            raw_count = raw_counts[index]
            if isinstance(raw_count, bool) or not isinstance(raw_count, int) or raw_count <= 0:
                raise ValueError(f"history count {index} must be a positive integer")
            counts.append(raw_count)

            raw_state = raw_states[index]
            if not isinstance(raw_state, list) or len(raw_state) != 360:
                raise ValueError(f"history state {index} must contain 360 values")
            state = [float(value) for value in raw_state]
            if not all(math.isfinite(value) for value in state):
                raise ValueError(f"history state {index} contains a non-finite value")
            states.append(state)

            score = float(raw_scores[index])
            if not math.isfinite(score):
                raise ValueError(f"history score {index} is not finite")
            scores.append(score)

        return cls(
            history_nodes=nodes,
            history_count=counts,
            history_states=states,
            history_score=scores,
        )

    def prepare_visit(self, position_rc: tuple[int, int]) -> tuple[int, bool]:
        """Register a visit before Judgment, matching ``main.py`` ordering.

        Source creates a new node, or increments the nearest node's count,
        immediately after Perception.  The updated node list is then included
        in the Judgment prompt; its directional score is filled only after
        Judgment returns.  ``is_new`` preserves which source branch to use.
        """
        new_row, new_col = position_rc
        closest_index = -1
        min_distance = float("inf")
        for i, (row, col) in enumerate(self.history_nodes):
            distance = math.sqrt((row - new_row) ** 2 + (col - new_col) ** 2)
            if distance < self.NEAR_THRESHOLD_CELLS and distance < min_distance:
                min_distance = distance
                closest_index = i
        is_new = closest_index == -1

        if is_new:
            self.history_nodes.append((new_row, new_col))
            self.history_count.append(1)
            self.history_states.append([0.0] * 360)
            closest_index = len(self.history_nodes) - 1
        else:
            self.history_count[closest_index] += 1

        return closest_index, is_new

    def apply_score(
        self,
        history_index: int,
        is_new: bool,
        heading_deg: float,
        angle_score: float,
    ) -> None:
        """Apply the post-Judgment directional score with source arithmetic."""
        if history_index < 0 or history_index >= len(self.history_nodes):
            raise ValueError("history index is out of range")
        if is_new and history_index != len(self.history_score):
            raise ValueError("new history score must be appended in source order")
        if not is_new and history_index >= len(self.history_score):
            raise ValueError("existing history node has no score state")

        c_angle = int(heading_deg % 360)
        w = self.WEDGE_HALF_WIDTH_DEG
        state = self.history_states[history_index]

        if is_new:
            if w <= c_angle < 360 - w:
                for a in range(c_angle - w, c_angle + w):
                    state[a] = angle_score
            elif c_angle < w:
                for a in range(0, c_angle + w):
                    state[a] = angle_score
                for a in range(360 - c_angle - w, 360):
                    state[a] = angle_score
            else:  # c_angle >= 360 - w
                for a in range(c_angle - w, 360):
                    state[a] = angle_score
                for a in range(0, c_angle + w - 360):
                    state[a] = angle_score
            self.history_score.append(sum(state))
        else:
            if w <= c_angle < 360 - w:
                for a in range(c_angle - w, c_angle + w):
                    state[a] = angle_score
            elif c_angle < w:
                # Upstream inconsistency preserved: window width here is
                # c_angle itself, not WEDGE_HALF_WIDTH_DEG, unlike the
                # is_new branch above.
                for a in range(0, c_angle):
                    state[a] = angle_score
                for a in range(360 - c_angle, 360):
                    state[a] = angle_score
            else:
                for a in range(c_angle, 360):
                    state[a] = angle_score
                for a in range(0, 360 - c_angle):
                    state[a] = angle_score
            self.history_score[history_index] = (
                sum(state) / self.history_count[history_index]
            )

    def update(
        self,
        position_rc: tuple[int, int],
        heading_deg: float,
        angle_score: float,
    ) -> int:
        """Compatibility helper that performs one complete source visit."""
        history_index, is_new = self.prepare_visit(position_rc)
        self.apply_score(history_index, is_new, heading_deg, angle_score)

        return history_index
