"""Per-robot directional exploration memory, ported from `main.py`'s
`history_nodes` / `history_count` / `history_state` bookkeeping (the block
right after the Perception+Judgment VLM calls, upstream lines ~1331-1410).

This is a real part of upstream's decision algorithm that the hub's first
VLM port missed entirely (see `vlm_prompts.py`'s module docstring for the
full story): each step, the combined Perception+Judgment score gets smeared
across a ±39-degree wedge of a 360-bucket "how promising was facing this
direction" array, keyed to the nearest known position (new position if none
within 25 cells). This is genuinely a *memory* — it persists across the
robot's whole episode/session, not just the current step.

**Known upstream inconsistency, preserved on purpose, not "fixed":**
the first-visit and revisit branches use different wraparound arithmetic
near the 0/360 boundary (first-visit uses a consistent ±39 window even when
it wraps around 0; revisit uses `c_angle` itself as the window width in the
wraparound case, not 39). This looks like a real bug in the original source,
but per this project's standing rule, upstream is authoritative even where
it looks wrong — see `hpc-fidelity-directive` — so it is reproduced exactly,
not corrected.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class DirectionalMemory:
    """One instance per robot. Coordinates are in map cells (row, col), same
    convention as upstream's `start = [row_cell, col_cell]`.
    """

    history_nodes: list[tuple[int, int]] = field(default_factory=list)
    history_count: list[int] = field(default_factory=list)
    history_states: list[list[float]] = field(default_factory=list)
    history_score: list[float] = field(default_factory=list)

    NEAR_THRESHOLD_CELLS: float = 25.0
    WEDGE_HALF_WIDTH_DEG: int = 39

    def update(self, position_rc: tuple[int, int], heading_deg: float, angle_score: float) -> int:
        """Records one step's observation. Returns the history_nodes index
        that was created or updated (matches upstream's `closest_index`,
        or `len(history_nodes) - 1` on first visit / a brand-new node).
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

        c_angle = int(heading_deg % 360)
        w = self.WEDGE_HALF_WIDTH_DEG
        state = self.history_states[closest_index]

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
            self.history_score[closest_index] = sum(state) / self.history_count[closest_index]

        return closest_index
