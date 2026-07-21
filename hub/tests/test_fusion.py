from __future__ import annotations

import numpy as np
import pytest

from focus_hub.frontiers import Frontier
from focus_hub.fusion import align_and_fuse_grids, allocate_frontiers_sequential, fuse_grids


def make_frontier(letter: str, size: int) -> Frontier:
    return Frontier(frontier_id=letter, row=0, col=0, x_m=0.0, y_m=0.0, size_cells=size)


class StubChoice:
    def __init__(self, frontier):
        self.frontier = frontier
        self.source = "stub"
        self.probabilities = {}


def test_fuse_grids_is_elementwise_max_and_monotonic():
    a = np.zeros((3, 4, 4), dtype=np.float32)
    b = np.zeros((3, 4, 4), dtype=np.float32)
    a[0, 1, 1] = 1.0
    b[0, 2, 2] = 1.0
    a[1, 0, 0] = 0.3
    b[1, 0, 0] = 0.7
    fused = fuse_grids([a, b])
    assert fused[0, 1, 1] == 1.0 and fused[0, 2, 2] == 1.0
    assert fused[1, 0, 0] == pytest.approx(0.7)
    assert np.all(fused >= a) and np.all(fused >= b)
    np.testing.assert_array_equal(fused, fuse_grids([b, a]))  # commutative


def test_fuse_grids_rejects_mismatched_shapes():
    with pytest.raises(ValueError, match="shapes differ"):
        fuse_grids([np.zeros((3, 4, 4)), np.zeros((3, 5, 5))])


def test_align_and_fuse_grids_identical_origin_matches_fuse_grids():
    a = np.zeros((3, 4, 4), dtype=np.float32)
    b = np.zeros((3, 4, 4), dtype=np.float32)
    a[0, 1, 1] = 1.0
    b[1, 0, 0] = 0.7
    fused, origin = align_and_fuse_grids([a, b], [(0.0, 0.0), (0.0, 0.0)], resolution_m=0.05)
    np.testing.assert_array_equal(fused, fuse_grids([a, b]))
    assert origin == (0.0, 0.0)


def test_align_and_fuse_grids_disjoint_origins_places_each_robot_in_its_own_region():
    # Robot A's grid covers world x in [0, 0.2), robot B's covers x in [0.2, 0.4)
    # at the same resolution -- disjoint, non-overlapping regions of a shared frame.
    resolution_m = 0.05
    a = np.zeros((3, 4, 4), dtype=np.float32)
    a[0, 0, 0] = 0.9  # a real value at A's own origin cell
    b = np.zeros((3, 4, 4), dtype=np.float32)
    b[0, 0, 0] = 0.6  # a real value at B's own origin cell

    fused, origin = align_and_fuse_grids(
        [a, b], [(0.0, 0.0), (0.2, 0.0)], resolution_m=resolution_m)

    assert origin == (0.0, 0.0)
    assert fused.shape == (3, 4, 8)  # union spans x in [0, 0.4) -> 8 cells wide
    # A's data lands at columns 0-3, unaffected by B.
    assert fused[0, 0, 0] == pytest.approx(0.9)
    # B's data lands at columns 4-7 (0.2m / 0.05m = 4 cell offset).
    assert fused[0, 0, 4] == pytest.approx(0.6)
    # No cross-contamination: A's origin cell doesn't see B's value or vice versa.
    assert fused[0, 0, 4] != fused[0, 0, 0]


def test_align_and_fuse_grids_overlapping_region_takes_max():
    resolution_m = 0.05
    a = np.zeros((2, 4, 4), dtype=np.float32)
    b = np.zeros((2, 4, 4), dtype=np.float32)
    # Both grids share their origin exactly -> fully overlapping; real upstream
    # max-fusion rule must still apply in the overlap.
    a[0, 2, 2] = 0.3
    b[0, 2, 2] = 0.8
    fused, origin = align_and_fuse_grids([a, b], [(1.0, 1.0), (1.0, 1.0)], resolution_m=resolution_m)
    assert origin == (1.0, 1.0)
    assert fused.shape == a.shape
    assert fused[0, 2, 2] == pytest.approx(0.8)


def test_align_and_fuse_grids_rejects_mismatched_lengths():
    with pytest.raises(ValueError, match="same length"):
        align_and_fuse_grids([np.zeros((3, 4, 4))], [(0.0, 0.0), (1.0, 1.0)], resolution_m=0.05)


def test_sequential_allocation_yields_distinct_frontiers():
    frontiers = [make_frontier(letter, size) for letter, size in
                 (("A", 40), ("B", 30), ("C", 20))]

    def choose(_robot_id, remaining):
        return StubChoice(remaining[0])   # both robots want the current best

    allocations = allocate_frontiers_sequential(["robot-0", "robot-1"], frontiers, choose)
    assert [a.frontier.frontier_id for a in allocations] == ["A", "B"]
    assert allocations[0].robot_id == "robot-0"
    assert allocations[1].robot_id == "robot-1"


def test_sequential_allocation_stops_when_candidates_run_out():
    frontiers = [make_frontier("A", 10)]

    def choose(_robot_id, remaining):
        return StubChoice(remaining[0])

    allocations = allocate_frontiers_sequential(["robot-0", "robot-1"], frontiers, choose)
    assert len(allocations) == 1


def test_sequential_allocation_rejects_foreign_frontier():
    frontiers = [make_frontier("A", 10), make_frontier("B", 5)]

    def choose(_robot_id, _remaining):
        return StubChoice(make_frontier("Z", 1))

    with pytest.raises(ValueError, match="not in the remaining set"):
        allocate_frontiers_sequential(["robot-0"], frontiers, choose)
