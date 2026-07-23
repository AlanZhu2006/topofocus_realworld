"""Tests for focus_hub.central_mapping's free-space ray marking (2026-07-21)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from focus_hub.central_mapping import CentralMapper, MapperConfig


@dataclass
class _FakeFrame:
    depth_m: np.ndarray
    T_world_infra1: np.ndarray


def _make_single_point_mapper(
    ray_trace_steps: int,
    *,
    map_size_m: float = 10.0,
    ray_trace_chunk_points: int = 8192,
    obstacle_fusion_mode: str = "max",
    obstacle_min_hits: int = 1,
    semantic_fusion_mode: str = "max",
    semantic_min_hits: int = 1,
    semantic_winner_margin_hits: int = 0,
    cat_pred_threshold: float = 5.0,
) -> tuple[CentralMapper, _FakeFrame, np.ndarray]:
    """One valid depth pixel at image (row=0, col=6), camera at the world
    origin facing +infra1-z. With K=identity intrinsics this places the
    world point at (x=6, y=0, z=1) -- a clean horizontal ray in the grid.
    """
    K = np.array([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]])
    config = MapperConfig(
        resolution_m=0.05,
        map_size_m=map_size_m,
        ray_trace_steps=ray_trace_steps,
        ray_trace_chunk_points=ray_trace_chunk_points,
        obstacle_fusion_mode=obstacle_fusion_mode,
        obstacle_min_hits=obstacle_min_hits,
        semantic_fusion_mode=semantic_fusion_mode,
        semantic_min_hits=semantic_min_hits,
        semantic_winner_margin_hits=semantic_winner_margin_hits,
        cat_pred_threshold=cat_pred_threshold,
    )
    mapper = CentralMapper(
        config=config,
        K_infra1=K,
        K_rgb=K,
        T_rgb_to_infra1=np.eye(4),
        origin_xy_m=(-1.0, -1.0),
        floor_z_m=0.0,
    )
    depth = np.zeros((1, 10), dtype=np.float64)
    depth[0, 6] = 1.0  # point_infra1 = (6, 0, 1) * depth = (6, 0, 1)
    frame = _FakeFrame(depth_m=depth, T_world_infra1=np.eye(4))
    semantic_pred = np.zeros((1, 10), dtype=np.int16)
    return mapper, frame, semantic_pred


def test_ray_tracing_fills_explored_between_camera_and_endpoint():
    mapper, frame, semantic_pred = _make_single_point_mapper(ray_trace_steps=40)
    mapper.integrate(frame, semantic_pred)
    grid = mapper.map.grid

    # Endpoint: world (6, 0) -> row=20 (from y=0), col=140 (from x=6).
    assert grid[1, 20, 140] == pytest.approx(1.0)  # explored at the true endpoint
    assert grid[0, 20, 140] == pytest.approx(
        1.0
    )  # obstacle at the true endpoint (z_rel=1, in band)

    # A cell strictly between the camera (world 0,0 -> row=20,col=20) and the
    # endpoint should now be marked explored by ray tracing.
    assert grid[1, 20, 80] == pytest.approx(1.0)
    # But never marked as an obstacle -- only the true endpoint carries that.
    assert grid[0, 20, 80] == pytest.approx(0.0)


def test_ray_trace_steps_zero_preserves_endpoint_only_behavior():
    mapper, frame, semantic_pred = _make_single_point_mapper(ray_trace_steps=0)
    mapper.integrate(frame, semantic_pred)
    grid = mapper.map.grid

    # Endpoint still marked as before.
    assert grid[1, 20, 140] == pytest.approx(1.0)
    assert grid[0, 20, 140] == pytest.approx(1.0)
    # No ray tracing means the intermediate cell stays unexplored.
    assert grid[1, 20, 80] == pytest.approx(0.0)


def test_tilted_floor_plane_prevents_scalar_height_false_obstacle():
    mapper, frame, semantic_pred = _make_single_point_mapper(ray_trace_steps=0)
    # Endpoint is world (6, 0, 1).  Relative to this measured tilted floor it
    # is only 10 cm high, below the default 25 cm obstacle band.  A scalar
    # floor_z=0 would incorrectly classify it as a 1 m obstacle.
    mapper.map.floor_plane_coefficients = (0.15, 0.0, 0.0)

    mapper.integrate(frame, semantic_pred)

    row, col = mapper.map.world_to_cell(np.array([6.0]), np.array([0.0]))
    assert mapper.map.grid[1, row[0], col[0]] == pytest.approx(1.0)
    assert mapper.map.grid[0, row[0], col[0]] == pytest.approx(0.0)


def test_ray_tracing_does_not_mark_cells_beyond_the_endpoint():
    mapper, frame, semantic_pred = _make_single_point_mapper(ray_trace_steps=40)
    mapper.integrate(frame, semantic_pred)
    grid = mapper.map.grid

    # Well past the endpoint (col=140) along the same row: never touched.
    assert grid[1, 20, 180] == pytest.approx(0.0)


def test_out_of_map_endpoint_still_marks_ray_segment_inside_map():
    mapper, frame, semantic_pred = _make_single_point_mapper(
        ray_trace_steps=40,
        map_size_m=2.0,
    )
    mapper.integrate(frame, semantic_pred)
    grid = mapper.map.grid

    # The endpoint at x=6 is outside the [-1, 1) map, but the ray crosses the
    # map from the camera at x=0 toward +x. That swept free space is explored.
    row, col = mapper.map.world_to_cell(np.array([0.75]), np.array([0.0]))
    assert grid[1, row[0], col[0]] == pytest.approx(1.0)
    assert grid[0].sum() == pytest.approx(0.0)


def test_ray_trace_chunking_is_byte_identical():
    small_chunks, frame, semantic_pred = _make_single_point_mapper(
        ray_trace_steps=40,
        ray_trace_chunk_points=1,
    )
    large_chunks, _, _ = _make_single_point_mapper(
        ray_trace_steps=40,
        ray_trace_chunk_points=1000,
    )
    small_chunks.integrate(frame, semantic_pred)
    large_chunks.integrate(frame, semantic_pred)
    assert small_chunks.map.grid.tobytes() == large_chunks.map.grid.tobytes()


def test_log_odds_requires_multiple_hits_and_later_free_rays_can_clear():
    mapper, frame, semantic_pred = _make_single_point_mapper(
        ray_trace_steps=80,
        obstacle_fusion_mode="log_odds",
        obstacle_min_hits=2,
    )
    endpoint_row, endpoint_col = mapper.map.world_to_cell(
        np.array([6.0]), np.array([0.0])
    )
    rc = (endpoint_row[0], endpoint_col[0])

    mapper.integrate(frame, semantic_pred)
    assert mapper.map.grid[(0,) + rc] == pytest.approx(0.0)
    mapper.integrate(frame, semantic_pred)
    assert mapper.map.grid[(0,) + rc] == pytest.approx(1.0)

    # Doubling depth moves the endpoint outside the obstacle height band and
    # outside this map, while its ray still traverses the old endpoint cell.
    farther = _FakeFrame(
        depth_m=frame.depth_m.copy(), T_world_infra1=frame.T_world_infra1
    )
    farther.depth_m[0, 6] = 2.0
    for _ in range(3):
        mapper.integrate(farther, semantic_pred)
    assert mapper.map.grid[(0,) + rc] == pytest.approx(0.0)
    assert mapper.map.grid[(1,) + rc] == pytest.approx(1.0)


def test_multi_view_semantics_require_repeated_keyframes_and_unique_winner():
    mapper, frame, semantic_pred = _make_single_point_mapper(
        ray_trace_steps=0,
        semantic_fusion_mode="multi_view",
        semantic_min_hits=2,
        semantic_winner_margin_hits=1,
        cat_pred_threshold=1.0,
    )
    semantic_pred[0, 6] = 4  # source MP3D chair ID
    endpoint_row, endpoint_col = mapper.map.world_to_cell(
        np.array([6.0]), np.array([0.0])
    )
    rc = (endpoint_row[0], endpoint_col[0])

    mapper.integrate(frame, semantic_pred)
    assert mapper.map.grid[(2,) + rc] == pytest.approx(0.0)
    mapper.integrate(frame, semantic_pred)
    assert mapper.map.grid[(2,) + rc] == pytest.approx(1.0)
