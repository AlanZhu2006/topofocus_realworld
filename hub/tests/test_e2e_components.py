from __future__ import annotations

import numpy as np
import pytest

from focus_hub.depth_align import align_depth_to_rgb, decode_depth_png16, encode_depth_png16
from focus_hub.frontiers import extract_frontiers, render_annotated_bev

K = np.array([[100.0, 0.0, 32.0], [0.0, 100.0, 24.0], [0.0, 0.0, 1.0]])


def test_png16_depth_roundtrip_is_lossless_at_scale():
    depth = np.array([[0.0, 0.431], [1.936, 5.0]], dtype=np.float32)
    payload = encode_depth_png16(depth, 0.001)
    decoded = decode_depth_png16(payload, 0.001)
    np.testing.assert_allclose(decoded, depth, atol=0.0005)
    assert decoded[0, 0] == 0.0


def test_png16_decode_rejects_non_16bit_payload():
    import cv2

    ok, buffer = cv2.imencode(".png", np.zeros((4, 4), dtype=np.uint8))
    assert ok
    with pytest.raises(ValueError, match="16-bit"):
        decode_depth_png16(buffer.tobytes(), 0.001)


def test_align_depth_identity_extrinsics_preserves_depth():
    depth = np.zeros((48, 64), dtype=np.float32)
    depth[20:30, 25:40] = 2.0
    aligned = align_depth_to_rgb(depth, K, K, np.eye(4), (48, 64))
    np.testing.assert_allclose(aligned[22, 30], 2.0)
    assert aligned[0, 0] == 0.0


def test_align_depth_translation_shifts_projection():
    depth = np.zeros((48, 64), dtype=np.float32)
    depth[24, 32] = 2.0  # exactly on the optical axis
    # RGB camera 0.1 m to the right of infra1: p_infra1 = T @ p_rgb with
    # t = +0.1 means the rgb origin sits at +0.1 x in infra1 coordinates.
    T_rgb_to_infra1 = np.eye(4)
    T_rgb_to_infra1[0, 3] = 0.1
    aligned = align_depth_to_rgb(depth, K, K, T_rgb_to_infra1, (48, 64))
    vs, us = np.nonzero(aligned)
    assert len(us) == 1
    # Point at infra1 (0,0,2) -> rgb frame (-0.1, 0, 2) -> u = cx - 100*0.1/2 = 27
    assert (vs[0], us[0]) == (24, 27)
    np.testing.assert_allclose(aligned[24, 27], 2.0)


def test_align_depth_keeps_nearest_on_collision():
    depth = np.zeros((48, 64), dtype=np.float32)
    depth[24, 32] = 4.0
    depth[24, 33] = 1.0
    # Squash both pixels onto one output column with a tiny-fx rgb camera.
    K_rgb = np.array([[1.0, 0.0, 32.0], [0.0, 100.0, 24.0], [0.0, 0.0, 1.0]])
    aligned = align_depth_to_rgb(depth, K, K_rgb, np.eye(4), (48, 64))
    values = aligned[aligned > 0]
    assert values.min() == pytest.approx(1.0)
    assert 4.0 not in values  # farther hit lost the z-buffer


def make_grid() -> np.ndarray:
    grid = np.zeros((17, 40, 40), dtype=np.float32)
    # Explored free box with an obstacle wall on its right edge.
    grid[1, 5:25, 5:25] = 1.0
    grid[0, 5:25, 24] = 1.0
    return grid


def test_extract_frontiers_finds_free_unknown_boundary():
    grid = make_grid()
    frontiers = extract_frontiers(grid, (0.0, 0.0), 0.1, min_cluster_cells=5)
    assert frontiers, "expected at least one frontier"
    assert frontiers[0].frontier_id == "A"
    # Frontier cells must be free and explored, adjacent to unknown; the wall
    # column (col 24) is an obstacle, so no frontier may sit on it.
    for frontier in frontiers:
        assert grid[0, frontier.row, frontier.col] <= 0.5
    # World coordinates map back into the grid extent.
    assert 0.0 < frontiers[0].x_m < 4.0
    assert 0.0 < frontiers[0].y_m < 4.0


def test_extract_frontiers_empty_when_fully_explored():
    grid = np.zeros((17, 10, 10), dtype=np.float32)
    grid[1] = 1.0
    assert extract_frontiers(grid, (0.0, 0.0), 0.1) == []


def test_extract_frontiers_respects_candidate_limit():
    grid = make_grid()
    frontiers = extract_frontiers(grid, (0.0, 0.0), 0.1, max_candidates=2, min_cluster_cells=1)
    assert len(frontiers) <= 2


def test_render_annotated_bev_shape_and_markers():
    grid = make_grid()
    frontiers = extract_frontiers(grid, (0.0, 0.0), 0.1, min_cluster_cells=5)
    image = render_annotated_bev(grid, frontiers, (10, 10), scale=2)
    assert image.shape == (80, 80, 3)
    # Red frontier markers must be present.
    red = (image[:, :, 2] > 200) & (image[:, :, 1] < 100)
    assert red.any()
