import numpy as np
import pytest

from semantic_mapping.depth_backprojection import (
    CameraIntrinsics,
    backproject_depth,
    depth_image_to_meters,
    transform_points,
)


def test_depth_encoding_conversion() -> None:
    depth_mm = np.array([[0, 250, 1000]], dtype=np.uint16)
    np.testing.assert_allclose(
        depth_image_to_meters(depth_mm, "16UC1"),
        [[0.0, 0.25, 1.0]],
    )

    depth_m = np.array([[0.5, np.nan]], dtype=np.float32)
    converted = depth_image_to_meters(depth_m, "32FC1")
    assert converted.dtype == np.float32
    assert converted is not depth_m
    assert converted[0, 0] == pytest.approx(0.5)
    assert np.isnan(converted[0, 1])


def test_known_pixel_backprojection_uses_optical_axes() -> None:
    intrinsics = CameraIntrinsics(3, 3, fx=100.0, fy=100.0, cx=1.0, cy=1.0)
    depth = np.full((3, 3), 2.0, dtype=np.float32)
    result = backproject_depth(
        depth,
        intrinsics,
        stride=1,
        min_depth_m=0.1,
        max_depth_m=3.0,
        edge_filter=False,
    )

    by_pixel = {
        tuple(pixel): point
        for pixel, point in zip(result.pixels_uv, result.points_camera)
    }
    np.testing.assert_allclose(by_pixel[(1, 1)], [0.0, 0.0, 2.0])
    np.testing.assert_allclose(by_pixel[(2, 0)], [0.02, -0.02, 2.0])


def test_invalid_depth_and_rgb_colors_are_filtered_together() -> None:
    intrinsics = CameraIntrinsics(2, 2, fx=1.0, fy=1.0, cx=0.0, cy=0.0)
    depth = np.array([[1.0, 0.0], [np.nan, 6.0]], dtype=np.float32)
    rgb = np.array([[[10, 20, 30], [1, 2, 3]], [[4, 5, 6], [7, 8, 9]]], dtype=np.uint8)
    result = backproject_depth(
        depth,
        intrinsics,
        rgb_image=rgb,
        stride=1,
        min_depth_m=0.25,
        max_depth_m=5.0,
        edge_filter=False,
    )

    np.testing.assert_array_equal(result.pixels_uv, [[0, 0]])
    np.testing.assert_array_equal(result.colors_rgb, [[10, 20, 30]])


def test_depth_discontinuity_filter_rejects_both_edge_sides() -> None:
    intrinsics = CameraIntrinsics(4, 1, fx=1.0, fy=1.0, cx=0.0, cy=0.0)
    depth = np.array([[1.0, 1.0, 2.0, 2.0]], dtype=np.float32)
    result = backproject_depth(
        depth,
        intrinsics,
        stride=1,
        min_depth_m=0.1,
        max_depth_m=3.0,
        edge_filter=True,
        edge_threshold_m=0.1,
    )

    np.testing.assert_array_equal(result.pixels_uv, [[0, 0], [3, 0]])


def test_transform_points_applies_explicit_se3() -> None:
    points = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    transform = np.eye(4)
    transform[:3, 3] = [10.0, -2.0, 0.5]
    np.testing.assert_allclose(transform_points(points, transform), [[11.0, 0.0, 3.5]])


def test_backprojection_rejects_unaligned_shape() -> None:
    intrinsics = CameraIntrinsics(2, 2, fx=1.0, fy=1.0, cx=0.0, cy=0.0)
    with pytest.raises(ValueError, match="does not match CameraInfo"):
        backproject_depth(np.ones((2, 3), dtype=np.float32), intrinsics)
