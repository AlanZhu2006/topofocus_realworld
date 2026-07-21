import numpy as np
import pytest
from std_msgs.msg import Header

from semantic_mapping.pointcloud import (
    build_xyz_probability_cloud,
    build_xyz_semantic_cloud,
    build_xyzrgb_cloud,
    read_xyz_cloud,
    read_xyzuv_cloud,
)


def test_xyzrgb_cloud_layout_and_packed_color() -> None:
    header = Header(frame_id="map")
    points = np.array([[1.0, 2.0, 3.0], [-1.0, 0.5, 4.0]], dtype=np.float32)
    colors = np.array([[10, 20, 30], [255, 128, 0]], dtype=np.uint8)

    message = build_xyzrgb_cloud(points, colors, header)

    assert message.header.frame_id == "map"
    assert message.height == 1
    assert message.width == 2
    assert message.point_step == 16
    assert message.row_step == 32
    assert [(field.name, field.offset) for field in message.fields] == [
        ("x", 0),
        ("y", 4),
        ("z", 8),
        ("rgb", 12),
    ]
    dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    decoded = np.frombuffer(message.data, dtype=dtype)
    np.testing.assert_allclose(decoded["x"], [1.0, -1.0])
    np.testing.assert_array_equal(decoded["rgb"], [0x0A141E, 0xFF8000])
    np.testing.assert_allclose(read_xyz_cloud(message), points)


def test_xyzrgb_cloud_rejects_color_count_mismatch() -> None:
    with pytest.raises(ValueError, match="RGB colors"):
        build_xyzrgb_cloud(
            np.zeros((2, 3), dtype=np.float32),
            np.zeros((1, 3), dtype=np.uint8),
            Header(),
        )


def test_xyzrgbuv_cloud_preserves_pixel_correspondence() -> None:
    points = np.asarray([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    colors = np.asarray([[10, 20, 30], [40, 50, 60]], dtype=np.uint8)
    pixels = np.asarray([[7, 8], [900, 400]], dtype=np.int32)
    message = build_xyzrgb_cloud(points, colors, Header(), pixels)
    assert message.point_step == 24
    assert [(field.name, field.offset) for field in message.fields[-2:]] == [
        ("u", 16),
        ("v", 20),
    ]
    decoded_points, decoded_pixels = read_xyzuv_cloud(message)
    np.testing.assert_allclose(decoded_points, points)
    np.testing.assert_array_equal(decoded_pixels, pixels)


def test_xyzuv_reader_requires_pixel_fields() -> None:
    message = build_xyzrgb_cloud(
        np.zeros((1, 3), dtype=np.float32),
        np.zeros((1, 3), dtype=np.uint8),
        Header(),
    )
    with pytest.raises(ValueError, match="pixel fields"):
        read_xyzuv_cloud(message)


def test_xyz_probability_cloud_round_trip() -> None:
    points = np.array([[1.0, -2.0, 0.5], [3.0, 4.0, 5.0]], dtype=np.float32)
    probabilities = np.array([0.75, 0.95], dtype=np.float32)
    message = build_xyz_probability_cloud(points, probabilities, Header(frame_id="map"))

    assert message.fields[-1].name == "occupancy"
    assert message.point_step == 16
    np.testing.assert_allclose(read_xyz_cloud(message), points)


def test_xyz_semantic_cloud_layout() -> None:
    points = np.asarray([[1.0, 2.0, 3.0]], dtype=np.float32)
    message = build_xyz_semantic_cloud(
        points,
        np.asarray([5], dtype=np.uint8),
        np.asarray([0.75], dtype=np.float32),
        np.asarray([3], dtype=np.uint32),
        np.asarray([[220, 160, 60]], dtype=np.uint8),
        Header(frame_id="map"),
    )
    assert message.point_step == 28
    assert [(field.name, field.offset) for field in message.fields[-3:]] == [
        ("semantic_label", 16),
        ("confidence", 20),
        ("observations", 24),
    ]
    np.testing.assert_allclose(read_xyz_cloud(message), points)
