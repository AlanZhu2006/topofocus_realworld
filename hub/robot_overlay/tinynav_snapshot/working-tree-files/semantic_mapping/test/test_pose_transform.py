import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from semantic_mapping.pose_provider import (
    PoseBuffer,
    PoseLookupError,
    interpolate_transform,
    make_transform_matrix,
    transform_components,
)


def test_make_transform_matrix_normalizes_quaternion() -> None:
    matrix = make_transform_matrix([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 2.0])
    np.testing.assert_allclose(
        matrix,
        np.array(
            [
                [1.0, 0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0, 2.0],
                [0.0, 0.0, 1.0, 3.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
    )


def test_transform_components_round_trip() -> None:
    expected = make_transform_matrix(
        [1.0, -2.0, 3.0], Rotation.from_euler("xyz", [10, 20, 30], degrees=True).as_quat()
    )
    translation, quaternion = transform_components(expected)
    restored = make_transform_matrix(translation, quaternion)
    np.testing.assert_allclose(restored, expected, atol=1e-7)


def test_interpolate_transform_uses_linear_translation_and_slerp() -> None:
    first = np.eye(4)
    second = np.eye(4)
    second[:3, :3] = Rotation.from_euler("z", 180.0, degrees=True).as_matrix()
    second[:3, 3] = [2.0, 4.0, 6.0]

    middle = interpolate_transform(first, second, 0.5)
    np.testing.assert_allclose(middle[:3, 3], [1.0, 2.0, 3.0])
    rotated_x = middle[:3, :3] @ np.array([1.0, 0.0, 0.0])
    np.testing.assert_allclose(rotated_x, [0.0, 1.0, 0.0], atol=1e-7)


def test_pose_buffer_interpolates_bracketing_samples() -> None:
    buffer = PoseBuffer(max_samples=4)
    first = np.eye(4)
    second = np.eye(4)
    second[0, 3] = 2.0
    buffer.add(1_000_000_000, first)
    buffer.add(1_100_000_000, second)

    result = buffer.lookup(1_050_000_000, max_time_error_ns=50_000_000)
    assert result.interpolated
    assert result.time_error_ns == 50_000_000
    assert result.matrix[0, 3] == pytest.approx(1.0)


def test_pose_buffer_nearest_and_error_limit() -> None:
    buffer = PoseBuffer(max_samples=4)
    pose = np.eye(4)
    buffer.add(1_000, pose)

    result = buffer.lookup(1_010, max_time_error_ns=10)
    assert not result.interpolated
    assert result.time_error_ns == 10
    with pytest.raises(PoseLookupError, match="Nearest pose"):
        buffer.lookup(1_011, max_time_error_ns=10)


def test_pose_buffer_replaces_duplicate_timestamp_and_bounds_size() -> None:
    buffer = PoseBuffer(max_samples=2)
    for timestamp in (1, 2, 3):
        pose = np.eye(4)
        pose[0, 3] = timestamp
        buffer.add(timestamp, pose)
    assert len(buffer) == 2

    replacement = np.eye(4)
    replacement[0, 3] = 30.0
    buffer.add(3, replacement)
    assert len(buffer) == 2
    assert buffer.lookup(3, 0).matrix[0, 3] == pytest.approx(30.0)
