from __future__ import annotations

from collections.abc import Sequence


def invert_rigid(matrix: Sequence[float]) -> tuple[float, ...]:
    if len(matrix) != 16:
        raise ValueError("expected a row-major 4x4 matrix")
    rotation = (
        (matrix[0], matrix[1], matrix[2]),
        (matrix[4], matrix[5], matrix[6]),
        (matrix[8], matrix[9], matrix[10]),
    )
    translation = (matrix[3], matrix[7], matrix[11])
    inverse_rotation = tuple(tuple(rotation[column][row] for column in range(3)) for row in range(3))
    inverse_translation = tuple(
        -sum(inverse_rotation[row][column] * translation[column] for column in range(3))
        for row in range(3)
    )
    return (
        inverse_rotation[0][0], inverse_rotation[0][1], inverse_rotation[0][2], inverse_translation[0],
        inverse_rotation[1][0], inverse_rotation[1][1], inverse_rotation[1][2], inverse_translation[1],
        inverse_rotation[2][0], inverse_rotation[2][1], inverse_rotation[2][2], inverse_translation[2],
        0.0, 0.0, 0.0, 1.0,
    )


def transform_point(matrix: Sequence[float], point: Sequence[float]) -> tuple[float, float, float]:
    if len(matrix) != 16 or len(point) != 3:
        raise ValueError("expected a row-major 4x4 matrix and a 3-vector")
    return tuple(
        sum(matrix[row * 4 + column] * point[column] for column in range(3)) + matrix[row * 4 + 3]
        for row in range(3)
    )  # type: ignore[return-value]


def compose_rigid(a: Sequence[float], b: Sequence[float]) -> tuple[float, ...]:
    """T_x_z = T_x_y @ T_y_z for row-major flat 4x4 matrices."""
    if len(a) != 16 or len(b) != 16:
        raise ValueError("expected row-major 4x4 matrices")
    return tuple(
        sum(a[row * 4 + k] * b[k * 4 + column] for k in range(4))
        for row in range(4)
        for column in range(4)
    )

