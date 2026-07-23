"""Strict loader for robot body-to-command-camera calibration artifacts."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class BaseCameraCalibration:
    robot_id: str
    camera_frame: str
    matrix: tuple[float, ...]
    source_path: str
    source_size_bytes: int
    source_sha256: str
    measurement_status: str

    def wire_transform(self) -> dict[str, object]:
        return {
            "parent_frame": "base_link",
            "child_frame": self.camera_frame,
            "matrix": list(self.matrix),
        }


def _validate_matrix(values: object) -> tuple[float, ...]:
    if not isinstance(values, list) or len(values) != 16:
        raise ValueError("base_T_camera must contain 16 row-major values")
    matrix = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in matrix):
        raise ValueError("base_T_camera contains a non-finite value")
    if any(
        abs(matrix[12 + index] - expected) > 1e-5
        for index, expected in enumerate((0.0, 0.0, 0.0, 1.0))
    ):
        raise ValueError("base_T_camera last row is not homogeneous")
    rotation = (matrix[0:3], matrix[4:7], matrix[8:11])
    for row in rotation:
        if abs(sum(value * value for value in row) - 1.0) > 2e-2:
            raise ValueError("base_T_camera rotation row is not unit length")
    for left, right in ((0, 1), (0, 2), (1, 2)):
        if abs(sum(rotation[left][i] * rotation[right][i] for i in range(3))) > 2e-2:
            raise ValueError("base_T_camera rotation is not orthogonal")
    determinant = (
        rotation[0][0]
        * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1]
        * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2]
        * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(determinant - 1.0) > 2e-2:
        raise ValueError("base_T_camera rotation must have determinant +1")
    return matrix


def load_base_camera_calibration(
    path: Path,
    *,
    expected_robot_id: str,
    expected_camera_frame: str,
) -> BaseCameraCalibration:
    resolved = path.expanduser().resolve()
    payload = resolved.read_bytes()
    raw = json.loads(payload)
    if raw.get("schema_version") != "focus-base-camera-calibration-v1":
        raise ValueError("unsupported base-camera calibration schema")
    if raw.get("passed") is not True:
        raise ValueError("base-camera calibration did not pass")
    if raw.get("robot_id") != expected_robot_id:
        raise ValueError("base-camera calibration robot ID mismatch")
    transform = raw.get("base_T_camera")
    if not isinstance(transform, dict):
        raise ValueError("base-camera calibration has no base_T_camera")
    if transform.get("parent_frame") != "base_link":
        raise ValueError("base-camera calibration parent must be base_link")
    camera_frame = str(transform.get("child_frame", ""))
    if camera_frame != expected_camera_frame:
        raise ValueError("base-camera calibration camera frame mismatch")
    measurement = raw.get("measurement")
    if not isinstance(measurement, dict):
        raise ValueError("base-camera calibration has no measurement provenance")
    status = str(measurement.get("status", ""))
    if status not in {
        "operator_measured_physical_mount",
        "surveyed_physical_mount",
        "observed_robot_tf",
    }:
        raise ValueError("base-camera calibration is not classified as measured")
    return BaseCameraCalibration(
        robot_id=expected_robot_id,
        camera_frame=camera_frame,
        matrix=_validate_matrix(transform.get("matrix")),
        source_path=str(resolved),
        source_size_bytes=len(payload),
        source_sha256=hashlib.sha256(payload).hexdigest(),
        measurement_status=status,
    )
