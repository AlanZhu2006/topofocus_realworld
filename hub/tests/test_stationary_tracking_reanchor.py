from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import subprocess

import numpy as np


HUB = Path(__file__).resolve().parents[1]
TOOL = HUB / "tools" / "reanchor_stationary_tracking_epoch.py"
PYTHON = HUB / ".venv" / "bin" / "python"
CONFIRMATION = "OPERATOR_CONFIRMS_ROBOT_STATIONARY_ACROSS_TRACKING_RESTART"


def planar(x: float, y: float, yaw: float) -> np.ndarray:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            [cosine, -sine, 0.0, x],
            [sine, cosine, 0.0, y],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def identity(path: Path, workspace: Path) -> dict[str, object]:
    return {
        "path": str(path.relative_to(workspace)),
        "size_bytes": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "classification": "test artifact",
    }


def write_observation(
    spool: Path,
    *,
    sequence: int,
    capture_time_ns: int,
    pose: np.ndarray,
    transform_version: str,
    robot_id: str = "robot-1",
) -> None:
    directory = spool / f"{sequence:020d}"
    directory.mkdir(parents=True)
    rgb = f"rgb-{sequence}".encode()
    depth = f"depth-{sequence}".encode()
    (directory / "rgb.jpg").write_bytes(rgb)
    (directory / "depth.png").write_bytes(depth)
    metadata = {
        "robot_id": robot_id,
        "sequence": sequence,
        "capture_time_ns": capture_time_ns,
        "rgb_size_bytes": len(rgb),
        "rgb_sha256": sha(rgb),
        "depth_size_bytes": len(depth),
        "depth_sha256": sha(depth),
        "pose": {
            "transform_version": transform_version,
            "shared_T_camera": {
                "parent_frame": "shared_world",
                "child_frame": "camera",
                "matrix": pose.reshape(-1).tolist(),
            },
        },
    }
    (directory / "metadata.json").write_text(json.dumps(metadata))


def source_calibration(path: Path, transform: np.ndarray) -> None:
    path.write_text(
        json.dumps(
            {
                "passed": True,
                "reference_robot": "robot-0",
                "other_robot": "robot-1",
                "shared_frame_calibration_id": "board-v1",
                "transform_version": "other-old-v1",
                "calibration_frame": {
                    "reference": {"transform_version": "reference-v1"}
                },
                "shared_world_from_other_odom": {
                    "parent_frame": "shared_world",
                    "child_frame": "robot-1_odom",
                    "matrix": transform.reshape(-1).tolist(),
                },
                "holdout_validation": {
                    "checks": {
                        "sync_skew": True,
                        "board_center_residual": True,
                        "board_normal_residual": True,
                        "board_moved_independently": True,
                    }
                },
            }
        )
    )


def test_stationary_reanchor_recovers_new_planar_epoch(tmp_path: Path):
    spool = tmp_path / "spool"
    source = tmp_path / "board.json"
    output = tmp_path / "reanchored.json"
    old_transform = planar(1.0, -0.5, 0.2)
    source_calibration(source, old_transform)
    pre_pose = planar(2.0, 3.0, 0.4)
    correction = planar(-0.7, 0.2, -0.3)
    post_reported = np.linalg.inv(correction) @ pre_pose
    post_raw = np.linalg.inv(old_transform) @ post_reported
    for index in range(3):
        write_observation(
            spool,
            sequence=10 + index,
            capture_time_ns=100 + index,
            pose=pre_pose @ planar(index * 0.0001, 0.0, 0.0),
            transform_version="other-old-v1",
        )
        write_observation(
            spool,
            sequence=20 + index,
            capture_time_ns=200 + index,
            pose=post_raw @ planar(index * 0.0001, 0.0, 0.0),
            transform_version="other-raw-restart-v2",
        )

    result = subprocess.run(
        [
            str(PYTHON),
            str(TOOL),
            "--workspace",
            str(tmp_path),
            "--source-calibration",
            str(source),
            "--spool",
            str(spool),
            "--pre-first",
            "10",
            "--pre-last",
            "12",
            "--post-first",
            "20",
            "--post-last",
            "22",
            "--post-transform-version",
            "other-raw-restart-v2",
            "--post-observations-are-raw-tracking",
            "--new-calibration-id",
            "board-restart-v2",
            "--new-transform-version",
            "other-restart-v2",
            "--tracking-restart-boot-id",
            "boot-2",
            "--operator-confirmation",
            CONFIRMATION,
            "--output",
            str(output),
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    artifact = json.loads(output.read_text())
    expected = correction @ old_transform
    actual = np.asarray(
        artifact["shared_world_from_other_odom"]["matrix"]
    ).reshape(4, 4)
    np.testing.assert_allclose(actual, expected, atol=3e-4)
    assert artifact["other_reanchor_validation"]["passed"] is True
    assert (
        artifact["other_reanchor_validation"]["post_observation_pose_model"]
        == "raw_tracking_normalized_with_old_shared_transform"
    )
    assert artifact["safety"]["robot_commands_issued"] is False
    assert "holdout_validation" not in artifact


def test_stationary_reanchor_recovers_reference_epoch(tmp_path: Path):
    spool = tmp_path / "spool"
    source = tmp_path / "board.json"
    output = tmp_path / "reanchored.json"
    other_transform = planar(1.0, -0.5, 0.2)
    source_calibration(source, other_transform)
    pre_pose = planar(0.1, -0.2, 0.05)
    correction = planar(0.8, -0.4, 0.3)
    post_reported = np.linalg.inv(correction) @ pre_pose
    for index in range(3):
        write_observation(
            spool,
            sequence=10 + index,
            capture_time_ns=100 + index,
            pose=pre_pose @ planar(index * 0.0001, 0.0, 0.0),
            transform_version="reference-v1",
            robot_id="robot-0",
        )
        write_observation(
            spool,
            sequence=20 + index,
            capture_time_ns=200 + index,
            pose=post_reported @ planar(index * 0.0001, 0.0, 0.0),
            transform_version="reference-v1",
            robot_id="robot-0",
        )

    result = subprocess.run(
        [
            str(PYTHON),
            str(TOOL),
            "--workspace",
            str(tmp_path),
            "--source-calibration",
            str(source),
            "--spool",
            str(spool),
            "--robot-id",
            "robot-0",
            "--pre-first",
            "10",
            "--pre-last",
            "12",
            "--post-first",
            "20",
            "--post-last",
            "22",
            "--new-calibration-id",
            "board-reference-restart-v2",
            "--new-transform-version",
            "reference-restart-v2",
            "--tracking-restart-boot-id",
            "boot-2",
            "--operator-confirmation",
            CONFIRMATION,
            "--output",
            str(output),
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    artifact = json.loads(output.read_text())
    actual = np.asarray(
        artifact["shared_world_from_reference_tracking"]["matrix"]
    ).reshape(4, 4)
    np.testing.assert_allclose(actual, correction, atol=3e-4)
    np.testing.assert_allclose(
        np.asarray(
            artifact["shared_world_from_other_odom"]["matrix"]
        ).reshape(4, 4),
        other_transform,
    )
    assert (
        artifact["calibration_frame"]["reference"]["transform_version"]
        == "reference-restart-v2"
    )
    assert artifact["transform_version"] == "other-old-v1"
    assert artifact["reference_reanchor_validation"]["passed"] is True
    assert artifact["safety"]["robot_commands_issued"] is False


def test_stationary_reanchor_extends_a_verified_calibration_chain(
    tmp_path: Path,
):
    spool = tmp_path / "spool"
    board_path = tmp_path / "board.json"
    current_path = tmp_path / "current-dual.json"
    output = tmp_path / "reanchored.json"
    board_transform = planar(1.0, -0.5, 0.2)
    source_calibration(board_path, board_transform)
    board = json.loads(board_path.read_text())
    current_transform = planar(0.4, -0.7, -0.1) @ board_transform
    current_reference = {
        "transform_version": "reference-reanchored-v2"
    }
    current_path.write_text(
        json.dumps(
            {
                "passed": True,
                "reference_robot": "robot-0",
                "other_robot": "robot-1",
                "shared_frame_calibration_id": "dual-v2",
                "transform_version": "other-reanchored-v2",
                "calibration_frame": {"reference": current_reference},
                "shared_world_from_reference_tracking": {
                    "parent_frame": "shared_world",
                    "child_frame": "robot-0_tracking",
                    "matrix": planar(0.2, 0.3, 0.1).reshape(-1).tolist(),
                },
                "shared_world_from_other_odom": {
                    "parent_frame": "shared_world",
                    "child_frame": "robot-1_odom",
                    "matrix": current_transform.reshape(-1).tolist(),
                },
                "derived_from_board_calibration": identity(
                    board_path, tmp_path
                ),
            }
        )
    )
    pre_pose = planar(2.0, 3.0, 0.4)
    correction = planar(-0.7, 0.2, -0.3)
    post_reported = np.linalg.inv(correction) @ pre_pose
    for index in range(3):
        write_observation(
            spool,
            sequence=10 + index,
            capture_time_ns=100 + index,
            pose=pre_pose @ planar(index * 0.0001, 0.0, 0.0),
            transform_version="other-reanchored-v2",
        )
        write_observation(
            spool,
            sequence=20 + index,
            capture_time_ns=200 + index,
            pose=post_reported @ planar(index * 0.0001, 0.0, 0.0),
            transform_version="other-reanchored-v2",
        )

    result = subprocess.run(
        [
            str(PYTHON),
            str(TOOL),
            "--workspace",
            str(tmp_path),
            "--source-calibration",
            str(current_path),
            "--spool",
            str(spool),
            "--pre-first",
            "10",
            "--pre-last",
            "12",
            "--post-first",
            "20",
            "--post-last",
            "22",
            "--new-calibration-id",
            "dual-v3",
            "--new-transform-version",
            "other-reanchored-v3",
            "--tracking-restart-boot-id",
            "boot-3",
            "--operator-confirmation",
            CONFIRMATION,
            "--output",
            str(output),
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    artifact = json.loads(output.read_text())
    actual = np.asarray(
        artifact["shared_world_from_other_odom"]["matrix"]
    ).reshape(4, 4)
    np.testing.assert_allclose(
        actual, correction @ current_transform, atol=3e-4
    )
    assert artifact["calibration_frame"]["reference"] == current_reference
    assert "shared_world_from_reference_tracking" in artifact
    assert artifact["derived_from_calibration"][
        "sha256"
    ] == hashlib.sha256(current_path.read_bytes()).hexdigest()
    assert artifact["derived_from_board_calibration"][
        "sha256"
    ] == hashlib.sha256(board_path.read_bytes()).hexdigest()


def test_stationary_reanchor_rejects_missing_confirmation(tmp_path: Path):
    result = subprocess.run(
        [
            str(PYTHON),
            str(TOOL),
            "--workspace",
            str(tmp_path),
            "--source-calibration",
            str(tmp_path / "missing.json"),
            "--spool",
            str(tmp_path / "missing-spool"),
            "--pre-first",
            "1",
            "--pre-last",
            "3",
            "--post-first",
            "4",
            "--post-last",
            "6",
            "--new-calibration-id",
            "new",
            "--new-transform-version",
            "new",
            "--tracking-restart-boot-id",
            "boot",
            "--operator-confirmation",
            "NO",
            "--output",
            str(tmp_path / "out.json"),
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0
    assert CONFIRMATION in result.stderr
    assert not (tmp_path / "out.json").exists()
