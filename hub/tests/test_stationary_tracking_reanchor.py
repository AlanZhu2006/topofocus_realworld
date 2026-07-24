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


def write_observation(
    spool: Path,
    *,
    sequence: int,
    capture_time_ns: int,
    pose: np.ndarray,
    transform_version: str,
) -> None:
    directory = spool / f"{sequence:020d}"
    directory.mkdir(parents=True)
    rgb = f"rgb-{sequence}".encode()
    depth = f"depth-{sequence}".encode()
    (directory / "rgb.jpg").write_bytes(rgb)
    (directory / "depth.png").write_bytes(depth)
    metadata = {
        "robot_id": "robot-1",
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
            pose=post_reported @ planar(index * 0.0001, 0.0, 0.0),
            transform_version="other-old-v1",
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
    assert artifact["safety"]["robot_commands_issued"] is False
    assert "holdout_validation" not in artifact


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
