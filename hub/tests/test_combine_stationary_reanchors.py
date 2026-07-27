from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess


HUB = Path(__file__).resolve().parents[1]
TOOL = HUB / "tools" / "combine_stationary_reanchors.py"
PYTHON = HUB / ".venv" / "bin" / "python"


def identity(path: Path, workspace: Path) -> dict[str, object]:
    return {
        "path": str(path.relative_to(workspace)),
        "size_bytes": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "classification": "test artifact",
    }


def write_source(path: Path) -> dict[str, object]:
    payload = {
        "passed": True,
        "reference_robot": "robot-0",
        "other_robot": "robot-1",
        "transform_version": "other-v1",
        "calibration_frame": {
            "reference": {"transform_version": "reference-v1"}
        },
        "shared_world_from_other_odom": {"matrix": list(range(16))},
        "holdout_validation": {
            "checks": {
                "sync_skew": True,
                "board_center_residual": True,
                "board_normal_residual": True,
                "board_moved_independently": True,
            }
        },
    }
    path.write_text(json.dumps(payload))
    return payload


def write_component(
    path: Path,
    *,
    source: dict[str, object],
    source_identity: dict[str, object],
    role: str,
) -> None:
    payload = {
        "schema_version": 3,
        "passed": True,
        "calibration_method": (
            "stationary_tracking_epoch_reanchor_of_validated_board_alignment"
        ),
        "reference_robot": "robot-0",
        "other_robot": "robot-1",
        "shared_frame_calibration_id": f"{role}-calibration",
        "transform_version": (
            "other-v2" if role == "other" else source["transform_version"]
        ),
        "calibration_frame": {
            "reference": {
                "transform_version": (
                    "reference-v2" if role == "reference" else "reference-v1"
                )
            }
        },
        "shared_world_from_other_odom": (
            {"matrix": list(reversed(range(16)))}
            if role == "other"
            else source["shared_world_from_other_odom"]
        ),
        "derived_from_board_calibration": source_identity,
        f"{role}_reanchor_validation": {
            "passed": True,
            "robot_role": role,
            "old_transform_version": f"{role}-v1",
            "new_transform_version": f"{role}-v2",
        },
        "safety": {
            "archived_observations_only": True,
            "robot_commands_issued": False,
            "robot_interfaces_used": False,
        },
    }
    if role == "reference":
        payload["shared_world_from_reference_tracking"] = {
            "matrix": list(reversed(range(16)))
        }
    path.write_text(json.dumps(payload))


def test_combines_two_reanchors_from_the_same_board_source(tmp_path: Path):
    source_path = tmp_path / "source.json"
    reference_path = tmp_path / "reference.json"
    other_path = tmp_path / "other.json"
    output_path = tmp_path / "combined.json"
    source = write_source(source_path)
    source_identity = identity(source_path, tmp_path)
    write_component(
        reference_path,
        source=source,
        source_identity=source_identity,
        role="reference",
    )
    write_component(
        other_path,
        source=source,
        source_identity=source_identity,
        role="other",
    )

    result = subprocess.run(
        [
            str(PYTHON),
            str(TOOL),
            "--workspace",
            str(tmp_path),
            "--source-calibration",
            str(source_path),
            "--reference-reanchor",
            str(reference_path),
            "--other-reanchor",
            str(other_path),
            "--new-calibration-id",
            "dual-v1",
            "--output",
            str(output_path),
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    payload = json.loads(output_path.read_text())
    assert payload["passed"] is True
    assert payload["shared_frame_calibration_id"] == "dual-v1"
    assert payload["calibration_frame"]["reference"]["transform_version"] == (
        "reference-v2"
    )
    assert payload["transform_version"] == "other-v2"
    assert payload["reference_reanchor_validation"]["passed"] is True
    assert payload["other_reanchor_validation"]["passed"] is True
    assert payload["safety"]["robot_commands_issued"] is False


def test_rejects_components_from_a_different_board_source(tmp_path: Path):
    source_path = tmp_path / "source.json"
    reference_path = tmp_path / "reference.json"
    other_path = tmp_path / "other.json"
    source = write_source(source_path)
    source_identity = identity(source_path, tmp_path)
    write_component(
        reference_path,
        source=source,
        source_identity=source_identity,
        role="reference",
    )
    source_identity["sha256"] = "0" * 64
    write_component(
        other_path,
        source=source,
        source_identity=source_identity,
        role="other",
    )

    result = subprocess.run(
        [
            str(PYTHON),
            str(TOOL),
            "--workspace",
            str(tmp_path),
            "--source-calibration",
            str(source_path),
            "--reference-reanchor",
            str(reference_path),
            "--other-reanchor",
            str(other_path),
            "--new-calibration-id",
            "dual-v1",
            "--output",
            str(tmp_path / "combined.json"),
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0
    assert "not derived from the named source" in result.stderr


def test_combines_a_chained_other_reanchor_with_current_reference(
    tmp_path: Path,
):
    source_path = tmp_path / "source.json"
    reference_path = tmp_path / "reference.json"
    current_path = tmp_path / "current-dual.json"
    other_path = tmp_path / "other-chained.json"
    output_path = tmp_path / "combined.json"
    source = write_source(source_path)
    source_identity = identity(source_path, tmp_path)
    write_component(
        reference_path,
        source=source,
        source_identity=source_identity,
        role="reference",
    )
    current = {
        "passed": True,
        "reference_robot": "robot-0",
        "other_robot": "robot-1",
        "transform_version": "other-v2",
        "calibration_frame": {
            "reference": {"transform_version": "reference-v2"}
        },
        "shared_world_from_other_odom": {"matrix": list(range(16))},
        "derived_from_board_calibration": source_identity,
    }
    current_path.write_text(json.dumps(current))
    other = {
        "schema_version": 4,
        "passed": True,
        "calibration_method": (
            "stationary_tracking_epoch_reanchor_of_validated_board_alignment"
        ),
        "reference_robot": "robot-0",
        "other_robot": "robot-1",
        "shared_frame_calibration_id": "other-v3-calibration",
        "transform_version": "other-v3",
        "calibration_frame": current["calibration_frame"],
        "shared_world_from_other_odom": {
            "matrix": list(reversed(range(16)))
        },
        "derived_from_board_calibration": source_identity,
        "derived_from_calibration": identity(current_path, tmp_path),
        "other_reanchor_validation": {
            "passed": True,
            "robot_role": "other",
            "old_transform_version": "other-v2",
            "new_transform_version": "other-v3",
        },
        "safety": {
            "archived_observations_only": True,
            "robot_commands_issued": False,
            "robot_interfaces_used": False,
        },
    }
    other_path.write_text(json.dumps(other))

    result = subprocess.run(
        [
            str(PYTHON),
            str(TOOL),
            "--workspace",
            str(tmp_path),
            "--source-calibration",
            str(source_path),
            "--reference-reanchor",
            str(reference_path),
            "--other-reanchor",
            str(other_path),
            "--new-calibration-id",
            "dual-v3",
            "--output",
            str(output_path),
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    payload = json.loads(output_path.read_text())
    assert payload["shared_frame_calibration_id"] == "dual-v3"
    assert payload["calibration_frame"]["reference"]["transform_version"] == (
        "reference-v2"
    )
    assert payload["transform_version"] == "other-v3"
