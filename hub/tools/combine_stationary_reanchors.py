#!/usr/bin/env python3
"""Combine independently validated stationary reanchors for both robots.

The two component artifacts must have been produced from the same independently
validated moved-board calibration.  This tool only composes archived evidence;
it never contacts a robot or emits a command.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import time


REQUIRED_HOLDOUT_CHECKS = (
    "sync_skew",
    "board_center_residual",
    "board_normal_residual",
    "board_moved_independently",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact_identity(
    path: Path, *, workspace: Path, classification: str
) -> dict[str, object]:
    resolved = path.resolve()
    try:
        display_path = str(resolved.relative_to(workspace.resolve()))
    except ValueError:
        display_path = str(resolved)
    return {
        "path": display_path,
        "size_bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved),
        "classification": classification,
    }


def identity_core(identity: object) -> tuple[object, object, object]:
    if not isinstance(identity, dict):
        raise ValueError("artifact identity must be an object")
    return (
        identity.get("path"),
        identity.get("size_bytes"),
        identity.get("sha256"),
    )


def require_component(
    payload: dict[str, object],
    *,
    role: str,
    source: dict[str, object],
    source_identity: dict[str, object],
) -> dict[str, object]:
    if payload.get("passed") is not True:
        raise ValueError(f"{role} reanchor did not pass")
    if (
        payload.get("reference_robot") != source.get("reference_robot")
        or payload.get("other_robot") != source.get("other_robot")
    ):
        raise ValueError(f"{role} reanchor robot identities differ from source")
    validation_key = f"{role}_reanchor_validation"
    opposite_key = (
        "other_reanchor_validation"
        if role == "reference"
        else "reference_reanchor_validation"
    )
    validation = payload.get(validation_key)
    if (
        not isinstance(validation, dict)
        or validation.get("passed") is not True
        or validation.get("robot_role") != role
        or payload.get(opposite_key) is not None
    ):
        raise ValueError(f"{role} component is not a single passed reanchor")
    if identity_core(payload.get("derived_from_board_calibration")) != identity_core(
        source_identity
    ):
        raise ValueError(f"{role} reanchor was not derived from the named source")
    safety = payload.get("safety")
    if (
        not isinstance(safety, dict)
        or safety.get("archived_observations_only") is not True
        or safety.get("robot_commands_issued") is not False
        or safety.get("robot_interfaces_used") is not False
    ):
        raise ValueError(f"{role} reanchor lacks fail-closed safety provenance")

    source_reference = source.get("calibration_frame", {})
    component_reference = payload.get("calibration_frame", {})
    if not isinstance(source_reference, dict) or not isinstance(
        component_reference, dict
    ):
        raise ValueError("calibration frame must be an object")
    source_reference = source_reference.get("reference")
    component_reference = component_reference.get("reference")
    if not isinstance(source_reference, dict) or not isinstance(
        component_reference, dict
    ):
        raise ValueError("reference calibration frame must be an object")

    if role == "reference":
        expected_old_version = source_reference.get("transform_version")
        emitted_version = component_reference.get("transform_version")
        if (
            payload.get("transform_version") != source.get("transform_version")
            or payload.get("shared_world_from_other_odom")
            != source.get("shared_world_from_other_odom")
            or not isinstance(
                payload.get("shared_world_from_reference_tracking"), dict
            )
        ):
            raise ValueError("reference component changed the other robot alignment")
    else:
        expected_old_version = source.get("transform_version")
        emitted_version = payload.get("transform_version")
        if (
            component_reference != source_reference
            or not isinstance(payload.get("shared_world_from_other_odom"), dict)
        ):
            raise ValueError("other component changed the reference alignment")
    if (
        validation.get("old_transform_version") != expected_old_version
        or validation.get("new_transform_version") != emitted_version
    ):
        raise ValueError(f"{role} reanchor transform provenance is inconsistent")
    return validation


def build_artifact(args: argparse.Namespace) -> dict[str, object]:
    workspace = args.workspace.resolve()
    source_path = args.source_calibration.resolve()
    reference_path = args.reference_reanchor.resolve()
    other_path = args.other_reanchor.resolve()
    source = json.loads(source_path.read_text(encoding="utf-8"))
    reference = json.loads(reference_path.read_text(encoding="utf-8"))
    other = json.loads(other_path.read_text(encoding="utf-8"))

    if source.get("passed") is not True:
        raise ValueError("source board calibration did not pass")
    holdout = source.get("holdout_validation")
    checks = holdout.get("checks") if isinstance(holdout, dict) else None
    if not isinstance(checks, dict) or not all(
        checks.get(name) is True for name in REQUIRED_HOLDOUT_CHECKS
    ):
        raise ValueError(
            "source calibration lacks an independent moved-board holdout"
        )
    source_identity = artifact_identity(
        source_path,
        workspace=workspace,
        classification=(
            "observed_board_calibration_with_independent_moved_board_holdout"
        ),
    )
    reference_validation = require_component(
        reference,
        role="reference",
        source=source,
        source_identity=source_identity,
    )
    other_validation = require_component(
        other,
        role="other",
        source=source,
        source_identity=source_identity,
    )
    reference_identity = artifact_identity(
        reference_path,
        workspace=workspace,
        classification="validated_reference_stationary_reanchor",
    )
    reference_identity["robot_role"] = "reference"
    other_identity = artifact_identity(
        other_path,
        workspace=workspace,
        classification="validated_other_stationary_reanchor",
    )
    other_identity["robot_role"] = "other"

    return {
        "schema_version": 4,
        "passed": True,
        "calibration_method": (
            "dual_stationary_tracking_epoch_reanchor_of_validated_board_alignment"
        ),
        "computed_at_ns": time.time_ns(),
        "reference_robot": source.get("reference_robot"),
        "other_robot": source.get("other_robot"),
        "shared_frame_calibration_id": args.new_calibration_id,
        "transform_version": other.get("transform_version"),
        "calibration_frame": {
            "reference": reference.get("calibration_frame", {}).get("reference")
        },
        "shared_world_from_reference_tracking": reference.get(
            "shared_world_from_reference_tracking"
        ),
        "shared_world_from_other_odom": other.get(
            "shared_world_from_other_odom"
        ),
        "derived_from_board_calibration": source_identity,
        "reference_reanchor_validation": reference_validation,
        "other_reanchor_validation": other_validation,
        "reanchor_components": {
            "reference": reference_identity,
            "other": other_identity,
        },
        "input_provenance": {
            "status": (
                "two_operator_observed_stationary_robot_epoch_handovers_"
                "composed_from_archived_validated_artifacts"
            ),
            "source_board_calibration": source_identity,
            "reference_reanchor": reference_identity,
            "other_reanchor": other_identity,
        },
        "safety": {
            "archived_observations_only": True,
            "robot_commands_issued": False,
            "robot_interfaces_used": False,
        },
        "note": (
            "Both tracking epochs changed while their robots were "
            "operator-confirmed stationary. Each transform remains tied to "
            "the same independently validated moved-board calibration."
        ),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--source-calibration", type=Path, required=True)
    parser.add_argument("--reference-reanchor", type=Path, required=True)
    parser.add_argument("--other-reanchor", type=Path, required=True)
    parser.add_argument("--new-calibration-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    artifact = build_artifact(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_name(f".{args.output.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(artifact, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, args.output)
    print(
        json.dumps(
            {
                "output": str(args.output.resolve()),
                "calibration_id": artifact["shared_frame_calibration_id"],
                "reference_transform_version": artifact["calibration_frame"][
                    "reference"
                ]["transform_version"],
                "other_transform_version": artifact["transform_version"],
                "robot_commands_issued": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
