"""Validate the public real-world deployment baseline without touching robots."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
from typing import Any


SCHEMA_VERSION = "focus-realworld-deployment-v1"
DEFAULT_MANIFEST = Path("hub/config/deployments/realworld_dual_robot_v1.json")
PROVENANCE_CLASSES = {"observed", "source-derived", "unverified"}
REQUIRED_HARDWARE_ROLES = {"hub", "robot-0", "robot-1"}
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
GIT_OBJECT_RE = re.compile(r"^[0-9a-f]{40}$")


class BaselineValidationError(ValueError):
    """Raised when the checked deployment no longer matches its contract."""


@dataclass(frozen=True)
class BaselineSummary:
    baseline_id: str
    file_count: int
    total_bytes: int


def _mapping(value: Any, context: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise BaselineValidationError(f"{context} must be an object")
    return value


def _nonempty_string(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise BaselineValidationError(f"{context} must be a non-empty string")
    return value


def _expect(value: Any, expected: Any, context: str) -> None:
    if value != expected:
        raise BaselineValidationError(
            f"{context} must be {expected!r}, found {value!r}"
        )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _workspace_file(workspace: Path, relative: str) -> Path:
    logical = PurePosixPath(relative)
    if logical.is_absolute() or ".." in logical.parts:
        raise BaselineValidationError(
            f"file contract escapes the workspace: {relative!r}"
        )
    candidate = (workspace / Path(*logical.parts)).resolve()
    try:
        candidate.relative_to(workspace)
    except ValueError as exc:
        raise BaselineValidationError(
            f"file contract resolves outside the workspace: {relative!r}"
        ) from exc
    if not candidate.is_file():
        raise BaselineValidationError(
            f"contracted deployment file is missing: {relative}"
        )
    return candidate


def _validate_hardware(manifest: dict[str, Any]) -> None:
    hardware = _mapping(manifest.get("hardware"), "hardware")
    _expect(set(hardware), REQUIRED_HARDWARE_ROLES, "hardware roles")
    for role, raw_entry in hardware.items():
        entry = _mapping(raw_entry, f"hardware.{role}")
        _nonempty_string(entry.get("compute"), f"hardware.{role}.compute")
        _nonempty_string(
            entry.get("responsibility"),
            f"hardware.{role}.responsibility",
        )
        evidence = _mapping(
            entry.get("evidence"), f"hardware.{role}.evidence"
        )
        classification = evidence.get("classification")
        if classification not in PROVENANCE_CLASSES:
            raise BaselineValidationError(
                f"hardware.{role}.evidence.classification is invalid"
            )
        _nonempty_string(
            evidence.get("source"), f"hardware.{role}.evidence.source"
        )


def _validate_control_contract(manifest: dict[str, Any]) -> None:
    contract = _mapping(
        manifest.get("control_contract"), "control_contract"
    )
    _expect(
        contract.get("high_level_commands"),
        ["GOAL", "HOLD", "STOP"],
        "control_contract.high_level_commands",
    )
    _expect(
        contract.get("target_expiry_required"),
        True,
        "control_contract.target_expiry_required",
    )
    _expect(
        contract.get("low_level_motion_authority"),
        "robot-local",
        "control_contract.low_level_motion_authority",
    )
    _expect(
        contract.get("raw_velocity_topic"),
        "/cmd_vel",
        "control_contract.raw_velocity_topic",
    )
    _expect(
        contract.get("guarded_velocity_topic"),
        "/focus_guarded_cmd_vel",
        "control_contract.guarded_velocity_topic",
    )

    planning = _mapping(manifest.get("planning"), "planning")
    router = _mapping(planning.get("route_planner"), "planning.route_planner")
    _expect(
        router.get("algorithm"),
        "bounded-8-connected-a-star",
        "planning.route_planner.algorithm",
    )
    _expect(
        router.get("unknown_cells_traversable"),
        False,
        "planning.route_planner.unknown_cells_traversable",
    )
    _expect(
        router.get("diagonal_corner_cutting"),
        False,
        "planning.route_planner.diagonal_corner_cutting",
    )
    local = _mapping(planning.get("local_planner"), "planning.local_planner")
    _expect(
        local.get("implementation"),
        "TinyNav trajectory lattice with ESDF scoring",
        "planning.local_planner.implementation",
    )
    _expect(
        local.get("all_candidates_in_collision"),
        "STOP",
        "planning.local_planner.all_candidates_in_collision",
    )
    controller = _mapping(planning.get("controller"), "planning.controller")
    _expect(
        controller.get("implementation"),
        "TinyNav path follower",
        "planning.controller.implementation",
    )
    _expect(
        controller.get("continuous_turn_timeout_s"),
        12.0,
        "planning.controller.continuous_turn_timeout_s",
    )
    _expect(
        controller.get("continuous_turn_timeout_result"),
        "LOCAL_PLANNER_TURN_STALLED and fresh source replan",
        "planning.controller.continuous_turn_timeout_result",
    )
    semantic = _mapping(
        planning.get("semantic_execution_confirmation"),
        "planning.semantic_execution_confirmation",
    )
    _expect(
        semantic.get("source_semantic_map_preserved"),
        True,
        "planning.semantic_execution_confirmation."
        "source_semantic_map_preserved",
    )
    _expect(
        semantic.get("minimum_component_cells"),
        3,
        "planning.semantic_execution_confirmation."
        "minimum_component_cells",
    )
    _expect(
        semantic.get("strong_component_cells"),
        25,
        "planning.semantic_execution_confirmation."
        "strong_component_cells",
    )
    _expect(
        semantic.get(
            "independent_current_frame_detector_required_for_"
            "compact_components"
        ),
        True,
        "planning.semantic_execution_confirmation."
        "independent_current_frame_detector_required_for_compact_components",
    )
    _expect(
        semantic.get("semantic_map_reinforcement"),
        False,
        "planning.semantic_execution_confirmation."
        "semantic_map_reinforcement",
    )
    continuity = _mapping(
        planning.get("frontier_goal_continuity"),
        "planning.frontier_goal_continuity",
    )
    _expect(
        continuity.get("source_switch_distance_m"),
        1.25,
        "planning.frontier_goal_continuity."
        "source_switch_distance_m",
    )
    _expect(
        continuity.get("physical_completion_distance_m"),
        0.5,
        "planning.frontier_goal_continuity."
        "physical_completion_distance_m",
    )


def _validate_software(manifest: dict[str, Any]) -> None:
    software = _mapping(manifest.get("software"), "software")
    tinynav = _mapping(software.get("tinynav"), "software.tinynav")
    source_url = _nonempty_string(
        tinynav.get("source_url"), "software.tinynav.source_url"
    )
    if not source_url.startswith("https://"):
        raise BaselineValidationError(
            "software.tinynav.source_url must use HTTPS"
        )
    for field in ("base_commit", "reconstructed_commit", "reconstructed_tree"):
        value = _nonempty_string(
            tinynav.get(field), f"software.tinynav.{field}"
        )
        if not GIT_OBJECT_RE.fullmatch(value):
            raise BaselineValidationError(
                f"software.tinynav.{field} must be a 40-character Git object"
            )

    unitree = _mapping(software.get("unitree_sdk2"), "software.unitree_sdk2")
    for field in ("upstream_url", "resolved_source_url"):
        value = _nonempty_string(
            unitree.get(field), f"software.unitree_sdk2.{field}"
        )
        if not value.startswith("https://"):
            raise BaselineValidationError(
                f"software.unitree_sdk2.{field} must use HTTPS"
            )
    revision = _nonempty_string(
        unitree.get("revision"), "software.unitree_sdk2.revision"
    )
    if not GIT_OBJECT_RE.fullmatch(revision):
        raise BaselineValidationError(
            "software.unitree_sdk2.revision must be a 40-character Git object"
        )


def validate_public_baseline(
    workspace: str | Path,
    manifest_path: str | Path | None = None,
) -> BaselineSummary:
    """Validate metadata, safety invariants, and byte-level file contracts."""

    root = Path(workspace).resolve()
    if not root.is_dir():
        raise BaselineValidationError(f"workspace does not exist: {root}")
    manifest_file = (
        Path(manifest_path)
        if manifest_path is not None
        else root / DEFAULT_MANIFEST
    )
    try:
        manifest = _mapping(
            json.loads(manifest_file.read_text(encoding="utf-8")),
            "manifest",
        )
    except (OSError, json.JSONDecodeError) as exc:
        raise BaselineValidationError(
            f"cannot read deployment manifest {manifest_file}: {exc}"
        ) from exc

    _expect(
        manifest.get("schema_version"),
        SCHEMA_VERSION,
        "schema_version",
    )
    baseline_id = _nonempty_string(
        manifest.get("baseline_id"), "baseline_id"
    )
    publication = _mapping(manifest.get("publication"), "publication")
    _expect(publication.get("code_scope"), "hub/", "publication.code_scope")
    _expect(
        publication.get("repository_license_status"),
        "owner-review-required",
        "publication.repository_license_status",
    )
    _validate_hardware(manifest)
    _validate_control_contract(manifest)
    _validate_software(manifest)

    contracts = manifest.get("file_contracts")
    if not isinstance(contracts, list) or not contracts:
        raise BaselineValidationError("file_contracts must be a non-empty list")
    seen: set[str] = set()
    total_bytes = 0
    for index, raw_contract in enumerate(contracts):
        contract = _mapping(raw_contract, f"file_contracts[{index}]")
        relative = _nonempty_string(
            contract.get("path"), f"file_contracts[{index}].path"
        )
        if relative in seen:
            raise BaselineValidationError(
                f"duplicate file contract path: {relative}"
            )
        seen.add(relative)
        classification = contract.get("classification")
        if classification not in PROVENANCE_CLASSES:
            raise BaselineValidationError(
                f"file_contracts[{index}].classification is invalid"
            )
        expected_size = contract.get("size_bytes")
        if (
            isinstance(expected_size, bool)
            or not isinstance(expected_size, int)
            or expected_size < 0
        ):
            raise BaselineValidationError(
                f"file_contracts[{index}].size_bytes must be non-negative"
            )
        expected_sha = _nonempty_string(
            contract.get("sha256"), f"file_contracts[{index}].sha256"
        )
        if not SHA256_RE.fullmatch(expected_sha):
            raise BaselineValidationError(
                f"file_contracts[{index}].sha256 is malformed"
            )
        path = _workspace_file(root, relative)
        actual_size = path.stat().st_size
        if actual_size != expected_size:
            raise BaselineValidationError(
                f"{relative} size mismatch: {actual_size} != {expected_size}"
            )
        actual_sha = _sha256(path)
        if actual_sha != expected_sha:
            raise BaselineValidationError(
                f"{relative} SHA-256 mismatch: {actual_sha} != {expected_sha}"
            )
        total_bytes += actual_size

    return BaselineSummary(
        baseline_id=baseline_id,
        file_count=len(contracts),
        total_bytes=total_bytes,
    )
