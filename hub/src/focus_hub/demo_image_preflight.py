"""Historical-image preflight contracts for the supervised lab demo.

The preflight deliberately stops at image/VLM repeatability.  Static RGB-D
frames contain neither a newly executed path nor a robot-local STOP result,
so they can never be promoted to navigation SR/SPL evidence.
"""
from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path
from typing import Literal

from pydantic import Field, model_validator

from .models import ObservationMetadata, ROBOT_ID_PATTERN, SHA256_PATTERN, StrictModel
from .shadow_coordination import sha256_file
from .source_episode import SOURCE_HM3D_OBJECTNAV_GOALS


IMAGE_PREFLIGHT_SCHEMA_VERSION = "focus-demo-image-preflight-v1"


class HistoricalArtifact(StrictModel):
    path: str = Field(min_length=1, max_length=1024)
    size_bytes: int = Field(gt=0)
    sha256: str = Field(pattern=SHA256_PATTERN)
    classification: Literal["observed_local_runtime"] = "observed_local_runtime"


class HistoricalObservation(StrictModel):
    robot_id: str = Field(pattern=ROBOT_ID_PATTERN)
    sequence: int = Field(ge=0)
    capture_time_ns: int = Field(gt=0)
    camera_frame: str = Field(min_length=1, max_length=128)
    transform_version: str = Field(min_length=1, max_length=128)
    wire_object_goal_category: str = Field(min_length=1, max_length=128)
    metadata: HistoricalArtifact
    rgb: HistoricalArtifact
    depth: HistoricalArtifact


class ImagePreflightCase(StrictModel):
    case_id: str = Field(pattern=r"^[a-z0-9][a-z0-9_-]{0,63}$")
    target_category: str = Field(min_length=1, max_length=64)
    purpose: str = Field(min_length=1, max_length=512)
    target_assignment: Literal["operator_selected_replay_target"]
    observations: tuple[HistoricalObservation, ...] = Field(min_length=2, max_length=2)

    @model_validator(mode="after")
    def validate_case(self) -> "ImagePreflightCase":
        if self.target_category not in SOURCE_HM3D_OBJECTNAV_GOALS:
            raise ValueError(
                f"unsupported HPC ObjectNav target {self.target_category!r}"
            )
        robot_ids = [item.robot_id for item in self.observations]
        if set(robot_ids) != {"robot-0", "robot-1"} or len(set(robot_ids)) != 2:
            raise ValueError("each image case needs exactly robot-0 and robot-1")
        return self


class ImagePreflightManifest(StrictModel):
    schema_version: Literal["focus-demo-image-preflight-v1"] = (
        IMAGE_PREFLIGHT_SCHEMA_VERSION
    )
    preflight_id: str = Field(pattern=r"^[a-z0-9][a-z0-9_-]{0,127}$")
    classification: Literal[
        "observed_historical_inputs_plus_operator_selected_replay_targets"
    ]
    official_navigation_metrics_eligible: Literal[False] = False
    ineligibility_reason: str = Field(min_length=1, max_length=512)
    repetitions_per_case: int = Field(gt=0, le=100)
    maximum_capture_skew_s: float = Field(gt=0.0, le=10.0)
    cases: tuple[ImagePreflightCase, ...] = Field(min_length=1, max_length=32)

    @model_validator(mode="after")
    def validate_manifest(self) -> "ImagePreflightManifest":
        case_ids = [case.case_id for case in self.cases]
        if len(case_ids) != len(set(case_ids)):
            raise ValueError("image preflight case IDs must be unique")
        observation_ids = [
            (item.robot_id, item.sequence)
            for case in self.cases
            for item in case.observations
        ]
        if len(observation_ids) != len(set(observation_ids)):
            raise ValueError("historical observations must not be reused across cases")
        return self


def _resolved_workspace_path(workspace: Path, relative_path: str) -> Path:
    candidate = Path(relative_path)
    if candidate.is_absolute():
        raise ValueError(f"artifact path must be workspace-relative: {relative_path}")
    root = workspace.resolve()
    resolved = (root / candidate).resolve()
    if not resolved.is_relative_to(root):
        raise ValueError(f"artifact escapes workspace: {relative_path}")
    return resolved


def _validate_artifact(workspace: Path, artifact: HistoricalArtifact) -> Path:
    path = _resolved_workspace_path(workspace, artifact.path)
    if not path.is_file():
        raise FileNotFoundError(f"missing historical artifact: {path}")
    observed_size = path.stat().st_size
    if observed_size != artifact.size_bytes:
        raise ValueError(
            f"artifact size drift for {artifact.path}: "
            f"expected {artifact.size_bytes}, observed {observed_size}"
        )
    observed_sha256 = sha256_file(path)
    if observed_sha256 != artifact.sha256:
        raise ValueError(f"artifact hash drift for {artifact.path}")
    return path


def validate_historical_inputs(
    manifest: ImagePreflightManifest,
    *,
    workspace: Path,
    expected_cases: int = 4,
    expected_repetitions: int = 5,
) -> dict[str, object]:
    """Validate every byte and wire identity referenced by the manifest."""

    if len(manifest.cases) != expected_cases:
        raise ValueError(
            f"expected {expected_cases} image cases, found {len(manifest.cases)}"
        )
    if manifest.repetitions_per_case != expected_repetitions:
        raise ValueError(
            f"expected {expected_repetitions} repetitions, found "
            f"{manifest.repetitions_per_case}"
        )

    case_records: list[dict[str, object]] = []
    total_bytes = 0
    artifact_count = 0
    for case in manifest.cases:
        capture_times: list[int] = []
        observation_records: list[dict[str, object]] = []
        for observation in case.observations:
            metadata_path = _validate_artifact(workspace, observation.metadata)
            rgb_path = _validate_artifact(workspace, observation.rgb)
            depth_path = _validate_artifact(workspace, observation.depth)
            artifact_count += 3
            total_bytes += sum(
                item.size_bytes
                for item in (observation.metadata, observation.rgb, observation.depth)
            )

            metadata = ObservationMetadata.model_validate_json(
                metadata_path.read_text(encoding="utf-8")
            )
            mismatches: list[str] = []
            if metadata.robot_id != observation.robot_id:
                mismatches.append("robot_id")
            if metadata.sequence != observation.sequence:
                mismatches.append("sequence")
            if metadata.capture_time_ns != observation.capture_time_ns:
                mismatches.append("capture_time_ns")
            if metadata.pose.shared_T_camera.child_frame != observation.camera_frame:
                mismatches.append("camera_frame")
            if metadata.pose.transform_version != observation.transform_version:
                mismatches.append("transform_version")
            if metadata.object_goal.category != observation.wire_object_goal_category:
                mismatches.append("wire_object_goal_category")
            if metadata.rgb_size_bytes != observation.rgb.size_bytes:
                mismatches.append("rgb_size_bytes")
            if metadata.depth_size_bytes != observation.depth.size_bytes:
                mismatches.append("depth_size_bytes")
            if metadata.rgb_sha256 != observation.rgb.sha256:
                mismatches.append("rgb_sha256")
            if metadata.depth_sha256 != observation.depth.sha256:
                mismatches.append("depth_sha256")
            if mismatches:
                raise ValueError(
                    f"manifest/wire mismatch for {observation.robot_id}/"
                    f"{observation.sequence}: {', '.join(mismatches)}"
                )

            capture_times.append(metadata.capture_time_ns)
            observation_records.append(
                {
                    "robot_id": metadata.robot_id,
                    "sequence": metadata.sequence,
                    "rgb_path": str(rgb_path),
                    "depth_path": str(depth_path),
                    "camera_frame": observation.camera_frame,
                    "transform_version": observation.transform_version,
                    "wire_object_goal_category": observation.wire_object_goal_category,
                    "replay_target_category": case.target_category,
                    "status": "observed_historical_input_identity_verified",
                }
            )

        skew_s = (max(capture_times) - min(capture_times)) / 1e9
        if skew_s > manifest.maximum_capture_skew_s:
            raise ValueError(
                f"case {case.case_id} capture skew {skew_s:.6f}s exceeds "
                f"{manifest.maximum_capture_skew_s:.6f}s"
            )
        case_records.append(
            {
                "case_id": case.case_id,
                "target_category": case.target_category,
                "capture_skew_s": skew_s,
                "observations": observation_records,
                "planned_repetitions": manifest.repetitions_per_case,
            }
        )

    return {
        "schema_version": IMAGE_PREFLIGHT_SCHEMA_VERSION,
        "preflight_id": manifest.preflight_id,
        "status": "historical_inputs_verified",
        "classification": manifest.classification,
        "official_navigation_metrics_eligible": False,
        "ineligibility_reason": manifest.ineligibility_reason,
        "case_count": len(manifest.cases),
        "planned_model_calls_per_robot": manifest.repetitions_per_case,
        "planned_case_repetitions": len(manifest.cases)
        * manifest.repetitions_per_case,
        "planned_agent_image_calls": len(manifest.cases)
        * manifest.repetitions_per_case
        * 2,
        "artifact_count": artifact_count,
        "artifact_bytes": total_bytes,
        "cases": case_records,
    }


def iter_observations(
    manifest: ImagePreflightManifest,
) -> Iterable[tuple[ImagePreflightCase, HistoricalObservation]]:
    for case in manifest.cases:
        for observation in case.observations:
            yield case, observation
