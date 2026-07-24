"""Persistent, fail-closed deployment sessions for the physical demo.

The session file is the single source of truth consumed by the one-click
launcher.  It binds one calibration artifact, transform epoch, pair of map
directories, robot deployment roots, generated Hub policies, and the exact
Git commit that was debug-validated.  Runtime files remain ignored by Git;
this module records their byte identities instead of copying sensor data or
credentials into the repository.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time
from typing import Literal

from pydantic import Field, field_validator, model_validator

from .map_snapshot import load_map_snapshot
from .models import ROBOT_ID_PATTERN, SHA256_PATTERN, StrictModel


SESSION_SCHEMA_VERSION = "focus-realworld-session-v1"
SESSION_POINTER_SCHEMA_VERSION = "focus-realworld-session-pointer-v1"
MAP_SESSION_CONTRACT_SCHEMA_VERSION = "focus-realworld-map-session-contract-v1"
SESSION_ID_PATTERN = r"^[a-z0-9][a-z0-9_.-]{0,95}$"
SAFE_RUNTIME_NAME_PATTERN = r"^[A-Za-z0-9_.:-]{1,128}$"
GIT_COMMIT_PATTERN = r"^[0-9a-f]{40,64}$"
RUNTIME_CODE_PATHS = ("hub", "source", "dependencies")


class ArtifactIdentity(StrictModel):
    path: str = Field(min_length=1, max_length=2048)
    size_bytes: int = Field(gt=0)
    sha256: str = Field(pattern=SHA256_PATTERN)
    classification: str = Field(min_length=1, max_length=256)


class CodeIdentity(StrictModel):
    git_commit: str = Field(pattern=GIT_COMMIT_PATTERN)
    working_tree_clean: Literal[True] = True


class CalibrationIdentity(StrictModel):
    calibration_id: str = Field(min_length=1, max_length=128)
    artifact: ArtifactIdentity
    validation_kind: Literal[
        "independent_moved_board_holdout",
        "validated_stationary_reanchor_of_board_calibration",
    ]


class RobotSession(StrictModel):
    robot_id: str = Field(pattern=ROBOT_ID_PATTERN)
    name: str = Field(pattern=r"^[a-z][a-z0-9_-]{0,31}$")
    transform_version: str = Field(min_length=1, max_length=128)
    map_dir: str = Field(min_length=1, max_length=2048)
    map_start_after_sequence: int = Field(ge=0)
    remote_root: str = Field(min_length=1, max_length=2048)
    remote_calibration_path: str = Field(min_length=1, max_length=2048)
    remote_base_camera_calibration_path: str = Field(
        min_length=1, max_length=2048
    )
    remote_hub_url: str = Field(min_length=1, max_length=256)
    remote_preview_url: str | None = Field(
        default=None, min_length=1, max_length=256
    )
    ssh_tmux_target: str = Field(pattern=SAFE_RUNTIME_NAME_PATTERN)

    @field_validator("transform_version")
    @classmethod
    def reject_unset_transform(cls, value: str) -> str:
        if value == "UNSET":
            raise ValueError("transform_version must be an explicit deployment ID")
        return value

    @field_validator(
        "remote_root",
        "remote_calibration_path",
        "remote_base_camera_calibration_path",
    )
    @classmethod
    def require_absolute_remote_path(cls, value: str) -> str:
        if not value.startswith("/"):
            raise ValueError("robot deployment paths must be absolute")
        return value

    @field_validator("remote_hub_url", "remote_preview_url")
    @classmethod
    def require_loopback_remote_url(cls, value: str | None) -> str | None:
        if value is not None and not re.fullmatch(
            r"http://127\.0\.0\.1:[0-9]+", value
        ):
            raise ValueError("robot tunnel endpoints must remain loopback-only")
        return value


class RuntimeIdentity(StrictModel):
    hub_port: int = Field(ge=1024, le=65535)
    hub_session: str = Field(pattern=SAFE_RUNTIME_NAME_PATTERN)
    glm_url: str = Field(min_length=1, max_length=256)
    glm_session: str = Field(pattern=SAFE_RUNTIME_NAME_PATTERN)
    map_session: str = Field(pattern=SAFE_RUNTIME_NAME_PATTERN)
    foxglove_session: str = Field(pattern=SAFE_RUNTIME_NAME_PATTERN)
    foxglove_port: int = Field(ge=1024, le=65535)
    preview_port: int = Field(ge=1024, le=65535)
    map_goal_category: Literal["chair", "bed", "plant", "toilet", "tv", "sofa"]
    semantic_backend: Literal["rednet", "segformer-ade20k"] = (
        "segformer-ade20k"
    )
    spool_dir: str = Field(min_length=1, max_length=2048)
    admin_token_file: str = Field(min_length=1, max_length=2048)
    debug_robot_config: ArtifactIdentity
    live_robot_config: ArtifactIdentity

    @field_validator("glm_url")
    @classmethod
    def require_loopback_glm(cls, value: str) -> str:
        if not re.fullmatch(r"http://127\.0\.0\.1:[0-9]+/v1", value):
            raise ValueError("GLM URL must be a loopback /v1 endpoint")
        return value

    @model_validator(mode="after")
    def distinct_ports(self) -> "RuntimeIdentity":
        if len({self.hub_port, self.foxglove_port, self.preview_port}) != 3:
            raise ValueError("Hub, Foxglove and preview ports must be distinct")
        return self


class DebugValidation(StrictModel):
    passed_at_ns: int = Field(gt=0)
    code_git_commit: str = Field(pattern=GIT_COMMIT_PATTERN)
    session_contract_sha256: str = Field(pattern=SHA256_PATTERN)
    shadow_manifest: ArtifactIdentity
    goal_category: Literal["chair", "bed", "plant", "toilet", "tv", "sofa"]
    strict_freshness: Literal[True] = True
    strict_mapping_health: Literal[True] = True
    hub_goal_output_disabled: Literal[True] = True
    robot_command_paths_disabled: Literal[True] = True


class RealworldSession(StrictModel):
    schema_version: Literal["focus-realworld-session-v1"] = SESSION_SCHEMA_VERSION
    session_id: str = Field(pattern=SESSION_ID_PATTERN)
    created_at_ns: int = Field(gt=0)
    code: CodeIdentity
    calibration: CalibrationIdentity
    robots: tuple[RobotSession, RobotSession]
    runtime: RuntimeIdentity
    debug_validation: DebugValidation | None = None

    @model_validator(mode="after")
    def validate_robot_pair(self) -> "RealworldSession":
        by_id = {robot.robot_id: robot for robot in self.robots}
        if set(by_id) != {"robot-0", "robot-1"} or len(by_id) != 2:
            raise ValueError("session requires exactly robot-0 and robot-1")
        if by_id["robot-0"].name != "wsj" or by_id["robot-1"].name != "yunji":
            raise ValueError("robot-0/robot-1 must be named wsj/yunji")
        if by_id["robot-0"].remote_preview_url is None:
            raise ValueError("WSJ session requires its camera-preview tunnel URL")
        if by_id["robot-0"].map_dir == by_id["robot-1"].map_dir:
            raise ValueError("robots must not share one map directory")
        expected_map_dirs = {
            "robot-0": f"hub/runtime/map_out_wsj_{self.session_id}",
            "robot-1": f"hub/runtime/map_out_yunji_{self.session_id}",
        }
        for robot_id, expected in expected_map_dirs.items():
            if by_id[robot_id].map_dir != expected:
                raise ValueError(
                    f"{robot_id} map directory must be session-derived: "
                    f"{expected}"
                )
        if self.runtime.map_session != f"shared_maps_{self.session_id}":
            raise ValueError("map tmux name must be derived from the session ID")
        if (
            self.runtime.foxglove_session
            != f"foxglove_relay_{self.session_id}"
        ):
            raise ValueError(
                "Foxglove tmux name must be derived from the session ID"
            )
        return self


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_workspace_path(workspace: Path, value: str) -> Path:
    root = workspace.resolve()
    candidate = Path(value)
    resolved = (
        candidate.expanduser().resolve()
        if candidate.is_absolute()
        else (root / candidate).resolve()
    )
    if not resolved.is_relative_to(root):
        raise ValueError(f"local session path escapes workspace: {value}")
    return resolved


def workspace_relative_path(workspace: Path, path: Path) -> str:
    root = workspace.resolve()
    resolved = path.expanduser().resolve()
    if not resolved.is_relative_to(root):
        raise ValueError(f"path must remain inside workspace: {resolved}")
    return str(resolved.relative_to(root))


def artifact_identity(
    workspace: Path,
    path: Path,
    *,
    classification: str,
) -> ArtifactIdentity:
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise FileNotFoundError(resolved)
    return ArtifactIdentity(
        path=workspace_relative_path(workspace, resolved),
        size_bytes=resolved.stat().st_size,
        sha256=sha256_file(resolved),
        classification=classification,
    )


def verify_artifact(workspace: Path, artifact: ArtifactIdentity) -> Path:
    resolved = resolve_workspace_path(workspace, artifact.path)
    if not resolved.is_file():
        raise FileNotFoundError(f"missing session artifact: {resolved}")
    observed_size = resolved.stat().st_size
    if observed_size != artifact.size_bytes:
        raise ValueError(
            f"session artifact size drift for {artifact.path}: "
            f"{observed_size} != {artifact.size_bytes}"
        )
    observed_hash = sha256_file(resolved)
    if observed_hash != artifact.sha256:
        raise ValueError(f"session artifact hash drift for {artifact.path}")
    return resolved


def git_identity(workspace: Path) -> CodeIdentity:
    commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=workspace, text=True
    ).strip()
    dirty = subprocess.check_output(
        [
            "git",
            "status",
            "--porcelain",
            "--untracked-files=normal",
            "--",
            *RUNTIME_CODE_PATHS,
        ],
        cwd=workspace,
        text=True,
    ).strip()
    if dirty:
        raise ValueError(
            "real-world sessions require clean runtime code under "
            f"{', '.join(RUNTIME_CODE_PATHS)}; commit and verify those paths "
            "before calibration/debug"
        )
    return CodeIdentity(git_commit=commit, working_tree_clean=True)


def _canonical_json(payload: object) -> bytes:
    return json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")


def session_contract_sha256(session: RealworldSession) -> str:
    payload = session.model_dump(mode="json", exclude={"debug_validation"})
    return hashlib.sha256(_canonical_json(payload)).hexdigest()


def calibration_validation_kind(payload: dict[str, object]) -> str:
    holdout = payload.get("holdout_validation")
    if isinstance(holdout, dict):
        checks = holdout.get("checks")
        required_checks = {
            "sync_skew",
            "board_center_residual",
            "board_normal_residual",
            "board_moved_independently",
        }
        if (
            isinstance(checks, dict)
            and required_checks.issubset(checks)
            and all(checks[name] is True for name in required_checks)
        ):
            return "independent_moved_board_holdout"
    derived = payload.get("derived_from_board_calibration")
    reanchor = payload.get("other_reanchor_validation")
    if (
        isinstance(derived, dict)
        and re.fullmatch(r"[0-9a-f]{64}", str(derived.get("sha256", "")))
        and isinstance(reanchor, dict)
        and reanchor.get("passed") is True
    ):
        return "validated_stationary_reanchor_of_board_calibration"
    raise ValueError(
        "calibration lacks a passed independent moved-board holdout or a "
        "validated stationary reanchor of one"
    )


def validate_calibration_contract(
    workspace: Path,
    calibration: CalibrationIdentity,
    robots: tuple[RobotSession, RobotSession],
) -> dict[str, object]:
    path = verify_artifact(workspace, calibration.artifact)
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("passed") is not True:
        raise ValueError("session calibration artifact did not pass")
    if payload.get("shared_frame_calibration_id") != calibration.calibration_id:
        raise ValueError("session calibration ID differs from its artifact")
    if payload.get("reference_robot") != "robot-0":
        raise ValueError("session calibration reference must be robot-0")
    if payload.get("other_robot") != "robot-1":
        raise ValueError("session calibration other robot must be robot-1")
    by_id = {robot.robot_id: robot for robot in robots}
    reference_version = str(
        payload.get("calibration_frame", {})
        .get("reference", {})
        .get("transform_version", "")
    )
    if reference_version != by_id["robot-0"].transform_version:
        raise ValueError("WSJ transform differs from calibration artifact")
    if str(payload.get("transform_version", "")) != by_id["robot-1"].transform_version:
        raise ValueError("Yunji transform differs from calibration artifact")
    observed_kind = calibration_validation_kind(payload)
    if observed_kind != calibration.validation_kind:
        raise ValueError("calibration validation classification drift")
    if observed_kind == "validated_stationary_reanchor_of_board_calibration":
        derived = payload.get("derived_from_board_calibration")
        if not isinstance(derived, dict):
            raise ValueError("stationary reanchor lacks its board-calibration source")
        source_path = resolve_workspace_path(
            workspace, str(derived.get("path", ""))
        )
        if not source_path.is_file():
            raise FileNotFoundError(source_path)
        expected_size = int(derived.get("size_bytes", -1))
        if source_path.stat().st_size != expected_size:
            raise ValueError("source board calibration size drift")
        if sha256_file(source_path) != str(derived.get("sha256", "")):
            raise ValueError("source board calibration hash drift")
        source_payload = json.loads(source_path.read_text(encoding="utf-8"))
        if source_payload.get("passed") is not True:
            raise ValueError("source board calibration did not pass")
        if (
            calibration_validation_kind(source_payload)
            != "independent_moved_board_holdout"
        ):
            raise ValueError(
                "stationary reanchor source lacks an independent moved-board "
                "holdout"
            )
    return payload


def expected_robot_config(session: RealworldSession, *, allow_goal: bool) -> dict:
    return {
        "schema_version": "1.0",
        "shared_frame": "shared_world",
        "robots": {
            robot.robot_id: {
                "transform_version": robot.transform_version,
                "allow_goal": allow_goal,
            }
            for robot in sorted(session.robots, key=lambda item: item.robot_id)
        },
    }


def validate_robot_configs(workspace: Path, session: RealworldSession) -> None:
    for artifact, allow_goal in (
        (session.runtime.debug_robot_config, False),
        (session.runtime.live_robot_config, True),
    ):
        path = verify_artifact(workspace, artifact)
        observed = json.loads(path.read_text(encoding="utf-8"))
        if observed != expected_robot_config(session, allow_goal=allow_goal):
            raise ValueError(f"robot policy content drift: {artifact.path}")


def expected_map_session_contract(
    session: RealworldSession,
    robot: RobotSession,
) -> dict[str, object]:
    return {
        "schema_version": MAP_SESSION_CONTRACT_SCHEMA_VERSION,
        "session_id": session.session_id,
        "code_git_commit": session.code.git_commit,
        "robot_id": robot.robot_id,
        "map_dir": robot.map_dir,
        "start_after_sequence": robot.map_start_after_sequence,
        "transform_version": robot.transform_version,
        "shared_frame_calibration_id": session.calibration.calibration_id,
        "goal_category": session.runtime.map_goal_category,
        "semantic_backend": session.runtime.semantic_backend,
        "semantic_yolo": {
            "enabled": True,
            "confidence": 0.2,
            "evidence_only": True,
        },
    }


def validate_map_contracts(
    workspace: Path,
    session: RealworldSession,
    *,
    require_fresh_s: float | None = None,
    now_ns: int | None = None,
) -> dict[str, dict[str, object]]:
    now = time.time_ns() if now_ns is None else now_ns
    reports: dict[str, dict[str, object]] = {}
    for robot in session.robots:
        directory = resolve_workspace_path(workspace, robot.map_dir)
        summary_path = directory / "map_summary.json"
        status_path = directory / "live_status.json"
        map_path = directory / "central_map.npz"
        contract_path = directory / "map_session_contract.json"
        for required in (summary_path, status_path, map_path, contract_path):
            if not required.is_file():
                raise FileNotFoundError(f"missing live map artifact: {required}")
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        status = json.loads(status_path.read_text(encoding="utf-8"))
        contract = json.loads(contract_path.read_text(encoding="utf-8"))
        if contract != expected_map_session_contract(session, robot):
            raise ValueError(
                f"{robot.robot_id} map session contract differs from session"
            )
        snapshot = load_map_snapshot(map_path)
        if snapshot is None:
            raise FileNotFoundError(map_path)
        if snapshot.frame_id != "shared_world":
            raise ValueError(f"{robot.robot_id} map snapshot frame mismatch")
        if snapshot.transform_version != robot.transform_version:
            raise ValueError(
                f"{robot.robot_id} map snapshot transform mismatch"
            )
        if (
            snapshot.shared_frame_calibration_id
            != session.calibration.calibration_id
        ):
            raise ValueError(
                f"{robot.robot_id} map snapshot calibration mismatch"
            )
        for payload, label in ((summary, "summary"), (status, "status")):
            if payload.get("robot_id") != robot.robot_id:
                raise ValueError(f"{robot.robot_id} map {label} robot mismatch")
            if payload.get("frame_id") != "shared_world":
                raise ValueError(f"{robot.robot_id} map {label} frame mismatch")
            if payload.get("transform_version") != robot.transform_version:
                raise ValueError(f"{robot.robot_id} map {label} transform mismatch")
            if (
                payload.get("shared_frame_calibration_id")
                != session.calibration.calibration_id
            ):
                raise ValueError(f"{robot.robot_id} map {label} calibration mismatch")
            if payload.get("mapping_blocked_reason") is not None:
                raise ValueError(
                    f"{robot.robot_id} map blocked: "
                    f"{payload.get('mapping_blocked_reason')}"
                )
        last_sequence = int(summary.get("last_observation_sequence", -1))
        if last_sequence <= robot.map_start_after_sequence:
            raise ValueError(
                f"{robot.robot_id} map has not advanced beyond its session "
                f"boundary {robot.map_start_after_sequence}"
            )
        semantic_mapping = summary.get("semantic_mapping")
        yolo = (
            semantic_mapping.get("yolo_reinforcement")
            if isinstance(semantic_mapping, dict)
            else None
        )
        if not isinstance(yolo, dict) or yolo.get("enabled") is not True:
            raise ValueError(f"{robot.robot_id} map has no enabled YOLO evidence")
        yolo_sequence = int(yolo.get("last_sequence", -1))
        if yolo_sequence <= robot.map_start_after_sequence:
            raise ValueError(
                f"{robot.robot_id} YOLO evidence predates the session boundary"
            )
        capture_ns = int(status.get("last_capture_time_ns", 0))
        age_s = (now - capture_ns) / 1e9 if capture_ns > 0 else float("inf")
        if require_fresh_s is not None and age_s > require_fresh_s:
            raise ValueError(
                f"{robot.robot_id} map input age {age_s:.3f}s exceeds "
                f"{require_fresh_s:.3f}s"
            )
        reports[robot.robot_id] = {
            "map_dir": str(directory),
            "last_capture_time_ns": capture_ns,
            "age_s": age_s,
            "last_sequence": last_sequence,
            "yolo_source_sequence": yolo_sequence,
            "mapping_blocked_reason": None,
        }
    return reports


def validate_debug_manifest(
    workspace: Path,
    session: RealworldSession,
    debug: DebugValidation,
) -> dict[str, object]:
    path = verify_artifact(workspace, debug.shadow_manifest)
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("status") != "complete_shadow_only":
        raise ValueError("debug shadow manifest has the wrong authority status")
    if payload.get("realworld_session_id") != session.session_id:
        raise ValueError("debug shadow belongs to a different real-world session")
    if (
        payload.get("realworld_session_contract_sha256")
        != session_contract_sha256(session)
    ):
        raise ValueError("debug shadow belongs to a different session contract")
    if payload.get("shared_frame_calibration_id") != session.calibration.calibration_id:
        raise ValueError("debug shadow calibration differs from session")
    if payload.get("allow_stale_shadow_input") is not False:
        raise ValueError("debug validation used stale-input override")
    if payload.get("allow_blocked_shadow_input") is not False:
        raise ValueError("debug validation used blocked-map override")
    timing = payload.get("input_timing")
    if not isinstance(timing, dict) or timing.get("status") != "accepted_fresh":
        raise ValueError("debug validation did not use fresh synchronized inputs")
    safety = payload.get("safety")
    if not isinstance(safety, dict):
        raise ValueError("debug shadow manifest lacks safety evidence")
    command_flags = [
        safety[name]
        for name in ("robot_commands_sent", "robot_commands_issued")
        if name in safety
    ]
    if not command_flags or any(value is not False for value in command_flags):
        raise ValueError("debug validation claims a robot command")
    if safety.get("hub_decision_mode_if_published") != "HOLD":
        raise ValueError("debug validation did not constrain Hub decisions to HOLD")
    if safety.get("allow_goal_changed") is not False:
        raise ValueError("debug validation changed GOAL authority")
    if safety.get("goal_publication_code_path_present") is not False:
        raise ValueError("debug validation exposed a goal-publication code path")
    publications = payload.get("hub_hold_publications")
    if not isinstance(publications, dict) or set(publications) != {
        "robot-0",
        "robot-1",
    }:
        raise ValueError("debug validation lacks two explicit HOLD publications")
    for robot_id, publication in publications.items():
        status_code = (
            publication.get("status_code")
            if isinstance(publication, dict)
            else None
        )
        if (
            not isinstance(publication, dict)
            or publication.get("mode") != "HOLD"
            or isinstance(status_code, bool)
            or not isinstance(status_code, int)
            or not 200 <= status_code < 300
        ):
            raise ValueError(
                f"debug validation has an invalid HOLD publication for {robot_id}"
            )
    return payload


def validate_session(
    workspace: Path,
    session: RealworldSession,
    *,
    require_maps: bool = False,
    require_debug: bool = False,
    require_current_code: bool = False,
    require_fresh_s: float | None = None,
) -> dict[str, object]:
    validate_calibration_contract(workspace, session.calibration, session.robots)
    validate_robot_configs(workspace, session)
    report: dict[str, object] = {
        "schema_version": session.schema_version,
        "session_id": session.session_id,
        "session_contract_sha256": session_contract_sha256(session),
        "calibration_id": session.calibration.calibration_id,
        "maps": None,
        "debug_validation": None,
    }
    if require_maps:
        report["maps"] = validate_map_contracts(
            workspace,
            session,
            require_fresh_s=require_fresh_s,
        )
    if require_current_code:
        observed_code = git_identity(workspace)
        if observed_code.git_commit != session.code.git_commit:
            raise ValueError(
                "current Git commit differs from the calibrated session; "
                "create a new session or rerun debug after an explicit migration"
            )
    if require_debug:
        if session.debug_validation is None:
            raise ValueError("live mode requires a passed debug validation")
        debug = session.debug_validation
        if debug.session_contract_sha256 != session_contract_sha256(session):
            raise ValueError("debug validation belongs to a different session contract")
        if debug.code_git_commit != session.code.git_commit:
            raise ValueError("debug validation used a different Git commit")
        validate_debug_manifest(workspace, session, debug)
        report["debug_validation"] = {
            "passed_at_ns": debug.passed_at_ns,
            "goal_category": debug.goal_category,
            "shadow_manifest": debug.shadow_manifest.path,
        }
    return report


def atomic_write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)
    directory_fd = os.open(path.parent, os.O_DIRECTORY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def load_session_file(path: Path) -> RealworldSession:
    return RealworldSession.model_validate_json(path.read_text(encoding="utf-8"))


def write_session_file(path: Path, session: RealworldSession) -> None:
    atomic_write_json(path, session.model_dump(mode="json"))
