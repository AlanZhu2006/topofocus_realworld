#!/usr/bin/env python3
"""Create, validate and resolve one persistent physical-demo session."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shlex
import shutil
import sys
import time

from pydantic import ValidationError


WORKSPACE = Path(__file__).resolve().parents[2]
HUB_DIR = WORKSPACE / "hub"
sys.path.insert(0, str(HUB_DIR / "src"))

from focus_hub.realworld_session import (  # noqa: E402
    ArtifactIdentity,
    CalibrationIdentity,
    DebugValidation,
    RealworldSession,
    RobotSession,
    RuntimeIdentity,
    SESSION_ID_PATTERN,
    SESSION_POINTER_SCHEMA_VERSION,
    artifact_identity,
    atomic_write_json,
    calibration_validation_kind,
    expected_robot_config,
    git_identity,
    load_session_file,
    session_contract_sha256,
    validate_debug_manifest,
    validate_session,
    workspace_relative_path,
    write_session_file,
)


DEBUG_SAFETY_CONFIRMATION = "DEBUG_STACK_NO_MOTION_VERIFIED"
DEFAULT_POINTER = HUB_DIR / "runtime/sessions/current.json"


def resolve_local_argument(value: Path) -> Path:
    path = value.expanduser()
    return path.resolve() if path.is_absolute() else (WORKSPACE / path).resolve()


def resolve_session_argument(value: str) -> Path:
    if value == "current":
        pointer_path = DEFAULT_POINTER
        if not pointer_path.is_file():
            raise FileNotFoundError(
                "no current session pointer; run the calibration/session "
                "preparation command first"
            )
        pointer = json.loads(pointer_path.read_text(encoding="utf-8"))
        if pointer.get("schema_version") != SESSION_POINTER_SCHEMA_VERSION:
            raise ValueError("current session pointer has the wrong schema")
        value = str(pointer.get("session_file", ""))
    path = Path(value)
    resolved = (
        path.expanduser().resolve()
        if path.is_absolute()
        else (WORKSPACE / path).resolve()
    )
    if not resolved.is_relative_to(WORKSPACE.resolve()):
        raise ValueError("session file must remain inside the workspace")
    if not resolved.is_file():
        raise FileNotFoundError(resolved)
    return resolved


def set_current_pointer(session_path: Path) -> None:
    atomic_write_json(
        DEFAULT_POINTER,
        {
            "schema_version": SESSION_POINTER_SCHEMA_VERSION,
            "session_file": workspace_relative_path(WORKSPACE, session_path),
            "updated_at_ns": time.time_ns(),
        },
    )


def calibration_versions(payload: dict) -> tuple[str, str]:
    reference = payload.get("calibration_frame", {}).get("reference", {})
    wsj_transform = str(reference.get("transform_version", ""))
    yunji_transform = str(payload.get("transform_version", ""))
    if not wsj_transform or not yunji_transform:
        raise ValueError("calibration artifact lacks deployment transform versions")
    return wsj_transform, yunji_transform


def config_identity(path: Path, *, allow_goal: bool) -> ArtifactIdentity:
    return artifact_identity(
        WORKSPACE,
        path,
        classification=(
            "generated_live_goal_policy"
            if allow_goal
            else "generated_fail_closed_debug_policy"
        ),
    )


def create_command(args: argparse.Namespace) -> int:
    if re.fullmatch(SESSION_ID_PATTERN, args.session_id) is None:
        raise ValueError("session ID is not lowercase/filesystem-safe")
    session_dir = HUB_DIR / "runtime/sessions" / args.session_id
    session_path = session_dir / "session.json"
    if session_dir.exists():
        raise ValueError(f"refusing to replace existing session: {session_dir}")

    calibration_path = resolve_local_argument(args.calibration_file)
    calibration_payload = json.loads(calibration_path.read_text(encoding="utf-8"))
    if calibration_payload.get("passed") is not True:
        raise ValueError("calibration artifact did not pass")
    calibration_id = str(calibration_payload.get("shared_frame_calibration_id", ""))
    if not calibration_id:
        raise ValueError("calibration artifact lacks shared_frame_calibration_id")
    wsj_transform, yunji_transform = calibration_versions(calibration_payload)
    validation_kind = calibration_validation_kind(calibration_payload)
    code = git_identity(WORKSPACE)

    session_dir.mkdir(parents=True)
    try:
        debug_config_path = session_dir / "robots_debug.json"
        live_config_path = session_dir / "robots_live.json"
        placeholder = ArtifactIdentity(
            path="pending",
            size_bytes=1,
            sha256="0" * 64,
            classification="pending",
        )
        robots = (
            RobotSession(
                robot_id="robot-0",
                name="wsj",
                transform_version=wsj_transform,
                map_dir=workspace_relative_path(
                    WORKSPACE, resolve_local_argument(args.wsj_map)
                ),
                map_start_after_sequence=args.wsj_start_after,
                remote_root=args.wsj_remote_root,
                remote_calibration_path=args.wsj_remote_calibration,
                remote_base_camera_calibration_path=(
                    args.wsj_base_camera_calibration
                ),
                remote_hub_url=args.wsj_remote_hub_url,
                remote_preview_url=args.wsj_remote_preview_url,
                ssh_tmux_target=args.wsj_ssh_tmux,
            ),
            RobotSession(
                robot_id="robot-1",
                name="yunji",
                transform_version=yunji_transform,
                map_dir=workspace_relative_path(
                    WORKSPACE, resolve_local_argument(args.yunji_map)
                ),
                map_start_after_sequence=args.yunji_start_after,
                remote_root=args.yunji_remote_root,
                remote_calibration_path=args.yunji_remote_calibration,
                remote_base_camera_calibration_path=(
                    args.yunji_base_camera_calibration
                ),
                remote_hub_url=args.yunji_remote_hub_url,
                ssh_tmux_target=args.yunji_ssh_tmux,
            ),
        )
        provisional = RealworldSession(
            session_id=args.session_id,
            created_at_ns=time.time_ns(),
            code=code,
            calibration=CalibrationIdentity(
                calibration_id=calibration_id,
                artifact=artifact_identity(
                    WORKSPACE,
                    calibration_path,
                    classification=(
                        "observed_and_source_derived_shared_frame_calibration"
                    ),
                ),
                validation_kind=validation_kind,
            ),
            robots=robots,
            runtime=RuntimeIdentity(
                hub_port=args.hub_port,
                hub_session=args.hub_session,
                glm_url=args.glm_url,
                glm_session=args.glm_session,
                map_session=args.map_session,
                foxglove_session=args.foxglove_session,
                foxglove_port=args.foxglove_port,
                preview_port=args.preview_port,
                map_goal_category=args.map_goal_category,
                semantic_backend=args.semantic_backend,
                spool_dir=workspace_relative_path(
                    WORKSPACE, resolve_local_argument(args.spool_dir)
                ),
                admin_token_file=workspace_relative_path(
                    WORKSPACE, resolve_local_argument(args.admin_token_file)
                ),
                debug_robot_config=placeholder,
                live_robot_config=placeholder,
            ),
        )
        atomic_write_json(
            debug_config_path,
            expected_robot_config(provisional, allow_goal=False),
        )
        atomic_write_json(
            live_config_path,
            expected_robot_config(provisional, allow_goal=True),
        )
        session = provisional.model_copy(
            update={
                "runtime": provisional.runtime.model_copy(
                    update={
                        "debug_robot_config": config_identity(
                            debug_config_path, allow_goal=False
                        ),
                        "live_robot_config": config_identity(
                            live_config_path, allow_goal=True
                        ),
                    }
                )
            }
        )
        write_session_file(session_path, session)
        validate_session(
            WORKSPACE,
            session,
            require_maps=args.require_maps,
            require_current_code=True,
        )
    except Exception:
        shutil.rmtree(session_dir, ignore_errors=True)
        raise
    if args.set_current:
        set_current_pointer(session_path)
    print(
        json.dumps(
            {
                "session_file": workspace_relative_path(WORKSPACE, session_path),
                "session_id": session.session_id,
                "session_contract_sha256": session_contract_sha256(session),
                "calibration_id": calibration_id,
                "code_git_commit": code.git_commit,
                "current_pointer_updated": args.set_current,
                "robot_commands_issued": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


def shell_assignments(path: Path, session: RealworldSession, mode: str) -> str:
    robots = {robot.robot_id: robot for robot in session.robots}
    runtime = session.runtime
    config = (
        runtime.live_robot_config
        if mode == "live"
        else runtime.debug_robot_config
    )
    values = {
        "FOCUS_SESSION_FILE": str(path),
        "FOCUS_SESSION_ID": session.session_id,
        "FOCUS_SESSION_CODE_COMMIT": session.code.git_commit,
        "FOCUS_SESSION_CONTRACT_SHA256": session_contract_sha256(session),
        "FOCUS_CALIBRATION_ID": session.calibration.calibration_id,
        "FOCUS_WSJ_TRANSFORM": robots["robot-0"].transform_version,
        "FOCUS_YUNJI_TRANSFORM": robots["robot-1"].transform_version,
        "FOCUS_WSJ_MAP": str(
            (WORKSPACE / robots["robot-0"].map_dir).resolve()
        ),
        "FOCUS_YUNJI_MAP": str(
            (WORKSPACE / robots["robot-1"].map_dir).resolve()
        ),
        "FOCUS_WSJ_START_AFTER": str(
            robots["robot-0"].map_start_after_sequence
        ),
        "FOCUS_YUNJI_START_AFTER": str(
            robots["robot-1"].map_start_after_sequence
        ),
        "FOCUS_HUB_PORT": str(runtime.hub_port),
        "FOCUS_HUB_SESSION": runtime.hub_session,
        "FOCUS_GLM_URL": runtime.glm_url,
        "FOCUS_GLM_SESSION": runtime.glm_session,
        "FOCUS_MAP_SESSION": runtime.map_session,
        "FOCUS_FOXGLOVE_SESSION": runtime.foxglove_session,
        "FOCUS_FOXGLOVE_PORT": str(runtime.foxglove_port),
        "FOCUS_PREVIEW_PORT": str(runtime.preview_port),
        "FOCUS_MAP_GOAL_CATEGORY": runtime.map_goal_category,
        "FOCUS_SEMANTIC_BACKEND": runtime.semantic_backend,
        "FOCUS_SPOOL_DIR": str((WORKSPACE / runtime.spool_dir).resolve()),
        "FOCUS_ADMIN_TOKEN_FILE": str(
            (WORKSPACE / runtime.admin_token_file).resolve()
        ),
        "FOCUS_ROBOT_CONFIG": str((WORKSPACE / config.path).resolve()),
        "FOCUS_DEBUG_ROBOT_CONFIG": str(
            (WORKSPACE / runtime.debug_robot_config.path).resolve()
        ),
        "FOCUS_LIVE_ROBOT_CONFIG": str(
            (WORKSPACE / runtime.live_robot_config.path).resolve()
        ),
        "FOCUS_WSJ_SSH_TMUX_RESOLVED": robots["robot-0"].ssh_tmux_target,
        "FOCUS_YUNJI_SSH_TMUX_RESOLVED": robots["robot-1"].ssh_tmux_target,
        "FOCUS_WSJ_ROOT_RESOLVED": robots["robot-0"].remote_root,
        "FOCUS_YUNJI_ROOT_RESOLVED": robots["robot-1"].remote_root,
        "FOCUS_WSJ_REMOTE_CALIBRATION": robots[
            "robot-0"
        ].remote_calibration_path,
        "FOCUS_YUNJI_REMOTE_CALIBRATION": robots[
            "robot-1"
        ].remote_calibration_path,
        "FOCUS_WSJ_REMOTE_BASE_CAMERA": robots[
            "robot-0"
        ].remote_base_camera_calibration_path,
        "FOCUS_YUNJI_REMOTE_BASE_CAMERA": robots[
            "robot-1"
        ].remote_base_camera_calibration_path,
        "FOCUS_WSJ_REMOTE_HUB_URL": robots["robot-0"].remote_hub_url,
        "FOCUS_YUNJI_REMOTE_HUB_URL": robots["robot-1"].remote_hub_url,
        "FOCUS_WSJ_REMOTE_PREVIEW_URL": (
            robots["robot-0"].remote_preview_url or ""
        ),
    }
    return "\n".join(
        f"{name}={shlex.quote(value)}" for name, value in values.items()
    ) + "\n"


def resolve_command(args: argparse.Namespace) -> int:
    path = resolve_session_argument(args.session_file)
    session = load_session_file(path)
    report = validate_session(
        WORKSPACE,
        session,
        require_maps=(
            args.mode in {"debug", "live"} and not args.allow_map_rebuild
        ),
        require_debug=args.mode == "live",
        require_current_code=args.mode in {"debug", "live"},
    )
    if args.format == "shell":
        sys.stdout.write(shell_assignments(path, session, args.mode))
    else:
        report["session_file"] = workspace_relative_path(WORKSPACE, path)
        print(json.dumps(report, indent=2, sort_keys=True))
    return 0


def mark_debug_command(args: argparse.Namespace) -> int:
    if args.debug_safety_confirmation != DEBUG_SAFETY_CONFIRMATION:
        raise ValueError(
            "debug mark requires " + DEBUG_SAFETY_CONFIRMATION
        )
    path = resolve_session_argument(args.session_file)
    session = load_session_file(path)
    validate_session(
        WORKSPACE,
        session,
        require_maps=True,
        require_current_code=True,
    )
    shadow_path = resolve_local_argument(args.shadow_manifest)
    shadow_payload = json.loads(shadow_path.read_text(encoding="utf-8"))
    debug = DebugValidation(
        passed_at_ns=time.time_ns(),
        code_git_commit=session.code.git_commit,
        session_contract_sha256=session_contract_sha256(session),
        shadow_manifest=artifact_identity(
            WORKSPACE,
            shadow_path,
            classification=(
                "observed_strict_no_motion_fullstack_debug_manifest"
            ),
        ),
        goal_category=str(shadow_payload.get("goal_category", "")),
        strict_freshness=True,
        strict_mapping_health=True,
        hub_goal_output_disabled=True,
        robot_command_paths_disabled=True,
    )
    validate_debug_manifest(WORKSPACE, session, debug)
    updated = session.model_copy(update={"debug_validation": debug})
    write_session_file(path, updated)
    validate_session(
        WORKSPACE,
        updated,
        require_maps=True,
        require_debug=True,
        require_current_code=True,
    )
    print(
        json.dumps(
            {
                "session_file": workspace_relative_path(WORKSPACE, path),
                "session_id": session.session_id,
                "debug_passed": True,
                "debug_goal_category": debug.goal_category,
                "shadow_manifest_sha256": debug.shadow_manifest.sha256,
                "live_mode_unlocked_for_same_session_and_code": True,
                "robot_commands_issued": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


def add_create_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--calibration-file", type=Path, required=True)
    parser.add_argument("--wsj-map", type=Path, required=True)
    parser.add_argument("--yunji-map", type=Path, required=True)
    parser.add_argument("--wsj-start-after", type=int, required=True)
    parser.add_argument("--yunji-start-after", type=int, required=True)
    parser.add_argument(
        "--wsj-remote-root",
        default="/home/nvidia/topofocus_realworld/current",
    )
    parser.add_argument(
        "--yunji-remote-root",
        default="/home/nyu/topofocus_realworld/current",
    )
    parser.add_argument("--wsj-remote-calibration", required=True)
    parser.add_argument("--yunji-remote-calibration", required=True)
    parser.add_argument(
        "--wsj-remote-hub-url", default="http://127.0.0.1:18089"
    )
    parser.add_argument(
        "--yunji-remote-hub-url", default="http://127.0.0.1:18089"
    )
    parser.add_argument(
        "--wsj-remote-preview-url", default="http://127.0.0.1:18766"
    )
    parser.add_argument(
        "--wsj-base-camera-calibration",
        default=(
            "/home/nvidia/.local/state/topofocus/calibration/"
            "wsj_tinynav_camera_base_20260723_operator.json"
        ),
    )
    parser.add_argument(
        "--yunji-base-camera-calibration",
        default=(
            "/home/nyu/.local/state/topofocus/calibration/"
            "yunji_odin1_base_camera_20260723_operator.json"
        ),
    )
    parser.add_argument(
        "--wsj-ssh-tmux", default="focus_wsj_tunnel_20260722:sensor-audit"
    )
    parser.add_argument(
        "--yunji-ssh-tmux", default="focus_yunji_tunnel_20260722:sensor-audit"
    )
    parser.add_argument("--hub-port", type=int, default=8188)
    parser.add_argument("--hub-session", default="focus_hub_realworld")
    parser.add_argument("--glm-url", default="http://127.0.0.1:31511/v1")
    parser.add_argument("--glm-session", default="glm_realworld")
    parser.add_argument("--map-session", required=True)
    parser.add_argument("--foxglove-session", required=True)
    parser.add_argument("--foxglove-port", type=int, default=8765)
    parser.add_argument("--preview-port", type=int, default=8766)
    parser.add_argument(
        "--map-goal-category",
        choices=("chair", "bed", "plant", "toilet", "tv", "sofa"),
        default="chair",
    )
    parser.add_argument(
        "--semantic-backend",
        choices=("rednet", "segformer-ade20k"),
        default="segformer-ade20k",
    )
    parser.add_argument(
        "--spool-dir", type=Path, default=HUB_DIR / "runtime/spool"
    )
    parser.add_argument(
        "--admin-token-file",
        type=Path,
        default=HUB_DIR / "runtime/admin_token",
    )
    parser.add_argument("--require-maps", action="store_true")
    parser.add_argument("--set-current", action="store_true")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    create = subparsers.add_parser("create")
    add_create_arguments(create)
    create.set_defaults(handler=create_command)

    resolve = subparsers.add_parser("resolve")
    resolve.add_argument("--session-file", default="current")
    resolve.add_argument(
        "--mode", choices=("status", "debug", "live"), default="status"
    )
    resolve.add_argument(
        "--allow-map-rebuild",
        action="store_true",
        help=(
            "validate the immutable session/code/debug contract before the "
            "launcher reconstructs missing or blocked maps from its recorded "
            "sequence boundary"
        ),
    )
    resolve.add_argument("--format", choices=("json", "shell"), default="json")
    resolve.set_defaults(handler=resolve_command)

    mark_debug = subparsers.add_parser("mark-debug")
    mark_debug.add_argument("--session-file", default="current")
    mark_debug.add_argument("--shadow-manifest", type=Path, required=True)
    mark_debug.add_argument("--debug-safety-confirmation", required=True)
    mark_debug.set_defaults(handler=mark_debug_command)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return int(args.handler(args))
    except (
        FileNotFoundError,
        OSError,
        ValueError,
        ValidationError,
        json.JSONDecodeError,
    ) as exc:
        parser.error(str(exc))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
