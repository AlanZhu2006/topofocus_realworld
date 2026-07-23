#!/usr/bin/env python3
"""Run the complete v2 Hub/adapter path with synthetic inputs and no motion."""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import sys
import time
from pathlib import Path

import cv2
from fastapi.testclient import TestClient
import numpy as np

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

from focus_hub.api import create_app  # noqa: E402
from focus_hub.models import ObservationMetadata  # noqa: E402
from focus_hub.registry import RobotPolicy  # noqa: E402
from focus_hub.settings import Settings  # noqa: E402
from focus_hub.transport_v2 import DecisionBatchV2, HighLevelDecisionV2  # noqa: E402
from focus_hub.v2_goal_adapter import (  # noqa: E402
    V2AdapterAction,
    V2GoalAdapter,
    V2GoalAdapterConfig,
)


IDENTITY = (
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
)
YAW_90 = (
    0.0, -1.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
)
ROBOTS = ("robot-0", "robot-1")
ROBOT_NAMES = {"robot-0": "wsj", "robot-1": "yunji"}
TRANSFORMS = {
    "robot-0": "dryrun-wsj-v1",
    "robot-1": "dryrun-yunji-v1",
}
CALIBRATION_ID = "synthetic-shared-board-v1"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact(path: Path, classification: str) -> dict[str, object]:
    return {
        "path": str(path.resolve()),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
        "classification": classification,
    }


def encode_inputs() -> tuple[bytes, bytes]:
    rgb = np.zeros((16, 16, 3), dtype=np.uint8)
    rgb[:, :, 1] = 96
    depth = np.full((16, 16), 1000, dtype=np.uint16)
    ok_rgb, rgb_encoded = cv2.imencode(".jpg", rgb)
    ok_depth, depth_encoded = cv2.imencode(".png", depth)
    if not ok_rgb or not ok_depth:
        raise RuntimeError("failed to encode synthetic RGB-D")
    return rgb_encoded.tobytes(), depth_encoded.tobytes()


def observation_metadata(
    robot_id: str,
    *,
    sequence: int,
    capture_time_ns: int,
    rgb: bytes,
    depth: bytes,
) -> ObservationMetadata:
    return ObservationMetadata.model_validate({
        "robot_id": robot_id,
        "sequence": sequence,
        "capture_time_ns": capture_time_ns,
        "sent_time_ns": capture_time_ns + 1_000_000,
        "pose": {
            "shared_T_camera": {
                "parent_frame": "shared_world",
                "child_frame": f"{robot_id}_camera",
                "matrix": IDENTITY,
            },
            "covariance_6x6": [0.0] * 36,
            "transform_version": TRANSFORMS[robot_id],
        },
        "base_T_camera": {
            "parent_frame": "base_link",
            "child_frame": f"{robot_id}_camera",
            "matrix": IDENTITY,
        },
        "intrinsics": {
            "width": 16,
            "height": 16,
            "fx": 12.0,
            "fy": 12.0,
            "cx": 8.0,
            "cy": 8.0,
            "distortion_model": "none",
            "distortion": [],
        },
        "depth_scale_m": 0.001,
        "depth_min_m": 0.1,
        "depth_max_m": 10.0,
        "rgb_encoding": "jpeg",
        "depth_encoding": "png16",
        "rgb_size_bytes": len(rgb),
        "depth_size_bytes": len(depth),
        "rgb_sha256": hashlib.sha256(rgb).hexdigest(),
        "depth_sha256": hashlib.sha256(depth).hexdigest(),
        "object_goal": {"goal_id": "demo-chair", "category": "chair"},
        "health": {
            "safety_state": "READY",
            "localization_state": "TRACKING",
            "estop_engaged": False,
            "collision_avoidance_ready": True,
            "motor_controller_ready": True,
            "detail": "synthetic dry-run only",
        },
        "mapping_only": False,
    })


def write_synthetic_map(path: Path, robot_id: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    grid = np.zeros((17, 25, 25), dtype=np.float32)
    grid[1] = 1.0
    np.savez_compressed(
        path,
        grid=grid,
        origin_xy_m=np.asarray([-0.625, -0.625]),
        resolution_m=np.asarray(0.05),
        frame_id=np.asarray("shared_world"),
        transform_version=np.asarray(TRANSFORMS[robot_id]),
        shared_frame_calibration_id=np.asarray(CALIBRATION_ID),
        map_format_version=np.asarray("focus-hub-central-map-v3"),
    )


def semantic_target() -> tuple[dict[str, object], bytes]:
    mask = np.zeros((25, 25), dtype=np.uint8)
    mask[12, 12] = 255
    ok, encoded = cv2.imencode(".png", mask)
    if not ok:
        raise RuntimeError("failed to encode synthetic semantic mask")
    png = encoded.tobytes()
    return ({
        "kind": "SEMANTIC_REGION",
        "category": "chair",
        "source_robot_id": "robot-0",
        "evidence_status": "model_inference_map_projected_unverified",
        "source_goal_dilation_cells": 10,
        "region": {
            "frame_id": "shared_world",
            "origin_xy_m": [-0.625, -0.625],
            "resolution_m": 0.05,
            "height": 25,
            "width": 25,
            "row_axis": "+y",
            "column_axis": "+x",
            "encoding": "png_u8_0_255_base64",
            "component_size_cells": 1,
            "payload_size_bytes": len(png),
            "payload_sha256": hashlib.sha256(png).hexdigest(),
            "payload_base64": base64.b64encode(png).decode("ascii"),
        },
        "display_centroid": {
            "frame_id": "shared_world",
            "x": 0.0,
            "y": 0.0,
            "authority": "display_only",
        },
    }, png)


def build_batch(
    app,
    map_paths: dict[str, Path],
    semantic: dict[str, object],
) -> DecisionBatchV2:
    issued_at_ns = time.time_ns()
    inputs = {}
    for robot_id in ROBOTS:
        state = app.state.registry.snapshot(robot_id)
        if state.last_observation is None:
            raise RuntimeError(f"{robot_id} synthetic observation disappeared")
        inputs[robot_id] = {
            "sequence": state.last_sequence,
            "capture_time_ns": state.last_observation.capture_time_ns,
            "payload_sha256": state.last_payload_digest,
        }
    decisions = []
    for robot_id in ROBOTS:
        target: dict[str, object]
        if robot_id == "robot-0":
            target = semantic
        else:
            # With shared_T_yunji_map=+90 degrees, this becomes local (1,0)
            # and local yaw 0. It proves that yaw is transformed, not copied.
            target = {
                "kind": "FRONTIER_POINT",
                "frontier_id": "frontier-yunji-0",
                "source_goal_dilation_cells": 10,
                "pose": {
                    "frame_id": "shared_world",
                    "x": 0.0,
                    "y": 1.0,
                    "z": 0.0,
                    "yaw_rad": np.pi / 2,
                },
            }
        decisions.append(HighLevelDecisionV2.model_validate({
            "robot_id": robot_id,
            "scene_id": "synthetic-scene-1",
            "episode_id": "synthetic-scene-1-trial-1",
            "round_index": 0,
            "source_step": 0,
            "decision_batch_id": "synthetic-batch-0",
            "leg_id": f"synthetic-leg-{robot_id}-0",
            "decision_id": f"synthetic-decision-{robot_id}-0",
            "lease_sequence": 0,
            "mode": "GOAL",
            "coordination": {
                "execution_epoch": 0,
                "active_robot_ids": list(ROBOTS),
            },
            "goal_category": "chair",
            "input_observations": inputs,
            "map_provenance": {
                "map_version": app.state.registry.snapshot(robot_id).map_version,
                "map_snapshot_sha256": sha256_file(map_paths[robot_id]),
                "map_format_version": "focus-hub-central-map-v3",
                "frame_id": "shared_world",
                "resolution_m": 0.05,
                "transform_version": TRANSFORMS[robot_id],
                "shared_frame_calibration_id": CALIBRATION_ID,
            },
            "issued_at_ns": issued_at_ns,
            "expires_at_ns": issued_at_ns + 8_000_000_000,
            "target": target,
            "reason": "synthetic concurrent full-stack dry-run",
        }))
    return DecisionBatchV2(decisions=tuple(decisions))


def post_event(
    client: TestClient,
    token: str,
    decision: HighLevelDecisionV2,
    *,
    status: str,
    index: int,
    path_length_m: float,
    velocity_zero: bool,
    local_goal,
    terminal_sequence: int | None = None,
) -> dict[str, object]:
    resolved = None
    if status == "ACCEPTED" and local_goal.source_region_sha256 is not None:
        resolved = {
            "frame_id": local_goal.frame_id,
            "x": local_goal.x,
            "y": local_goal.y,
            "yaw_rad": local_goal.yaw_rad,
            "source_region_sha256": local_goal.source_region_sha256,
            "arrival_radius_m": local_goal.arrival_radius_m,
            "adapter_name": (
                "tinynav-semantic-approach"
                if decision.robot_id == "robot-0"
                else "water-semantic-approach"
            ),
            "adapter_version": "1",
        }
    event = {
        "robot_id": decision.robot_id,
        "scene_id": decision.scene_id,
        "episode_id": decision.episode_id,
        "decision_batch_id": decision.decision_batch_id,
        "leg_id": decision.leg_id,
        "decision_id": decision.decision_id,
        "lease_sequence": decision.lease_sequence,
        "event_id": f"synthetic-{decision.robot_id}-{index}-{status.lower()}",
        "status": status,
        "reason_code": {
            "RECEIVED": "DECISION_RECEIVED",
            "ACCEPTED": "LOCAL_GOAL_ACCEPTED",
            "NAVIGATING": "LOCAL_PLANNER_ACTIVE",
            "ARRIVED": "LOCAL_PLANNER_ARRIVED",
        }[status],
        "observed_at_ns": time.time_ns(),
        "local_pose": {
            "frame_id": local_goal.frame_id,
            "x": 0.0 if status != "ARRIVED" else local_goal.x,
            "y": 0.0 if status != "ARRIVED" else local_goal.y,
            "yaw_rad": 0.0 if status != "ARRIVED" else local_goal.yaw_rad,
        },
        "path_length_m_from_episode_start": path_length_m,
        "velocity_zero_confirmed": velocity_zero,
        "terminal_observation_sequence": terminal_sequence,
        "resolved_local_goal": resolved,
        "detail": "synthetic dry-run event",
    }
    response = client.post(
        f"/v2/robots/{decision.robot_id}/navigation-events",
        json=event,
        headers={"X-Robot-Token": token},
    )
    response.raise_for_status()
    return response.json()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    if output.exists():
        print(f"refusing to overwrite existing dry-run output: {output}", file=sys.stderr)
        return 2
    output.mkdir(parents=True)

    map_paths = {
        robot_id: output / "synthetic_inputs" / robot_id / "central_map.npz"
        for robot_id in ROBOTS
    }
    for robot_id, path in map_paths.items():
        write_synthetic_map(path, robot_id)
    semantic, semantic_png = semantic_target()
    semantic_path = (
        output / "synthetic_inputs" / "robot-0" / "chair_goal_mask.png"
    )
    semantic_path.write_bytes(semantic_png)

    settings = Settings(
        policies={
            robot_id: RobotPolicy(TRANSFORMS[robot_id], allow_goal=True)
            for robot_id in ROBOTS
        },
        robot_tokens={robot_id: f"synthetic-token-{robot_id}" for robot_id in ROBOTS},
        admin_token="synthetic-admin-token",
        spool_dir=output / "hub_state" / "spool",
        state_dir=output / "hub_state" / "state",
        min_free_bytes=0,
    )
    app = create_app(settings)
    client = TestClient(app)
    rgb, depth = encode_inputs()
    metadata_by_robot: dict[str, ObservationMetadata] = {}
    base_capture_ns = time.time_ns() - 100_000_000
    for index, robot_id in enumerate(ROBOTS):
        metadata = observation_metadata(
            robot_id,
            sequence=index,
            capture_time_ns=base_capture_ns + index,
            rgb=rgb,
            depth=depth,
        )
        response = client.post(
            f"/v1/robots/{robot_id}/observations",
            headers={"X-Robot-Token": f"synthetic-token-{robot_id}"},
            data={"metadata_json": metadata.model_dump_json()},
            files={
                "rgb": ("rgb.jpg", rgb, "image/jpeg"),
                "depth": ("depth.png", depth, "image/png"),
            },
        )
        response.raise_for_status()
        metadata_by_robot[robot_id] = metadata

    batch = build_batch(app, map_paths, semantic)
    publish_response = client.post(
        "/v2/admin/decision-batches",
        headers={"X-Admin-Token": "synthetic-admin-token"},
        json=json.loads(batch.model_dump_json()),
    )
    publish_response.raise_for_status()

    adapters = {
        "robot-0": V2GoalAdapter(V2GoalAdapterConfig(
            robot_id="robot-0",
            transform_version=TRANSFORMS["robot-0"],
            shared_frame_calibration_id=CALIBRATION_ID,
            shared_T_robot_map=IDENTITY,
            output_kind="tinynav_poi",
            local_frame_id="wsj/map",
        )),
        "robot-1": V2GoalAdapter(V2GoalAdapterConfig(
            robot_id="robot-1",
            transform_version=TRANSFORMS["robot-1"],
            shared_frame_calibration_id=CALIBRATION_ID,
            shared_T_robot_map=YAW_90,
            output_kind="water_move",
            local_frame_id="yunji/map",
        )),
    }
    robot_records: dict[str, object] = {}
    for decision in batch.decisions:
        response = client.get(
            f"/v2/robots/{decision.robot_id}/decisions/latest",
            headers={
                "X-Robot-Token": f"synthetic-token-{decision.robot_id}"
            },
        )
        response.raise_for_status()
        polled = HighLevelDecisionV2.model_validate(response.json())
        adapter_result = adapters[decision.robot_id].evaluate(
            polled,
            now_ns=time.time_ns(),
            health=metadata_by_robot[decision.robot_id].health,
            current_position_robot_map=(
                -1.0 if decision.robot_id == "robot-0" else 0.0,
                0.0,
                0.0,
            ),
            is_local_goal_reachable=(
                (lambda _x, _y: True)
                if decision.robot_id == "robot-0"
                else None
            ),
        )
        if adapter_result.action != V2AdapterAction.GOAL:
            raise RuntimeError(
                f"{decision.robot_id} dry-run adapter rejected: "
                f"{adapter_result.reason_code} {adapter_result.detail}"
            )
        if adapter_result.local_goal is None:
            raise RuntimeError(f"{decision.robot_id} adapter produced no local goal")
        acknowledgements = []
        for index, (status, path_length, zero) in enumerate((
            ("RECEIVED", 0.0, True),
            ("ACCEPTED", 0.0, True),
            ("NAVIGATING", 0.1, False),
            ("ARRIVED", 1.0, True),
        )):
            acknowledgements.append(post_event(
                client,
                f"synthetic-token-{decision.robot_id}",
                decision,
                status=status,
                index=index,
                path_length_m=path_length,
                velocity_zero=zero,
                local_goal=adapter_result.local_goal,
                terminal_sequence=(
                    metadata_by_robot[decision.robot_id].sequence
                    if status == "ARRIVED"
                    else None
                ),
            ))
        robot_records[decision.robot_id] = {
            "physical_name": ROBOT_NAMES[decision.robot_id],
            "target_kind": decision.target.kind if decision.target else None,
            "local_goal": adapter_result.local_goal.__dict__,
            "command_preview": adapter_result.command_preview,
            "command_preview_authority": "preview_only_never_sent",
            "navigation_event_acks": acknowledgements,
        }

    event_log_path = settings.state_dir / "decision_events.jsonl"
    report = {
        "schema_version": "focus-v2-fullstack-dryrun-v1",
        "status": "complete_synthetic_no_motion",
        "classification": "synthetic locally observed; unverified on robot",
        "robot_commands_sent": False,
        "external_network_used": False,
        "official_navigation_metrics_eligible": False,
        "concurrent_goal_robot_ids": list(ROBOTS),
        "batch_response": publish_response.json(),
        "robots": robot_records,
        "artifacts": [
            artifact(map_paths[robot_id], "synthetic valid v3 map input")
            for robot_id in ROBOTS
        ] + [
            artifact(semantic_path, "synthetic semantic-region input"),
            artifact(event_log_path, "locally observed durable v2 event log"),
            artifact(Path(__file__), "implemented dry-run orchestrator"),
            artifact(
                WORKSPACE / "hub/src/focus_hub/transport_v2.py",
                "implemented v2 wire models",
            ),
            artifact(
                WORKSPACE / "hub/src/focus_hub/v2_goal_adapter.py",
                "implemented no-send robot adapter",
            ),
        ],
    }
    report_path = output / "report.json"
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "status": report["status"],
        "robot_commands_sent": False,
        "concurrent_goal_robot_ids": list(ROBOTS),
        "report": str(report_path),
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
