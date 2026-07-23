from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from focus_hub.realworld_session import RobotSession


TOOLS = Path(__file__).resolve().parents[1] / "tools"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "freeze_realworld_inputs",
        TOOLS / "freeze_realworld_inputs.py",
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_inputs(
    tmp_path: Path,
    observation,
) -> tuple[SimpleNamespace, Path, Path]:
    robot = RobotSession(
        robot_id="robot-0",
        name="wsj",
        transform_version="calib-test-v1",
        map_dir="hub/runtime/map_out_wsj_test",
        map_start_after_sequence=5,
        remote_root="/robot/release",
        remote_calibration_path="/robot/shared.json",
        remote_base_camera_calibration_path="/robot/base.json",
        remote_hub_url="http://127.0.0.1:18089",
        remote_preview_url="http://127.0.0.1:18766",
        ssh_tmux_target="robot:ssh",
    )
    session = SimpleNamespace(
        session_id="test",
        code=SimpleNamespace(git_commit="a" * 40),
        robots=(robot,),
        calibration=SimpleNamespace(calibration_id="shared-test-v1"),
        runtime=SimpleNamespace(
            map_goal_category="chair",
            semantic_backend="segformer-ade20k",
        ),
    )
    frozen = tmp_path / "frozen"
    frozen.mkdir()
    np.savez_compressed(
        frozen / "central_map.npz",
        grid=np.zeros((2, 3, 3), dtype=np.float32),
        origin_xy_m=np.zeros(2),
        resolution_m=np.asarray(0.05),
        frame_id=np.asarray("shared_world"),
        transform_version=np.asarray("calib-test-v1"),
        shared_frame_calibration_id=np.asarray("shared-test-v1"),
        map_format_version=np.asarray("focus-hub-central-map-v3"),
        snapshot_id=np.asarray("robot-0:10:test"),
    )
    common = {
        "robot_id": "robot-0",
        "frame_id": "shared_world",
        "transform_version": "calib-test-v1",
        "shared_frame_calibration_id": "shared-test-v1",
        "mapping_blocked_reason": None,
    }
    (frozen / "map_summary.json").write_text(
        json.dumps(
            {
                **common,
                "snapshot_id": "robot-0:10:test",
                "semantic_mapping": {
                    "yolo_reinforcement": {
                        "enabled": True,
                        "last_sequence": observation.sequence,
                    }
                },
            }
        )
    )
    (frozen / "live_status.json").write_text(json.dumps(common))
    (frozen / "map_session_contract.json").write_text(
        json.dumps(
            {
                "schema_version": "focus-realworld-map-session-contract-v1",
                "session_id": "test",
                "code_git_commit": "a" * 40,
                "robot_id": "robot-0",
                "map_dir": "hub/runtime/map_out_wsj_test",
                "start_after_sequence": 5,
                "transform_version": "calib-test-v1",
                "shared_frame_calibration_id": "shared-test-v1",
                "goal_category": "chair",
                "semantic_backend": "segformer-ade20k",
                "semantic_yolo": {
                    "enabled": True,
                    "confidence": 0.2,
                    "evidence_only": True,
                },
            }
        )
    )
    spool = tmp_path / "spool"
    source = spool / "robot-0" / f"{observation.sequence:020d}"
    source.mkdir(parents=True)
    (source / "metadata.json").write_text(
        observation.model_dump_json(), encoding="utf-8"
    )
    (source / "rgb.jpg").write_bytes(b"rgb")
    (source / "depth.png").write_bytes(b"depth")
    return session, frozen, spool


def test_frozen_input_requires_current_epoch_and_ready_command_metadata(
    tmp_path, observation_factory
):
    module = load_module()
    now_ns = 100_000_000_000
    observation = observation_factory(
        sequence=10,
        now_ns=now_ns,
        mapping_only=False,
        health_ready=True,
    )
    session, frozen, spool = build_inputs(tmp_path, observation)

    record, snapshot, metadata = module.validate_frozen_robot(
        session,
        "robot-0",
        frozen,
        tmp_path / "accepted/wsj",
        spool,
        now_ns=now_ns,
        max_input_age_s=1.0,
        minimum_source_sequence=10,
    )

    assert record["source_sequence"] == 10
    assert snapshot.transform_version == "calib-test-v1"
    assert metadata.mapping_only is False


def test_frozen_input_rejects_source_before_clean_hub_epoch(
    tmp_path, observation_factory
):
    module = load_module()
    now_ns = 100_000_000_000
    observation = observation_factory(
        sequence=10,
        now_ns=now_ns,
        mapping_only=False,
        health_ready=True,
    )
    session, frozen, spool = build_inputs(tmp_path, observation)

    with pytest.raises(ValueError, match="current Hub epoch"):
        module.validate_frozen_robot(
            session,
            "robot-0",
            frozen,
            tmp_path / "accepted/wsj",
            spool,
            now_ns=now_ns,
            max_input_age_s=1.0,
            minimum_source_sequence=11,
        )
