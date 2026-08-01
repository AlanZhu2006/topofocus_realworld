from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from focus_hub.models import ObservationMetadata
from focus_hub.v2_scene_batch import build_batch_from_shadow_manifest
from test_v2_scene_batch import prepare_round

TOOLS = Path(__file__).resolve().parents[1] / "tools"


def load_module():
    name = "focus_test_v2_source_episode"
    spec = importlib.util.spec_from_file_location(
        name, TOOLS / "run_v2_source_episode.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def load_tool(name: str):
    spec = importlib.util.spec_from_file_location(
        f"focus_test_{name}",
        TOOLS / f"{name}.py",
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def navigation_event(
    decision,
    status: str,
    *,
    event_id: str,
    path_m: float = 1.0,
) -> dict[str, object]:
    return {
        "event_id": event_id,
        "robot_id": decision.robot_id,
        "decision_id": decision.decision_id,
        "leg_id": decision.leg_id,
        "status": status,
        "reason_code": "TEST",
        "observed_at_ns": 1_000_000_000,
        "episode_start_local_pose": {
            "x": 0.0,
            "y": 0.0,
            "yaw_rad": 0.0,
        },
        "local_pose": {"x": 1.0, "y": 0.0, "yaw_rad": 0.0},
        "path_length_m_from_episode_start": path_m,
        "velocity_zero_confirmed": status == "ARRIVED",
        "terminal_observation_sequence": 10,
    }


def test_source_round_step_quota_matches_source_clock():
    module = load_module()

    assert (
        module.source_round_step_quota(
            {
                "source_episode": {
                    "enabled": True,
                    "logical_l_step": 0,
                    "next_logical_l_step": 24,
                }
            }
        )
        == 24
    )
    assert (
        module.source_round_step_quota(
            {
                "source_episode": {
                    "enabled": True,
                    "logical_l_step": 24,
                    "next_logical_l_step": 49,
                }
            }
        )
        == 25
    )
    with pytest.raises(ValueError, match="persistent scene state"):
        module.source_round_step_quota(
            {
                "source_episode": {
                    "enabled": False,
                    "logical_l_step": 0,
                    "next_logical_l_step": 24,
                }
            }
        )
    with pytest.raises(ValueError, match="invalid source logical step delta"):
        module.source_round_step_quota(
            {
                "source_episode": {
                    "enabled": True,
                    "logical_l_step": 24,
                    "next_logical_l_step": 50,
                }
            }
        )


@pytest.mark.parametrize(
    ("flag", "value"),
    (
        ("--goal-continuity-retain-distance-m", "1.30"),
        ("--cross-round-min-progress-m", "0.20"),
        ("--max-consecutive-stagnant-intervals", "2"),
    ),
)
def test_formal_runner_rejects_source_threshold_drift(
    monkeypatch,
    flag,
    value,
):
    module = load_module()
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "run_v2_source_episode.py",
            "--output",
            "/tmp/source-contract-test",
            "--scene-id",
            "scene",
            "--episode-id",
            "episode",
            "--admin-token-file",
            "/tmp/admin-token",
            "--robot-config",
            "/tmp/robot-config",
            flag,
            value,
        ],
    )

    with pytest.raises(SystemExit):
        module.parse_args()


def test_current_goal_evidence_maps_detector_name_to_goal_category():
    module = load_module()

    evidence = module.current_goal_evidence_by_robot(
        {
            "robots": [
                {
                    "robot_id": "robot-0",
                    "detections": {
                        "potted plant": 0.56,
                        "chair": 0.91,
                    },
                },
                {
                    "robot_id": "robot-1",
                    "detections": {
                        "potted plant": 0.88,
                        "airplane": 0.5,
                    },
                },
            ]
        },
        "plant",
    )

    assert evidence == {"robot-0": 0.56, "robot-1": 0.88}


def test_every_explicit_recoverable_failure_revokes_goal_continuity_once():
    module = load_module()

    rejected = module.continuity_rejected_robot_ids(
        {
            "robot-0": {
                "event": {
                    "status": "REJECTED",
                    "reason_code": "LOCAL_PLANNER_TURN_STALLED",
                }
            },
            "robot-1": {
                "event": {
                    "status": "REJECTED",
                    "reason_code": "LOCAL_GOAL_UNREACHABLE",
                }
            },
            "robot-ignored": {
                "event": {
                    "status": "ACCEPTED",
                    "reason_code": "LOCAL_PLANNER_PATH_STALE",
                }
            },
        }
    )

    assert rejected == frozenset({"robot-0", "robot-1"})


@pytest.mark.parametrize(
    "tool_name",
    (
        "build_v2_decision_batch",
        "run_v2_supervised_episode",
    ),
)
def test_every_frozen_manifest_entrypoint_applies_semantic_execution_guard(
    tmp_path,
    observation_factory,
    monkeypatch: pytest.MonkeyPatch,
    tool_name,
):
    _now, manifest, registry, config = prepare_round(
        tmp_path / "inputs",
        observation_factory,
    )
    output = tmp_path / f"{tool_name}-output"
    arguments = [
        f"{tool_name}.py",
        "--manifest",
        str(manifest),
        "--registry-state",
        str(registry),
        "--robot-config",
        str(config),
        "--scene-id",
        "scene-guard-test",
        "--episode-id",
        "episode-guard-test",
        "--output",
        str(output),
    ]
    monkeypatch.setattr(sys, "argv", arguments)

    assert load_tool(tool_name).main() == 0

    execution = json.loads(
        (
            output
            / (
                "decision_batch.json"
                if tool_name == "build_v2_decision_batch"
                else "batch_000_initial.json"
            )
        ).read_text(encoding="utf-8")
    )
    source = json.loads(
        (output / "source_candidate_batch.json").read_text(encoding="utf-8")
    )
    source_by_robot = {item["robot_id"]: item for item in source["decisions"]}
    execution_by_robot = {item["robot_id"]: item for item in execution["decisions"]}
    assert source_by_robot["robot-0"]["target"]["kind"] == ("SEMANTIC_REGION")
    assert execution_by_robot["robot-0"]["target"]["kind"] == ("FRONTIER_POINT")
    guard = json.loads(
        (output / "semantic_execution_guard.json").read_text(encoding="utf-8")
    )
    assert guard["rejected_robot_ids"] == ["robot-0"]


def test_freeze_next_round_fails_immediately_for_latched_map(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
):
    module = load_module()
    calls = 0

    def blocked(*_args, **_kwargs):
        nonlocal calls
        calls += 1
        raise ValueError("robot-0 frozen map blocked: ground plane tilt drift")

    monkeypatch.setattr(module, "freeze", blocked)
    rejection_log = tmp_path / "freeze_rejections.jsonl"

    with pytest.raises(RuntimeError, match="non-recoverable round input"):
        module.freeze_next_round(
            session_path=tmp_path / "session.json",
            session=SimpleNamespace(),
            output=tmp_path / "accepted",
            minimum_sequences={"robot-0": 1, "robot-1": 1},
            max_input_age_s=60.0,
            max_sync_skew_s=5.0,
            timeout_s=45.0,
            poll_s=0.01,
            rejection_log=rejection_log,
        )

    assert calls == 1
    assert "frozen map blocked" in rejection_log.read_text(encoding="utf-8")


def test_freeze_next_round_fails_immediately_for_runtime_map_wording(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
):
    module = load_module()
    calls = 0

    def blocked(*_args, **_kwargs):
        nonlocal calls
        calls += 1
        raise ValueError(
            "robot-0 map blocked: pose discontinuity requires a fresh map session"
        )

    monkeypatch.setattr(module, "freeze", blocked)

    with pytest.raises(RuntimeError, match="non-recoverable round input"):
        module.freeze_next_round(
            session_path=tmp_path / "session.json",
            session=SimpleNamespace(),
            output=tmp_path / "accepted",
            minimum_sequences={"robot-0": 1, "robot-1": 1},
            max_input_age_s=60.0,
            max_sync_skew_s=5.0,
            timeout_s=45.0,
            poll_s=0.01,
            rejection_log=tmp_path / "freeze_rejections.jsonl",
        )

    assert calls == 1


def test_live_mapping_blocks_reports_only_explicit_latches(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
):
    module = load_module()
    monkeypatch.setattr(module, "WORKSPACE", tmp_path)
    robots = []
    for robot_id, reason in (
        ("robot-0", "pose discontinuity"),
        ("robot-1", None),
    ):
        map_dir = tmp_path / "maps" / robot_id
        map_dir.mkdir(parents=True)
        (map_dir / "live_status.json").write_text(
            json.dumps({"mapping_blocked_reason": reason}),
            encoding="utf-8",
        )
        robots.append(
            SimpleNamespace(
                robot_id=robot_id,
                map_dir=str(map_dir.relative_to(tmp_path)),
            )
        )

    assert module.live_mapping_blocks(SimpleNamespace(robots=robots)) == {
        "robot-0": "pose discontinuity"
    }


def test_round_inspection_distinguishes_semantic_and_frontier_arrival(
    tmp_path, observation_factory
):
    module = load_module()
    now, manifest, registry, config = prepare_round(tmp_path, observation_factory)
    built = build_batch_from_shadow_manifest(
        manifest,
        registry,
        scene_id="scene-1",
        episode_id="scene-1-trial-1",
        execution_epoch=0,
        now_ns=now,
        robot_config_path=config,
    )
    decisions = {item.robot_id: item for item in built.batch.decisions}
    server_time_ns = 2_000_000_000
    states = {
        robot_id: {
            "server_time_ns": server_time_ns,
            "latest_event_received_at_ns": server_time_ns - 10_000_000,
            "latest_event": navigation_event(
                decision,
                "ARRIVED",
                event_id=f"{robot_id}-arrived",
            ),
        }
        for robot_id, decision in decisions.items()
    }

    inspected = module.inspect_round_states(states, built.batch, set(decisions))

    assert set(inspected.semantic_arrivals) == {"robot-0"}
    assert set(inspected.frontier_arrivals) == {"robot-1"}
    assert inspected.failures == {}
    assert inspected.current_feedback_ready is True

    states["robot-1"]["latest_event"] = navigation_event(
        decisions["robot-1"],
        "REJECTED",
        event_id="robot-1-rejected",
    )
    inspected = module.inspect_round_states(states, built.batch, set(decisions))
    assert set(inspected.failures) == {"robot-1"}
    assert inspected.frontier_arrivals == {}


def test_semantic_local_failure_keeps_healthy_semantic_peer_active(
    tmp_path, observation_factory
):
    module = load_module()
    now, manifest, registry, config = prepare_round(tmp_path, observation_factory)
    built = build_batch_from_shadow_manifest(
        manifest,
        registry,
        scene_id="scene-1",
        episode_id="scene-1-trial-1",
        execution_epoch=0,
        now_ns=now,
        robot_config_path=config,
    )
    decisions = {item.robot_id: item for item in built.batch.decisions}
    semantic_target = decisions["robot-0"].target
    assert semantic_target is not None
    decisions["robot-1"] = decisions["robot-1"].model_copy(
        update={"target": semantic_target}
    )

    (
        failed_semantic,
        remaining_active,
        remaining_semantic,
    ) = module.partition_recoverable_failures(
        decisions,
        {"robot-0", "robot-1"},
        {
            "robot-1": {
                "status": "REJECTED",
                "reason_code": "LOCAL_GOAL_UNREACHABLE",
            }
        },
    )

    assert failed_semantic == {"robot-1"}
    assert remaining_active == {"robot-0"}
    assert remaining_semantic == {"robot-0"}


def test_semantic_local_failure_replans_when_only_frontier_peer_remains(
    tmp_path, observation_factory
):
    module = load_module()
    now, manifest, registry, config = prepare_round(tmp_path, observation_factory)
    built = build_batch_from_shadow_manifest(
        manifest,
        registry,
        scene_id="scene-1",
        episode_id="scene-1-trial-1",
        execution_epoch=0,
        now_ns=now,
        robot_config_path=config,
    )
    decisions = {item.robot_id: item for item in built.batch.decisions}

    (
        failed_semantic,
        remaining_active,
        remaining_semantic,
    ) = module.partition_recoverable_failures(
        decisions,
        {"robot-0", "robot-1"},
        {
            "robot-0": {
                "status": "REJECTED",
                "reason_code": "LOCAL_GOAL_UNREACHABLE",
            }
        },
    )

    assert failed_semantic == {"robot-0"}
    assert remaining_active == {"robot-1"}
    assert remaining_semantic == set()


def test_foxglove_target_events_record_the_actual_post_guard_batch(
    tmp_path,
    observation_factory,
    monkeypatch: pytest.MonkeyPatch,
):
    module = load_module()
    monkeypatch.setattr(module, "WORKSPACE", tmp_path)
    now, manifest, registry, config = prepare_round(tmp_path, observation_factory)
    built = build_batch_from_shadow_manifest(
        manifest,
        registry,
        scene_id="scene-1",
        episode_id="scene-1-trial-1",
        execution_epoch=3,
        now_ns=now,
        robot_config_path=config,
    )
    robots = []
    for robot_id in ("robot-0", "robot-1"):
        map_dir = tmp_path / "hub" / "runtime" / f"map-{robot_id}"
        map_dir.mkdir(parents=True)
        robots.append(
            SimpleNamespace(
                robot_id=robot_id,
                map_dir=str(map_dir.relative_to(tmp_path)),
            )
        )
    session = SimpleNamespace(robots=tuple(robots))

    paths = module.write_foxglove_vlm_target_events(
        session,
        built.batch,
        publication_reason="round_0_goal",
        published_at_ns=now + 1,
    )

    assert set(paths) == {"robot-0", "robot-1"}
    events = {
        robot.robot_id: json.loads(
            (tmp_path / robot.map_dir / module.VLM_TARGET_EVENT_FILENAME).read_text(
                encoding="utf-8"
            )
        )
        for robot in robots
    }
    assert all(
        event["schema_version"] == module.VLM_TARGET_EVENT_SCHEMA_VERSION
        for event in events.values()
    )
    assert all(
        event["status"] == "hub_accepted_high_level_decision"
        for event in events.values()
    )
    assert events["robot-0"]["target"]["kind"] == "SEMANTIC_REGION"
    assert events["robot-0"]["target"]["coordinate_authority"] == (
        "display_only_semantic_region_centroid"
    )
    assert events["robot-1"]["target"]["kind"] == "FRONTIER_POINT"
    assert events["robot-1"]["target"]["coordinate_authority"] == (
        "authoritative_frontier_goal_pose"
    )


def test_semantic_arrival_seed_survives_following_hold():
    module = load_module()
    arrival = {
        "status": "ARRIVED",
        "observed_at_ns": 1_000_000_000,
        "episode_start_local_pose": {
            "x": 0.0,
            "y": 0.0,
            "yaw_rad": 0.0,
        },
        "local_pose": {"x": 1.0, "y": 0.0, "yaw_rad": 0.0},
        "path_length_m_from_episode_start": 2.5,
        "velocity_zero_confirmed": True,
        "terminal_observation_sequence": 42,
    }
    following_hold = {
        **arrival,
        "status": "HOLDING",
        "velocity_zero_confirmed": True,
    }

    seed = module.evaluation_seed_from_events(
        {"robot-0": following_hold},
        {"robot-0": arrival},
    )

    assert seed["robot-0"]["latest_navigation_status"] == "ARRIVED"
    assert seed["robot-0"]["arrival_target_kind"] == "SEMANTIC_REGION"
    assert seed["robot-0"]["local_planner_stopped"] is True
    assert seed["robot-0"]["actual_path_length_m"] == 2.5


class ReadinessClient:
    def __init__(self, readiness: dict[str, dict[str, object]]) -> None:
        self._readiness = readiness

    def readiness(self, robot_id: str) -> dict[str, object]:
        return dict(self._readiness[robot_id])


def transient_slam_readiness(*, ready: bool = False) -> dict[str, object]:
    return {
        "ready_for_goal": ready,
        "blockers": [] if ready else ["HEALTH_NOT_READY"],
        "health": {
            "safety_state": "READY" if ready else "HOLD",
            "localization_state": "TRACKING" if ready else "LOST",
            "estop_engaged": False,
            "collision_avoidance_ready": True,
            "motor_controller_ready": True,
            "detail": (
                "slam_optimizer_imu_valid"
                if ready
                else ("optimizer_status=skipped_imu_invalid; " "odom_age=0.693s/5.000s")
            ),
        },
    }


def test_pre_goal_readiness_waits_only_for_transient_slam_hold(
    monkeypatch: pytest.MonkeyPatch,
):
    module = load_module()

    assert module.transient_slam_readiness_waitable(transient_slam_readiness())
    hard = transient_slam_readiness()
    hard["health"]["detail"] = "optimizer_status=failed"
    assert not module.transient_slam_readiness_waitable(hard)
    stale = transient_slam_readiness()
    stale["blockers"] = ["HEALTH_NOT_READY", "HEALTH_STALE"]
    assert not module.transient_slam_readiness_waitable(stale)

    class RecoveringClient:
        calls = 0

        def readiness(self, _robot_id: str) -> dict[str, object]:
            self.calls += 1
            return transient_slam_readiness(ready=self.calls >= 2)

    monkeypatch.setattr(module.time, "sleep", lambda _seconds: None)
    client = RecoveringClient()
    reports, waited_s = module.wait_for_goal_readiness(
        client,
        {"robot-0"},
        timeout_s=1.0,
        poll_s=0.01,
    )

    assert reports["robot-0"]["ready_for_goal"] is True
    assert client.calls == 2
    assert waited_s >= 0.0


def test_pre_goal_readiness_rejects_nontransient_gate_immediately():
    module = load_module()
    hard = transient_slam_readiness()
    hard["health"]["detail"] = "optimizer_status=failed"

    with pytest.raises(RuntimeError, match="runtime readiness blocked GOAL"):
        module.wait_for_goal_readiness(
            ReadinessClient({"robot-0": hard}),
            {"robot-0"},
            timeout_s=1.0,
            poll_s=0.01,
        )


def test_pre_goal_readiness_partitions_hard_block_from_ready_peer():
    module = load_module()
    ready = transient_slam_readiness(ready=True)
    hard = transient_slam_readiness()
    hard["health"]["detail"] = "slam_optimizer_imu_valid; odom_age=7.949s/5.000s"

    reports, ready_ids, blocked_ids, waited_s = module.partition_goal_readiness(
        ReadinessClient({"robot-0": hard, "robot-1": ready}),
        {"robot-0", "robot-1"},
        timeout_s=1.0,
        poll_s=0.01,
    )

    assert reports["robot-0"]["ready_for_goal"] is False
    assert ready_ids == {"robot-1"}
    assert blocked_ids == {"robot-0"}
    assert waited_s >= 0.0


def write_terminal_observation(
    spool: Path,
    observation_factory,
    *,
    robot_id: str,
    sequence: int,
    now_ns: int,
) -> None:
    source = spool / robot_id / f"{sequence:020d}"
    source.mkdir(parents=True)
    rgb = f"rgb-{robot_id}-{sequence}".encode()
    depth = f"depth-{robot_id}-{sequence}".encode()
    raw = observation_factory(
        robot_id=robot_id,
        sequence=sequence,
        now_ns=now_ns,
    ).model_dump(mode="json")
    raw["rgb_size_bytes"] = len(rgb)
    raw["depth_size_bytes"] = len(depth)
    raw["rgb_sha256"] = hashlib.sha256(rgb).hexdigest()
    raw["depth_sha256"] = hashlib.sha256(depth).hexdigest()
    metadata = ObservationMetadata.model_validate(raw)
    (source / "rgb.jpg").write_bytes(rgb)
    (source / "depth.png").write_bytes(depth)
    (source / "metadata.json").write_text(metadata.model_dump_json(), encoding="utf-8")


def test_terminal_evidence_seals_verified_post_arrival_bytes(
    tmp_path,
    observation_factory,
    monkeypatch: pytest.MonkeyPatch,
):
    module = load_module()
    monkeypatch.setattr(module, "WORKSPACE", tmp_path)
    spool = tmp_path / "spool"
    robots = []
    for index, robot_id in enumerate(("robot-0", "robot-1")):
        map_dir = tmp_path / "hub" / "runtime" / f"map-{robot_id}"
        map_dir.mkdir(parents=True)
        for name, payload in (
            ("central_map.npz", b"map"),
            ("map_summary.json", b"{}"),
            ("live_status.json", b"{}"),
            ("map_session_contract.json", b"{}"),
        ):
            (map_dir / name).write_bytes(payload)
        robots.append(
            SimpleNamespace(
                robot_id=robot_id,
                map_dir=str(map_dir.relative_to(tmp_path)),
            )
        )
        write_terminal_observation(
            spool,
            observation_factory,
            robot_id=robot_id,
            sequence=11 + index,
            now_ns=3_000_000_000,
        )
    session = SimpleNamespace(robots=tuple(robots))
    client = ReadinessClient(
        {
            "robot-0": {
                "last_observation_sequence": 11,
                "last_observation_received_at_ns": 2_100_000_000,
            },
            "robot-1": {
                "last_observation_sequence": 12,
                "last_observation_received_at_ns": 2_200_000_000,
            },
        }
    )
    arrival = {
        "status": "ARRIVED",
        "observed_at_ns": 1_000_000_000,
        "velocity_zero_confirmed": True,
    }
    output = tmp_path / "terminal"

    result = module.wait_and_seal_terminal_evidence(
        client=client,
        session=session,
        spool=spool,
        semantic_arrivals={"robot-0": arrival},
        arrival_received_at_ns={"robot-0": 2_000_000_000},
        decision_inputs={"robot-0": 10, "robot-1": 10},
        output=output,
        timeout_s=0.1,
        poll_s=0.01,
    )

    manifest = json.loads(
        (output / "terminal_evidence.json").read_text(encoding="utf-8")
    )
    assert result["status"] == "complete_post_arrival_candidate"
    assert manifest["official_success_verified"] is False
    assert (
        manifest["robots"]["robot-0"]["observation_timing_status"]
        == "observed_post_arrival_hub_observation"
    )
    assert (
        manifest["robots"]["robot-1"]["observation_timing_status"]
        == "observed_scene_terminal_hub_observation"
    )
    for robot_id in ("robot-0", "robot-1"):
        for key in ("rgb", "depth", "metadata"):
            record = manifest["robots"][robot_id][key]
            preserved = Path(record["path"])
            assert preserved.stat().st_size == record["size_bytes"]
            assert module.sha256_file(preserved) == record["sha256"]
        assert len(manifest["robots"][robot_id]["map_artifacts"]) == 4


def test_frozen_shared_robot_positions_preserve_status_provenance(tmp_path):
    module = load_module()
    rows = []
    expected = {
        "robot-0": (0.3362092841198523, -0.19971329635089863),
        "robot-1": (-0.9426940506797459, -0.1337700123489402),
    }
    for robot_id, point in expected.items():
        map_dir = tmp_path / robot_id
        map_dir.mkdir()
        (map_dir / "live_status.json").write_text(
            json.dumps(
                {
                    "frame_id": "shared_world",
                    "last_robot_xy_m": list(point),
                }
            ),
            encoding="utf-8",
        )
        rows.append({"robot_id": robot_id, "map_dir": str(map_dir)})

    positions, provenance, errors = module.frozen_shared_robot_positions(
        {"robots": rows}
    )

    assert positions == expected
    assert errors == {}
    assert set(provenance) == set(expected)
    for record in provenance.values():
        assert record["classification"] == (
            "source_derived_frozen_shared_frame_robot_pose"
        )
        assert record["size_bytes"] > 0
        assert len(record["sha256"]) == 64
        assert record["last_robot_heading_deg"] is None


def test_live_failure_pose_snapshot_requires_full_session_identity(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
):
    module = load_module()
    monkeypatch.setattr(module, "WORKSPACE", tmp_path)
    robots = []
    source_bytes: dict[str, bytes] = {}
    for index, robot_id in enumerate(("robot-0", "robot-1")):
        transform = f"{robot_id}-transform-v1"
        map_dir = tmp_path / "hub" / "runtime" / f"map-{robot_id}"
        map_dir.mkdir(parents=True)
        raw = json.dumps(
            {
                "robot_id": robot_id,
                "frame_id": "shared_world",
                "transform_version": transform,
                "shared_frame_calibration_id": (
                    "calibration-v1" if robot_id == "robot-0" else "wrong-calibration"
                ),
                "mapping_blocked_reason": None,
                "last_observation_sequence": 20 + index,
                "last_capture_time_ns": 5_000_000_000 + index,
                "last_robot_xy_m": [1.0 + index, 2.0 + index],
                "last_robot_heading_deg": 30.0 + index,
            },
            separators=(",", ":"),
        ).encode("utf-8")
        (map_dir / "live_status.json").write_bytes(raw)
        source_bytes[robot_id] = raw
        robots.append(
            SimpleNamespace(
                robot_id=robot_id,
                map_dir=str(map_dir.relative_to(tmp_path)),
                transform_version=transform,
            )
        )
    session = SimpleNamespace(
        robots=tuple(robots),
        calibration=SimpleNamespace(calibration_id="calibration-v1"),
    )

    positions, provenance, errors = module.capture_live_shared_robot_positions(
        session,
        tmp_path / "failure-pose",
        robot_ids={"robot-0", "robot-1"},
        minimum_sequences={"robot-0": 20, "robot-1": 21},
        minimum_capture_times_ns={
            "robot-0": 4_000_000_000,
            "robot-1": 4_000_000_000,
        },
    )

    assert positions == {"robot-0": (1.0, 2.0)}
    assert set(provenance) == {"robot-0", "robot-1"}
    assert set(errors) == {"robot-1"}
    assert "calibration differs" in errors["robot-1"]
    assert provenance["robot-0"]["validation"] == (
        "accepted_for_failure_pose_authority"
    )
    assert provenance["robot-0"]["minimum_failure_capture_time_ns"] == (4_000_000_000)
    assert provenance["robot-1"]["validation"] == "rejected"
    for robot_id in ("robot-0", "robot-1"):
        preserved = tmp_path / "failure-pose" / f"{robot_id}_live_status.json"
        assert preserved.read_bytes() == source_bytes[robot_id]
        assert module.sha256_file(preserved) == provenance[robot_id]["sha256"]


def test_cross_round_progress_guard_applies_source_stationary_rule_once():
    module = load_module()
    first, counts = module.cross_round_progress_guard(
        previous_positions={"robot-1": (1.0, 2.0)},
        current_positions={"robot-1": (1.03, 2.01)},
        previous_active_robot_ids={"robot-1"},
        current_active_robot_ids={"robot-1"},
        previous_stagnant_intervals={},
    )
    assert first["status"] == "blocked"
    assert first["robots"]["robot-1"]["consecutive_stagnant_intervals"] == 1
    assert first["blocked_robot_ids"] == ["robot-1"]
    assert counts == {"robot-1": 1}


def test_cross_round_progress_guard_resets_after_real_progress_or_inactivity():
    module = load_module()
    progressed, counts = module.cross_round_progress_guard(
        previous_positions={"robot-1": (0.0, 0.0)},
        current_positions={"robot-1": (0.13, 0.0)},
        previous_active_robot_ids={"robot-1"},
        current_active_robot_ids={"robot-1"},
        previous_stagnant_intervals={"robot-1": 1},
    )
    baseline, counts = module.cross_round_progress_guard(
        previous_positions={"robot-1": (0.13, 0.0)},
        current_positions={"robot-1": (0.13, 0.0)},
        previous_active_robot_ids=set(),
        current_active_robot_ids={"robot-1"},
        previous_stagnant_intervals=counts,
    )

    assert progressed["status"] == "pass"
    assert progressed["robots"]["robot-1"]["status"] == "progressed"
    assert baseline["robots"]["robot-1"]["status"] == "baseline_only"
    assert counts == {"robot-1": 0}


def test_coordination_hold_is_not_cross_round_stall_evidence():
    module = load_module()

    report, counts = module.cross_round_progress_guard(
        previous_positions={"robot-1": (0.0, 0.0)},
        current_positions={"robot-1": (0.0, 0.0)},
        # The source allocated robot-1, but the published execution batch
        # held it.  Only the effective execution set belongs here.
        previous_active_robot_ids=set(),
        current_active_robot_ids={"robot-1"},
        previous_stagnant_intervals={"robot-1": 1},
    )

    assert report["status"] == "pass"
    assert report["blocked_robot_ids"] == []
    assert report["robots"]["robot-1"] == {
        "status": "baseline_only",
        "consecutive_stagnant_intervals": 0,
    }
    assert counts == {"robot-1": 0}


def test_semantic_path_replan_does_not_charge_peer_stagnation():
    module = load_module()
    result = module.RoundResult(
        status="replan",
        reason=module.SEMANTIC_PATH_REPLAN_REASON,
        final_states={},
        semantic_arrivals={},
        latest_events={},
        feedback_counts={"robot-0": 3, "robot-1": 3},
    )

    active, counts = module.progress_memory_after_round(
        result,
        active_robot_ids={"robot-0", "robot-1"},
        stagnant_intervals={"robot-0": 1, "robot-1": 0},
    )

    assert active == set()
    assert counts == {}


def test_completed_round_preserves_cross_round_progress_memory():
    module = load_module()
    result = module.RoundResult(
        status="replan",
        reason="source-derived 25-tick round completed",
        final_states={},
        semantic_arrivals={},
        latest_events={},
        feedback_counts={"robot-0": 25},
    )

    active, counts = module.progress_memory_after_round(
        result,
        active_robot_ids={"robot-0"},
        stagnant_intervals={"robot-0": 1},
    )

    assert active == {"robot-0"}
    assert counts == {"robot-0": 1}


def test_interrupted_frontier_is_removed_from_progress_comparison():
    module = load_module()
    result = module.RoundResult(
        status="replan",
        reason="source-derived 25-tick round completed",
        final_states={},
        semantic_arrivals={},
        latest_events={},
        feedback_counts={"robot-0": 25, "robot-1": 2},
        interrupted_robot_ids=frozenset({"robot-1"}),
    )

    active, counts = module.progress_memory_after_round(
        result,
        active_robot_ids={"robot-0", "robot-1"},
        stagnant_intervals={"robot-0": 0, "robot-1": 1},
    )

    assert active == {"robot-0"}
    assert counts == {"robot-0": 0}
