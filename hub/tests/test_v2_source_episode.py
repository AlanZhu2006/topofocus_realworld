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

    assert module.source_round_step_quota(
        {
            "source_episode": {
                "enabled": True,
                "logical_l_step": 0,
                "next_logical_l_step": 24,
            }
        }
    ) == 24
    assert module.source_round_step_quota(
        {
            "source_episode": {
                "enabled": True,
                "logical_l_step": 24,
                "next_logical_l_step": 49,
            }
        }
    ) == 25
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


def test_round_inspection_distinguishes_semantic_and_frontier_arrival(
    tmp_path, observation_factory
):
    module = load_module()
    now, manifest, registry, config = prepare_round(
        tmp_path, observation_factory
    )
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

    inspected = module.inspect_round_states(
        states, built.batch, set(decisions)
    )

    assert set(inspected.semantic_arrivals) == {"robot-0"}
    assert set(inspected.frontier_arrivals) == {"robot-1"}
    assert inspected.failures == {}
    assert inspected.current_feedback_ready is True

    states["robot-1"]["latest_event"] = navigation_event(
        decisions["robot-1"],
        "REJECTED",
        event_id="robot-1-rejected",
    )
    inspected = module.inspect_round_states(
        states, built.batch, set(decisions)
    )
    assert set(inspected.failures) == {"robot-1"}
    assert inspected.frontier_arrivals == {}


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
    (source / "metadata.json").write_text(
        metadata.model_dump_json(), encoding="utf-8"
    )


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
