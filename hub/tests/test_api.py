from __future__ import annotations

from fastapi.testclient import TestClient

from focus_hub.api import create_app
from focus_hub.registry import RobotPolicy
from focus_hub.settings import Settings


def test_authenticated_observation_is_spooled_and_retry_is_idempotent(tmp_path, observation_factory):
    settings = Settings(
        policies={"robot-0": RobotPolicy("calib-test-v1", allow_goal=False)},
        robot_tokens={"robot-0": "robot-secret"},
        admin_token="admin-secret",
        spool_dir=tmp_path / "spool",
        state_dir=tmp_path / "state",
        min_free_bytes=0,
    )
    client = TestClient(create_app(settings))
    metadata = observation_factory(sequence=4)
    request = {
        "data": {"metadata_json": metadata.model_dump_json()},
        "files": {
            "rgb": ("rgb.jpg", b"rgb", "image/jpeg"),
            "depth": ("depth.png", b"depth", "image/png"),
        },
    }

    assert client.post("/v1/robots/robot-0/observations", **request).status_code == 401
    first = client.post(
        "/v1/robots/robot-0/observations",
        headers={"X-Robot-Token": "robot-secret"},
        **request,
    )
    assert first.status_code == 200
    assert first.json()["status"] == "accepted"
    frame_dir = tmp_path / "spool/robot-0/00000000000000000004"
    assert (frame_dir / "rgb.jpg").read_bytes() == b"rgb"
    assert (frame_dir / "depth.png").read_bytes() == b"depth"

    retry = client.post(
        "/v1/robots/robot-0/observations",
        headers={"X-Robot-Token": "robot-secret"},
        **request,
    )
    assert retry.status_code == 200
    assert retry.json()["status"] == "duplicate"

    decision = client.get(
        "/v1/robots/robot-0/decisions/latest",
        headers={"X-Robot-Token": "robot-secret"},
    )
    assert decision.status_code == 200
    assert decision.json()["mode"] == "HOLD"


def test_heartbeat_endpoint_requires_auth_and_matching_robot_id(tmp_path):
    import time

    settings = Settings(
        policies={"robot-0": RobotPolicy("calib-test-v1", allow_goal=False)},
        robot_tokens={"robot-0": "robot-secret"},
        admin_token="admin-secret",
        spool_dir=tmp_path / "spool",
        state_dir=tmp_path / "state",
        min_free_bytes=0,
    )
    client = TestClient(create_app(settings))
    body = {
        "robot_id": "robot-0",
        "sent_time_ns": time.time_ns(),
        "health": {
            "safety_state": "READY",
            "localization_state": "TRACKING",
            "estop_engaged": False,
            "collision_avoidance_ready": True,
            "motor_controller_ready": True,
        },
    }

    unauthenticated = client.post("/v1/robots/robot-0/heartbeat", json=body)
    assert unauthenticated.status_code == 401

    mismatched = dict(body, robot_id="robot-1")
    mismatched_resp = client.post(
        "/v1/robots/robot-0/heartbeat", json=mismatched, headers={"X-Robot-Token": "robot-secret"})
    assert mismatched_resp.status_code == 422

    ok = client.post("/v1/robots/robot-0/heartbeat", json=body, headers={"X-Robot-Token": "robot-secret"})
    assert ok.status_code == 200
    assert ok.json()["status"] == "accepted"
    assert ok.json()["robot_id"] == "robot-0"

