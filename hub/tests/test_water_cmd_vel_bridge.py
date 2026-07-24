from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


OVERLAY = Path(__file__).resolve().parents[1] / "robot_overlay"


def load_bridge():
    path = OVERLAY / "water_cmd_vel_bridge.py"
    spec = importlib.util.spec_from_file_location("test_water_cmd_vel_bridge", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_water_velocity_bridge_clamps_only_supported_axes() -> None:
    bridge = load_bridge()
    command = bridge.sanitize_velocity(
        linear_x=0.30,
        linear_y=0.0,
        linear_z=0.0,
        angular_x=0.0,
        angular_y=0.0,
        angular_z=-0.90,
        max_linear_mps=0.15,
        max_angular_radps=0.40,
    )

    assert command.accepted is True
    assert command.linear_mps == pytest.approx(0.15)
    assert command.angular_radps == pytest.approx(-0.40)

    rejected = bridge.sanitize_velocity(
        linear_x=0.1,
        linear_y=0.01,
        linear_z=0.0,
        angular_x=0.0,
        angular_y=0.0,
        angular_z=0.0,
        max_linear_mps=0.15,
        max_angular_radps=0.40,
    )
    assert rejected.accepted is False
    assert rejected.zero is True


def test_water_velocity_watchdog_fails_closed() -> None:
    bridge = load_bridge()
    command = bridge.SanitizedVelocity(0.1, 0.2, True, "accepted")

    active, reason = bridge.effective_velocity(
        command,
        received_monotonic=10.0,
        now_monotonic=10.2,
        input_timeout_s=0.3,
        water_ready=True,
    )
    assert active.linear_mps == pytest.approx(0.1)
    assert active.angular_radps == pytest.approx(0.2)
    assert reason == "active"

    stale, reason = bridge.effective_velocity(
        command,
        received_monotonic=10.0,
        now_monotonic=10.31,
        input_timeout_s=0.3,
        water_ready=True,
    )
    assert stale.zero is True
    assert reason == "guarded_command_stale"

    unhealthy, reason = bridge.effective_velocity(
        command,
        received_monotonic=10.0,
        now_monotonic=10.1,
        input_timeout_s=0.3,
        water_ready=False,
    )
    assert unhealthy.zero is True
    assert reason == "water_health_not_ready"


def test_water_joy_command_is_newline_terminated_and_bounded() -> None:
    bridge = load_bridge()
    raw = bridge.joy_command_line(0.125, -0.25, request_id="test-id")

    assert raw.endswith(b"\n")
    assert raw.startswith(b"/api/joy_control?")
    assert b"linear_velocity=0.125" in raw
    assert b"angular_velocity=-0.250" in raw
    assert b"uuid=test-id" in raw
    with pytest.raises(ValueError):
        bridge.joy_command_line(0.51, 0.0, request_id="too-fast")


def test_water_status_parser_rejects_estop_and_errors() -> None:
    bridge = load_bridge()

    def response(results):
        return {
            "type": "response",
            "status": "OK",
            "results": results,
        }

    assert bridge.parse_water_health(
        response({"estop_state": False, "error_code": "00000000"})
    )["ready"] is True
    assert bridge.parse_water_health(
        response({"estop_state": True, "error_code": "00000000"})
    )["ready"] is False
    assert bridge.parse_water_health(
        response({"estop_state": False, "error_code": "123"})
    )["ready"] is False


def test_water_bridge_source_has_no_high_level_move_endpoint() -> None:
    source = (OVERLAY / "water_cmd_vel_bridge.py").read_text(encoding="utf-8")
    assert "/api/joy_control" in source
    assert '"/api/move"' not in source
    assert "OPERATOR_PRESENT_AND_YUNJI_CLEAR" in source
