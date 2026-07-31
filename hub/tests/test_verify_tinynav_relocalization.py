from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


OVERLAY = Path(__file__).parents[1] / "robot_overlay"


def load_module():
    path = OVERLAY / "verify_tinynav_relocalization.py"
    spec = importlib.util.spec_from_file_location(
        "test_verify_tinynav_relocalization_module", path
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def status_message(**updates):
    payload = {
        "schema_version": "focus-tinynav-relocalized-odometry-v1",
        "ready": True,
        "reason": "READY",
        "tracking_frame": "world",
        "map_frame": "map",
        "support": 3,
        "minimum_support": 2,
        "latest_supported_age_s": 0.2,
        "map": {"map_id": "map-1"},
        "raw_pose_fallback_enabled": False,
        "robot_commands_issued": False,
    }
    payload.update(updates)
    return SimpleNamespace(data=json.dumps(payload))


def test_ready_status_is_strictly_validated() -> None:
    module = load_module()

    result = module.validate_status(
        status_message(),
        expected_map_id="map-1",
        maximum_age_s=30.0,
    )

    assert result["reason"] == "READY"


@pytest.mark.parametrize(
    ("updates", "match"),
    [
        ({"ready": False}, "not ready"),
        ({"raw_pose_fallback_enabled": True}, "fallback"),
        ({"latest_supported_age_s": 31.0}, "stale"),
        ({"map": {"map_id": "other"}}, "identity"),
        ({"support": 1}, "support"),
    ],
)
def test_bad_status_fails_closed(updates, match) -> None:
    module = load_module()

    with pytest.raises(ValueError, match=match):
        module.validate_status(
            status_message(**updates),
            expected_map_id="map-1",
            maximum_age_s=30.0,
        )
