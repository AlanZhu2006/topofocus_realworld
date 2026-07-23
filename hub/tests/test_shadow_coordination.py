from __future__ import annotations

import numpy as np
import pytest

from focus_hub.map_snapshot import MapSnapshot
from focus_hub.shadow_coordination import (
    build_shadow_target_payload,
    collapse_detection_records,
    filter_semantic_categories,
    heading_deg_from_camera_pose,
    validate_shadow_input_timing,
    validate_shadow_target_payload,
    world_to_cell,
)


CATEGORIES = ("chair", "plant", "table")


def make_snapshot() -> MapSnapshot:
    return MapSnapshot(
        grid=np.zeros((5, 10, 10), dtype=np.float32),
        origin_xy_m=(-1.0, -2.0),
        resolution_m=0.05,
        frame_id="shared_world",
        transform_version="test-transform",
        shared_frame_calibration_id="test-calibration",
        map_format_version="focus-hub-central-map-v3",
    )


def test_filter_semantics_hides_operator_rejected_plant_without_mutating_source():
    grid = np.zeros((5, 4, 4), dtype=np.float32)
    grid[0, 0, 0] = 1.0
    grid[1, :, :] = 1.0
    grid[2, 1, 1] = 0.8
    grid[3, 2, 2] = 0.9
    source = grid.copy()

    filtered, hidden = filter_semantic_categories(
        grid, CATEGORIES, ("chair",)
    )

    np.testing.assert_array_equal(grid, source)
    np.testing.assert_array_equal(filtered[:3], grid[:3])
    assert not np.any(filtered[3:])
    assert hidden == {"plant": 1}


def test_shadow_geometry_helpers_are_frame_consistent():
    pose = np.eye(4)
    pose[:2, 2] = [0.0, 1.0]

    assert heading_deg_from_camera_pose(pose) == pytest.approx(90.0)
    assert world_to_cell((0.15, -1.85), (-1.0, -2.0), 0.05, (40, 40)) == (
        2,
        22,
    )
    with pytest.raises(ValueError, match="outside fused grid"):
        world_to_cell((10.0, 10.0), (-1.0, -2.0), 0.05, (40, 40))


def test_detection_records_collapse_duplicate_classes_to_max_confidence():
    records = [
        {"class_name": "chair", "confidence": 0.4},
        {"class_name": "chair", "confidence": 0.8},
        {"class_name": "tv", "confidence": 0.5},
    ]

    assert collapse_detection_records(records) == {"chair": 0.8, "tv": 0.5}


def test_shadow_input_timing_accepts_fresh_synchronized_sources():
    result = validate_shadow_input_timing(
        [95_000_000_000, 97_000_000_000],
        now_ns=100_000_000_000,
        max_input_age_s=10.0,
        max_sync_skew_s=3.0,
        allow_stale_forensic_input=False,
    )

    assert result["status"] == "accepted_fresh"
    assert result["oldest_input_age_s"] == pytest.approx(5.0)
    assert result["cross_robot_capture_skew_s"] == pytest.approx(2.0)


def test_shadow_input_timing_rejects_stale_skew_without_forensic_override():
    arguments = {
        "capture_times_ns": [60_000_000_000, 90_000_000_000],
        "now_ns": 100_000_000_000,
        "max_input_age_s": 20.0,
        "max_sync_skew_s": 5.0,
    }

    with pytest.raises(ValueError, match="oldest input age.*capture skew"):
        validate_shadow_input_timing(
            **arguments,
            allow_stale_forensic_input=False,
        )
    result = validate_shadow_input_timing(
        **arguments,
        allow_stale_forensic_input=True,
    )
    assert result["status"] == "accepted_stale_forensic_override"
    assert len(result["violations"]) == 2


def test_shadow_target_round_trip_and_expiry_fail_closed():
    snapshot = make_snapshot()
    payload = build_shadow_target_payload(
        robot_id="robot-0",
        frontier_id="B",
        goal_category="chair",
        target_xy_m=(1.2, 3.4),
        yaw_rad=0.5,
        snapshot=snapshot,
        created_at_ns=1_000,
        expires_at_ns=2_000,
        run_manifest="/tmp/shadow_manifest.json",
        map_snapshot_sha256="a" * 64,
    )

    target = validate_shadow_target_payload(
        payload,
        robot_id="robot-0",
        snapshot=snapshot,
        now_ns=1_500,
    )
    assert target.frontier_id == "B"
    assert (target.x_m, target.y_m, target.yaw_rad) == (1.2, 3.4, 0.5)

    with pytest.raises(ValueError, match="expired"):
        validate_shadow_target_payload(
            payload,
            robot_id="robot-0",
            snapshot=snapshot,
            now_ns=2_000,
        )
    with pytest.raises(ValueError, match="transform mismatch"):
        wrong_snapshot = MapSnapshot(
            **{
                **snapshot.__dict__,
                "transform_version": "wrong-transform",
            }
        )
        validate_shadow_target_payload(
            payload,
            robot_id="robot-0",
            snapshot=wrong_snapshot,
            now_ns=1_500,
        )

    payload["authority"] = "robot_command"
    with pytest.raises(ValueError, match="display-only authority"):
        validate_shadow_target_payload(
            payload,
            robot_id="robot-0",
            snapshot=snapshot,
            now_ns=1_500,
        )
