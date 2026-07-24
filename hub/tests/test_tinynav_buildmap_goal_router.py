from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import pytest

from focus_hub.v2_robot_runtime import OccupancyGrid2D


OVERLAY = Path(__file__).resolve().parents[1] / "robot_overlay"


def load_router():
    path = OVERLAY / "tinynav_buildmap_goal_router.py"
    module_name = "test_tinynav_buildmap_goal_router_overlay"
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def goal_payload(*, expires_at_ns: int = 2_000_000_000) -> str:
    return json.dumps(
        {
            "0": {
                "id": 0,
                "name": "focus_hub_goal",
                "position": [4.5, 2.5, 0.0],
                "yaw_rad": 0.2,
                "source": "focus_hub_v2",
                "target_kind": "FRONTIER_POINT",
                "decision_id": "decision-1",
                "leg_id": "leg-1",
                "lease_sequence": 0,
                "expires_at_ns": expires_at_ns,
                "arrival_radius_m": 0.5,
            }
        }
    )


def grid(
    data: list[int],
    *,
    width: int,
    height: int,
    resolution_m: float = 1.0,
) -> OccupancyGrid2D:
    return OccupancyGrid2D(
        width=width,
        height=height,
        resolution_m=resolution_m,
        origin_x_m=0.0,
        origin_y_m=0.0,
        data=tuple(data),
    )


def test_goal_parser_accepts_only_fresh_single_hub_goal():
    router = load_router()

    parsed = router.parse_goal_payload(goal_payload(), now_ns=1_000_000_000)

    assert parsed.decision_id == "decision-1"
    assert parsed.leg_id == "leg-1"
    assert parsed.arrival_radius_m == pytest.approx(0.5)
    assert parsed.target_kind == "FRONTIER_POINT"

    with pytest.raises(ValueError, match="expired"):
        router.parse_goal_payload(
            goal_payload(expires_at_ns=1_000_000_000),
            now_ns=1_000_000_000,
        )
    with pytest.raises(ValueError, match="exactly one"):
        router.parse_goal_payload("{}", now_ns=1_000_000_000)
    foreign = json.loads(goal_payload())
    foreign["0"]["source"] = "rviz"
    with pytest.raises(ValueError, match="source"):
        router.parse_goal_payload(json.dumps(foreign), now_ns=1_000_000_000)


def test_same_leg_lease_renewal_must_be_newer_and_target_stable():
    router = load_router()
    current = router.parse_goal_payload(goal_payload(), now_ns=1_000_000_000)
    renewed_payload = json.loads(
        goal_payload(expires_at_ns=3_000_000_000)
    )
    renewed_payload["0"]["decision_id"] = "decision-2"
    renewed_payload["0"]["lease_sequence"] = 1
    renewed = router.parse_goal_payload(
        json.dumps(renewed_payload), now_ns=1_000_000_000
    )

    assert router.is_seamless_lease_renewal(current, renewed)

    changed_payload = json.loads(json.dumps(renewed_payload))
    changed_payload["0"]["position"][0] += 0.01
    changed = router.parse_goal_payload(
        json.dumps(changed_payload), now_ns=1_000_000_000
    )
    assert not router.is_seamless_lease_renewal(current, changed)

    old_sequence_payload = json.loads(json.dumps(renewed_payload))
    old_sequence_payload["0"]["lease_sequence"] = 0
    old_sequence = router.parse_goal_payload(
        json.dumps(old_sequence_payload), now_ns=1_000_000_000
    )
    assert not router.is_seamless_lease_renewal(current, old_sequence)


def test_a_star_uses_known_free_gap_and_never_crosses_unknown():
    router = load_router()
    data = [0] * (7 * 5)
    for row in range(5):
        data[row * 7 + 3] = 100
    data[2 * 7 + 3] = 0
    data[1 * 7 + 2] = -1
    occupancy = grid(data, width=7, height=5)

    plan = router.plan_route(
        occupancy,
        start_x=0.5,
        start_y=2.5,
        goal_x=5.5,
        goal_y=2.5,
        arrival_radius_m=0.1,
        clearance_cells=0,
    )

    assert plan is not None
    assert (2, 3) in plan.cells
    assert (1, 2) not in plan.cells
    assert all(occupancy.data[row * occupancy.width + column] == 0 for row, column in plan.cells)


def test_a_star_fails_closed_for_solid_wall():
    router = load_router()
    data = [0] * (7 * 5)
    for row in range(5):
        data[row * 7 + 3] = 100
    occupancy = grid(data, width=7, height=5)

    plan = router.plan_route(
        occupancy,
        start_x=0.5,
        start_y=2.5,
        goal_x=5.5,
        goal_y=2.5,
        arrival_radius_m=0.1,
        clearance_cells=0,
    )

    assert plan is None


def test_frontier_route_can_make_partial_progress_without_crossing_unknown():
    router = load_router()
    occupancy = grid(
        [0, 0, 0, 0, -1, -1, -1],
        width=7,
        height=1,
    )

    plan = router.plan_route(
        occupancy,
        start_x=0.5,
        start_y=0.5,
        goal_x=6.5,
        goal_y=0.5,
        arrival_radius_m=0.5,
        clearance_cells=0,
        allow_partial_progress=True,
        minimum_progress_m=0.1,
    )

    assert plan is not None
    assert plan.reaches_arrival_region is False
    assert plan.target_cell == (0, 3)
    assert plan.remaining_goal_distance_m == pytest.approx(2.5)
    assert all(
        occupancy.data[row * occupancy.width + column] == 0
        for row, column in plan.cells
    )


def test_partial_route_requires_actual_progress():
    router = load_router()
    occupancy = grid([0, -1, -1], width=3, height=1)

    assert router.plan_route(
        occupancy,
        start_x=0.5,
        start_y=0.5,
        goal_x=2.5,
        goal_y=0.5,
        arrival_radius_m=0.1,
        clearance_cells=0,
        allow_partial_progress=True,
        minimum_progress_m=0.1,
    ) is None


def test_latched_map_stays_valid_only_within_bounded_base_motion():
    router = load_router()

    assert router.cached_map_valid_for_pose(
        map_age_s=1.0,
        map_timeout_s=6.0,
        map_anchor_base_xy=None,
        current_base_xy=None,
        max_cached_map_motion_m=0.1,
    ) == (True, 0.0)
    valid, displacement = router.cached_map_valid_for_pose(
        map_age_s=30.0,
        map_timeout_s=6.0,
        map_anchor_base_xy=(1.0, 2.0),
        current_base_xy=(1.06, 2.0),
        max_cached_map_motion_m=0.1,
    )
    assert valid is True
    assert displacement == pytest.approx(0.06)
    valid, displacement = router.cached_map_valid_for_pose(
        map_age_s=30.0,
        map_timeout_s=6.0,
        map_anchor_base_xy=(1.0, 2.0),
        current_base_xy=(1.11, 2.0),
        max_cached_map_motion_m=0.1,
    )
    assert valid is False
    assert displacement == pytest.approx(0.11)
    valid, displacement = router.cached_map_valid_for_pose(
        map_age_s=30.0,
        map_timeout_s=6.0,
        map_anchor_base_xy=(1.0, 2.0),
        current_base_xy=(1.24, 2.0),
        max_cached_map_motion_m=0.25,
    )
    assert valid is True
    assert displacement == pytest.approx(0.24)
    valid, displacement = router.cached_map_valid_for_pose(
        map_age_s=30.0,
        map_timeout_s=6.0,
        map_anchor_base_xy=(1.0, 2.0),
        current_base_xy=(1.26, 2.0),
        max_cached_map_motion_m=0.25,
    )
    assert valid is False
    assert displacement == pytest.approx(0.26)
    assert router.cached_map_valid_for_pose(
        map_age_s=30.0,
        map_timeout_s=6.0,
        map_anchor_base_xy=None,
        current_base_xy=(1.0, 2.0),
        max_cached_map_motion_m=0.1,
    ) == (False, None)


def test_wsj_launcher_bridges_one_source_keyframe_plus_one_grid_cell():
    source = (
        OVERLAY / "start_tinynav_buildmap_online_nav.sh"
    ).read_text(encoding="utf-8")

    assert 'FOCUS_MAX_CACHED_MAP_MOTION_M:-0.25' in source
    assert '--max-cached-map-motion-m \\"$MAX_CACHED_MAP_MOTION_M\\"' in source


def test_a_star_can_use_bounded_known_free_start_seed():
    router = load_router()
    occupancy = grid([0] * 100, width=10, height=10)

    assert router.plan_route(
        occupancy,
        start_x=0.1,
        start_y=0.1,
        goal_x=8.5,
        goal_y=8.5,
        arrival_radius_m=0.1,
        clearance_cells=1,
        start_snap_radius_m=1.5,
    ) is None
    plan = router.plan_route(
        occupancy,
        start_x=0.1,
        start_y=0.1,
        goal_x=8.5,
        goal_y=8.5,
        arrival_radius_m=0.1,
        clearance_cells=1,
        start_snap_radius_m=2.1,
    )

    assert plan is not None
    assert plan.cells[0] == (1, 1)
    assert plan.start_snap_distance_m == pytest.approx(
        1.4 * 2 ** 0.5
    )


def test_a_star_can_leave_a_self_occupied_measured_base_footprint():
    router = load_router()
    data = [0] * 121
    for row in range(3, 6):
        for column in range(3, 6):
            data[row * 11 + column] = 100
    occupancy = grid(data, width=11, height=11)

    assert router.plan_route(
        occupancy,
        start_x=4.5,
        start_y=4.5,
        goal_x=9.5,
        goal_y=9.5,
        arrival_radius_m=0.1,
        clearance_cells=1,
        start_snap_radius_m=4.0,
    ) is None
    plan = router.plan_route(
        occupancy,
        start_x=4.5,
        start_y=4.5,
        goal_x=9.5,
        goal_y=9.5,
        arrival_radius_m=0.1,
        clearance_cells=1,
        start_snap_radius_m=4.0,
        start_footprint_override_m=1.1,
    )

    assert plan is not None
    assert occupancy.free_with_clearance(
        *plan.cells[0], clearance_cells=1
    )
    assert plan.start_snap_distance_m == pytest.approx(3.0)


def test_arrival_disk_can_end_before_an_unknown_target_cell():
    router = load_router()
    data = [0] * 7
    data[5] = -1
    occupancy = grid(data, width=7, height=1)

    plan = router.plan_route(
        occupancy,
        start_x=0.5,
        start_y=0.5,
        goal_x=5.5,
        goal_y=0.5,
        arrival_radius_m=1.1,
        clearance_cells=0,
    )

    assert plan is not None
    assert plan.target_cell == (0, 4)
    assert (0, 5) not in plan.cells


def test_lookahead_is_bounded_to_the_route():
    router = load_router()
    occupancy = grid([0] * 6, width=6, height=1)
    plan = router.RoutePlan(
        cells=((0, 0), (0, 1), (0, 2), (0, 3)),
        target_cell=(0, 3),
        length_m=3.0,
    )

    assert router.select_lookahead(
        occupancy, plan, lookahead_m=1.5
    ) == pytest.approx((2.5, 0.5))
    assert router.select_lookahead(
        occupancy, plan, lookahead_m=20.0
    ) == pytest.approx((3.5, 0.5))


def test_router_has_no_robot_sdk_or_velocity_output():
    source = (OVERLAY / "tinynav_buildmap_goal_router.py").read_text(
        encoding="utf-8"
    )
    assert "unitree" not in source.lower()
    assert "WaterTcpClient" not in source
    assert "Twist" not in source
    assert "/cmd_vel" not in source


def test_router_parameterizes_robot_and_camera_identity():
    source = (OVERLAY / "tinynav_buildmap_goal_router.py").read_text(
        encoding="utf-8"
    )

    assert 'parser.add_argument(\n        "--robot-id"' in source
    assert 'parser.add_argument(\n        "--base-camera-frame"' in source
    assert "expected_robot_id=args.robot_id" in source
    assert "expected_camera_frame=args.base_camera_frame" in source
    assert (
        "message.child_frame_id = base_camera_calibration.camera_frame"
        in source
    )


def test_router_keeps_sensor_callbacks_responsive_during_replanning():
    source = (OVERLAY / "tinynav_buildmap_goal_router.py").read_text(
        encoding="utf-8"
    )

    assert "MultiThreadedExecutor(num_threads=3)" in source
    assert "self.odom_callback_group = MutuallyExclusiveCallbackGroup()" in source
    assert (
        "self.occupancy_callback_group = MutuallyExclusiveCallbackGroup()"
        in source
    )
    assert "self.control_callback_group = MutuallyExclusiveCallbackGroup()" in source
    assert "callback_group=self.odom_callback_group" in source
    assert "callback_group=self.occupancy_callback_group" in source
    assert "with self.sensor_lock:" in source
