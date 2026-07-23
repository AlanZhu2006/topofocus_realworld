from __future__ import annotations

import pytest

from focus_hub.transport_v2 import NavigationStatusV2
from focus_hub.v2_robot_runtime import (
    HubV2RobotClient,
    OccupancyGrid2D,
    PathAccumulator,
    navigation_event,
    parse_water_current_pose,
    require_water_ok,
    water_move_state,
    water_robot_health,
)
from test_v2_goal_adapter import make_target_decision


def test_water_pose_parser_accepts_observed_api_shapes():
    assert parse_water_current_pose(
        {"current_pose": {"x": 1, "y": 2, "theta": 0.5}}
    ) == (1.0, 2.0, 0.5)
    assert parse_water_current_pose({"current_pose": [1, 2, 0.5]}) == (
        1.0, 2.0, 0.5
    )
    assert parse_water_current_pose({"current_pose": "1,2,0.5"}) == (
        1.0, 2.0, 0.5
    )


def test_water_response_and_state_are_fail_closed():
    require_water_ok(
        {"type": "response", "status": "OK", "error_message": ""},
        command="/api/move",
    )
    with pytest.raises(RuntimeError, match="rejected"):
        require_water_ok(
            {"status": "ERROR", "error_message": "bad goal"},
            command="/api/move",
        )
    assert water_move_state({"move_status": "running"}) == "ACTIVE"
    assert water_move_state({"move_status": "succeeded"}) == "ARRIVED"
    assert water_move_state({"move_status": "vendor-new-state"}) == "UNKNOWN"


def test_water_health_requires_fresh_local_odometry():
    ready = water_robot_health(
        {"error_code": "00000000", "move_status": "idle", "power_percent": 80},
        odometry_fresh=True,
    )
    stale = water_robot_health(
        {"error_code": "00000000", "move_status": "idle", "power_percent": 80},
        odometry_fresh=False,
    )
    assert ready.ready_for_goal()
    assert not stale.ready_for_goal()


def test_receiver_heartbeat_uses_local_health(monkeypatch):
    health = water_robot_health(
        {"error_code": "00000000", "move_status": "idle"},
        odometry_fresh=True,
    )
    client = object.__new__(HubV2RobotClient)
    client.robot_id = "robot-1"
    captured = {}

    def request(method, path, *, body=None):
        captured.update(method=method, path=path, body=body)
        return 200, (
            b'{"robot_id":"robot-1","received_at_ns":10,"status":"accepted"}'
        )

    monkeypatch.setattr(client, "_request", request)

    ack = client.post_heartbeat(health)

    assert ack.status == "accepted"
    assert captured["path"] == "/v1/robots/robot-1/heartbeat"
    assert captured["body"]["health"]["safety_state"] == "READY"


def test_path_accumulator_ignores_localization_jump():
    path = PathAccumulator(max_step_m=1.0)
    assert path.update(0, 0) == 0
    assert path.update(0.3, 0.4) == pytest.approx(0.5)
    assert path.update(10, 10) == pytest.approx(0.5)
    assert path.rejected_jumps == 1


def test_occupancy_reachability_keeps_unknown_and_obstacle_blocked():
    # A vertical obstacle wall separates the left and right halves. The one
    # unknown cell on the left is blocked as well.
    data = [0] * 35
    for row in range(5):
        data[row * 7 + 3] = 100
    data[1 * 7 + 1] = -1
    grid = OccupancyGrid2D(
        width=7,
        height=5,
        resolution_m=1.0,
        origin_x_m=0.0,
        origin_y_m=0.0,
        data=tuple(data),
    )

    component = grid.reachable_component(0.5, 2.5, clearance_cells=0)

    assert grid.point_in_component(2.5, 2.5, component)
    assert not grid.point_in_component(4.5, 2.5, component)
    assert not grid.point_in_component(1.5, 1.5, component)


def test_occupancy_clearance_blocks_cells_next_to_wall():
    data = [0] * 49
    data[3 * 7 + 3] = 100
    grid = OccupancyGrid2D(7, 7, 1.0, 0.0, 0.0, tuple(data))

    component = grid.reachable_component(1.5, 1.5, clearance_cells=1)

    assert grid.point_in_component(1.5, 1.5, component)
    assert not grid.point_in_component(2.5, 3.5, component)


def test_occupancy_can_bound_start_seed_without_crossing_unknown():
    data = [0] * 49
    grid = OccupancyGrid2D(7, 7, 1.0, 0.0, 0.0, tuple(data))

    assert not grid.reachable_component(
        0.1, 0.1, clearance_cells=1
    )
    component = grid.reachable_component(
        0.1,
        0.1,
        clearance_cells=1,
        start_snap_radius_m=2.1,
    )
    assert grid.point_in_component(1.5, 1.5, component)

    blocked = list(data)
    blocked[0] = -1
    blocked_grid = OccupancyGrid2D(
        7, 7, 1.0, 0.0, 0.0, tuple(blocked)
    )
    assert not blocked_grid.reachable_component(
        0.1,
        0.1,
        clearance_cells=1,
        start_snap_radius_m=2.1,
    )


def test_occupancy_can_escape_only_the_measured_blocked_start_footprint():
    data = [0] * 121
    for row in range(3, 6):
        for column in range(3, 6):
            data[row * 11 + column] = 100
    grid = OccupancyGrid2D(11, 11, 1.0, 0.0, 0.0, tuple(data))

    assert not grid.reachable_component(
        4.5,
        4.5,
        clearance_cells=1,
        start_snap_radius_m=4.0,
    )
    component = grid.reachable_component(
        4.5,
        4.5,
        clearance_cells=1,
        start_snap_radius_m=4.0,
        start_footprint_override_m=1.1,
    )

    assert component
    assert all(grid.free_with_clearance(*cell) for cell in component)
    assert not grid.point_in_component(4.5, 4.5, component)
    assert grid.point_in_component(4.5, 1.5, component)


def test_occupancy_arrival_disk_can_touch_reachable_free_space():
    data = [0] * 7
    data[5] = -1
    grid = OccupancyGrid2D(7, 1, 1.0, 0.0, 0.0, tuple(data))
    component = grid.reachable_component(0.5, 0.5, clearance_cells=0)

    assert grid.cell_center(0, 4) == (4.5, 0.5)
    assert not grid.point_in_component(5.5, 0.5, component)
    assert grid.component_within_radius(5.5, 0.5, 1.1, component)
    assert not grid.component_within_radius(5.5, 0.5, 0.9, component)
    with pytest.raises(ValueError, match="radius_m"):
        grid.component_within_radius(5.5, 0.5, -0.1, component)


def test_terminal_event_uses_own_frozen_observation():
    decision = make_target_decision(robot_id="robot-1")
    event = navigation_event(
        decision,
        status=NavigationStatusV2.ARRIVED,
        reason_code="LOCAL_PLANNER_ARRIVED",
        local_pose=(1, 2, 0.5),
        path_length_m=3.0,
        velocity_zero_confirmed=True,
        terminal=True,
    )
    assert event.terminal_observation_sequence == 2
    assert event.path_length_m_from_episode_start == 3.0
