from __future__ import annotations

import importlib.util
import json
import math
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


def test_target_refresh_request_is_versioned_and_decision_bound():
    router = load_router()
    payload = json.dumps(
        {
            "schema_version": (
                "focus-tinynav-target-refresh-request-v1"
            ),
            "decision_id": "decision-1",
            "requested_at_ns": 1_000_000_000,
            "path_age_s": 1.2,
        }
    )

    assert router.parse_target_refresh_request(payload) == (
        "decision-1",
        1_000_000_000,
    )
    invalid = json.loads(payload)
    invalid["schema_version"] = "unknown"
    with pytest.raises(ValueError, match="schema"):
        router.parse_target_refresh_request(json.dumps(invalid))


def test_new_leg_and_bounded_repair_reuse_process_stable_target_publisher():
    source = (OVERLAY / "tinynav_buildmap_goal_router.py").read_text(
        encoding="utf-8"
    )
    new_leg = source.split(
        'self.clear_target("GOAL_REPLACED", discard_goal=True)', 1
    )[1].split(
        'self.publish_status("ACCEPTED", "FRESH_VERSIONED_GOAL")', 1
    )[0]
    assert "self.recreate_target_publisher()" not in new_leg
    assert "self.destroy_publisher(" not in new_leg
    assert "self.create_publisher(" not in new_leg
    assert "self.goal = goal" in new_leg
    repair = source.split(
        "def on_target_refresh_request(self, message: String)", 1
    )[1].split("def on_occupancy(", 1)[0]
    assert "self.goal.decision_id != decision_id" in repair
    assert "not self.target_active" in repair
    assert "odom_age_s > args.input_timeout_s" in repair
    assert "self.recreate_target_publisher()" not in repair
    assert "self.destroy_publisher(" not in repair
    assert "self.create_publisher(" not in repair
    assert "self.publish_target(waypoint[0], waypoint[1], odom)" in repair
    assert "stable_generation=" in repair


def test_target_publisher_is_created_once_for_process_lifetime():
    source = (OVERLAY / "tinynav_buildmap_goal_router.py").read_text(
        encoding="utf-8"
    )
    assert source.count("self.target_publisher = self.create_publisher(") == 1
    assert "def recreate_target_publisher(" not in source
    assert "self.destroy_publisher(previous_publisher)" not in source
    assert "self.target_publisher_generation = 1" in source
    assert '"target_publisher_lifecycle": "process_stable"' in source


def test_semantic_planner_targets_inside_unchanged_arrival_radius():
    router = load_router()

    assert router.planning_arrival_radius_m(
        target_kind="SEMANTIC_REGION",
        arrival_radius_m=0.50,
        semantic_terminal_margin_m=0.15,
        grid_resolution_m=0.05,
    ) == pytest.approx(0.35)
    assert router.planning_arrival_radius_m(
        target_kind="FRONTIER_POINT",
        arrival_radius_m=0.50,
        semantic_terminal_margin_m=0.15,
        grid_resolution_m=0.05,
    ) == pytest.approx(0.50)
    assert router.planning_arrival_radius_m(
        target_kind="SEMANTIC_REGION",
        arrival_radius_m=0.10,
        semantic_terminal_margin_m=0.15,
        grid_resolution_m=0.05,
    ) == pytest.approx(0.05)


def test_semantic_inner_disk_gives_arrival_boundary_crossing_margin():
    router = load_router()
    occupancy = grid(
        [0] * 41,
        width=41,
        height=1,
        resolution_m=0.05,
    )
    arrival_radius_m = 0.50
    planning_radius_m = router.planning_arrival_radius_m(
        target_kind="SEMANTIC_REGION",
        arrival_radius_m=arrival_radius_m,
        semantic_terminal_margin_m=0.15,
        grid_resolution_m=occupancy.resolution_m,
    )

    plan = router.plan_route(
        occupancy,
        start_x=0.025,
        start_y=0.025,
        goal_x=1.525,
        goal_y=0.025,
        arrival_radius_m=planning_radius_m,
        clearance_cells=0,
    )

    assert plan is not None
    target_x, target_y = occupancy.cell_center(*plan.target_cell)
    target_distance_m = (
        (target_x - 1.525) ** 2 + (target_y - 0.025) ** 2
    ) ** 0.5
    assert target_distance_m <= planning_radius_m + 1e-12
    assert arrival_radius_m - target_distance_m >= 0.149


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


def test_decision_id_is_immutable_and_only_exact_replay_is_accepted():
    router = load_router()
    current = router.parse_goal_payload(goal_payload(), now_ns=1_000_000_000)

    assert router.is_exact_decision_replay(current, current)

    mutated_payload = json.loads(goal_payload())
    mutated_payload["0"]["position"][0] += 0.01
    mutated = router.parse_goal_payload(
        json.dumps(mutated_payload), now_ns=1_000_000_000
    )
    assert not router.is_exact_decision_replay(current, mutated)

    source = (OVERLAY / "tinynav_buildmap_goal_router.py").read_text(
        encoding="utf-8"
    )
    assert '"DECISION_ID_MUTATED"' in source
    assert '"EXACT_DECISION_REPLAY"' in source


def test_precomputed_clearance_mask_matches_original_cell_predicate():
    router = load_router()
    data = [0] * (8 * 7)
    data[2 * 8 + 3] = 100
    data[5 * 8 + 6] = -1
    occupancy = grid(data, width=8, height=7)

    for clearance_cells in (0, 1, 2):
        traversable = router.clearance_traversability(
            occupancy,
            clearance_cells=clearance_cells,
        )
        assert traversable.shape == (occupancy.height, occupancy.width)
        for row in range(occupancy.height):
            for column in range(occupancy.width):
                assert bool(traversable[row, column]) is (
                    occupancy.free_with_clearance(
                        row,
                        column,
                        clearance_cells=clearance_cells,
                    )
                )


def test_terminal_obstacle_clearance_does_not_treat_frontier_unknown_as_wall():
    router = load_router()
    data = [0] * (7 * 7)
    data[3 * 7 + 4] = -1
    occupancy = grid(data, width=7, height=7)

    route_clearance = router.clearance_traversability(
        occupancy,
        clearance_cells=1,
    )
    terminal_clearance = router.obstacle_clearance_traversability(
        occupancy,
        clearance_cells=1,
    )

    assert not route_clearance[3, 3]
    assert terminal_clearance[3, 3]
    assert not terminal_clearance[3, 4]
    assert router.point_has_obstacle_clearance(
        occupancy,
        x_m=3.5,
        y_m=3.5,
        clearance_cells=1,
    )

    data[3 * 7 + 2] = 100
    occupancy_with_wall = grid(data, width=7, height=7)
    assert not router.point_has_obstacle_clearance(
        occupancy_with_wall,
        x_m=3.5,
        y_m=3.5,
        clearance_cells=1,
    )


def test_a_star_prefers_robot1_clearance_but_retains_narrow_connectivity():
    router = load_router()
    data = [0] * (9 * 7)
    for row in range(7):
        data[row * 9 + 4] = 100
    data[3 * 9 + 4] = 0
    occupancy = grid(data, width=9, height=7)

    plan = router.plan_route(
        occupancy,
        start_x=1.5,
        start_y=3.5,
        goal_x=7.5,
        goal_y=3.5,
        arrival_radius_m=0.1,
        clearance_cells=0,
        preferred_clearance_cells=1,
    )

    assert plan is not None
    assert (3, 4) in plan.cells
    assert plan.low_preferred_clearance_length_m > 0.0


def test_a_star_chooses_longer_centered_route_when_preferred_route_exists():
    router = load_router()
    data = [0] * (15 * 15)
    for column in range(6, 9):
        data[7 * 15 + column] = 100
    occupancy = grid(data, width=15, height=15)

    plan = router.plan_route(
        occupancy,
        start_x=2.5,
        start_y=7.5,
        goal_x=12.5,
        goal_y=7.5,
        arrival_radius_m=0.1,
        clearance_cells=0,
        preferred_clearance_cells=2,
    )

    assert plan is not None
    assert plan.low_preferred_clearance_length_m == pytest.approx(0.0)
    preferred = router.clearance_traversability(
        occupancy,
        clearance_cells=2,
    )
    assert all(preferred[row, column] for row, column in plan.cells)


def test_terminal_obstacle_clearance_stops_before_observed_wall():
    router = load_router()
    data = [0] * (9 * 7)
    for row in range(7):
        data[row * 9 + 6] = 100
    occupancy = grid(data, width=9, height=7)

    plan = router.plan_route(
        occupancy,
        start_x=1.5,
        start_y=3.5,
        goal_x=5.5,
        goal_y=3.5,
        arrival_radius_m=0.1,
        clearance_cells=0,
        preferred_clearance_cells=1,
        terminal_obstacle_clearance_cells=2,
        allow_partial_progress=True,
        minimum_progress_m=0.1,
    )

    assert plan is not None
    assert plan.reaches_arrival_region is False
    assert plan.target_cell == (3, 3)
    assert occupancy.cell_center(*plan.target_cell)[0] == pytest.approx(3.5)


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
    assert plan.partial_termination_reason == "known_free_map_edge"
    assert plan.expanded_cells == 4
    assert all(
        occupancy.data[row * occupancy.width + column] == 0
        for row, column in plan.cells
    )


def test_a_star_search_budget_returns_progress_or_fails_closed():
    router = load_router()
    occupancy = grid([0] * 100, width=10, height=10)

    with pytest.raises(router.PlanningBudgetExceeded) as caught:
        router.plan_route(
            occupancy,
            start_x=0.5,
            start_y=5.5,
            goal_x=20.5,
            goal_y=5.5,
            arrival_radius_m=0.1,
            clearance_cells=0,
            max_expansions=2,
        )
    assert caught.value.expanded_cells == 2
    assert caught.value.limit_reason == "expansion_budget"

    partial = router.plan_route(
        occupancy,
        start_x=0.5,
        start_y=5.5,
        goal_x=20.5,
        goal_y=5.5,
        arrival_radius_m=0.1,
        clearance_cells=0,
        allow_partial_progress=True,
        minimum_progress_m=0.1,
        max_expansions=2,
    )
    assert partial is not None
    assert partial.target_cell == (5, 1)
    assert partial.partial_termination_reason == "search_expansion_budget"
    assert partial.expanded_cells == 2

    with pytest.raises(ValueError, match="positive integer"):
        router.plan_route(
            occupancy,
            start_x=0.5,
            start_y=5.5,
            goal_x=20.5,
            goal_y=5.5,
            arrival_radius_m=0.1,
            clearance_cells=0,
            max_expansions=1.5,
        )
    with pytest.raises(ValueError, match="finite and positive"):
        router.plan_route(
            occupancy,
            start_x=0.5,
            start_y=5.5,
            goal_x=20.5,
            goal_y=5.5,
            arrival_radius_m=0.1,
            clearance_cells=0,
            max_planning_duration_s=0.0,
        )


def test_a_star_has_a_hardware_independent_wall_clock_bound(monkeypatch):
    router = load_router()
    occupancy = grid([0] * 100, width=10, height=10)
    timestamps = iter((10.0, 10.6))
    monkeypatch.setattr(router.time, "monotonic", lambda: next(timestamps))

    with pytest.raises(router.PlanningBudgetExceeded) as caught:
        router.plan_route(
            occupancy,
            start_x=0.5,
            start_y=5.5,
            goal_x=20.5,
            goal_y=5.5,
            arrival_radius_m=0.1,
            clearance_cells=0,
            max_expansions=1_000,
            max_planning_duration_s=0.5,
        )

    assert caught.value.expanded_cells == 32
    assert caught.value.limit_reason == "wall_clock_budget"


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


def test_wsj_launcher_uses_continuous_geometry_plus_one_grid_cell():
    source = (
        OVERLAY / "start_tinynav_buildmap_online_nav.sh"
    ).read_text(encoding="utf-8")
    online_mapping = (
        OVERLAY / "run_tinynav_buildmap_online_mapping.py"
    ).read_text(encoding="utf-8")

    assert 'FOCUS_MAX_CACHED_MAP_MOTION_M:-0.25' in source
    assert '--max-cached-map-motion-m \\"$MAX_CACHED_MAP_MOTION_M\\"' in source
    assert 'FOCUS_WSJ_MAP_TIMEOUT_S:-12.0' in source
    assert '--map-timeout-s \\"$MAP_TIMEOUT_S\\"' in source
    assert 'FOCUS_WSJ_ODOMETRY_INPUT_TIMEOUT_S:-3.0' in source
    assert '--input-timeout-s \\"$ODOMETRY_INPUT_TIMEOUT_S\\"' in source
    assert 'FOCUS_WSJ_REACHABILITY_CLEARANCE_M:-0.05' in source
    assert 'FOCUS_WSJ_START_SNAP_RADIUS_M:-0.75' in source
    assert 'FOCUS_WSJ_START_FOOTPRINT_OVERRIDE_M:-0.35' in source
    assert (
        "FOCUS_WSJ_SEMANTIC_TERMINAL_PLANNING_MARGIN_M:-0.15"
        in source
    )
    assert '--start-snap-radius-m \\"$START_SNAP_RADIUS_M\\"' in source
    assert '--clearance-m \\"$REACHABILITY_CLEARANCE_M\\"' in source
    assert (
        '--start-footprint-override-m '
        '\\"$START_FOOTPRINT_OVERRIDE_M\\"'
    ) in source
    assert (
        '--semantic-terminal-planning-margin-m '
        '\\"$SEMANTIC_TERMINAL_PLANNING_MARGIN_M\\"'
    ) in source
    assert "FOCUS_TINYNAV_MAX_PLAN_EXPANSIONS:-20000" in source
    assert "FOCUS_TINYNAV_MAX_PLAN_DURATION_S:-0.50" in source
    assert (
        '--max-plan-expansions \\"$MAX_PLAN_EXPANSIONS\\"'
        in source
    )
    assert (
        '--max-plan-duration-s \\"$MAX_PLAN_DURATION_S\\"'
        in source
    )
    assert '"/slam/depth",' in online_mapping
    assert '"ros_continuous_depth_geometry_rgb.py"' in online_mapping
    assert '"--approved-size",' in online_mapping
    assert '"848x480",' in online_mapping
    assert '"640x480",' in online_mapping
    assert '"/slam/keyframe_depth"' not in online_mapping
    assert '"keyframe.pose_jump_translation_m": 1.0' in online_mapping
    assert '"keyframe.pose_jump_rotation_deg": 90.0' in online_mapping
    assert '"keyframe.pause_frames_after_jump": 0' in online_mapping
    assert '"navigation_occupancy_mapper.py"' in online_mapping


def test_wsj_launcher_uses_short_segment_velocity_floor_wrapper():
    source = (
        OVERLAY / "start_tinynav_buildmap_online_nav.sh"
    ).read_text(encoding="utf-8")

    assert '"$SCRIPT_DIR/yunji_tinynav_cmd_vel_control.py"' in source
    assert (
        'uv run python \\"$SCRIPT_DIR/'
        'yunji_tinynav_cmd_vel_control.py\\"'
    ) in source
    assert (
        "uv run python /tinynav/tinynav/platforms/cmd_vel_control.py"
        not in source
    )


def test_router_default_map_deadline_matches_verified_data_plane():
    source = (OVERLAY / "tinynav_buildmap_goal_router.py").read_text(
        encoding="utf-8"
    )

    assert (
        'parser.add_argument("--map-timeout-s", type=float, default=12.0)'
        in source
    )
    assert 'default=20_000' in source


def test_both_robot_launchers_share_the_same_planning_work_bound():
    for name in (
        "start_tinynav_buildmap_online_nav.sh",
        "start_wsj_buildmap_v2.sh",
        "start_yunji_v2.sh",
    ):
        source = (OVERLAY / name).read_text(encoding="utf-8")
        assert "FOCUS_TINYNAV_MAX_PLAN_EXPANSIONS:-20000" in source
        assert "FOCUS_TINYNAV_MAX_PLAN_DURATION_S:-0.50" in source
        assert "--max-plan-expansions" in source
        assert "--max-plan-duration-s" in source


def test_router_paths_carry_current_measured_orientation():
    source = (OVERLAY / "tinynav_buildmap_goal_router.py").read_text(
        encoding="utf-8"
    )

    assert "pose.pose.orientation = odom.pose.pose.orientation" in source
    assert "self.publish_route(plan, grid, odom)" in source
    assert "pose.pose.orientation.w = 1.0" not in source


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
    assert plan.cells[0] == (0, 0)
    assert plan.cells[2] == (1, 1)
    assert all(
        max(abs(first[0] - second[0]), abs(first[1] - second[1])) == 1
        for first, second in zip(plan.cells, plan.cells[1:])
    )
    assert plan.start_snap_distance_m == pytest.approx(
        1.4 * 2 ** 0.5
    )


def test_a_star_can_leave_a_cropped_unknown_measured_base_footprint():
    router = load_router()
    data = [0] * 121
    for row in range(3, 6):
        for column in range(3, 6):
            data[row * 11 + column] = -1
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
    assert plan.cells[0] == (4, 4)
    safe_cells = [
        cell
        for cell in plan.cells
        if occupancy.free_with_clearance(*cell, clearance_cells=1)
    ]
    assert safe_cells
    assert plan.cells.index(safe_cells[0]) > 0
    assert all(
        max(abs(first[0] - second[0]), abs(first[1] - second[1])) == 1
        for first, second in zip(plan.cells, plan.cells[1:])
    )
    assert plan.start_snap_distance_m == pytest.approx(3.0)


def test_a_star_never_overrides_observed_obstacle_in_current_footprint():
    router = load_router()
    data = [0] * 121
    data[4 * 11 + 4] = 100
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
        start_footprint_override_m=1.1,
    ) is None


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


def test_start_snap_route_does_not_teleport_to_remote_clearance_seed():
    router = load_router()
    data = [0] * (9 * 5)
    data[2 * 9 + 1] = -1
    occupancy = grid(data, width=9, height=5)

    plan = router.plan_route(
        occupancy,
        start_x=1.5,
        start_y=2.5,
        goal_x=7.5,
        goal_y=2.5,
        arrival_radius_m=0.1,
        clearance_cells=1,
        start_snap_radius_m=3.0,
        start_footprint_override_m=1.1,
    )

    assert plan is not None
    assert plan.cells[0] == (2, 1)
    assert all(
        max(abs(first[0] - second[0]), abs(first[1] - second[1])) == 1
        for first, second in zip(plan.cells, plan.cells[1:])
    )


def test_start_snap_prefers_forward_seed_for_forward_only_controller():
    router = load_router()
    data = [0] * (11 * 5)
    data[2 * 11 + 4] = -1
    occupancy = grid(data, width=11, height=5)

    plan = router.plan_route(
        occupancy,
        start_x=4.4,
        start_y=2.5,
        goal_x=9.5,
        goal_y=2.5,
        arrival_radius_m=0.1,
        clearance_cells=1,
        start_snap_radius_m=3.0,
        start_footprint_override_m=1.1,
        preferred_seed_heading_rad=0.0,
    )

    assert plan is not None
    assert plan.cells[:3] == ((2, 4), (2, 5), (2, 6))
    lookahead = router.select_lookahead(
        occupancy, plan, lookahead_m=1.0
    )
    assert lookahead[0] > 4.4


def test_start_snap_is_anchored_to_goal_when_goal_is_behind_robot():
    router = load_router()
    data = [0] * (15 * 7)
    data[3 * 15 + 7] = -1
    occupancy = grid(data, width=15, height=7)

    # The robot may currently face +X, but a fixed goal at -X must keep the
    # clearance seed on the goal side while the controller turns.  Reusing the
    # changing robot yaw here caused the physical seed to switch sides and the
    # control segment to stay at +/-180 degrees indefinitely.
    goal_heading = math.pi
    plan = router.plan_route(
        occupancy,
        start_x=7.4,
        start_y=3.5,
        goal_x=2.5,
        goal_y=3.5,
        arrival_radius_m=0.1,
        clearance_cells=1,
        start_snap_radius_m=4.0,
        start_footprint_override_m=1.1,
        preferred_seed_heading_rad=goal_heading,
    )

    assert plan is not None
    assert plan.cells[:3] == ((3, 7), (3, 6), (3, 5))
    first_x, _ = occupancy.cell_center(*plan.cells[0])
    second_x, _ = occupancy.cell_center(*plan.cells[1])
    assert second_x < first_x


def test_wsj_launcher_uses_forward_only_planner_and_bounded_rotate_first():
    initial = (
        OVERLAY / "start_tinynav_buildmap_online_nav.sh"
    ).read_text(encoding="utf-8")
    reload = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text(
        encoding="utf-8"
    )

    for source in (initial, reload):
        assert "run_yunji_tinynav_planner.py" in source
        assert "--robot-profile source-default" in source
        assert "--robot-id robot-0" in source
        assert "--base-camera-frame camera" in source
        assert (
            '--base-camera-calibration-file \\"$BASE_CAMERA_CALIBRATION_FILE\\"'
            in source
        )
        assert "--rotate-first-on-reverse" in source
        assert "--stabilize-large-turn" in source
        assert "--verified-forward-only-planner" in source
        assert "--rotate-first-max-angular-radps 0.35" in source
        assert "--rotate-first-timeout-s 12.0" in source
    assert "--reject-reverse-trajectory" in reload


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
