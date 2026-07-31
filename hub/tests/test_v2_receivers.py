from __future__ import annotations

import ast
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest


OVERLAY = Path(__file__).resolve().parents[1] / "robot_overlay"


def load_overlay(name: str):
    path = OVERLAY / name
    spec = importlib.util.spec_from_file_location(f"test_{path.stem}", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def pose(*, x=1.0, y=2.0, z=0.0, yaw=0.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2),
            w=math.cos(yaw / 2),
        ),
    )


def valid_slam_payload(
    *,
    coverage_ratio: float = 1.0,
    max_sample_gap_s: float = 0.005,
    end_error_s: float = 0.0,
    overwritten: int = 0,
) -> str:
    import json

    return json.dumps({
        "stats": {
            "optimizer_status": "ok",
            "imu_messages_overwritten": overwritten,
        },
        "metrics": {
            "initial_error": 1.0,
            "final_error": 0.5,
            "num_factors": 4,
            "num_variables": 6,
            "imu_intervals_valid": True,
            "imu_intervals": [{
                "duration_s": 0.5,
                "sample_count": 100,
                "expected_count": 100,
                "coverage_ratio": coverage_ratio,
                "max_sample_gap_s": max_sample_gap_s,
                "end_error_s": end_error_s,
                "valid": True,
            }],
        },
    })


def test_data_plane_verifier_waits_for_resolved_endpoint_identity():
    verifier = load_overlay("verify_tinynav_data_plane.py")
    endpoints = [
        SimpleNamespace(
            node_namespace="_NODE_NAMESPACE_UNKNOWN_",
            node_name="_NODE_NAME_UNKNOWN_",
        ),
        SimpleNamespace(
            node_namespace="/",
            node_name="cmd_vel_control_node",
        ),
    ]

    assert verifier.endpoint_names(endpoints) == ["/cmd_vel_control_node"]


def test_data_plane_verifier_uses_source_clock_for_map_freshness():
    verifier = load_overlay("verify_tinynav_data_plane.py")
    header = SimpleNamespace(
        frame_id="camera",
        stamp=SimpleNamespace(sec=10, nanosec=0),
    )
    image = SimpleNamespace(width=640, height=480, header=header)
    camera_info = SimpleNamespace(
        width=640,
        height=480,
        header=header,
        k=[400.0, 0.0, 319.5, 0.0, 400.0, 239.5, 0.0, 0.0, 1.0],
    )

    geometry = verifier.validate_geometry_contract(
        image, camera_info, expected_frame="camera"
    )
    assert geometry["width"] == 640
    fresh_image = SimpleNamespace(
        header=SimpleNamespace(
            frame_id="camera",
            # A boot-relative clock is intentional: this must never be
            # compared with Unix wall time.
            stamp=SimpleNamespace(sec=12, nanosec=0),
        )
    )
    assert verifier.message_lag_s(
        image, reference_message=fresh_image
    ) == pytest.approx(
        2.0
    )

    camera_info.width = 848
    with pytest.raises(ValueError, match="dimensions differ"):
        verifier.validate_geometry_contract(
            image, camera_info, expected_frame="camera"
        )


def test_data_plane_verifier_rejects_mixed_source_clock_epochs():
    verifier = load_overlay("verify_tinynav_data_plane.py")
    occupancy = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0))
    )
    restarted_depth = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=0))
    )

    with pytest.raises(ValueError, match="ahead of the fresh sensor"):
        verifier.message_lag_s(
            occupancy, reference_message=restarted_depth
        )


def test_data_plane_camera_info_is_a_lightweight_geometry_witness():
    verifier = load_overlay("verify_tinynav_data_plane.py")
    camera_info = SimpleNamespace(
        width=848,
        height=480,
        header=SimpleNamespace(
            frame_id="camera",
            stamp=SimpleNamespace(sec=12, nanosec=0),
        ),
        k=[430.0, 0.0, 423.5, 0.0, 430.0, 239.5, 0.0, 0.0, 1.0],
    )

    profile = verifier.validate_camera_info_contract(
        camera_info,
        expected_frame="camera",
        expected_width=848,
        expected_height=480,
    )
    assert profile["frame_id"] == "camera"
    assert profile["width"] == 848

    camera_info.width = 640
    with pytest.raises(ValueError, match="locked profile"):
        verifier.validate_camera_info_contract(
            camera_info,
            expected_frame="camera",
            expected_width=848,
            expected_height=480,
        )


def test_data_plane_cached_occupancy_requires_stationary_no_goal_hold():
    verifier = load_overlay("verify_tinynav_data_plane.py")
    hold = {"state": "HOLD", "reason": "NO_GOAL", "decision_id": None}

    valid, motion_m = verifier.cached_occupancy_start_is_valid(
        occupancy_age_s=49.77,
        maximum_age_s=12.0,
        anchor_xy=(1.0, 2.0),
        current_xy=(1.079, 2.0),
        maximum_motion_m=0.25,
        router_status=hold,
    )
    assert valid is True
    assert motion_m == pytest.approx(0.079)

    for current_xy, router in (
        ((1.251, 2.0), hold),
        (
            (1.079, 2.0),
            {
                "state": "NAVIGATING",
                "reason": "ONLINE_PATH_READY",
                "decision_id": "active",
            },
        ),
    ):
        valid, _ = verifier.cached_occupancy_start_is_valid(
            occupancy_age_s=49.77,
            maximum_age_s=12.0,
            anchor_xy=(1.0, 2.0),
            current_xy=current_xy,
            maximum_motion_m=0.25,
            router_status=router,
        )
        assert valid is False


def test_data_plane_rejects_nonempty_map_when_base_has_no_free_component():
    verifier = load_overlay("verify_tinynav_data_plane.py")
    width = 5
    height = 5
    values = [-1] * (width * height)
    for row in range(height):
        values[row * width + 3] = 0
    occupancy = SimpleNamespace(
        header=SimpleNamespace(frame_id="world"),
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=0.1,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0)
            ),
        ),
        data=values,
    )
    odometry = SimpleNamespace(
        pose=SimpleNamespace(pose=pose(x=0.7, y=0.25))
    )
    identity = (
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    )

    with pytest.raises(ValueError, match="no start-connected"):
        verifier.validate_start_reachability(
            occupancy,
            odometry,
            frame_id="world",
            base_T_camera=identity,
            clearance_m=0.0,
            start_snap_radius_m=0.5,
            start_footprint_override_m=0.19,
        )

    report = verifier.validate_start_reachability(
        occupancy,
        odometry,
        frame_id="world",
        base_T_camera=identity,
        clearance_m=0.0,
        start_snap_radius_m=0.5,
        start_footprint_override_m=0.21,
    )
    assert report["base_inside_grid"] is False
    assert report["reachable_component_cells"] == height


def test_data_plane_collects_messages_before_expensive_graph_queries():
    source = (OVERLAY / "verify_tinynav_data_plane.py").read_text(
        encoding="utf-8"
    )
    loop = source.split(
        "# Full sensor/map validation loop.", 1
    )[1].split("missing = sorted(", 1)[0]

    assert loop.index(
        "if not required_messages.issubset(latest):"
    ) < loop.index("observed_graph = graph()")


def test_data_plane_phase_specific_command_graph_contract():
    verifier = load_overlay("verify_tinynav_data_plane.py")
    pre_bridge = {
        "raw": {
            "publishers": ["/cmd_vel_control_node"],
            "subscriptions": ["/focus_v2_wsj_receiver"],
        },
        "guarded": {
            "publishers": ["/focus_v2_wsj_receiver"],
            "subscriptions": [],
        },
        "target": {
            "publishers": ["/focus_tinynav_buildmap_goal_router"],
            "subscriptions": [
                "/cmd_vel_control_node",
                "/planning_node",
            ],
        },
        "poi": {
            "publishers": ["/focus_v2_wsj_receiver"],
            "subscriptions": ["/focus_tinynav_buildmap_goal_router"],
        },
    }

    verifier.validate_command_graph(
        pre_bridge,
        robot_id="robot-0",
        mode="live",
        pre_bridge_command_check=True,
    )
    with pytest.raises(ValueError, match="guarded chassis subscriber"):
        verifier.validate_command_graph(
            pre_bridge,
            robot_id="robot-0",
            mode="live",
        )
    for invalid_target_subscribers in (
        ["/planning_node"],
        [
            "/cmd_vel_control_node",
            "/planning_node",
            "/unexpected_target_consumer",
        ],
    ):
        invalid_graph = {
            section: {
                direction: list(endpoints)
                for direction, endpoints in routes.items()
            }
            for section, routes in pre_bridge.items()
        }
        invalid_graph["target"]["subscriptions"] = invalid_target_subscribers
        with pytest.raises(ValueError, match="TinyNav target subscribers"):
            verifier.validate_command_graph(
                invalid_graph,
                robot_id="robot-0",
                mode="live",
                pre_bridge_command_check=True,
            )

    post_bridge = {
        "guarded": {
            "publishers": ["/focus_v2_wsj_receiver"],
            "subscriptions": ["/go2_cmd_bridge"],
        }
    }
    verifier.validate_command_graph(
        post_bridge,
        robot_id="robot-0",
        mode="live",
        guarded_only=True,
    )


def test_wsj_verification_is_split_around_sender_and_go2_bridge():
    launcher = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text(
        encoding="utf-8"
    )
    park_index = launcher.index(
        'bash "$SCRIPT_DIR/start_wsj_command_observation.sh"'
    )
    park_only_index = launcher.index("--park-only", park_index)
    sensor_index = launcher.index("--sensor-map-only")
    sender_index = launcher.index(
        'bash "$SCRIPT_DIR/start_wsj_command_observation.sh"',
        sensor_index,
    )
    command_index = launcher.index("--command-graph-only")
    pre_index = launcher.index("--pre-bridge-command-check")
    bridge_index = launcher.index(
        'tmux new-window -d -t "$SESSION" -n go2-bridge'
    )
    post_index = launcher.index("--post-bridge-command-check")

    assert (
        park_index
        < park_only_index
        < sensor_index
        < sender_index
        < command_index
        < pre_index
        < bridge_index
        < post_index
    )
    assert "single verifier replaces the former three serial" in launcher
    assert 'tmux kill-window -t "$SESSION:hub-sender"' not in launcher


def test_calibration_geometry_verifier_is_read_only_and_reuses_contract():
    verifier = (OVERLAY / "verify_ros_geometry_profile.py").read_text(
        encoding="utf-8"
    )

    assert "validate_geometry_contract" in verifier
    assert "create_subscription" in verifier
    assert "create_publisher" not in verifier
    assert "robot_commands_issued" in verifier
    assert "focus-ros-geometry-profile-v1" in verifier


def test_receiver_pose_conversions_preserve_planar_yaw():
    wsj = load_overlay("v2_wsj_receiver.py")
    yunji = load_overlay("v2_yunji_receiver.py")

    wsj_matrix = wsj.quaternion_pose_matrix(pose(yaw=0.7))
    yunji_message = SimpleNamespace(pose=SimpleNamespace(pose=pose(yaw=-0.4)))
    yunji_matrix = yunji.quaternion_pose_matrix(yunji_message)

    assert math.atan2(wsj_matrix[4], wsj_matrix[0]) == pytest.approx(0.7)
    assert math.atan2(yunji_matrix[4], yunji_matrix[0]) == pytest.approx(-0.4)


def test_wsj_uses_measured_mount_to_recover_robot_base_pose():
    wsj = load_overlay("v2_wsj_receiver.py")
    tracking_T_camera = (
        1.0, 0.0, 0.0, 1.3,
        0.0, 1.0, 0.0, 2.0,
        0.0, 0.0, 1.0, 0.3,
        0.0, 0.0, 0.0, 1.0,
    )
    base_T_camera = (
        1.0, 0.0, 0.0, 0.3,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.3,
        0.0, 0.0, 0.0, 1.0,
    )
    identity = (
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    )

    x_m, y_m, yaw_rad = wsj.robot_map_base_pose(
        tracking_T_map=identity,
        tracking_T_camera=tracking_T_camera,
        base_T_camera=base_T_camera,
    )

    assert (x_m, y_m, yaw_rad) == pytest.approx((1.0, 2.0, 0.0))


def test_wsj_slam_gate_rejects_bad_imu_and_accepts_complete_report():
    wsj = load_overlay("v2_wsj_receiver.py")
    assert wsj.slam_metrics_gate(valid_slam_payload()) == (
        True,
        "slam_optimizer_imu_valid",
    )
    bad = valid_slam_payload().replace('"imu_intervals_valid": true', '"imu_intervals_valid": false')
    assert wsj.slam_metrics_gate(bad)[0] is False


def test_wsj_receiver_recovers_only_after_stable_valid_overwrite_count():
    wsj = load_overlay("v2_wsj_receiver.py")
    gate = wsj.SlamHealthDebouncer()
    payload = valid_slam_payload(overwritten=17)

    assert gate.update(payload, received_ns=10_000_000_000)[0] is False
    assert gate.update(payload, received_ns=11_000_000_000)[0] is False
    passed, detail = gate.update(payload, received_ns=12_100_000_000)
    assert passed is True
    assert detail.endswith(":17")

    increased = valid_slam_payload(overwritten=18)
    assert gate.update(increased, received_ns=13_000_000_000)[0] is False


def test_wsj_slam_gate_matches_sender_numeric_thresholds():
    wsj = load_overlay("v2_wsj_receiver.py")
    sender_tree = ast.parse(
        (OVERLAY / "focus_ros_sender.py").read_text(encoding="utf-8")
    )
    sender_thresholds = {
        node.targets[0].id: ast.literal_eval(node.value)
        for node in sender_tree.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id
        in {
            "SLAM_IMU_MIN_COVERAGE_RATIO",
            "SLAM_IMU_MAX_SAMPLE_GAP_S",
            "SLAM_IMU_END_TOLERANCE_S",
        }
    }

    assert sender_thresholds == {
        "SLAM_IMU_MIN_COVERAGE_RATIO": wsj.SLAM_IMU_MIN_COVERAGE_RATIO,
        "SLAM_IMU_MAX_SAMPLE_GAP_S": wsj.SLAM_IMU_MAX_SAMPLE_GAP_S,
        "SLAM_IMU_END_TOLERANCE_S": wsj.SLAM_IMU_END_TOLERANCE_S,
    }
    assert wsj.slam_metrics_gate(
        valid_slam_payload(
            coverage_ratio=0.80,
            max_sample_gap_s=0.05,
            end_error_s=0.01,
        )
    )[0] is True
    assert wsj.slam_metrics_gate(
        valid_slam_payload(coverage_ratio=0.799)
    ) == (False, "imu_interval_threshold")
    assert wsj.slam_metrics_gate(
        valid_slam_payload(max_sample_gap_s=0.0501)
    ) == (False, "imu_interval_threshold")
    assert wsj.slam_metrics_gate(
        valid_slam_payload(end_error_s=0.0101)
    ) == (False, "imu_interval_threshold")


def test_wsj_slam_gate_tolerates_only_one_transient_interval_blip():
    wsj = load_overlay("v2_wsj_receiver.py")
    gate = wsj.SlamHealthDebouncer(
        max_transient_failures=1,
        max_last_good_age_s=2.0,
    )
    valid = valid_slam_payload()
    transient = valid.replace(
        '"imu_intervals_valid": true',
        '"imu_intervals_valid": false',
    )

    assert gate.update(valid, received_ns=1_000_000_000)[0] is True
    first_pass, first_detail = gate.update(
        transient,
        received_ns=1_500_000_000,
    )
    assert first_pass is True
    assert "transient_tolerated_1/1" in first_detail
    assert gate.update(
        transient,
        received_ns=1_900_000_000,
    ) == (False, "imu_intervals_invalid")

    assert gate.update(valid, received_ns=2_000_000_000)[0] is True
    hard = valid.replace(
        '"optimizer_status": "ok"',
        '"optimizer_status": "failed"',
    )
    assert gate.update(hard, received_ns=2_100_000_000) == (
        False,
        "optimizer_status=failed",
    )


def test_wsj_slam_gate_bounds_skipped_invalid_imu_optimizer_report():
    wsj = load_overlay("v2_wsj_receiver.py")
    gate = wsj.SlamHealthDebouncer(
        max_transient_failures=1,
        max_last_good_age_s=2.0,
    )
    valid = valid_slam_payload()
    skipped = valid.replace(
        '"optimizer_status": "ok"',
        '"optimizer_status": "skipped_imu_invalid"',
    )

    assert gate.update(skipped, received_ns=900_000_000) == (
        False,
        "optimizer_status=skipped_imu_invalid",
    )
    assert gate.update(valid, received_ns=1_000_000_000)[0] is True
    first_pass, first_detail = gate.update(
        skipped,
        received_ns=1_500_000_000,
    )
    assert first_pass is True
    assert first_detail == (
        "optimizer_status=skipped_imu_invalid_transient_tolerated_1/1"
    )
    assert gate.update(
        skipped,
        received_ns=1_900_000_000,
    ) == (False, "optimizer_status=skipped_imu_invalid")


def test_wsj_tracking_freshness_keeps_odom_deadline_stricter_than_slam():
    wsj = load_overlay("v2_wsj_receiver.py")

    fresh, odom_age_s, slam_age_s = wsj.local_tracking_freshness(
        now_ns=10_000_000_000,
        odom_received_ns=8_500_000_000,
        slam_received_ns=7_100_000_000,
        odom_timeout_s=2.0,
        slam_timeout_s=3.0,
    )
    assert fresh is True
    assert odom_age_s == pytest.approx(1.5)
    assert slam_age_s == pytest.approx(2.9)

    assert wsj.local_tracking_freshness(
        now_ns=10_000_000_000,
        odom_received_ns=7_900_000_000,
        slam_received_ns=9_900_000_000,
        odom_timeout_s=2.0,
        slam_timeout_s=3.0,
    )[0] is False
    assert wsj.local_tracking_freshness(
        now_ns=10_000_000_000,
        odom_received_ns=9_900_000_000,
        slam_received_ns=6_900_000_000,
        odom_timeout_s=2.0,
        slam_timeout_s=3.0,
    )[0] is False


def test_wsj_trajectory_gate_stops_before_terminal_recovery_deadline():
    wsj = load_overlay("v2_wsj_receiver.py")
    kwargs = {
        "authority_started_ns": 1_000_000_000,
        "trajectory_received_ns": 1_100_000_000,
        "stale_timeout_s": 1.0,
        "start_grace_s": 1.5,
        "recovery_timeout_s": 5.0,
    }

    fresh, failed, age_s, observed = wsj.trajectory_gate_state(
        now_ns=2_116_000_000,
        **kwargs,
    )
    assert fresh is False
    assert failed is False
    assert age_s == pytest.approx(1.016)
    assert observed is True

    fresh, failed, age_s, observed = wsj.trajectory_gate_state(
        now_ns=4_465_452_042,
        **kwargs,
    )
    assert fresh is False
    assert failed is False
    assert age_s == pytest.approx(3.365452042)
    assert observed is True

    fresh, failed, age_s, observed = wsj.trajectory_gate_state(
        now_ns=6_101_000_000,
        **kwargs,
    )
    assert fresh is False
    assert failed is True
    assert age_s == pytest.approx(5.001)
    assert observed is True


def test_wsj_trajectory_gate_keeps_never_started_path_grace_bounded():
    wsj = load_overlay("v2_wsj_receiver.py")

    state = wsj.trajectory_gate_state(
        now_ns=2_400_000_000,
        authority_started_ns=1_000_000_000,
        trajectory_received_ns=900_000_000,
        stale_timeout_s=1.0,
        start_grace_s=1.5,
        recovery_timeout_s=3.0,
    )
    assert state == (False, False, pytest.approx(1.4), False)

    state = wsj.trajectory_gate_state(
        now_ns=2_501_000_000,
        authority_started_ns=1_000_000_000,
        trajectory_received_ns=900_000_000,
        stale_timeout_s=1.0,
        start_grace_s=1.5,
        recovery_timeout_s=3.0,
    )
    assert state == (False, True, pytest.approx(1.501), False)


def test_planner_target_refresh_requires_same_active_ready_leg_at_zero_velocity():
    wsj = load_overlay("v2_wsj_receiver.py")
    ready = {
        "authorized": True,
        "router_recovery_gate_closed": False,
        "trajectory_fresh": False,
        "trajectory_failed": False,
        "trajectory_age_s": 1.1,
        "trajectory_stale_timeout_s": 1.0,
        "router_state": "NAVIGATING",
        "router_reason": "ONLINE_PATH_READY",
        "router_decision_id": "decision-1",
        "active_decision_id": "decision-1",
        "router_waypoint": (2.0, 3.0),
    }

    assert wsj.planner_target_refresh_eligible(**ready) is True
    for field, value in (
        ("trajectory_fresh", True),
        ("trajectory_failed", True),
        ("all_candidates_in_collision", True),
        ("router_recovery_gate_closed", True),
        ("router_state", "HOLD"),
        ("router_decision_id", "another-decision"),
        ("router_waypoint", None),
    ):
        candidate = {**ready, field: value}
        assert wsj.planner_target_refresh_eligible(**candidate) is False


def test_planner_candidate_status_is_strictly_validated():
    wsj = load_overlay("v2_wsj_receiver.py")
    payload = {
        "schema_version": wsj.PLANNER_CANDIDATE_STATUS_SCHEMA_VERSION,
        "all_candidates_in_collision": True,
        "candidate_count": 416,
        "finite_candidate_count": 0,
        "finite_in_place_candidate_count": 0,
        "evaluated_at_ns": 2_000_000_000,
        "robot_profile": "source-default",
    }

    assert wsj.parse_planner_candidate_status(json.dumps(payload)) == {
        "all_candidates_in_collision": True,
        "candidate_count": 416,
        "finite_candidate_count": 0,
        "finite_in_place_candidate_count": 0,
        "evaluated_at_ns": 2_000_000_000,
        "robot_profile": "source-default",
    }
    for update in (
        {"schema_version": "unsupported"},
        {"all_candidates_in_collision": False},
        {"finite_candidate_count": 417},
        {"finite_in_place_candidate_count": 1},
        {"evaluated_at_ns": 0},
    ):
        with pytest.raises(ValueError):
            wsj.parse_planner_candidate_status(
                json.dumps({**payload, **update})
            )


def test_all_collision_gate_is_authority_scoped_fresh_and_bounded():
    wsj = load_overlay("v2_wsj_receiver.py")
    common = {
        "authority_started_ns": 1_000_000_000,
        "collision_since_ns": 1_100_000_000,
        "all_candidates_in_collision": True,
        "status_timeout_s": 1.0,
        "rejection_timeout_s": 7.0,
    }

    # A report from before this authority epoch cannot reject a new leg.
    assert wsj.planner_collision_gate_state(
        now_ns=1_200_000_000,
        status_received_ns=900_000_000,
        **common,
    ) == (False, False, 0.0, False)
    assert wsj.planner_collision_gate_state(
        now_ns=1_600_000_000,
        status_received_ns=1_550_000_000,
        **common,
    ) == (True, False, pytest.approx(0.5), True)
    assert wsj.planner_collision_gate_state(
        now_ns=8_200_000_000,
        status_received_ns=8_150_000_000,
        **common,
    ) == (True, True, pytest.approx(7.1), True)
    # Stale collision evidence reopens this specific gate; independent path
    # freshness still remains fail-closed in the receiver.
    assert wsj.planner_collision_gate_state(
        now_ns=9_200_000_000,
        status_received_ns=8_150_000_000,
        **common,
    ) == (False, False, pytest.approx(8.1), True)


def test_observed_six_second_collision_recovery_is_not_terminal():
    wsj = load_overlay("v2_wsj_receiver.py")

    # The physical gate remains closed throughout the observed transient, but
    # the leg is not permanently rejected before a new finite candidate set
    # arrives.  This covers the 5.85 s and 5.94 s live recoveries.
    gate_closed, terminal, age_s, observed = (
        wsj.planner_collision_gate_state(
            now_ns=6_940_000_000,
            authority_started_ns=500_000_000,
            status_received_ns=6_900_000_000,
            collision_since_ns=1_000_000_000,
            all_candidates_in_collision=True,
            status_timeout_s=1.0,
            rejection_timeout_s=7.0,
        )
    )
    assert gate_closed is True
    assert terminal is False
    assert age_s == pytest.approx(5.94)
    assert observed is True
    assert wsj.planner_collision_gate_state(
        now_ns=6_950_000_000,
        authority_started_ns=500_000_000,
        status_received_ns=6_950_000_000,
        collision_since_ns=0,
        all_candidates_in_collision=False,
        status_timeout_s=1.0,
        rejection_timeout_s=7.0,
    ) == (False, False, 0.0, False)


def test_final_velocity_gate_rechecks_all_health_at_control_rate():
    wsj = load_overlay("v2_wsj_receiver.py")
    base = {
        "now_ns": 10_000_000_000,
        "authorized": True,
        "authority_deadline_ns": 11_000_000_000,
        "trajectory_fresh": True,
        "reverse_required": False,
        "health_pass": True,
        "health_evaluated_ns": 9_500_000_000,
        "health_timeout_s": 1.5,
        "odom_received_ns": 9_800_000_000,
        "slam_received_ns": 9_000_000_000,
        "odom_timeout_s": 2.0,
        "slam_timeout_s": 3.0,
        "slam_pass": True,
        "occupancy_received_ns": 9_700_000_000,
        "occupancy_timeout_s": 3.0,
        "platform_required": True,
        "platform_received_ns": 9_800_000_000,
        "platform_timeout_s": 2.0,
        "platform_pass": True,
        "router_recovery_gate_closed": False,
        "all_candidates_in_collision": False,
    }

    assert wsj.physical_velocity_gate_reason(**base) is None
    assert (
        wsj.physical_velocity_gate_reason(
            **{
                **base,
                "occupancy_received_ns": 6_999_999_999,
                "cached_occupancy_motion_valid": True,
            }
        )
        is None
    )
    cases = {
        "authorized": (False, "authority_closed"),
        "health_pass": (False, "health_not_ready"),
        "health_evaluated_ns": (
            8_499_999_999,
            "health_evaluation_stale",
        ),
        "odom_received_ns": (7_999_999_999, "local_tracking_stale"),
        "slam_pass": (False, "localization_not_tracking"),
        "occupancy_received_ns": (
            6_999_999_999,
            "occupancy_missing_or_stale",
        ),
        "platform_received_ns": (
            7_999_999_999,
            "platform_health_stale",
        ),
        "platform_pass": (False, "platform_health_not_ready"),
        "reverse_required": (True, "reverse_trajectory_rejected"),
        "all_candidates_in_collision": (
            True,
            "all_trajectories_in_collision",
        ),
        "turn_stalled": (True, "turn_recovery_stalled"),
        "trajectory_fresh": (False, "trajectory_missing_or_stale"),
        "router_recovery_gate_closed": (
            True,
            "router_recovery_gate_closed",
        ),
    }
    for field, (value, expected) in cases.items():
        assert wsj.physical_velocity_gate_reason(
            **{**base, field: value}
        ) == expected


def test_occupancy_episode_recovery_is_bounded_after_motion_gate_closes():
    wsj = load_overlay("v2_wsj_receiver.py")
    common = {
        "recovery_grace_s": 7.0,
        "all_other_health_ready": True,
        "occupancy_observed": True,
    }

    assert wsj.occupancy_recovery_eligible(
        recovery_elapsed_s=0.0, **common
    )
    assert wsj.occupancy_recovery_eligible(
        recovery_elapsed_s=7.0, **common
    )
    assert not wsj.occupancy_recovery_eligible(
        recovery_elapsed_s=7.001, **common
    )
    assert not wsj.occupancy_recovery_eligible(
        recovery_elapsed_s=0.105,
        **{**common, "all_other_health_ready": False},
    )
    assert not wsj.occupancy_recovery_eligible(
        recovery_elapsed_s=0.105,
        **{**common, "occupancy_observed": False},
    )
    assert not wsj.occupancy_recovery_eligible(
        recovery_elapsed_s=float("inf"), **common
    )


def test_heartbeat_delivery_gap_stops_motion_but_preserves_one_bounded_leg():
    wsj = load_overlay("v2_wsj_receiver.py")
    common = {
        "recovery_grace_s": 3.0,
        "sensor_ready": True,
        "heartbeat_delivery_ready": False,
    }

    assert wsj.heartbeat_delivery_recovery_eligible(
        recovery_elapsed_s=0.0, **common
    )
    assert wsj.heartbeat_delivery_recovery_eligible(
        recovery_elapsed_s=3.0, **common
    )
    assert not wsj.heartbeat_delivery_recovery_eligible(
        recovery_elapsed_s=3.001, **common
    )
    assert not wsj.heartbeat_delivery_recovery_eligible(
        recovery_elapsed_s=0.5,
        **{**common, "sensor_ready": False},
    )
    assert not wsj.heartbeat_delivery_recovery_eligible(
        recovery_elapsed_s=0.5,
        **{**common, "heartbeat_delivery_ready": True},
    )
    assert not wsj.heartbeat_delivery_recovery_eligible(
        recovery_elapsed_s=float("inf"), **common
    )
    with pytest.raises(ValueError, match="must be positive"):
        wsj.heartbeat_delivery_recovery_eligible(
            recovery_elapsed_s=0.0,
            recovery_grace_s=0.0,
            sensor_ready=True,
            heartbeat_delivery_ready=False,
        )


def test_heartbeat_recovery_renewal_overrides_only_delivery_safety_state():
    wsj = load_overlay("v2_wsj_receiver.py")
    health = wsj.RobotHealth(
        safety_state=wsj.SafetyState.HOLD,
        localization_state=wsj.LocalizationState.TRACKING,
        estop_engaged=False,
        collision_avoidance_ready=True,
        motor_controller_ready=True,
        detail="robot local health ready; heartbeat delivery is not fresh",
    )

    renewal = wsj.heartbeat_delivery_recovery_renewal_health(health)

    assert renewal.ready_for_goal()
    assert renewal.detail == health.detail
    for update in (
        {"estop_engaged": True},
        {"localization_state": wsj.LocalizationState.LOST},
        {"collision_avoidance_ready": False},
        {"motor_controller_ready": False},
    ):
        unsafe = wsj.heartbeat_delivery_recovery_renewal_health(
            health.model_copy(update=update)
        )
        assert not unsafe.ready_for_goal()


def test_occupancy_recovery_renewal_health_only_overrides_map_gate():
    wsj = load_overlay("v2_wsj_receiver.py")
    health = wsj.RobotHealth(
        safety_state=wsj.SafetyState.HOLD,
        localization_state=wsj.LocalizationState.TRACKING,
        estop_engaged=False,
        collision_avoidance_ready=False,
        motor_controller_ready=True,
        detail="occupancy stale",
    )

    renewal = wsj.occupancy_recovery_renewal_health(health)

    assert renewal.ready_for_goal()
    assert renewal.detail == "occupancy stale"
    estopped = wsj.occupancy_recovery_renewal_health(
        health.model_copy(update={"estop_engaged": True})
    )
    assert not estopped.ready_for_goal()


def test_slam_episode_recovery_stops_motion_but_bounds_leg_wait():
    wsj = load_overlay("v2_wsj_receiver.py")
    common = {
        "recovery_grace_s": 2.0,
        "slam_detail": "optimizer_status=skipped_imu_invalid",
        "all_non_slam_health_ready": True,
    }

    assert wsj.slam_recovery_eligible(
        recovery_elapsed_s=0.0, **common
    )
    assert wsj.slam_recovery_eligible(
        recovery_elapsed_s=2.0, **common
    )
    assert not wsj.slam_recovery_eligible(
        recovery_elapsed_s=2.001, **common
    )
    assert not wsj.slam_recovery_eligible(
        recovery_elapsed_s=0.1,
        **{**common, "slam_detail": "optimizer_status=failed"},
    )
    assert not wsj.slam_recovery_eligible(
        recovery_elapsed_s=0.1,
        **{**common, "all_non_slam_health_ready": False},
    )


def test_slam_recovery_renewal_health_only_overrides_slam_gate():
    wsj = load_overlay("v2_wsj_receiver.py")
    health = wsj.RobotHealth(
        safety_state=wsj.SafetyState.HOLD,
        localization_state=wsj.LocalizationState.LOST,
        estop_engaged=False,
        collision_avoidance_ready=True,
        motor_controller_ready=True,
        detail="optimizer_status=skipped_imu_invalid",
    )

    renewal = wsj.slam_recovery_renewal_health(health)

    assert renewal.ready_for_goal()
    assert renewal.detail == health.detail
    estopped = wsj.slam_recovery_renewal_health(
        health.model_copy(update={"estop_engaged": True})
    )
    assert not estopped.ready_for_goal()


def test_odometry_recovery_is_bounded_after_velocity_gate_closes():
    wsj = load_overlay("v2_wsj_receiver.py")
    common = {
        "recovery_grace_s": 7.0,
        "all_non_odometry_health_ready": True,
        "odometry_observed": True,
    }

    assert wsj.odometry_recovery_eligible(
        recovery_elapsed_s=0.0, **common
    )
    assert wsj.odometry_recovery_eligible(
        recovery_elapsed_s=7.0, **common
    )
    assert not wsj.odometry_recovery_eligible(
        recovery_elapsed_s=7.001, **common
    )
    assert not wsj.odometry_recovery_eligible(
        recovery_elapsed_s=0.1,
        **{**common, "all_non_odometry_health_ready": False},
    )
    assert not wsj.odometry_recovery_eligible(
        recovery_elapsed_s=0.1,
        **{**common, "odometry_observed": False},
    )


def test_odometry_recovery_renewal_overrides_only_localization_gate():
    wsj = load_overlay("v2_wsj_receiver.py")
    health = wsj.RobotHealth(
        safety_state=wsj.SafetyState.HOLD,
        localization_state=wsj.LocalizationState.LOST,
        estop_engaged=False,
        collision_avoidance_ready=True,
        motor_controller_ready=True,
        detail="slam_optimizer_imu_valid; odom stale",
    )

    renewal = wsj.odometry_recovery_renewal_health(health)

    assert renewal.ready_for_goal()
    assert renewal.detail == health.detail
    assert not wsj.odometry_recovery_renewal_health(
        health.model_copy(update={"estop_engaged": True})
    ).ready_for_goal()


def test_odometry_slam_recovery_keeps_both_original_bounds():
    wsj = load_overlay("v2_wsj_receiver.py")
    common = {
        "odometry_recovery_grace_s": 7.0,
        "slam_recovery_grace_s": 2.0,
        "slam_detail": "optimizer_status=skipped_imu_invalid",
        "all_non_odometry_slam_health_ready": True,
        "odometry_observed": True,
    }

    assert wsj.odometry_slam_recovery_eligible(
        odometry_recovery_elapsed_s=6.9,
        slam_recovery_elapsed_s=1.9,
        **common,
    )
    assert not wsj.odometry_slam_recovery_eligible(
        odometry_recovery_elapsed_s=7.001,
        slam_recovery_elapsed_s=0.1,
        **common,
    )
    assert not wsj.odometry_slam_recovery_eligible(
        odometry_recovery_elapsed_s=0.1,
        slam_recovery_elapsed_s=2.001,
        **common,
    )
    assert not wsj.odometry_slam_recovery_eligible(
        odometry_recovery_elapsed_s=0.1,
        slam_recovery_elapsed_s=0.1,
        **{**common, "slam_detail": "optimizer_status=failed"},
    )
    assert not wsj.odometry_slam_recovery_eligible(
        odometry_recovery_elapsed_s=0.1,
        slam_recovery_elapsed_s=0.1,
        **{**common, "all_non_odometry_slam_health_ready": False},
    )


def test_odometry_slam_handoff_preserves_timer_only_for_same_leg():
    wsj = load_overlay("v2_wsj_receiver.py")

    assert wsj.inherited_recovery_start_ns(
        now_ns=9_000_000_000,
        active_leg_id="leg-a",
        current_started_ns=0,
        current_leg_id=None,
        handoff_started_ns=7_500_000_000,
        handoff_leg_id="leg-a",
    ) == 7_500_000_000
    assert wsj.inherited_recovery_start_ns(
        now_ns=9_000_000_000,
        active_leg_id="leg-b",
        current_started_ns=0,
        current_leg_id=None,
        handoff_started_ns=7_500_000_000,
        handoff_leg_id="leg-a",
    ) == 9_000_000_000
    assert wsj.inherited_recovery_start_ns(
        now_ns=9_000_000_000,
        active_leg_id="leg-a",
        current_started_ns=7_000_000_000,
        current_leg_id="leg-a",
        handoff_started_ns=7_500_000_000,
        handoff_leg_id="leg-a",
    ) == 7_000_000_000


def test_combined_sensor_recovery_keeps_both_original_bounds():
    wsj = load_overlay("v2_wsj_receiver.py")
    common = {
        "occupancy_recovery_grace_s": 7.0,
        "slam_recovery_grace_s": 2.0,
        "slam_detail": "optimizer_status=skipped_imu_invalid",
        "all_non_sensor_health_ready": True,
        "occupancy_observed": True,
    }

    assert wsj.combined_sensor_recovery_eligible(
        occupancy_recovery_elapsed_s=6.9,
        slam_recovery_elapsed_s=1.9,
        **common,
    )
    assert not wsj.combined_sensor_recovery_eligible(
        occupancy_recovery_elapsed_s=7.001,
        slam_recovery_elapsed_s=0.1,
        **common,
    )
    assert not wsj.combined_sensor_recovery_eligible(
        occupancy_recovery_elapsed_s=0.1,
        slam_recovery_elapsed_s=2.001,
        **common,
    )
    assert not wsj.combined_sensor_recovery_eligible(
        occupancy_recovery_elapsed_s=0.1,
        slam_recovery_elapsed_s=0.1,
        **{**common, "slam_detail": "optimizer_status=failed"},
    )
    assert not wsj.combined_sensor_recovery_eligible(
        occupancy_recovery_elapsed_s=0.1,
        slam_recovery_elapsed_s=0.1,
        **{**common, "occupancy_observed": False},
    )


def test_combined_sensor_recovery_renewal_overrides_only_sensor_gates():
    wsj = load_overlay("v2_wsj_receiver.py")
    health = wsj.RobotHealth(
        safety_state=wsj.SafetyState.HOLD,
        localization_state=wsj.LocalizationState.LOST,
        estop_engaged=False,
        collision_avoidance_ready=False,
        motor_controller_ready=True,
        detail="optimizer_status=skipped_imu_invalid; occupancy stale",
    )

    renewal = wsj.combined_sensor_recovery_renewal_health(health)

    assert renewal.ready_for_goal()
    assert renewal.detail == health.detail
    estopped = wsj.combined_sensor_recovery_renewal_health(
        health.model_copy(update={"estop_engaged": True})
    )
    assert not estopped.ready_for_goal()


def test_closed_gate_recovery_kind_includes_combined_sensor_state():
    wsj = load_overlay("v2_wsj_receiver.py")

    assert wsj.closed_gate_recovery_kind(
        occupancy_recovery_active=True,
        slam_recovery_active=False,
        odometry_recovery_active=False,
        combined_sensor_recovery_active=False,
    ) == "occupancy"
    assert wsj.closed_gate_recovery_kind(
        occupancy_recovery_active=False,
        slam_recovery_active=True,
        odometry_recovery_active=False,
        combined_sensor_recovery_active=False,
    ) == "slam"
    assert wsj.closed_gate_recovery_kind(
        occupancy_recovery_active=False,
        slam_recovery_active=False,
        odometry_recovery_active=False,
        combined_sensor_recovery_active=True,
    ) == "combined"
    assert wsj.closed_gate_recovery_kind(
        occupancy_recovery_active=False,
        slam_recovery_active=False,
        odometry_recovery_active=True,
        combined_sensor_recovery_active=False,
    ) == "odometry"
    assert wsj.closed_gate_recovery_kind(
        occupancy_recovery_active=False,
        slam_recovery_active=False,
        odometry_recovery_active=False,
        combined_sensor_recovery_active=False,
    ) is None
    with pytest.raises(RuntimeError, match="mutually exclusive"):
        wsj.closed_gate_recovery_kind(
            occupancy_recovery_active=True,
            slam_recovery_active=True,
            odometry_recovery_active=False,
            combined_sensor_recovery_active=False,
        )
    source = (
        Path(__file__).resolve().parents[1]
        / "robot_overlay"
        / "v2_wsj_receiver.py"
    ).read_text(encoding="utf-8")
    assert '"closed_gate_recovery_handoff"' in source
    assert source.count(
        "physical velocity gate closed immediately"
    ) == 5
    assert '"LOCAL_SENSOR_RECOVERY_WAIT"' in source
    assert '"combined_sensor_recovery_wait"' in source
    assert '"LOCAL_ODOMETRY_RECOVERY_WAIT"' in source
    assert '"odometry_stale_recovery_wait"' in source
    assert '"LOCAL_HEARTBEAT_RECOVERY_WAIT"' in source
    assert '"HEARTBEAT_DELIVERY_TIMEOUT"' in source
    assert "ready = bool(sensor_ready and heartbeat_delivery_ready)" in source


def test_both_robot_launchers_share_bounded_heartbeat_delivery_recovery():
    wsj = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text(
        encoding="utf-8"
    )
    yunji = (OVERLAY / "start_yunji_v2.sh").read_text(encoding="utf-8")

    assert "FOCUS_WSJ_HEARTBEAT_DELIVERY_RECOVERY_GRACE_S:-3.0" in wsj
    assert "FOCUS_YUNJI_HEARTBEAT_DELIVERY_RECOVERY_GRACE_S:-3.0" in yunji
    for launcher in (wsj, yunji):
        assert "--heartbeat-delivery-recovery-grace-s" in launcher
        assert '"$HEARTBEAT_DELIVERY_RECOVERY_GRACE_S"' in launcher


def test_receiver_and_router_share_cached_occupancy_motion_gate():
    wsj = load_overlay("v2_wsj_receiver.py")

    assert wsj.cached_map_valid_for_pose(
        map_age_s=17.7,
        map_timeout_s=5.0,
        map_anchor_base_xy=(1.0, 2.0),
        current_base_xy=(1.24, 2.0),
        max_cached_map_motion_m=0.25,
    ) == (True, pytest.approx(0.24))
    assert wsj.cached_map_valid_for_pose(
        map_age_s=17.7,
        map_timeout_s=5.0,
        map_anchor_base_xy=(1.0, 2.0),
        current_base_xy=(1.26, 2.0),
        max_cached_map_motion_m=0.25,
    ) == (False, pytest.approx(0.26))


def test_receiver_trajectory_contract_rejects_stale_geometry_inputs():
    wsj = load_overlay("v2_wsj_receiver.py")
    valid = SimpleNamespace(
        header=SimpleNamespace(frame_id="world"),
        poses=[
            SimpleNamespace(pose=pose(x=0.0, y=0.0)),
            SimpleNamespace(pose=pose(x=0.2, y=0.0)),
        ],
    )

    assert wsj.trajectory_message_summary(
        valid, expected_frame="world"
    ) == (2, (0.0, 0.0), (0.2, 0.0))
    valid.poses.insert(
        1,
        SimpleNamespace(pose=pose(x=0.0, y=0.0)),
    )
    assert wsj.trajectory_message_summary(
        valid, expected_frame="world"
    ) == (3, (0.0, 0.0), (0.2, 0.0))
    valid.poses[2].pose.position.x = 0.005
    with pytest.raises(ValueError, match="geometrically distinct"):
        wsj.trajectory_message_summary(valid, expected_frame="world")
    valid.poses[1].pose = pose(x=0.0, y=0.0, yaw=math.radians(10.0))
    assert wsj.trajectory_message_summary(
        valid, expected_frame="world"
    ) == (3, (0.0, 0.0), (0.0, 0.0))
    valid.poses[2].pose.position.x = 0.2
    valid.header.frame_id = "map"
    with pytest.raises(ValueError, match="frame"):
        wsj.trajectory_message_summary(valid, expected_frame="world")
    valid.header.frame_id = "world"
    valid.poses[1].pose.position.x = float("nan")
    with pytest.raises(ValueError, match="non-finite"):
        wsj.trajectory_message_summary(valid, expected_frame="world")


def test_receiver_rejects_nonfinite_raw_twist():
    wsj = load_overlay("v2_wsj_receiver.py")
    message = SimpleNamespace(
        linear=SimpleNamespace(x=0.1, y=0.0, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=-0.2),
    )
    assert wsj.twist_components_finite(message)
    message.angular.z = float("inf")
    assert not wsj.twist_components_finite(message)


def test_wsj_recovers_only_transient_router_input_lag_with_ready_gate():
    wsj = load_overlay("v2_wsj_receiver.py")

    assert wsj.recoverable_router_hold(
        "ODOMETRY_STALE", receiver_runtime_ready=True
    )
    assert wsj.recoverable_router_hold(
        "OCCUPANCY_STALE_AFTER_MOTION", receiver_runtime_ready=True
    )
    assert not wsj.recoverable_router_hold(
        "ODOMETRY_STALE", receiver_runtime_ready=False
    )
    assert not wsj.recoverable_router_hold(
        "OCCUPANCY_STALE_AFTER_MOTION", receiver_runtime_ready=False
    )
    assert not wsj.recoverable_router_hold(
        "NO_KNOWN_FREE_PATH", receiver_runtime_ready=True
    )


@pytest.mark.parametrize(
    "target_kind",
    ["FRONTIER_POINT", "SEMANTIC_REGION"],
)
def test_no_path_gets_bounded_zero_velocity_map_recovery(target_kind):
    receiver = load_overlay("v2_wsj_receiver.py")

    assert receiver.router_hold_recovery_eligible(
        target_kind,
        "NO_KNOWN_FREE_PATH",
        receiver_runtime_ready=True,
    )
    assert not receiver.router_hold_recovery_eligible(
        target_kind,
        "NO_KNOWN_FREE_PATH",
        receiver_runtime_ready=False,
    )
    assert receiver.router_hold_recovery_eligible(
        target_kind,
        "ODOMETRY_STALE",
        receiver_runtime_ready=True,
    )


@pytest.mark.parametrize(
    "target_kind",
    ["FRONTIER_POINT", "SEMANTIC_REGION"],
)
def test_no_known_free_path_replans_both_high_level_target_kinds(
    target_kind,
):
    receiver = load_overlay("v2_wsj_receiver.py")

    assert receiver.no_known_free_path_requires_replan(
        target_kind,
        "NO_KNOWN_FREE_PATH",
    )
    assert not receiver.no_known_free_path_requires_replan(
        target_kind,
        "ODOMETRY_STALE",
    )
    assert not receiver.no_known_free_path_requires_replan(
        "UNSUPPORTED_TARGET",
        "NO_KNOWN_FREE_PATH",
    )


def test_goal_progress_watchdog_survives_leases_and_bounds_stall():
    receiver = load_overlay("v2_wsj_receiver.py")
    watchdog = receiver.GoalProgressWatchdog(
        timeout_s=20.0,
        minimum_improvement_m=0.05,
    )

    assert watchdog.observe(
        leg_id="leg-1",
        remaining_m=1.0,
        now_monotonic=10.0,
        position_xy=(0.0, 0.0),
    ) == (False, 0.0)
    assert watchdog.observe(
        leg_id="leg-1",
        remaining_m=0.97,
        now_monotonic=25.0,
        position_xy=(0.01, 0.0),
    ) == pytest.approx((False, 15.0))
    # A real 5 cm reduction resets the same-leg timer; short lease renewals
    # must not reset it merely by changing decision IDs.
    assert watchdog.observe(
        leg_id="leg-1",
        remaining_m=0.94,
        now_monotonic=26.0,
        position_xy=(0.02, 0.0),
    ) == (False, 0.0)
    stalled, stalled_s = watchdog.observe(
        leg_id="leg-1",
        remaining_m=0.93,
        now_monotonic=46.0,
        position_xy=(0.03, 0.0),
    )
    assert stalled is True
    assert stalled_s == pytest.approx(20.0)
    # A genuinely new navigation leg starts a fresh bounded window.
    assert watchdog.observe(
        leg_id="leg-2",
        remaining_m=2.0,
        now_monotonic=47.0,
        position_xy=(1.0, 1.0),
    ) == (False, 0.0)


def test_goal_progress_watchdog_accepts_bounded_detour_motion():
    receiver = load_overlay("v2_wsj_receiver.py")
    watchdog = receiver.GoalProgressWatchdog(
        timeout_s=20.0,
        minimum_improvement_m=0.05,
    )

    assert watchdog.observe(
        leg_id="leg-detour",
        remaining_m=1.0,
        now_monotonic=10.0,
        position_xy=(0.0, 0.0),
    ) == (False, 0.0)
    # A locally planned detour can move sideways before straight-line goal
    # distance falls.  Physical displacement is progress, while the source
    # 24/25-tick round still provides the outer execution bound.
    assert watchdog.observe(
        leg_id="leg-detour",
        remaining_m=1.01,
        now_monotonic=29.0,
        position_xy=(0.0, 0.06),
    ) == (False, 0.0)
    stalled, stalled_s = watchdog.observe(
        leg_id="leg-detour",
        remaining_m=1.0,
        now_monotonic=49.0,
        position_xy=(0.0, 0.07),
    )
    assert stalled is True
    assert stalled_s == pytest.approx(20.0)


def test_external_odin_odometry_health_uses_covariance_fail_closed():
    receiver = load_overlay("v2_wsj_receiver.py")
    covariance = [0.0] * 36
    covariance[0] = 0.001
    covariance[7] = 0.002
    covariance[35] = 0.003

    assert receiver.external_odometry_covariance_gate(covariance) == (
        True,
        "external_odometry_covariance_tracking",
    )
    covariance[7] = 0.02
    assert receiver.external_odometry_covariance_gate(covariance) == (
        False,
        "external_odometry_covariance_not_tracking",
    )
    assert receiver.external_odometry_covariance_gate([0.0] * 35) == (
        False,
        "external_odometry_covariance_malformed",
    )


def test_receiver_bounds_health_detail_without_losing_authority_evidence():
    receiver = load_overlay("v2_wsj_receiver.py")
    source = (
        "external_odometry_covariance_tracking; "
        + "graph_evidence=" * 80
        + "; WATER local status/watchdog retains final authority"
    )

    bounded = receiver.bounded_protocol_detail(source)

    assert len(bounded) == 512
    assert bounded.startswith("external_odometry_covariance_tracking")
    assert "truncated_sha256=" in bounded
    assert bounded.endswith(
        "WATER local status/watchdog retains final authority"
    )
    receiver.RobotHealth(
        safety_state=receiver.SafetyState.READY,
        localization_state=receiver.LocalizationState.TRACKING,
        estop_engaged=False,
        collision_avoidance_ready=True,
        motor_controller_ready=True,
        detail=bounded,
    )


def test_receiver_help_exposes_separate_explicit_live_gates():
    cases = {
        "v2_wsj_receiver.py": (
            "--enable-live-go2-motion",
            "OPERATOR_PRESENT_AND_WSJ_CLEAR",
        ),
        "v2_yunji_receiver.py": (
            "--enable-live-water-motion",
            "OPERATOR_PRESENT_AND_YUNJI_CLEAR",
        ),
    }
    for filename, expected in cases.items():
        result = subprocess.run(
            [sys.executable, str(OVERLAY / filename), "--help"],
            capture_output=True,
            text=True,
            check=False,
        )
        assert result.returncode == 0, result.stderr
        assert expected[0] in result.stdout
        source = (OVERLAY / filename).read_text(encoding="utf-8")
        assert expected[1] in source


def test_wsj_command_path_has_a_distinct_guarded_topic():
    source = (OVERLAY / "v2_wsj_receiver.py").read_text(encoding="utf-8")
    assert 'default="/cmd_vel"' in source
    assert 'default="/focus_guarded_cmd_vel"' in source
    assert "raw and guarded cmd_vel topics must differ" in source
    assert "poi_has_no_bypass_publisher" in source
    assert "raw_cmd_has_no_direct_bridge" in source
    assert "--trajectory-start-grace-s" in source
    assert "--trajectory-stale-timeout-s" in source
    assert "--semantic-arrival-radius-m" in source
    assert "--reverse-required-topic" in source
    assert "--reject-reverse-trajectory" in source
    assert "--turn-stalled-topic" in source
    assert "--reject-stalled-turn" in source
    assert "--controller-pause-service" in source
    assert "--controller-pause-ack-timeout-s" in source
    assert "--controller-pause-startup-timeout-s" in source
    assert "self.pause_client.call_async(request)" in source
    assert "node.set_controller_paused_confirmed(" in source
    assert 'phase="startup"' in source
    assert "set_controller_paused_confirmed(False)" in source
    assert '"controller_pause_acknowledged"' in source
    assert '"controller_pause_ack_timeout"' in source
    assert "HubHeartbeatPump" in source
    assert "heartbeat_pump.update(reported_health)" in source
    assert "hub.post_heartbeat(health)" not in source
    hold_priority = source.index(
        'decision.mode.value in {"HOLD", "STOP"}'
    )
    episode_path_binding = source.index(
        "path, path_episode_id = bind_path_to_episode("
    )
    assert episode_path_binding < hold_priority
    recovery_renewal = source.index(
        "active_combined_sensor_recovery = bool(",
        hold_priority,
    )
    assert hold_priority < recovery_renewal
    priority_branch = source[
        hold_priority:recovery_renewal
    ]
    assert "if acknowledged:" in priority_branch
    assert "last_decision_id = decision.decision_id" in priority_branch
    assert "continue" in priority_branch
    assert '"LOCAL_PATH_REVERSE_REQUIRED"' in source
    assert '"LOCAL_PLANNER_TURN_STALLED"' in source
    assert '"reverse_trajectory_rejected"' in source
    assert '"trajectory_missing_or_stale"' in source
    assert '"LOCAL_PLANNER_PATH_STALE"' in source
    assert 'default="/planning/candidate_status"' in source
    assert '"planner_candidate_status_publisher"' in source
    assert '"all_trajectories_in_collision"' in source
    assert '"local_planner_all_candidates_in_collision"' in source
    assert '"LOCAL_GOAL_UNREACHABLE"' in source
    path_stale_branch = source.split(
        'NavigationStatusV2.REJECTED,\n                        "LOCAL_PLANNER_PATH_STALE"',
        1,
    )[0].rsplit("elif (", 1)[1]
    assert "active_goal.target_kind" not in path_stale_branch
    assert "and trajectory_failed" in path_stale_branch
    assert (
        "and router_recovery_leg_id\n"
        "                    != active_decision.leg_id"
        in path_stale_branch
    )
    assert '"frontier_no_path_rejected"' in source
    assert '"semantic_no_path_rejected"' in source
    assert '"LOCAL_GOAL_UNREACHABLE"' in source
    assert "NavigationStatusV2.REJECTED" in source
    assert "no_known_free_path_requires_replan(" in source
    assert "self.router_status_lock = threading.Lock()" in source
    assert "node.router_status_snapshot()" in source
    assert (
        "router_decision_id == active_decision.decision_id" in source
    )
    assert "node.router_decision_id is None" not in source
    assert '"control_telemetry"' in source
    assert '"occupancy_recovery_lease_renewed"' in source
    assert '"slam_recovery_lease_renewed"' in source
    assert "renew_authority_while_gate_closed" in source
    assert "authorize_motion=False" in source
    assert "self.router_recovery_gate_closed = True" in source
    assert "node.close_router_recovery_gate()" in source
    assert "node.revoke(pause=False)" not in source
    router_gate_method = source.split(
        "def close_router_recovery_gate(self)", 1
    )[1].split("def set_controller_paused_confirmed(", 1)[0]
    assert "self.authority_started_ns =" not in router_gate_method
    assert "self.authority_deadline_ns =" not in router_gate_method
    assert '"LOCAL_AUTHORITY_LOST"' in source
    assert '"recovery_authority_lost_local_hold"' in source
    assert "--slam-recovery-grace-s" in source
    assert '"slam_transient_recovery_wait"' in source
    assert '"LOCAL_SLAM_RECOVERY_WAIT"' in source
    assert '"SLAM_TRANSIENT_TIMEOUT"' in source
    assert '"occupancy_recovery_decision_deferred"' not in source
    recovery_feedback = source.split(
        "elif router_recovery_leg_id == active_decision.leg_id:", 1
    )[1].split("elif not goal_published_this_cycle:", 1)[0]
    assert "NavigationStatusV2.ACCEPTED" in recovery_feedback
    assert '"LOCAL_ROUTER_RECOVERY_WAIT"' in recovery_feedback
    assert "last_feedback_monotonic = time.monotonic()" in recovery_feedback


def test_wsj_online_buildmap_mode_is_explicit_and_pause_is_latched():
    result = subprocess.run(
        [sys.executable, str(OVERLAY / "v2_wsj_receiver.py"), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "--online-buildmap-world" in result.stdout
    assert "--base-camera-calibration-file" in result.stdout

    source = (OVERLAY / "v2_wsj_receiver.py").read_text(encoding="utf-8")
    assert "source_derived_session_local_identity" in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    assert '"HEALTH_NOT_READY"' in source
    assert "component_within_radius" in source


def test_wsj_ros_callbacks_have_an_independent_executor() -> None:
    source = (OVERLAY / "v2_wsj_receiver.py").read_text(encoding="utf-8")
    assert "SingleThreadedExecutor" in source
    assert 'name="focus-v2-wsj-ros"' in source
    command_loop = source.split("while rclpy.ok():", 1)[1]
    assert "rclpy.spin_once(node" not in command_loop


def test_wsj_maploc_repair_is_no_bridge_and_fail_closed() -> None:
    buildmap = (OVERLAY / "start_go2_buildmap.sh").read_text(
        encoding="utf-8"
    )
    launcher = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text(
        encoding="utf-8"
    )

    assert "--repair-online-stack" in buildmap
    assert "Repair refuses any live physical command path." in buildmap
    assert "ros2 topic pub --once /nav/paused" in buildmap
    assert "v2_wsj_receiver\\.py.*--enable-live-go2-motion" in buildmap
    assert 'missing_windows[0]}" == "maploc"' in launcher
    assert "--repair-online-stack" in launcher
    assert "rebuild_verified_partial_online_stack" in launcher
    assert "Refusing partial-stack rebuild while a live command path exists." in (
        launcher
    )
    assert "Refusing unrecognized partial-stack window" in launcher
    assert "Rebuilding verified partial online stack" in launcher
    rebuild = launcher.split(
        "rebuild_verified_partial_online_stack() {", 1
    )[1].split("\n}\n", 1)[0]
    assert "/nav/paused" in rebuild
    assert "/focus_guarded_cmd_vel" in rebuild
    assert "go2-bridge" in rebuild
    assert "--enable-live-go2-motion" in rebuild
    assert "verified_online_window" in rebuild
    assert 'tmux kill-window -t "$SESSION:$window"' in rebuild
    assert "start_tinynav_buildmap_online_nav.sh" in rebuild


def test_wsj_live_bridge_uses_observed_effective_command_floors() -> None:
    launcher = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text(
        encoding="utf-8"
    )

    assert "GO2_MAX_VX=0.20" in launcher
    assert "GO2_MAX_WZ=0.50" in launcher
    assert 'FOCUS_WSJ_LINEAR_COMMAND_FLOOR_MPS:-0.18' in launcher
    assert 'GO2_MIN_CMD_V=\\"$LINEAR_COMMAND_FLOOR_MPS\\"' in launcher
    assert '--linear-command-floor-mps \\"$LINEAR_COMMAND_FLOOR_MPS\\"' in launcher
    assert "GO2_MIN_CMD_W=0.30" in launcher
    assert "GO2_SEND_ZERO_WHEN_IDLE=false" in launcher
    assert "Move(0)+StopMove" in launcher
    assert '--start-snap-radius-m "$START_SNAP_RADIUS_M"' in launcher
    assert (
        '--start-footprint-override-m "$START_FOOTPRINT_OVERRIDE_M"'
        in launcher
    )
    assert 'FOCUS_WSJ_START_FOOTPRINT_OVERRIDE_M:-0.35' in launcher


def test_wsj_stale_occupancy_recovery_is_publisher_last_and_no_bridge() -> None:
    launcher = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text(
        encoding="utf-8"
    )
    recovery = launcher.split("recover_online_map_publisher() {", 1)[1]
    recovery = recovery.split("\n}\n", 1)[0]

    assert "go2-bridge" in recovery
    assert "Refusing online-map recovery while a live command path exists." in (
        recovery
    )
    assert "/nav/paused" in recovery
    assert "/focus_guarded_cmd_vel" in recovery
    assert 'tmux send-keys -t "$SESSION:online-map" C-c' in recovery
    assert 'tmux respawn-pane -t "$SESSION:online-map"' in recovery
    assert 'tmux respawn-pane -k -t "$SESSION:online-map"' not in recovery
    assert (
        recovery.index("C-c")
        < recovery.index("respawn-pane")
        < recovery.index("occupancy_mapper_node")
    )
    assert (
        'mark_component_contract online-map "$online_map_contract"'
        in recovery
    )
    assert "continuous-depth-online-map-v1" in launcher
    assert (
        'component_contract_matches \\\n'
        '    online-map "$online_map_contract"'
        in launcher
    )
    assert (
        "WSJ warm online-map does not match the verified "
        "deployment contract."
        in launcher
    )
    fast_probe = launcher.index(
        '"${sensor_map_verifier[@]}" --timeout-s 8'
    )
    recovery_call = launcher.index(
        "\n  recover_online_map_publisher\n",
        fast_probe,
    )
    full_probe = launcher.index(
        '"${sensor_map_verifier[@]}" --timeout-s 35'
    )
    assert fast_probe < recovery_call < full_probe


def test_wsj_launcher_reloads_persistent_goal_router_before_receiver() -> None:
    launcher = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text(
        encoding="utf-8"
    )

    reload_index = launcher.index(
        'tmux respawn-pane -k -t "$SESSION:goal-router"'
    )
    controller_reload_index = launcher.index(
        'tmux respawn-pane -t "$SESSION:control"'
    )
    receiver_index = launcher.index(
        'tmux new-window -d -t "$SESSION" -n v2-receiver'
    )
    bridge_index = launcher.index(
        'tmux new-window -d -t "$SESSION" -n go2-bridge'
    )
    assert (
        controller_reload_index
        < reload_index
        < receiver_index
        < bridge_index
    )
    assert (
        "WSJ velocity controller reloaded from the current deployment"
        in launcher
    )
    assert 'tmux send-keys -t "$SESSION:control" C-c' in launcher
    assert 'tmux respawn-pane -k -t "$SESSION:control"' not in launcher
    assert "yunji_tinynav_cmd_vel_control.py" in launcher
    assert "--reject-stalled-turn" in launcher
    assert "WSJ goal-router reloaded from the current deployment" in launcher
    assert '--start-snap-radius-m \\"$START_SNAP_RADIUS_M\\"' in launcher
    assert (
        '--start-footprint-override-m \\"$START_FOOTPRINT_OVERRIDE_M\\"'
        in launcher
    )
    assert '--input-timeout-s \\"$ODOMETRY_INPUT_TIMEOUT_S\\"' in launcher
    assert (
        '--semantic-terminal-planning-margin-m '
        '\\"$SEMANTIC_TERMINAL_PLANNING_MARGIN_M\\"'
        in launcher
    )


def test_yunji_active_launcher_uses_tinynav_and_guarded_joy_not_native_maps():
    launcher = (OVERLAY / "start_yunji_v2.sh").read_text(encoding="utf-8")
    component = (OVERLAY / "run_yunji_tinynav_component.sh").read_text(
        encoding="utf-8"
    )
    bridge = (OVERLAY / "water_cmd_vel_bridge.py").read_text(encoding="utf-8")

    assert "odin1_tinynav_adapter.py" in launcher
    assert "run_yunji_tinynav_planner.py" in component
    assert "cmd_vel_control.py" in component
    assert "--external-odometry-health" in launcher
    assert "--enable-live-tinynav-motion" in launcher
    assert "--reject-reverse-trajectory" in launcher
    assert "--reject-stalled-turn" in launcher
    assert "--rotate-first-on-reverse" in launcher
    assert "--stabilize-large-turn" in launcher
    assert "--verified-forward-only-planner" in launcher
    assert "--rotate-first-max-angular-radps" in launcher
    assert "--rotate-first-timeout-s" in launcher
    assert "-p keyframe.pose_jump_translation_m:=1.0" in launcher
    assert "-p keyframe.pose_jump_rotation_deg:=90.0" in launcher
    assert "-p keyframe.pause_frames_after_jump:=0" in launcher
    assert "-p integration.max_rays_per_frame:=3000" in launcher
    assert "navigation_occupancy_mapper.py" in component
    assert "FOCUS_YUNJI_REVERSE_ROTATE_MAX_ANGULAR_RADPS:-0.35" in launcher
    assert "FOCUS_YUNJI_REVERSE_ROTATE_TIMEOUT_S:-12.0" in launcher
    assert "/focus_guarded_cmd_vel" in launcher
    assert "/api/joy_control" in bridge
    assert "/api/accessible_point_query" not in launcher + bridge
    assert '"/api/move"' not in launcher + bridge
    assert "verify_tinynav_data_plane.py" in launcher
    assert 'FOCUS_YUNJI_START_SNAP_RADIUS_M:-1.0' in launcher
    assert 'FOCUS_YUNJI_ODOMETRY_INPUT_TIMEOUT_S:-2.0' in launcher
    assert '--start-snap-radius-m "$START_SNAP_RADIUS_M"' in launcher
    assert '--input-timeout-s "$ODOMETRY_INPUT_TIMEOUT_S"' in launcher
    assert launcher.count(
        '--start-footprint-override-m "$START_FOOTPRINT_OVERRIDE_M"'
    ) == 3
    assert "--reuse-verified-debug-core" in launcher
    assert "without process restarts" in launcher
    assert 'systemctl is-active --quiet "$unit"' in launcher
    assert "unit_matches_core_contract" in launcher
    reuse_branch = launcher[
        launcher.index('if [[ "$reuse_verified_debug_core" == true ]]')
        : launcher.index(
            "else",
            launcher.index('if [[ "$reuse_verified_debug_core" == true ]]'),
        )
    ]
    assert "start_router" not in reuse_branch
    assert "start_controller" not in reuse_branch
    assert "unit_matches_core_contract" in reuse_branch
    assert '--setenv="OPENBLAS_NUM_THREADS=1"' in launcher
    assert '--setenv="OMP_NUM_THREADS=1"' in launcher
    assert '--setenv="MKL_NUM_THREADS=1"' in launcher
    assert '--setenv="NUMEXPR_NUM_THREADS=1"' in launcher


def test_robot_launchers_require_live_data_plane_verification():
    wsj = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text(
        encoding="utf-8"
    )
    yunji = (OVERLAY / "start_yunji_v2.sh").read_text(encoding="utf-8")
    verifier = (OVERLAY / "verify_tinynav_data_plane.py").read_text(
        encoding="utf-8"
    )

    assert "verify_tinynav_data_plane.py" in wsj
    assert "verify_tinynav_data_plane.py" in yunji
    assert "robot_commands_issued" in verifier
    assert "focus-tinynav-data-plane-verification-v1" in verifier
    assert "get_publishers_info_by_topic" in verifier
    assert "get_subscriptions_info_by_topic" in verifier
    assert "--fresh-image-topic" not in wsj
    assert "--fresh-image-topic" in yunji
    assert (
        "--fresh-camera-info-topic /camera/camera/color/camera_info"
        in wsj
    )
    assert "--fresh-camera-info-topic /slam/camera_info" in wsj
    assert "--geometry-image-topic /slam/depth" not in wsj
    assert "--camera-info-topic /slam/camera_info" in wsj
    assert "--geometry-width 848" in wsj
    assert "--geometry-height 480" in wsj
    assert "--require-reachable-start" in wsj
    assert "--require-reachable-start" in yunji
    assert "--minimum-occupancy-updates 2" in wsj
    assert "--minimum-occupancy-updates 2" in yunji
    assert "--maximum-occupancy-update-interval-s 4.0" in wsj
    assert "--maximum-occupancy-update-interval-s 4.0" in yunji
    assert "--odom-topic /slam/odometry_visual" in wsj
    assert "--max-occupancy-age-s 12" in wsj
    assert (
        '--max-cached-occupancy-motion-m "$MAX_CACHED_MAP_MOTION_M"'
        in wsj
    )
    assert "--geometry-image-topic /slam/depth" in yunji
    assert "--camera-info-topic /slam/camera_info" in yunji
    assert "--max-occupancy-age-s 12" in yunji
    assert "--fresh-image-topic /slam/keyframe_depth" not in wsj
    assert "fresh_topic_once" not in wsj
    assert "No full-resolution Image subscriber is created" in wsj
    assert "ReliabilityPolicy.BEST_EFFORT" in verifier
    assert "DurabilityPolicy.VOLATILE" in verifier
    assert "depth=1" in verifier
    assert 'FOCUS_WSJ_SLAM_DATA_TIMEOUT_S:-3.0' in wsj
    assert 'FOCUS_WSJ_MAP_TIMEOUT_S:-12.0' in wsj
    assert '--map-timeout-s \\"$MAP_TIMEOUT_S\\"' in wsj
    assert 'FOCUS_WSJ_ODOMETRY_INPUT_TIMEOUT_S:-3.0' in wsj
    assert 'FOCUS_WSJ_RECEIVER_LOCAL_DATA_TIMEOUT_S:-5.0' in wsj
    assert 'FOCUS_WSJ_RECEIVER_ODOMETRY_RECOVERY_GRACE_S:-7.0' in wsj
    assert 'FOCUS_WSJ_TRAJECTORY_START_GRACE_S:-12.0' in wsj
    assert 'FOCUS_WSJ_TRAJECTORY_STALE_TIMEOUT_S:-1.0' in wsj
    assert 'FOCUS_WSJ_TRAJECTORY_RECOVERY_TIMEOUT_S:-12.0' in wsj
    assert (
        '--local-data-timeout-s "$RECEIVER_LOCAL_DATA_TIMEOUT_S"'
        in wsj
    )
    assert (
        '--odometry-recovery-grace-s '
        '"$RECEIVER_ODOMETRY_RECOVERY_GRACE_S"'
        in wsj
    )
    assert (
        '--trajectory-start-grace-s "$TRAJECTORY_START_GRACE_S"'
        in wsj
    )
    assert (
        '--trajectory-stale-timeout-s "$TRAJECTORY_STALE_TIMEOUT_S"'
        in wsj
    )
    assert (
        '--trajectory-recovery-timeout-s '
        '"$TRAJECTORY_RECOVERY_TIMEOUT_S"'
        in wsj
    )
    assert 'FOCUS_YUNJI_TRAJECTORY_START_GRACE_S:-12.0' in yunji
    assert 'FOCUS_YUNJI_TRAJECTORY_STALE_TIMEOUT_S:-1.0' in yunji
    assert 'FOCUS_YUNJI_TRAJECTORY_RECOVERY_TIMEOUT_S:-12.0' in yunji
    assert (
        '--trajectory-start-grace-s "$TRAJECTORY_START_GRACE_S"'
        in yunji
    )
    assert (
        '--trajectory-stale-timeout-s "$TRAJECTORY_STALE_TIMEOUT_S"'
        in yunji
    )
    assert (
        '--trajectory-recovery-timeout-s '
        '"$TRAJECTORY_RECOVERY_TIMEOUT_S"'
        in yunji
    )
    assert '--slam-data-timeout-s "$SLAM_DATA_TIMEOUT_S"' in wsj
    sensor_verification = wsj[
        wsj.index(
            '"$PYTHON_BIN" -u "$SCRIPT_DIR/verify_tinynav_data_plane.py"'
        )
        : wsj.index('bash "$SCRIPT_DIR/start_wsj_command_observation.sh"')
    ]
    assert "/camera/camera/color/image_raw" not in sensor_verification
    assert "--fresh-image-topic /slam/depth" not in sensor_verification
    assert "--geometry-image-topic /slam/depth" not in sensor_verification
    assert "/slam/keyframe_depth" not in sensor_verification
    assert "/slam/keyframe_odom" not in sensor_verification
    assert "validates geometry, occupancy and router state" in wsj
    assert "fail_closed_on_error" in wsj
    assert "fail_closed_on_error" in yunji
    assert "focus-yunji-water-bridge-live-v1.service" in yunji
    assert 'FOCUS_YUNJI_MAP_TIMEOUT_S:-12.0' in yunji
    assert 'FOCUS_YUNJI_REACHABILITY_CLEARANCE_M:-0.30' in yunji
    assert '--map-timeout-s "$MAP_TIMEOUT_S"' in yunji
    assert 'tmux kill-window -t "$SESSION:go2-bridge"' in wsj


def test_wsj_calibration_recovers_the_sensor_epoch_before_board_capture():
    launcher = (
        OVERLAY / "start_wsj_calibration_observation.sh"
    ).read_text(encoding="utf-8")
    persistent_sender = launcher.index(
        'bash "$SCRIPT_DIR/start_wsj_command_observation.sh"'
    )
    calibration_sender = launcher.index("sender=(")
    subscribers_ready = launcher.index(
        "WSJ_DDS_SUBSCRIBERS_READY_BEFORE_PUBLISHERS"
    )
    stop_perception = launcher.index(
        'tmux set-option -w -t "$SESSION:perception" remain-on-exit on'
    )
    restart_camera = launcher.index(
        'tmux respawn-pane -k -t "$SESSION:camera"'
    )
    restart_perception = launcher.index(
        'tmux respawn-pane -k -t "$SESSION:perception"'
    )
    persistent_tuple = launcher.index(
        'wait_for_persistent_sender_tuple "$parked_tuple_baseline"'
    )
    sensor_ready = launcher.index("WSJ_MAPPING_ONLY_SENSOR_EPOCH_READY")
    continuous_sensor_gate = launcher[restart_perception:sensor_ready]

    assert 'tmux respawn-pane -k -t "$SESSION:camera"' in launcher
    assert 'tmux respawn-pane -k -t "$SESSION:perception"' in launcher
    assert "/slam/depth" in continuous_sensor_gate
    assert "/slam/odometry_visual" in continuous_sensor_gate
    assert "/slam/keyframe_depth" not in continuous_sensor_gate
    assert "/slam/keyframe_odom" not in continuous_sensor_gate
    assert "--depth-topic /slam/depth" in launcher
    assert "--pose-topic /slam/odometry_visual" in launcher
    assert "--depth-topic /slam/keyframe_depth" not in launcher
    assert "--pose-topic /slam/keyframe_odom" not in launcher
    assert "latest_sequence > baseline" in launcher
    assert 'calibration_epoch_baseline="$(latest_hub_sequence)"' in launcher
    assert "continuous_tuple_gate=sender_sequence" in launcher
    assert launcher.count("verify_ros_geometry_profile.py") >= 2
    assert "--image-topic /slam/depth" in launcher
    assert "--camera-info-topic /slam/camera_info" in launcher
    assert "stable TinyNav processed depth" in launcher
    assert "WSJ_MAPPING_ONLY_SENSOR_EPOCH_READY" in launcher
    assert '"purpose=$OBSERVATION_PURPOSE"' in launcher
    assert "--field header" in launcher
    assert "--qos-reliability best_effort" in launcher
    assert "--qos-durability volatile" in launcher
    assert "--qos-depth 1" in launcher
    assert (
        persistent_sender
        < calibration_sender
        < subscribers_ready
        < stop_perception
        < restart_camera
        < restart_perception
        < persistent_tuple
        < sensor_ready
    )
    assert "Persistent WSJ sender PID changed during publisher restart" in launcher


def test_yunji_direct_water_map_receiver_is_retained_as_legacy_only():
    legacy = (OVERLAY / "v2_yunji_receiver.py").read_text(encoding="utf-8")
    launcher = (OVERLAY / "start_yunji_v2.sh").read_text(encoding="utf-8")

    assert '"/api/move"' in legacy
    assert "v2_yunji_receiver.py" not in launcher


def test_yunji_old_firmware_capability_is_parsed_fail_closed():
    yunji = load_overlay("v2_yunji_receiver.py")

    assert yunji.water_version_tuple("0.3.179.2A") == (0, 3, 179, 2)
    assert yunji.water_version_tuple("0.10.7") >= yunji.ACCESSIBLE_POINT_MIN_VERSION
    assert yunji.water_version_tuple("unknown") == ()


def test_yunji_legacy_receding_horizon_bounds_each_native_goal():
    yunji = load_overlay("v2_yunji_receiver.py")
    final = yunji.LocalHighLevelGoal(
        frame_id="yunji/water_map",
        x=2.0,
        y=0.0,
        z=0.0,
        yaw_rad=1.2,
        target_kind="FRONTIER_POINT",
        arrival_radius_m=0.5,
    )

    first = yunji.bounded_legacy_subgoal((0.0, 0.0, 0.0), final, step_m=0.45)
    assert math.hypot(first.x, first.y) == pytest.approx(0.45)
    assert first.yaw_rad == pytest.approx(0.0)
    assert first.arrival_radius_m is None

    last = yunji.bounded_legacy_subgoal((1.7, 0.0, 0.0), final, step_m=0.45)
    assert last == final
    assert yunji.local_goal_arrival_radius(final) == pytest.approx(0.5)


def test_yunji_legacy_mode_retains_final_goal_and_checks_segment_progress():
    source = (OVERLAY / "v2_yunji_receiver.py").read_text(encoding="utf-8")
    assert "water-legacy-receding-horizon-v1" in source
    assert "lease renewal preserves the original local final goal" in source
    assert "legacy_segment_continuation" in source
    assert "legacy_firmware_min_segment_progress_m" in source
    assert "--legacy-firmware-subgoal-step-m" in source


def test_observation_senders_need_measured_mount_and_explicit_activation():
    for filename in ("focus_ros_sender.py", "odin1_sender.py"):
        source = (OVERLAY / filename).read_text(encoding="utf-8")
        assert "--enable-command-capable-observations" in source
        assert "--base-camera-calibration-file" in source
        assert "COMMAND_CAPABLE_OBSERVATION_ONLY" in source
        assert "the armed v2 receiver owns command health heartbeats" in source
