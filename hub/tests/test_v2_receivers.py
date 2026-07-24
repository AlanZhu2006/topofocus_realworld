from __future__ import annotations

import ast
import importlib.util
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


def test_data_plane_verifier_rejects_stale_or_mismatched_geometry():
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
    assert verifier.message_age_s(image, now_ns=12_000_000_000) == pytest.approx(
        2.0
    )

    camera_info.width = 848
    with pytest.raises(ValueError, match="dimensions differ"):
        verifier.validate_geometry_contract(
            image, camera_info, expected_frame="camera"
        )


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
    assert "Refusing ambiguous partial online stack" in launcher


def test_wsj_live_bridge_uses_observed_effective_command_floors() -> None:
    launcher = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text(
        encoding="utf-8"
    )

    assert "GO2_MAX_VX=0.20" in launcher
    assert "GO2_MAX_WZ=0.50" in launcher
    assert "GO2_MIN_CMD_V=0.15" in launcher
    assert "GO2_MIN_CMD_W=0.30" in launcher
    assert "--start-snap-radius-m 0.75" in launcher
    assert "--start-footprint-override-m 0.35" in launcher


def test_wsj_launcher_reloads_persistent_goal_router_before_receiver() -> None:
    launcher = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text(
        encoding="utf-8"
    )

    reload_index = launcher.index(
        'tmux respawn-pane -k -t "$SESSION:goal-router"'
    )
    receiver_index = launcher.index(
        'tmux new-window -d -t "$SESSION" -n v2-receiver'
    )
    bridge_index = launcher.index(
        'tmux new-window -d -t "$SESSION" -n go2-bridge'
    )
    assert reload_index < receiver_index < bridge_index
    assert "WSJ goal-router reloaded from the current deployment" in launcher
    assert '--start-snap-radius-m \\"$START_SNAP_RADIUS_M\\"' in launcher
    assert (
        '--start-footprint-override-m \\"$START_FOOTPRINT_OVERRIDE_M\\"'
        in launcher
    )
    assert '--input-timeout-s \\"$ODOMETRY_INPUT_TIMEOUT_S\\"' in launcher


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
    ) == 2
    assert "--reuse-verified-debug-core" in launcher
    assert "without interrupting /slam/depth" in launcher
    assert 'systemctl is-active --quiet "$unit"' in launcher
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
    assert "--fresh-image-topic" in wsj
    assert "--fresh-image-topic" in yunji
    assert "--fresh-image-topic /slam/depth" in wsj
    assert "--geometry-image-topic /slam/depth" in wsj
    assert "--camera-info-topic /slam/camera_info" in wsj
    assert "--max-occupancy-age-s 12" in wsj
    assert "--geometry-image-topic /slam/depth" in yunji
    assert "--camera-info-topic /slam/camera_info" in yunji
    assert "--max-occupancy-age-s 12" in yunji
    assert "--fresh-image-topic /slam/keyframe_depth" not in wsj
    assert "WSJ calibrated sensor epoch is stale" in wsj
    assert "Refusing to restart camera/perception after calibration" in wsj
    assert "--field header" in wsj
    assert "--qos-reliability best_effort" in wsj
    assert "--qos-durability volatile" in wsj
    assert "--qos-depth 1" in wsj
    assert 'FOCUS_WSJ_SLAM_DATA_TIMEOUT_S:-3.0' in wsj
    assert '--local-data-timeout-s "$ODOMETRY_INPUT_TIMEOUT_S"' in wsj
    assert '--slam-data-timeout-s "$SLAM_DATA_TIMEOUT_S"' in wsj
    continuous_stream_loop = wsj[
        wsj.index("for topic in \\\n  /camera/camera/color/image_raw")
        : wsj.index('bash "$SCRIPT_DIR/start_wsj_command_observation.sh"')
    ]
    assert "/slam/keyframe_depth" not in continuous_stream_loop
    assert "/slam/keyframe_odom" not in continuous_stream_loop
    assert "strict map-freshness gate" in wsj
    assert "fail_closed_on_error" in wsj
    assert "fail_closed_on_error" in yunji
    assert "focus-yunji-water-bridge-live-v1.service" in yunji
    assert 'tmux kill-window -t "$SESSION:go2-bridge"' in wsj


def test_wsj_calibration_recovers_the_sensor_epoch_before_board_capture():
    launcher = (
        OVERLAY / "start_wsj_calibration_observation.sh"
    ).read_text(encoding="utf-8")

    assert 'tmux respawn-pane -k -t "$SESSION:camera"' in launcher
    assert 'tmux respawn-pane -k -t "$SESSION:perception"' in launcher
    assert "/slam/depth" in launcher
    assert "/slam/keyframe_depth" in launcher
    assert "/slam/keyframe_odom" in launcher
    assert launcher.count("verify_ros_geometry_profile.py") >= 3
    assert "--image-topic /slam/depth" in launcher
    assert "--camera-info-topic /slam/camera_info" in launcher
    assert "stable TinyNav processed depth" in launcher
    assert "WSJ_CALIBRATION_SENSOR_EPOCH_READY" in launcher
    assert "--field header" in launcher
    assert "--qos-reliability best_effort" in launcher
    assert "--qos-durability volatile" in launcher
    assert "--qos-depth 1" in launcher
    assert launcher.index("WSJ_CALIBRATION_SENSOR_EPOCH_READY") < launcher.index(
        "sender=("
    )


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
