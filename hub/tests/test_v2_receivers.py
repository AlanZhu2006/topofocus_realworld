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
) -> str:
    import json

    return json.dumps({
        "stats": {"optimizer_status": "ok", "imu_messages_overwritten": 0},
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


def test_yunji_uses_high_level_move_cancel_and_reachability_not_joy_control():
    source = (OVERLAY / "v2_yunji_receiver.py").read_text(encoding="utf-8")
    assert '"/api/move"' in source
    assert '"/api/move/cancel"' in source
    assert '"/api/map/accessible_point_query"' in source
    assert '"/api/software/get_version"' in source
    assert "WATER move_base/local planner/controller" in source
    assert "/api/joy_control" not in source


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
