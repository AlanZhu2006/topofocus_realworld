from __future__ import annotations

import importlib.util
import json
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest
import numpy as np


def _load_sender_module(monkeypatch):
    """Import the robot-only sender without requiring ROS 2 on the hub."""

    def module(name: str, **attributes):
        value = types.ModuleType(name)
        for key, attribute in attributes.items():
            setattr(value, key, attribute)
        monkeypatch.setitem(sys.modules, name, value)
        return value

    module("cv2")
    module("cv_bridge", CvBridge=object)
    module("message_filters")
    module("rclpy")
    module("rclpy.node", Node=object)
    module("rclpy.qos", qos_profile_sensor_data=object())
    module("nav_msgs")
    module("nav_msgs.msg", Odometry=object)
    module("sensor_msgs")
    module("sensor_msgs.msg", CameraInfo=object, Image=object)
    module("std_msgs")
    module("std_msgs.msg", String=object)

    path = Path(__file__).resolve().parents[1] / "robot_overlay" / "focus_ros_sender.py"
    spec = importlib.util.spec_from_file_location("focus_ros_sender_health_under_test", path)
    assert spec is not None and spec.loader is not None
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def _odom(*, position=(1.0, 2.0, 3.0), quaternion=(0.0, 0.0, 0.0, 1.0)):
    p = SimpleNamespace(x=position[0], y=position[1], z=position[2])
    q = SimpleNamespace(
        x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=p, orientation=q)))


def _covariance(diagonal: float) -> list[float]:
    covariance = [0.0] * 36
    for index in (0, 7, 14, 21, 28, 35):
        covariance[index] = diagonal
    return covariance


class _DepthBridge:
    def __init__(self, array):
        self.array = array
        self.requested_encoding = None

    def imgmsg_to_cv2(self, _message, *, desired_encoding):
        self.requested_encoding = desired_encoding
        return self.array


def test_uint16_depth_is_preserved_for_hardware_path(monkeypatch):
    sender = _load_sender_module(monkeypatch)
    source = np.array([[0, 300, 1234, 65535]], dtype=np.uint16)
    bridge = _DepthBridge(source)

    converted = sender.depth_msg_to_png16_array(
        bridge, SimpleNamespace(encoding="16UC1"))

    assert bridge.requested_encoding == "16UC1"
    assert converted.dtype == np.uint16
    assert converted.flags.c_contiguous
    np.testing.assert_array_equal(converted, source)


def test_float_metre_depth_converts_to_png16_units(monkeypatch):
    sender = _load_sender_module(monkeypatch)
    source = np.array(
        [[0.0, 0.3, 1.2344, 1.2346, np.nan, np.inf, -1.0, 70.0]],
        dtype=np.float32,
    )
    bridge = _DepthBridge(source)

    converted = sender.depth_msg_to_png16_array(
        bridge, SimpleNamespace(encoding="32FC1"))

    assert bridge.requested_encoding == "32FC1"
    np.testing.assert_array_equal(
        converted,
        np.array([[0, 300, 1234, 1235, 0, 0, 0, 65535]], dtype=np.uint16),
    )


def test_depth_conversion_rejects_unknown_encoding_or_shape(monkeypatch):
    sender = _load_sender_module(monkeypatch)

    with pytest.raises(ValueError, match="unsupported depth encoding"):
        sender.depth_msg_to_png16_array(
            _DepthBridge(np.zeros((2, 2), dtype=np.uint16)),
            SimpleNamespace(encoding="mono16"),
        )
    with pytest.raises(ValueError, match="must be 2-D"):
        sender.depth_msg_to_png16_array(
            _DepthBridge(np.zeros((2, 2, 1), dtype=np.float32)),
            SimpleNamespace(encoding="32FC1"),
        )


def test_rgb_registration_identity_preserves_valid_depth_pixels(monkeypatch):
    sender = _load_sender_module(monkeypatch)
    rgb = np.arange(3 * 3 * 3, dtype=np.uint8).reshape(3, 3, 3)
    depth = np.ones((3, 3), dtype=np.float32)
    depth[0, 0] = 0.0
    intrinsics = np.array(
        [[2.0, 0.0, 1.0], [0.0, 2.0, 1.0], [0.0, 0.0, 1.0]]
    )

    registered, registered_depth, coverage = sender.register_rgb_onto_depth_grid(
        rgb, depth, intrinsics, intrinsics, np.eye(4)
    )

    np.testing.assert_array_equal(registered, rgb)
    np.testing.assert_array_equal(registered_depth, depth > 0.0)
    assert coverage == pytest.approx(1.0)


def test_rgb_registration_uses_static_extrinsic_for_valid_depth(monkeypatch):
    sender = _load_sender_module(monkeypatch)
    rgb = np.zeros((3, 5, 3), dtype=np.uint8)
    for column in range(rgb.shape[1]):
        rgb[:, column] = column * 10
    depth = np.ones((3, 5), dtype=np.float32)
    intrinsics = np.array(
        [[1.0, 0.0, 2.0], [0.0, 1.0, 1.0], [0.0, 0.0, 1.0]]
    )
    # At z=1 m, +1 m in color X moves the projected sample one pixel right.
    T_rgb_from_depth = np.eye(4)
    T_rgb_from_depth[0, 3] = 1.0

    registered, registered_depth, coverage = sender.register_rgb_onto_depth_grid(
        rgb, depth, intrinsics, intrinsics, T_rgb_from_depth
    )

    np.testing.assert_array_equal(registered[:, :4], rgb[:, 1:])
    assert np.count_nonzero(registered_depth) == 12
    assert coverage == pytest.approx(0.8)


def _slam_payload(
    *,
    initial=1.0,
    final=0.5,
    factors=4,
    variables=6,
    status="ok",
    intervals_valid=True,
    coverage=1.0,
    max_gap=0.005,
    end_error=0.0,
    overwritten=0,
) -> str:
    return json.dumps({
        "stats": {
            "optimizer_status": status,
            "imu_messages_overwritten": overwritten,
        },
        "metrics": {
            "initial_error": initial,
            "final_error": final,
            "num_factors": factors,
            "num_variables": variables,
            "imu_intervals_valid": intervals_valid,
            "imu_intervals": [{
                "duration_s": 0.5,
                "sample_count": 100,
                "expected_count": 100,
                "coverage_ratio": coverage,
                "max_sample_gap_s": max_gap,
                "end_error_s": end_error,
                "valid": intervals_valid,
            }],
        }
    })


def test_zero_covariance_is_unknown_not_tracking(monkeypatch):
    sender = _load_sender_module(monkeypatch)

    state, wire = sender.classify_localization_state([0.0] * 36)

    assert state == "UNKNOWN"
    assert wire == [0.0] * 36


def test_real_finite_covariance_preserves_existing_thresholds(monkeypatch):
    sender = _load_sender_module(monkeypatch)

    assert sender.classify_localization_state(_covariance(0.001))[0] == "TRACKING"
    assert sender.classify_localization_state(_covariance(0.1))[0] == "DEGRADED"
    assert sender.classify_localization_state(_covariance(2.0))[0] == "LOST"


def test_nonfinite_or_negative_covariance_is_unknown(monkeypatch):
    sender = _load_sender_module(monkeypatch)
    nonfinite = _covariance(0.01)
    nonfinite[7] = float("inf")
    negative = _covariance(0.01)
    negative[35] = -0.1

    assert sender.classify_localization_state(nonfinite) == ("UNKNOWN", [0.0] * 36)
    assert sender.classify_localization_state(negative) == ("UNKNOWN", [0.0] * 36)


def test_odometry_rejects_invalid_pose_instead_of_fabricating_identity(monkeypatch):
    sender = _load_sender_module(monkeypatch)

    with pytest.raises(ValueError, match="zero norm"):
        sender.odom_to_matrix(_odom(quaternion=(0.0, 0.0, 0.0, 0.0)))
    with pytest.raises(ValueError, match="non-finite"):
        sender.odom_to_matrix(_odom(position=(float("nan"), 0.0, 0.0)))


def test_shared_tracking_alignment_is_applied_before_upload(monkeypatch):
    sender = _load_sender_module(monkeypatch)
    tracking_T_camera = sender.odom_to_matrix(
        _odom(position=(1.0, 0.0, 0.0))
    )
    shared_T_tracking = [
        0.0, -1.0, 0.0, 2.0,
        1.0, 0.0, 0.0, 3.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    ]

    result = np.asarray(sender.apply_shared_tracking_alignment(
        tracking_T_camera, shared_T_tracking
    )).reshape(4, 4)

    np.testing.assert_allclose(result[:3, 3], [2.0, 4.0, 0.0])
    np.testing.assert_allclose(result[:3, :3], np.asarray(
        shared_T_tracking
    ).reshape(4, 4)[:3, :3])


def test_shared_tracking_alignment_rejects_bad_shape(monkeypatch):
    sender = _load_sender_module(monkeypatch)

    with pytest.raises(ValueError, match="two 4x4"):
        sender.apply_shared_tracking_alignment([1.0], [1.0] * 16)


def test_complete_slam_health_is_degraded_without_covariance(monkeypatch):
    sender = _load_sender_module(monkeypatch)
    metrics = sender.LatestSlamMetrics()
    metrics.update(_slam_payload(), received_monotonic=10.0)

    state, detail = metrics.apply("UNKNOWN", timeout_s=5.0, now_monotonic=11.0)

    assert state == "DEGRADED"
    assert detail == "slam_optimizer_imu_valid;covariance_unavailable"


def test_complete_slam_health_preserves_real_tracking_covariance(monkeypatch):
    sender = _load_sender_module(monkeypatch)
    metrics = sender.LatestSlamMetrics()
    metrics.update(_slam_payload(), received_monotonic=10.0)

    state, detail = metrics.apply("TRACKING", timeout_s=5.0, now_monotonic=11.0)

    assert state == "TRACKING"
    assert detail == "slam_optimizer_imu_valid"


def test_slam_health_recovers_only_after_stable_valid_overwrite_count(
    monkeypatch,
):
    sender = _load_sender_module(monkeypatch)
    metrics = sender.LatestSlamMetrics()
    payload = _slam_payload(overwritten=17)

    metrics.update(payload, received_monotonic=10.0)
    assert metrics.apply(
        "UNKNOWN", timeout_s=5.0, now_monotonic=10.1
    )[0] == "LOST"
    metrics.update(payload, received_monotonic=11.0)
    assert metrics.apply(
        "UNKNOWN", timeout_s=5.0, now_monotonic=11.1
    )[0] == "UNKNOWN"
    metrics.update(payload, received_monotonic=12.1)
    state, detail = metrics.apply(
        "UNKNOWN", timeout_s=5.0, now_monotonic=12.2
    )
    assert state == "DEGRADED"
    assert detail == (
        "slam_optimizer_imu_valid_after_overwrite_recovery:"
        "17;covariance_unavailable"
    )

    metrics.update(_slam_payload(overwritten=18), received_monotonic=13.0)
    assert metrics.apply(
        "UNKNOWN", timeout_s=5.0, now_monotonic=13.1
    )[0] == "LOST"


def test_nonfinite_optimizer_forces_lost(monkeypatch):
    sender = _load_sender_module(monkeypatch)
    metrics = sender.LatestSlamMetrics()
    metrics.update(_slam_payload(initial=float("inf"), final=float("inf")),
                   received_monotonic=10.0)

    state, detail = metrics.apply("TRACKING", timeout_s=5.0, now_monotonic=11.0)

    assert state == "LOST"
    assert detail == "slam_optimizer_nonfinite"


@pytest.mark.parametrize(
    ("payload", "detail"),
    [
        (_slam_payload(intervals_valid=False), "slam_imu_intervals_invalid"),
        (_slam_payload(coverage=0.2), "slam_imu_intervals_invalid"),
        (_slam_payload(max_gap=0.2), "slam_imu_intervals_invalid"),
        (_slam_payload(end_error=0.2), "slam_imu_intervals_invalid"),
        (_slam_payload(overwritten=1), "slam_imu_buffer_overwritten:1"),
        (_slam_payload(initial=0.5, final=1.0), "slam_optimizer_worsened"),
    ],
)
def test_slam_fault_telemetry_forces_lost(monkeypatch, payload, detail):
    sender = _load_sender_module(monkeypatch)
    metrics = sender.LatestSlamMetrics()
    metrics.update(payload, received_monotonic=10.0)

    assert metrics.apply("TRACKING", timeout_s=5.0, now_monotonic=11.0) == (
        "LOST", detail)


def test_warmup_status_stays_unknown(monkeypatch):
    sender = _load_sender_module(monkeypatch)
    metrics = sender.LatestSlamMetrics()
    metrics.update(_slam_payload(status="warmup_complete"), received_monotonic=10.0)

    assert metrics.apply("TRACKING", timeout_s=5.0, now_monotonic=11.0) == (
        "UNKNOWN", "slam_warmup_complete")


def test_missing_stale_or_empty_optimizer_report_fails_closed(monkeypatch):
    sender = _load_sender_module(monkeypatch)
    metrics = sender.LatestSlamMetrics()
    assert metrics.apply("TRACKING", timeout_s=5.0, now_monotonic=1.0) == (
        "UNKNOWN", "slam_metrics_missing")

    metrics.update(_slam_payload(), received_monotonic=10.0)
    state, detail = metrics.apply("TRACKING", timeout_s=5.0, now_monotonic=16.0)
    assert state == "UNKNOWN"
    assert detail == "slam_metrics_stale:6.0s"

    metrics.update(_slam_payload(factors=0, variables=0), received_monotonic=20.0)
    assert metrics.apply("TRACKING", timeout_s=5.0, now_monotonic=21.0) == (
        "UNKNOWN", "slam_graph_empty")
