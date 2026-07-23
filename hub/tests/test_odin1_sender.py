import importlib.util
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import subprocess
import sys

import numpy as np
import pytest


OVERLAY = Path(__file__).resolve().parents[1] / "robot_overlay"
TOOLS = Path(__file__).resolve().parents[1] / "tools"


def load_sender_module():
    sys.path.insert(0, str(OVERLAY))
    try:
        path = OVERLAY / "odin1_sender.py"
        spec = importlib.util.spec_from_file_location("focus_test_odin1_sender", path)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(OVERLAY))


def factory_calibration_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "config"
        / "calibration"
        / "odin1_O1-P070100205_factory_20260722.json"
    )


def shared_calibration_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "config"
        / "calibration"
        / "yunji_odin1_board_20260722_v1.json"
    )


def test_factory_calibration_loads_exact_device_contract():
    sender = load_sender_module()

    calibration = sender.load_odin_calibration(
        str(factory_calibration_path()), "O1-P070100205"
    )

    assert calibration.serial == "O1-P070100205"
    assert (calibration.width, calibration.height) == (1600, 1296)
    assert calibration.camera_frame == "odin1_camera_optical_frame"
    assert calibration.odometry_frame == "yunji_odin1_odom"
    assert np.isclose(np.linalg.det(calibration.T_imu_camera[:3, :3]), 1.0)
    with pytest.raises(ValueError, match="serial mismatch"):
        sender.load_odin_calibration(str(factory_calibration_path()), "wrong")


def test_old_realsense_shared_transform_is_rejected():
    sender = load_sender_module()
    old_path = (
        Path(__file__).resolve().parents[1]
        / "config"
        / "calibration"
        / "shared_board_gravity_20260722_v3.json"
    )

    with pytest.raises(ValueError, match="version mismatch"):
        sender.load_shared_transform(
            str(old_path), expected_transform_version="yunji-odin1-local-odom-20260722-v1"
        )


def test_current_odin_shared_transform_loads_with_exact_identity_and_provenance():
    sender = load_sender_module()

    matrix, calibration_id = sender.load_shared_transform(
        str(shared_calibration_path()),
        expected_transform_version="yunji-odin1-board-20260722-v1",
    )
    artifact = json.loads(shared_calibration_path().read_text())

    assert calibration_id == "shared-board-odin1-20260722-v1"
    assert matrix is not None
    assert np.linalg.det(matrix[:3, :3]) == pytest.approx(1.0)
    assert artifact["passed"] is True
    assert artifact["safety"]["robot_commands_issued"] is False
    assert artifact["holdout_validation"]["checks"] == {
        "board_center_residual": True,
        "board_normal_residual": True,
        "sync_skew": True,
    }


def _identity_calibration(sender):
    return sender.OdinCalibration(
        serial="test",
        width=8,
        height=6,
        fx=4.0,
        fy=4.0,
        cx=3.5,
        cy=2.5,
        skew=0.0,
        distortion=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        T_imu_camera=np.eye(4),
        camera_frame="camera",
        odometry_frame="odom",
    )


def test_projector_rectifies_identity_and_z_buffers_nearest_point():
    sender = load_sender_module()
    projector = sender.OdinProjector(
        _identity_calibration(sender),
        output_width=8,
        splat_radius=0,
        depth_min_m=0.1,
        depth_max_m=10.0,
    )
    raw = np.full((6, 8, 3), (10, 20, 30), dtype=np.uint8)
    rectified = projector.rectify(raw)
    # A constant input remains constant wherever the calibrated remap is valid.
    valid = rectified.any(axis=2)
    assert np.array_equal(rectified[valid], raw[valid])

    # Both points hit the principal pixel; the z-buffer must retain 1 m.
    points = np.array([[0.0, 0.0, 2.0], [0.0, 0.0, 1.0]])
    depth, diagnostics = projector.project(points, np.eye(4))

    assert np.count_nonzero(depth) == 1
    assert depth[2, 4] == pytest.approx(1.0)
    assert diagnostics["projected_points"] == 2
    assert diagnostics["depth_valid_pixels"] == 1


def test_projection_uses_odom_camera_pose():
    sender = load_sender_module()
    projector = sender.OdinProjector(
        _identity_calibration(sender), output_width=8, splat_radius=0
    )
    T_odom_camera = np.eye(4)
    T_odom_camera[0, 3] = 1.0
    # World x=1 is directly in front of a camera translated to world x=1.
    depth, _ = projector.project(np.array([[1.0, 0.0, 2.0]]), T_odom_camera)

    assert depth[2, 4] == pytest.approx(2.0)


def test_decode_cloud_reads_xyz_and_packed_bgr():
    sender = load_sender_module()
    dtype = np.dtype(
        {
            "names": ["x", "y", "z", "rgb"],
            "formats": ["<f4", "<f4", "<f4", "<u4"],
            "offsets": [0, 4, 8, 12],
            "itemsize": 16,
        }
    )
    records = np.zeros(2, dtype=dtype)
    records["x"] = [1.0, 2.0]
    records["y"] = [3.0, 4.0]
    records["z"] = [5.0, 6.0]
    records["rgb"] = [0x00112233, 0x00A0B0C0]
    fields = [
        SimpleNamespace(name=name, offset=offset)
        for name, offset in (("x", 0), ("y", 4), ("z", 8), ("rgb", 12))
    ]
    message = SimpleNamespace(
        fields=fields,
        is_bigendian=False,
        point_step=16,
        row_step=32,
        width=2,
        height=1,
        data=records.tobytes(),
    )

    xyz, bgr = sender.decode_cloud_xyz_bgr(message)

    assert np.array_equal(xyz, [[1.0, 3.0, 5.0], [2.0, 4.0, 6.0]])
    assert np.array_equal(bgr, [[0x33, 0x22, 0x11], [0xC0, 0xB0, 0xA0]])


def test_sync_gate_rejects_an_adjacent_odin_cycle_then_accepts_exact_stamp():
    sender = load_sender_module()
    source = object.__new__(sender.OdinRos2Source)
    target = 1_000_000_000
    cloud_message = SimpleNamespace(name="cloud")
    odom_message = SimpleNamespace(name="odom")
    source.sync_slop_ns = int(sender.DEFAULT_SYNC_SLOP_S * 1e9)
    source._last_cloud_stamp = -1
    source._clouds = sender.deque([(target, 10, cloud_message)], maxlen=6)
    source._odometry = sender.deque([(target, 11, odom_message)], maxlen=30)
    source._images = sender.deque(
        [(target + 97_000_000, 12, SimpleNamespace(name="next_image"))], maxlen=6
    )

    assert source._select_locked() is None

    exact_image = SimpleNamespace(name="exact_image")
    source._images.append((target, 13, exact_image))
    image, cloud, odom = source._select_locked()
    assert image[2] is exact_image
    assert cloud[2] is cloud_message
    assert odom[2] is odom_message


def test_sync_selection_never_replays_an_older_buffered_cloud():
    sender = load_sender_module()
    source = object.__new__(sender.OdinRos2Source)
    older = 1_000_000_000
    newer = 1_100_000_000
    source.sync_slop_ns = int(sender.DEFAULT_SYNC_SLOP_S * 1e9)
    source._last_cloud_stamp = -1
    source._clouds = sender.deque(
        [
            (older, 10, SimpleNamespace(name="older_cloud")),
            (newer, 20, SimpleNamespace(name="newer_cloud")),
        ],
        maxlen=6,
    )
    source._images = sender.deque(
        [
            (older, 11, SimpleNamespace(name="older_image")),
            (newer, 21, SimpleNamespace(name="newer_image")),
        ],
        maxlen=6,
    )
    source._odometry = sender.deque(
        [
            (older, 12, SimpleNamespace(name="older_odom")),
            (newer, 22, SimpleNamespace(name="newer_odom")),
        ],
        maxlen=30,
    )

    selected = source._select_locked()
    assert selected is not None
    assert selected[1][0] == newer
    assert source._select_locked() is None


def test_odin_metadata_uses_v1_wire_frame_and_extended_depth_range():
    path = OVERLAY / "yunji_sender.py"
    spec = importlib.util.spec_from_file_location("focus_test_yunji_metadata", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    metadata = module.build_metadata(
        robot_id="robot-1",
        sequence=1,
        rgb_bytes=b"rgb",
        depth_bytes=b"depth",
        pose_matrix=np.eye(4).reshape(-1).tolist(),
        transform_version="odin-test",
        goal_category="plant",
        status={"error_code": "00000000"},
        width=8,
        height=6,
        fx=4.0,
        fy=4.0,
        cx=3.5,
        cy=2.5,
        capture_time_ns=1,
        localization_state="TRACKING",
        covariance_6x6=[0.0] * 36,
        camera_frame="odin1_camera_optical_frame",
        depth_max_m=8.0,
    )

    assert metadata["pose"]["shared_T_camera"]["parent_frame"] == "shared_world"
    assert metadata["depth_max_m"] == 8.0


def test_sender_help_is_renderable():
    result = subprocess.run(
        [sys.executable, str(OVERLAY / "odin1_sender.py"), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "--calibration-file" in result.stdout
    assert "--dry-run" in result.stdout
    assert "--enable-command-capable-observations" in result.stdout
    assert "--base-camera-calibration-file" in result.stdout


def test_preview_defaults_are_loopback_only_and_reuse_robot_auth():
    sender_source = (OVERLAY / "odin1_sender.py").read_text()
    example = (OVERLAY / "config" / "odin1.env.example").read_text()

    assert 'os.environ.get("FOCUS_ODIN1_CAMERA_PREVIEW_URL")' in sender_source
    assert 'or token' in sender_source
    assert (
        "FOCUS_ODIN1_CAMERA_PREVIEW_URL=http://127.0.0.1:18766/camera/yunji"
        in example
    )


def test_gravity_board_calibrator_supports_direct_camera_pose():
    path = TOOLS / "calibrate_gravity_shared_frame_via_board.py"
    result = subprocess.run(
        [sys.executable, str(path), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "--other-pose-is-camera" in result.stdout


def test_headless_launch_and_services_contain_no_motion_stack():
    launch = (OVERLAY / "odin1_driver_headless.launch.py").read_text()
    driver_unit = (
        OVERLAY / "systemd" / "focus-yunji-odin1-driver.service"
    ).read_text()
    sender_unit = (
        OVERLAY / "systemd" / "focus-yunji-odin1-sender.service"
    ).read_text()
    combined = "\n".join((launch, driver_unit, sender_unit)).lower()

    assert 'executable="host_sdk_sample"' in launch
    assert "odin1_sender.py" in sender_unit
    assert "sendsigkill=no" in combined
    assert "/bin/bash -c " in driver_unit
    assert "/bin/bash -c " in sender_unit
    assert "/bin/bash -lc " not in combined
    for forbidden in (
        "rviz2",
        "/api/move",
        "cmd_vel",
        "move_base",
        "planning_node",
        "goal_receiver",
    ):
        assert forbidden not in combined


def test_captured_odin_driver_patch_matches_manifest():
    snapshot = OVERLAY / "odin1_snapshot"
    patch = snapshot / "odin_ros_driver_0.13.0_firmware_0.13.1_mode1.patch"
    digest, filename = (snapshot / "manifest.sha256").read_text().strip().split()

    assert filename == patch.name
    assert patch.stat().st_size == 10346
    assert hashlib.sha256(patch.read_bytes()).hexdigest() == digest


def test_calibration_artifact_is_valid_json():
    artifact = json.loads(factory_calibration_path().read_text())
    assert artifact["provenance"]["deployment_document"]["size_bytes"] == 4441
    assert artifact["validation"]["robot_commands_issued"] is False
