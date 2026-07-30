#!/usr/bin/env python3
"""Live ROS 2 keyframe sender for the Focus hub (standalone overlay).

Runs on the robot's ROS 2 Humble workspace (system Python: rclpy,
message_filters, cv_bridge, requests are all present there). It subscribes to
exactly the topics the TinyNav semantic-mapping stack already publishes live
during `tinynav_semantic_auto_nav.sh` and uploads authenticated mapping-only
keyframes to the hub. It never subscribes to any control/actuation topic and
cannot move the robot.

Topics (RGB/depth/info confirmed against `semantic_mapping/semantic_pointcloud_node.py`
on the robot, see `hub/docs/ROBOT_WSJ_AUDIT.md` and `audit/LIVE_ROS2_SENDER.md`;
pose topic per the 2026-07-19 revision below):
  /camera/camera/color/image_raw                       sensor_msgs/Image
  /camera/camera/aligned_depth_to_color/image_raw       sensor_msgs/Image (16UC1, mm)
  /camera/camera/aligned_depth_to_color/camera_info     sensor_msgs/CameraInfo
  /slam/keyframe_odom                                   nav_msgs/Odometry

Pose source, 2026-07-19 HPC-fidelity pivot (matches the same fix already
applied and live-validated on the Yunji sender — see
`audit/YUNJI_WATER_SENDER.md`, "Third pass"): this used to read
`/semantic_mapping/camera_pose`, which is `T_map_camera` in TinyNav's own
per-session "map" frame — i.e. relocalized against a pre-built map via
`map_node`'s SuperPoint+LightGlue+DINOv2+PnP pipeline. The original
Habitat/HPC codebase never does this: every episode resets each agent to the
same `episode.start_position` and tracks pose from that reset, live, with no
pre-built-map dependency (`merge_sim_episode_config`,
habitat-lab/habitat/tasks/nav/nav.py:104-131). `perception_node`'s own live
SLAM estimate — no relocalization, no saved map — is the faithful
real-machine analogue, exactly like `/sensors_fusion/odom` was for Yunji.

**Topic choice revised same-day, on stronger evidence.** The first version of
this pivot picked `/slam/odometry_visual` from `hub/docs/ROBOT_WSJ_AUDIT.md`
(a genuine direct-observation audit) alone, which documents the topic exists
but never records its message type or exact semantics — an informed guess,
stated as such. Reading `perception_node.py` and `build_map_node.py`
directly (via a cached local clone of the same GitHub mirror used earlier,
`AlanZhu2006/go2_tinynav` at commit `629c79b`, dated 2026-06-17 — still not
proven identical to wsj's exact deployed commit `933fce5...`, see
`experiment.md` 2026-07-18, but strong evidence for the general topic
architecture) settled it with much higher confidence:

  - `perception_node.py` publishes `keyframe_pose_pub` (`/slam/keyframe_odom`,
    `nav_msgs/Odometry`), `keyframe_image_pub` (`/slam/keyframe_image`) and
    `keyframe_depth_pub` (`/slam/keyframe_depth`) all in the SAME code block,
    stamped with the SAME `left_msg.header.stamp` — an exact-timestamp match
    by construction, not merely an approximate one.
  - `build_map_node.py`'s `BuildMapNode` — TinyNav's own online mapping node,
    see the "online loop closure" section below — subscribes to exactly
    these three topics via `ApproximateTimeSynchronizer` for precisely the
    same purpose this sender has (one pose per synchronized RGB-D keyframe).
    That TinyNav's own codebase uses `/slam/keyframe_odom` for this is strong
    corroboration it is the intended topic, not `/slam/odometry_visual`
    (which is continuous, not keyframe-paired) or `/slam/odometry` (a
    separate "high-rate" stream subscribed to separately by `BuildMapNode`
    as `continuous_odom_sub`, confirming it is NOT the per-keyframe pose).
  - `perception_node.py`'s `odom_pub`/`keyframe_pose_pub` both publish via
    `np2msg(pose, stamp, "world", "camera", velocity)` — i.e. `parent_frame=
    "world"` (this node's own live, session-fresh origin — not a persistent
    map name), `child_frame="camera"` — confirming the "reports the camera's
    own pose directly, no base_link involved" assumption from the first
    version of this pivot was correct, now from direct evidence rather than
    analogy.

RGB/depth were deliberately NOT switched to `/slam/keyframe_image`/
`/slam/keyframe_depth` despite their even tighter sync guarantee: that depth
comes from `perception_node`'s own `depth_engine.infer(...)` (a computed
depth, disparity-derived) rather than the RealSense driver's native hardware
depth this sender has always used — a real quality/characteristics trade-off
that needs live evaluation before adopting, not something to fold in as a
side effect of the pose-topic fix. `ApproximateTimeSynchronizer` still pairs
the raw driver's RGB/depth against `/slam/keyframe_odom`, same as before.

**Live verification update, 2026-07-21:** the exact wsj checkout at base
`933fce54...` was inspected, the health-gated overlay was deployed under a
new filename, and a no-actuation ten-frame run against these topics completed
with 10/10 Hub accepts and zero pose-sync skew. See
`audit/WSJ_IMU_SCHEDULING_FIX_20260721.md`. Physical camera/body extrinsics
and a moved-robot ground-truth trajectory remain unverified.

The default aligned-depth path is already resampled into the color frame by
the RealSense driver, so — unlike the replay sender, which has to reproject raw
infra1 depth — no extra geometry step is needed there.  A second, explicit
TinyNav-native path is also supported: `/slam/keyframe_image` and
`/slam/keyframe_depth` are already pixel-aligned by TinyNav, and the latter's
`32FC1` metres are converted to the same `png16` millimetre wire contract.
Topic defaults remain on the hardware-depth path; callers must opt into the
native path with CLI topic overrides so the depth-source trade-off is visible
in the session provenance.

Mapping-only runs may still upload the raw TinyNav tracking pose under a
distinctive test transform version.  Command-capable observation metadata is
stricter: it requires both the measured base/camera mount and a versioned
shared-tracking calibration, and composes the latter before claiming the pose
is in ``shared_world``.  This sender still has no control or actuator output.
"""
from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

import message_filters

HUB_SRC = Path(__file__).resolve().parents[1] / "src"
if HUB_SRC.is_dir():
    sys.path.insert(0, str(HUB_SRC))

from focus_hub.base_camera_calibration import load_base_camera_calibration
from focus_hub.robot_map_alignment import load_shared_tracking_calibration

DEPTH_SCALE_M = 0.001
CAMERA_FRAME = "camera_color_optical_frame"

# Same thresholds and caveat as hub/robot_overlay/yunji_sender.py's
# classify_localization_state: a reasonable order-of-magnitude guess, not
# calibrated against a real degraded/lost tracking event on THIS robot —
# doubly true here since this file hasn't even been run against wsj yet.
LOCALIZATION_TRACKING_MAX_POS_VAR_M2 = 0.01
LOCALIZATION_TRACKING_MAX_YAW_VAR_RAD2 = 0.01
LOCALIZATION_DEGRADED_MAX_POS_VAR_M2 = 1.0
LOCALIZATION_DEGRADED_MAX_YAW_VAR_RAD2 = 1.0
COVARIANCE_ZERO_EPS = 1e-15
SLAM_IMU_MIN_COVERAGE_RATIO = 0.80
SLAM_IMU_MAX_SAMPLE_GAP_S = 0.05
SLAM_IMU_END_TOLERANCE_S = 0.01
SLAM_OVERWRITE_RECOVERY_MIN_REPORTS = 3
SLAM_OVERWRITE_RECOVERY_MIN_S = 2.0
PARKED_RECEIPT_TUPLE_INTERVAL = 15
RUNTIME_COMMAND_CONTRACT_SCHEMA = (
    "focus-wsj-command-observation-contract-v1"
)
RUNTIME_COMMAND_ACTIVATION = "COMMAND_CAPABLE_OBSERVATION_ONLY"


class ObservationContract:
    """Checked metadata applied without replacing the ROS/DDS participant."""

    def __init__(
        self,
        *,
        transform_version: str,
        goal_category: str,
        base_camera_calibration: object | None,
        shared_tracking_calibration: object | None,
        runtime_contract_sha256: str | None = None,
    ) -> None:
        self.transform_version = transform_version
        self.goal_category = goal_category
        self.base_camera_calibration = base_camera_calibration
        self.shared_tracking_calibration = shared_tracking_calibration
        self.runtime_contract_sha256 = runtime_contract_sha256

    @property
    def mapping_only(self) -> bool:
        return self.base_camera_calibration is None


def sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validated_runtime_artifact(
    value: object, *, label: str
) -> Path:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an artifact object")
    path = Path(str(value.get("path", "")))
    if not path.is_absolute() or not path.is_file():
        raise ValueError(f"{label}.path must be an existing absolute file")
    expected_size = int(value.get("size_bytes", -1))
    expected_sha256 = str(value.get("sha256", ""))
    if path.stat().st_size != expected_size:
        raise ValueError(f"{label} size mismatch")
    if sha256_path(path) != expected_sha256:
        raise ValueError(f"{label} checksum mismatch")
    return path


def load_runtime_command_contract(
    path: Path,
    *,
    robot_id: str,
    camera_frame: str,
    deployment_commit: str,
) -> ObservationContract:
    encoded = path.read_bytes()
    contract_sha256 = hashlib.sha256(encoded).hexdigest()
    payload = json.loads(encoded)
    if not isinstance(payload, dict):
        raise ValueError("runtime command contract must be a JSON object")
    if payload.get("schema_version") != RUNTIME_COMMAND_CONTRACT_SCHEMA:
        raise ValueError("runtime command contract schema mismatch")
    if payload.get("activation_confirmation") != RUNTIME_COMMAND_ACTIVATION:
        raise ValueError("runtime command contract activation mismatch")
    if payload.get("robot_id") != robot_id:
        raise ValueError("runtime command contract robot mismatch")
    if payload.get("camera_frame") != camera_frame:
        raise ValueError("runtime command contract camera mismatch")
    if payload.get("deployment_commit") != deployment_commit:
        raise ValueError("runtime command contract deployment mismatch")
    transform_version = str(payload.get("transform_version", ""))
    calibration_id = str(payload.get("shared_frame_calibration_id", ""))
    goal_category = str(payload.get("goal_category", ""))
    for label, value in (
        ("transform_version", transform_version),
        ("shared_frame_calibration_id", calibration_id),
        ("goal_category", goal_category),
    ):
        if not value or any(
            character not in "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_.-"
            for character in value
        ):
            raise ValueError(f"runtime command contract {label} is invalid")
    base_path = _validated_runtime_artifact(
        payload.get("base_camera_calibration"),
        label="base_camera_calibration",
    )
    shared_path = _validated_runtime_artifact(
        payload.get("shared_tracking_calibration"),
        label="shared_tracking_calibration",
    )
    base = load_base_camera_calibration(
        base_path,
        expected_robot_id=robot_id,
        expected_camera_frame=camera_frame,
    )
    shared = load_shared_tracking_calibration(
        shared_path,
        robot_id=robot_id,
        expected_transform_version=transform_version,
        expected_calibration_id=calibration_id,
    )
    return ObservationContract(
        transform_version=transform_version,
        goal_category=goal_category,
        base_camera_calibration=base,
        shared_tracking_calibration=shared,
        runtime_contract_sha256=contract_sha256,
    )


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def depth_msg_to_png16_array(bridge: CvBridge, depth_msg: Image) -> np.ndarray:
    """Convert supported ROS depth encodings to uint16 wire units.

    RealSense aligned depth is ``16UC1`` in millimetres and therefore already
    matches ``DEPTH_SCALE_M``. TinyNav keyframe depth is ``32FC1`` in metres;
    convert it with the same rounding/clipping semantics as
    ``focus_hub.depth_align.encode_depth_png16``. Non-finite and negative
    values become the wire-format invalid sentinel (zero).
    """

    encoding = str(getattr(depth_msg, "encoding", "")).upper()
    if encoding == "16UC1":
        depth = np.asarray(
            bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1"))
        if depth.dtype != np.uint16:
            raise ValueError(
                f"16UC1 bridge output has unexpected dtype {depth.dtype}")
    elif encoding == "32FC1":
        depth_m = np.asarray(
            bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1"),
            dtype=np.float32,
        )
        safe_depth_m = np.nan_to_num(
            depth_m, nan=0.0, posinf=0.0, neginf=0.0)
        scaled = np.rint(safe_depth_m / DEPTH_SCALE_M)
        depth = np.clip(scaled, 0, np.iinfo(np.uint16).max).astype(np.uint16)
    else:
        raise ValueError(
            f"unsupported depth encoding {encoding!r}; expected 16UC1 or 32FC1")

    if depth.ndim != 2:
        raise ValueError(f"depth image must be 2-D, got shape {depth.shape}")
    return np.ascontiguousarray(depth)


def camera_info_matrix(message: CameraInfo) -> np.ndarray:
    """Return and validate the 3x3 pinhole matrix carried by CameraInfo."""

    values = np.asarray(message.k, dtype=np.float64)
    if values.shape != (9,) or not np.all(np.isfinite(values)):
        raise ValueError("camera_info.k must contain nine finite values")
    matrix = values.reshape(3, 3)
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("camera intrinsics must have positive focal lengths")
    return matrix


def transform_message_matrix(message) -> np.ndarray:
    """Convert one geometry_msgs/TransformStamped to ``T_target_source``."""

    transform = message.transform
    translation = transform.translation
    rotation = transform.rotation
    x, y, z, w = (
        float(rotation.x),
        float(rotation.y),
        float(rotation.z),
        float(rotation.w),
    )
    values = (
        float(translation.x),
        float(translation.y),
        float(translation.z),
        x,
        y,
        z,
        w,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("RGB/depth static transform contains a non-finite value")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("RGB/depth static transform has a zero quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), translation.x],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), translation.y],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), translation.z],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return matrix


def register_rgb_onto_depth_grid(
    rgb_bgr: np.ndarray,
    depth_m: np.ndarray,
    K_depth: np.ndarray,
    K_rgb: np.ndarray,
    T_rgb_from_depth: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Depth-register a real color frame onto TinyNav's left-IR pixel grid.

    TinyNav's keyframe depth and pose both live in the left-infrared optical
    frame.  The RealSense color stream is kept for learned semantics, but its
    pixels cannot merely be renamed as infrared pixels.  Every valid depth
    sample is therefore lifted with ``K_depth``, transformed through the
    RealSense static optical extrinsic, projected with ``K_rgb`` and sampled
    from the color image.  The returned image has exactly the depth shape, so
    the existing aligned-RGBD transport contract remains true.

    Invalid-depth pixels are initialized with a nearest-neighbour resized color
    view.  They help the 2-D detector see a complete image but can never enter
    the BEV: the Hub mapper already rejects their zero depth.  Valid-depth
    pixels always use the calibrated projection.
    """

    rgb = np.asarray(rgb_bgr)
    depth = np.asarray(depth_m, dtype=np.float64)
    K_depth = np.asarray(K_depth, dtype=np.float64)
    K_rgb = np.asarray(K_rgb, dtype=np.float64)
    transform = np.asarray(T_rgb_from_depth, dtype=np.float64)
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError(f"RGB image must have shape (H,W,3), got {rgb.shape}")
    if depth.ndim != 2:
        raise ValueError(f"depth image must be 2-D, got {depth.shape}")
    if K_depth.shape != (3, 3) or K_rgb.shape != (3, 3):
        raise ValueError("RGB and depth intrinsics must both be 3x3")
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("T_rgb_from_depth must be a finite 4x4 matrix")

    depth_h, depth_w = depth.shape
    rgb_h, rgb_w = rgb.shape[:2]
    row_lookup = np.rint(
        np.linspace(0.0, max(0, rgb_h - 1), depth_h)
    ).astype(np.int64)
    col_lookup = np.rint(
        np.linspace(0.0, max(0, rgb_w - 1), depth_w)
    ).astype(np.int64)
    registered = np.ascontiguousarray(
        rgb[row_lookup[:, None], col_lookup[None, :]].copy()
    )

    valid = np.isfinite(depth) & (depth > 0.0)
    valid_count = int(np.count_nonzero(valid))
    if valid_count == 0:
        return registered, np.zeros(depth.shape, dtype=bool), 0.0
    rows, cols = np.nonzero(valid)
    z_depth = depth[rows, cols]
    points_depth = np.stack(
        (
            (cols - K_depth[0, 2]) / K_depth[0, 0] * z_depth,
            (rows - K_depth[1, 2]) / K_depth[1, 1] * z_depth,
            z_depth,
        ),
        axis=-1,
    )
    points_rgb = (
        points_depth @ transform[:3, :3].T + transform[:3, 3]
    )
    in_front = points_rgb[:, 2] > 1e-6
    projected_u = np.full(points_rgb.shape[0], -1, dtype=np.int64)
    projected_v = np.full(points_rgb.shape[0], -1, dtype=np.int64)
    projected_u[in_front] = np.rint(
        K_rgb[0, 0]
        * points_rgb[in_front, 0]
        / points_rgb[in_front, 2]
        + K_rgb[0, 2]
    ).astype(np.int64)
    projected_v[in_front] = np.rint(
        K_rgb[1, 1]
        * points_rgb[in_front, 1]
        / points_rgb[in_front, 2]
        + K_rgb[1, 2]
    ).astype(np.int64)
    mapped = (
        in_front
        & (projected_u >= 0)
        & (projected_u < rgb_w)
        & (projected_v >= 0)
        & (projected_v < rgb_h)
    )
    registered[rows[mapped], cols[mapped]] = rgb[
        projected_v[mapped], projected_u[mapped]
    ]
    registered_depth_mask = np.zeros(depth.shape, dtype=bool)
    registered_depth_mask[rows[mapped], cols[mapped]] = True
    return (
        registered,
        registered_depth_mask,
        float(np.count_nonzero(mapped) / valid_count),
    )


def odom_to_matrix(odom: Odometry) -> list[float]:
    """Converts nav_msgs/Odometry's pose into this project's row-major flat
    4x4 wire format. Assumes odom.pose.pose IS the camera's own pose
    directly (see the UNVERIFIED note in the module docstring) — no
    base_link->camera composition is applied.
    """
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    pose_values = (p.x, p.y, p.z, x, y, z, w)
    if not all(math.isfinite(float(value)) for value in pose_values):
        raise ValueError("odometry pose contains a non-finite value")
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm <= 1e-12:
        raise ValueError("odometry quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    r00 = 1 - 2 * (y * y + z * z)
    r01 = 2 * (x * y - z * w)
    r02 = 2 * (x * z + y * w)
    r10 = 2 * (x * y + z * w)
    r11 = 1 - 2 * (x * x + z * z)
    r12 = 2 * (y * z - x * w)
    r20 = 2 * (x * z - y * w)
    r21 = 2 * (y * z + x * w)
    r22 = 1 - 2 * (x * x + y * y)
    return [
        r00, r01, r02, float(p.x),
        r10, r11, r12, float(p.y),
        r20, r21, r22, float(p.z),
        0.0, 0.0, 0.0, 1.0,
    ]


def apply_shared_tracking_alignment(
    tracking_T_camera: list[float] | tuple[float, ...],
    shared_T_tracking: list[float] | tuple[float, ...],
) -> list[float]:
    """Compose a calibrated tracking pose before labeling it shared-world.

    This is deliberately a full SE(3) composition for mapping geometry.  The
    physical receiver separately projects its shared-to-local goal transform
    onto SE(2), after enforcing the configured tilt bound.
    """

    tracking = np.asarray(tracking_T_camera, dtype=np.float64)
    shared = np.asarray(shared_T_tracking, dtype=np.float64)
    if tracking.size != 16 or shared.size != 16:
        raise ValueError("tracking alignment requires two 4x4 transforms")
    tracking = tracking.reshape(4, 4)
    shared = shared.reshape(4, 4)
    result = shared @ tracking
    if not np.all(np.isfinite(result)):
        raise ValueError("shared tracking composition is non-finite")
    return [float(value) for value in result.reshape(-1)]


def classify_localization_state(covariance_6x6) -> tuple[str, list[float]]:
    """Returns (localization_state, covariance_6x6_for_wire). Same logic as
    yunji_sender.py's function of the same name — see that module for the
    full rationale. Duplicated rather than imported: both senders are
    standalone single-file deployables with zero shared imports by design.
    """
    if not covariance_6x6 or len(covariance_6x6) != 36:
        return "UNKNOWN", [0.0] * 36
    cov = [float(v) for v in covariance_6x6]
    if not all(math.isfinite(v) for v in cov):
        return "UNKNOWN", [0.0] * 36
    diagonal = [cov[index] for index in (0, 7, 14, 21, 28, 35)]
    if any(value < 0 for value in diagonal):
        return "UNKNOWN", [0.0] * 36
    # nav_msgs/Odometry does not define an all-zero covariance as a proof of
    # perfect localization. TinyNav currently leaves every entry at zero, so
    # accepting it as low variance turns an absent uncertainty estimate into
    # a false TRACKING signal. Fail closed until a real covariance is present.
    if max(abs(value) for value in diagonal) <= COVARIANCE_ZERO_EPS:
        return "UNKNOWN", cov
    var_x, var_y, var_yaw = cov[0], cov[7], cov[35]
    pos_var = max(var_x, var_y)
    if pos_var <= LOCALIZATION_TRACKING_MAX_POS_VAR_M2 and var_yaw <= LOCALIZATION_TRACKING_MAX_YAW_VAR_RAD2:
        return "TRACKING", cov
    if pos_var <= LOCALIZATION_DEGRADED_MAX_POS_VAR_M2 and var_yaw <= LOCALIZATION_DEGRADED_MAX_YAW_VAR_RAD2:
        return "DEGRADED", cov
    return "LOST", cov


class HubTransport:
    """Same resume/retry contract as hub/robot_overlay/focus_sender.py."""

    def __init__(self, base_url, robot_id, token, timeout_s=10.0,
                 max_retries: int | None = 8, backoff_base_s=0.5,
                 backoff_cap_s=8.0):
        self.base_url = base_url.rstrip("/")
        self.robot_id = robot_id
        self.token = token
        self.timeout_s = timeout_s
        self.max_retries = max_retries
        self.backoff_base_s = backoff_base_s
        self.backoff_cap_s = backoff_cap_s
        self.retries_total = 0
        self.session_resets_total = 0
        self.session = self._new_session()

    def _new_session(self):
        import requests

        session = requests.Session()
        session.headers["X-Robot-Token"] = self.token
        return session

    def _reset_session(self) -> None:
        """Drop only the stale HTTP pool; preserve this ROS/DDS participant."""

        previous = self.session
        try:
            previous.close()
        finally:
            self.session = self._new_session()
            self.session_resets_total += 1

    def last_sequence(self) -> int:
        import requests

        url = f"{self.base_url}/v1/robots/{self.robot_id}/observations/latest"
        try:
            response = self.session.get(url, timeout=self.timeout_s)
        except (requests.ConnectionError, requests.Timeout):
            # Hub debug/live transitions briefly replace the local API behind
            # the reverse tunnel. A kept-alive socket can survive in a stale
            # pool even after the replacement is ready. Rebuild only that
            # pool; restarting this process also recreates the DDS participant
            # and has repeatedly delayed discovery of TinyNav publishers.
            self._reset_session()
            response = self.session.get(url, timeout=self.timeout_s)
        response.raise_for_status()
        return int(response.json()["last_sequence"])

    def upload(self, metadata: dict, rgb_bytes: bytes, depth_bytes: bytes, restamp):
        import requests

        attempt = 0
        while True:
            attempt += 1
            try:
                response = self.session.post(
                    f"{self.base_url}/v1/robots/{self.robot_id}/observations",
                    data={"metadata_json": json.dumps(metadata)},
                    files={
                        "rgb": ("rgb", rgb_bytes, "image/jpeg"),
                        "depth": ("depth", depth_bytes, "image/png"),
                    },
                    timeout=self.timeout_s,
                )
                if response.status_code in (200, 201):
                    return response.json(), attempt
                if 400 <= response.status_code < 500 and response.status_code not in (408, 429):
                    raise RuntimeError(
                        f"hub rejected seq {metadata['sequence']}: "
                        f"{response.status_code} {response.text[:300]}"
                    )
            except (requests.ConnectionError, requests.Timeout):
                # Keep ROS subscriptions and DDS discovery warm across a Hub
                # restart. Only the HTTP connection pool is disposable.
                self._reset_session()
            if (
                self.max_retries is not None
                and attempt > self.max_retries
            ):
                raise RuntimeError(f"giving up on seq {metadata['sequence']} after {attempt} attempts")
            self.retries_total += 1
            delay = min(self.backoff_cap_s, self.backoff_base_s * (2 ** (attempt - 1)))
            time.sleep(delay)
            metadata = restamp(metadata)


class LatestHealth:
    """Thread-safe holder for the most recently computed health snapshot,
    so the independent heartbeat thread can repost it without depending on
    whatever `process()` is currently doing (which can block for tens of
    seconds inside HubTransport.upload()'s retry loop — exactly the
    scenario a decoupled heartbeat needs to survive).
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: dict = {
            "safety_state": "UNKNOWN", "localization_state": "UNKNOWN",
            "estop_engaged": False,
            "collision_avoidance_ready": False, "motor_controller_ready": False,
        }

    def set(self, value: dict) -> None:
        with self._lock:
            self._value = dict(value)

    def get(self) -> dict:
        with self._lock:
            return dict(self._value)


class LatestSlamMetrics:
    """Fail-closed view of TinyNav's latest ``/slam/data`` optimizer report.

    ``perception_node.py`` publishes JSON containing factor/variable counts,
    initial/final graph errors and per-keyframe IMU coverage. This holder
    independently checks those fields so a producer-side boolean alone cannot
    turn incomplete inertial data into a healthy pose.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._received_monotonic: float | None = None
        self._gate = "UNKNOWN"
        self._detail = "slam_metrics_missing"
        self._last_overwritten_count: int | None = None
        self._overwrite_stable_reports = 0
        self._overwrite_stable_since: float | None = None

    def update(self, raw_json: str, *, received_monotonic: float | None = None) -> None:
        received = time.monotonic() if received_monotonic is None else received_monotonic
        gate = "UNKNOWN"
        detail = "slam_metrics_invalid"
        current_intervals_valid = False
        try:
            payload = json.loads(raw_json)
            stats = payload.get("stats")
            metrics = payload.get("metrics")
            if not isinstance(stats, dict) or not isinstance(metrics, dict):
                raise ValueError("missing stats or metrics object")

            optimizer_status = stats.get("optimizer_status")
            if optimizer_status in {"initializing", "anchoring", "warmup_complete"}:
                detail = f"slam_{optimizer_status}"
            elif optimizer_status == "skipped_imu_invalid":
                gate = "LOST"
                detail = "slam_imu_intervals_invalid"
            elif optimizer_status == "rejected_nonfinite":
                gate = "LOST"
                detail = "slam_optimizer_nonfinite"
            elif optimizer_status != "ok":
                detail = "slam_optimizer_status_invalid"
            else:
                initial_error = float(metrics["initial_error"])
                final_error = float(metrics["final_error"])
                num_factors = int(metrics["num_factors"])
                num_variables = int(metrics["num_variables"])
                intervals = metrics.get("imu_intervals")
                producer_intervals_valid = metrics.get("imu_intervals_valid")

                if not math.isfinite(initial_error) or not math.isfinite(final_error):
                    gate = "LOST"
                    detail = "slam_optimizer_nonfinite"
                elif final_error > initial_error + max(1e-9, abs(initial_error) * 1e-6):
                    gate = "LOST"
                    detail = "slam_optimizer_worsened"
                elif num_factors <= 0 or num_variables <= 0:
                    detail = "slam_graph_empty"
                elif producer_intervals_valid is not True:
                    gate = "LOST" if producer_intervals_valid is False else "UNKNOWN"
                    detail = (
                        "slam_imu_intervals_invalid"
                        if producer_intervals_valid is False
                        else "slam_imu_health_missing"
                    )
                elif not isinstance(intervals, list) or not intervals:
                    detail = "slam_imu_health_missing"
                elif not all(self._interval_is_valid(interval) for interval in intervals):
                    gate = "LOST"
                    detail = "slam_imu_intervals_invalid"
                else:
                    current_intervals_valid = True
                    gate, detail = self._overwrite_gate(
                        int(stats.get("imu_messages_overwritten", 0)),
                        received_monotonic=received,
                    )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            pass
        if not current_intervals_valid:
            self._overwrite_stable_reports = 0
            self._overwrite_stable_since = None
        with self._lock:
            self._received_monotonic = received
            self._gate = gate
            self._detail = detail

    def _overwrite_gate(
        self,
        count: int,
        *,
        received_monotonic: float,
    ) -> tuple[str, str]:
        if count < 0:
            return "LOST", "slam_imu_overwrite_counter_invalid"
        if count == 0:
            self._last_overwritten_count = 0
            self._overwrite_stable_reports = 0
            self._overwrite_stable_since = None
            return "PASS", "slam_optimizer_imu_valid"
        if self._last_overwritten_count != count:
            self._last_overwritten_count = count
            self._overwrite_stable_reports = 1
            self._overwrite_stable_since = received_monotonic
            return "LOST", f"slam_imu_buffer_overwritten:{count}"
        self._overwrite_stable_reports += 1
        if self._overwrite_stable_since is None:
            self._overwrite_stable_since = received_monotonic
        stable_s = max(
            0.0, received_monotonic - self._overwrite_stable_since
        )
        if (
            self._overwrite_stable_reports
            >= SLAM_OVERWRITE_RECOVERY_MIN_REPORTS
            and stable_s >= SLAM_OVERWRITE_RECOVERY_MIN_S
        ):
            return (
                "PASS",
                "slam_optimizer_imu_valid_after_overwrite_recovery:"
                f"{count}",
            )
        return (
            "UNKNOWN",
            "slam_imu_buffer_recovery:"
            f"{self._overwrite_stable_reports}/"
            f"{SLAM_OVERWRITE_RECOVERY_MIN_REPORTS}",
        )

    @staticmethod
    def _interval_is_valid(interval: object) -> bool:
        if not isinstance(interval, dict) or interval.get("valid") is not True:
            return False
        try:
            duration_s = float(interval["duration_s"])
            sample_count = int(interval["sample_count"])
            expected_count = int(interval["expected_count"])
            coverage_ratio = float(interval["coverage_ratio"])
            max_sample_gap_s = float(interval["max_sample_gap_s"])
            end_error_s = float(interval["end_error_s"])
        except (KeyError, TypeError, ValueError):
            return False
        numeric = (duration_s, coverage_ratio, max_sample_gap_s, end_error_s)
        return (
            all(math.isfinite(value) for value in numeric)
            and duration_s > 0.0
            and sample_count >= 2
            and expected_count > 0
            and coverage_ratio >= SLAM_IMU_MIN_COVERAGE_RATIO
            and max_sample_gap_s <= SLAM_IMU_MAX_SAMPLE_GAP_S
            and end_error_s <= SLAM_IMU_END_TOLERANCE_S
        )

    def apply(self, covariance_state: str, *, timeout_s: float,
              now_monotonic: float | None = None) -> tuple[str, str]:
        now = time.monotonic() if now_monotonic is None else now_monotonic
        with self._lock:
            received = self._received_monotonic
            gate = self._gate
            detail = self._detail
        if received is None:
            return "UNKNOWN", "slam_metrics_missing"
        age_s = max(0.0, now - received)
        if age_s > timeout_s:
            return "UNKNOWN", f"slam_metrics_stale:{age_s:.1f}s"
        if gate == "LOST":
            return "LOST", detail
        if gate != "PASS":
            return "UNKNOWN", detail
        if covariance_state == "UNKNOWN":
            # Complete IMU/optimizer telemetry makes the pose useful for the
            # mapping-only lane, but absent covariance must never become
            # command-capable TRACKING.
            return "DEGRADED", f"{detail};covariance_unavailable"
        if covariance_state != "TRACKING":
            return covariance_state, f"{detail};covariance_{covariance_state.lower()}"
        return "TRACKING", detail


class HeartbeatThread(threading.Thread):
    """Independent liveness/health ping, decoupled from rclpy's single-
    threaded spin loop and from `process()`'s synchronous upload-with-retry
    call (which can legitimately block for ~40s on a bad connection — see
    HubTransport.upload). A same-thread rclpy Timer would NOT be
    independent here, since `rclpy.spin_once` only runs one callback at a
    time; this uses a real OS thread instead, exactly like the Yunji
    sender's HeartbeatThread.

    Caveat carried over honestly from the rest of this file's health
    reporting: wsj has no independent fast health source at all right now
    (unlike Yunji's TCP API) — this thread only guarantees FASTER, more
    reliable DELIVERY of the same `localization_state` `process()` already
    computes per synced frame; it cannot invent estop/battery data this
    sender has no source for.
    """

    def __init__(self, *, base_url: str, robot_id: str, token: str,
                 latest_health: LatestHealth, period_s: float = 0.5) -> None:
        super().__init__(daemon=True, name="focus-ros-heartbeat")
        import requests

        self.session = requests.Session()
        self.session.headers["X-Robot-Token"] = token
        self.base_url = base_url.rstrip("/")
        self.robot_id = robot_id
        self.latest_health = latest_health
        self.period_s = period_s
        self.stop_event = threading.Event()
        self.beats_sent = 0
        self.beats_failed = 0

    def run(self) -> None:
        while not self.stop_event.is_set():
            t0 = time.monotonic()
            try:
                self._beat_once()
                self.beats_sent += 1
            except Exception:  # noqa: BLE001 - a failed heartbeat must not kill the thread
                self.beats_failed += 1
            elapsed = time.monotonic() - t0
            self.stop_event.wait(max(0.0, self.period_s - elapsed))

    def _beat_once(self) -> None:
        body = {
            "robot_id": self.robot_id,
            "sent_time_ns": time.time_ns(),
            "health": self.latest_health.get(),
        }
        self.session.post(
            f"{self.base_url}/v1/robots/{self.robot_id}/heartbeat", json=body, timeout=2.0,
        ).raise_for_status()

    def stop(self) -> None:
        self.stop_event.set()


class FocusRosSender(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("focus_ros_sender")
        self.args = args
        self.bridge = CvBridge()
        # This ROS/DDS participant is intentionally session-long. Network
        # outages rebuild only its HTTP pool and must not exhaust into a
        # process exit that would require publisher rediscovery.
        self.transport = HubTransport(
            args.base_url,
            args.robot_id,
            args.token,
            max_retries=None,
        )
        self.min_period_s = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.0
        self.last_upload_monotonic = 0.0
        self.frames_sent = 0
        self.frames_seen = 0
        self.last_frame_error: str | None = None
        self.last_frame_error_at_ns: int | None = None
        self.metrics: list[dict] = []
        self.done = False
        self.registration_T_rgb_from_depth: np.ndarray | None = None
        self.registration_tf_listener = None
        self.registration_tf_buffer = None
        self.registration_time_zero = None
        self.registration_timeout = None
        self.runtime_contract_file = args.runtime_command_contract_file
        self.runtime_receipt_file = args.runtime_command_receipt_file
        self.runtime_contract_sha256 = None
        self.runtime_rejected_sha256 = None
        self.runtime_receipt_state = None
        self.runtime_contract_activations = 0
        self.parked_geometry_tuples = 0
        self.last_parked_receipt_tuple = 0
        self.observation_contract = (
            None
            if self.runtime_contract_file is not None
            else ObservationContract(
                transform_version=args.transform_version,
                goal_category=args.goal_category,
                base_camera_calibration=args.base_camera_calibration,
                shared_tracking_calibration=args.shared_tracking_calibration,
            )
        )

        try:
            self.sequence = self.transport.last_sequence() + 1
            self.get_logger().info(f"resume: starting at sequence {self.sequence} [hub]")
        except Exception as exc:  # noqa: BLE001
            self.sequence = 0
            self.get_logger().warn(f"resume: hub unreachable at startup ({exc}); starting at sequence 0")

        self.latest_health = LatestHealth()
        self.latest_slam_metrics = LatestSlamMetrics()
        self.slam_data_sub = self.create_subscription(
            String, args.slam_data_topic, self.on_slam_data, 10)
        self.heartbeat_thread = None
        if args.heartbeat_hz > 0:
            self.heartbeat_thread = HeartbeatThread(
                base_url=args.base_url, robot_id=args.robot_id, token=args.token,
                latest_health=self.latest_health, period_s=1.0 / args.heartbeat_hz,
            )
            self.heartbeat_thread.start()
            self.get_logger().info(
                f"heartbeat thread started ({args.heartbeat_hz} Hz, independent of the sync callback)")

        depth_sub = message_filters.Subscriber(self, Image, args.depth_topic, qos_profile=qos_profile_sensor_data)
        info_sub = message_filters.Subscriber(self, CameraInfo, args.info_topic, qos_profile=qos_profile_sensor_data)
        pose_sub = message_filters.Subscriber(self, Odometry, args.pose_topic, qos_profile=qos_profile_sensor_data)
        self.rgb_cache = deque(maxlen=args.rgb_cache_size)
        self.latest_rgb_info_msg = None
        self.geometry_tuples_seen = 0
        self.rgb_cache_skew_rejections = 0
        self.closest_rgb_skew_ms = None
        self.latest_rgb_sub = None
        self.latest_rgb_info_sub = None
        if args.latest_rgb_for_depth:
            self.latest_rgb_sub = self.create_subscription(
                Image,
                args.rgb_topic,
                self.on_latest_rgb,
                qos_profile_sensor_data,
            )
            self.latest_rgb_info_sub = self.create_subscription(
                CameraInfo,
                args.rgb_info_topic,
                self.on_latest_rgb_info,
                qos_profile_sensor_data,
            )
            synchronized_inputs = [depth_sub, info_sub, pose_sub]
            synchronized_callback = self.on_synced_with_latest_rgb
        else:
            rgb_sub = message_filters.Subscriber(
                self,
                Image,
                args.rgb_topic,
                qos_profile=qos_profile_sensor_data,
            )
            synchronized_inputs = [rgb_sub, depth_sub, info_sub, pose_sub]
            synchronized_callback = self.on_synced
        if args.register_rgb_to_depth:
            from rclpy.duration import Duration
            from rclpy.time import Time
            from tf2_ros import Buffer, TransformListener

            self.registration_tf_buffer = Buffer()
            self.registration_tf_listener = TransformListener(
                self.registration_tf_buffer, self
            )
            self.registration_time_zero = Time()
            self.registration_timeout = Duration(
                seconds=args.registration_tf_timeout_s
            )
            if not args.latest_rgb_for_depth:
                rgb_info_sub = message_filters.Subscriber(
                    self,
                    CameraInfo,
                    args.rgb_info_topic,
                    qos_profile=qos_profile_sensor_data,
                )
                synchronized_inputs.append(rgb_info_sub)
                synchronized_callback = self.on_synced_registered
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            synchronized_inputs,
            queue_size=args.sync_queue_size, slop=args.sync_slop,
        )
        self.synchronizer.registerCallback(synchronized_callback)
        self.runtime_contract_timer = None
        if self.runtime_contract_file is not None:
            self.runtime_contract_timer = self.create_timer(
                0.25, self._refresh_runtime_contract
            )
            self._refresh_runtime_contract()
        self.get_logger().info(
            f"focus_ros_sender ready: rgb={args.rgb_topic} depth={args.depth_topic} "
            f"info={args.info_topic} pose={args.pose_topic} -> {args.base_url} "
            f"(robot_id={args.robot_id}, transform_version={args.transform_version}, "
            f"rate={args.rate_hz}Hz, max_frames={args.max_frames or 'unbounded'}, "
            f"capture_time_source={args.capture_time_source}, "
            f"register_rgb_to_depth={args.register_rgb_to_depth}, "
            f"latest_rgb_for_depth={args.latest_rgb_for_depth}, "
            f"rgb_cache_size={args.rgb_cache_size})"
        )
        if args.capture_time_source == "wall":
            self.get_logger().warn(
                "capture_time_source=wall: capture_time_ns is NOT the real sensor "
                "timestamp. Rehearsal/bag-replay use only."
            )

    def _write_runtime_receipt(
        self,
        *,
        status: str,
        contract_sha256: str | None,
        detail: str,
    ) -> None:
        if self.runtime_receipt_file is None:
            return
        receipt = {
            "schema_version": "focus-wsj-command-observation-receipt-v1",
            "status": status,
            "detail": detail,
            "robot_id": self.args.robot_id,
            "deployment_commit": self.args.deployment_commit,
            "contract_sha256": contract_sha256,
            "pid": os.getpid(),
            "written_at_ns": time.time_ns(),
            "transform_version": (
                None
                if self.observation_contract is None
                else self.observation_contract.transform_version
            ),
            "shared_tracking_calibration_sha256": (
                None
                if self.observation_contract is None
                or self.observation_contract.shared_tracking_calibration is None
                else self.observation_contract.shared_tracking_calibration.source_sha256
            ),
            "robot_commands_issued": False,
            "frames_seen": getattr(self, "frames_seen", 0),
            "frames_sent": getattr(self, "frames_sent", 0),
            "last_frame_error": getattr(self, "last_frame_error", None),
            "last_frame_error_at_ns": getattr(
                self, "last_frame_error_at_ns", None
            ),
            "parked_geometry_tuples": getattr(
                self, "parked_geometry_tuples", 0
            ),
        }
        destination = self.runtime_receipt_file
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(
            f".{destination.name}.tmp.{os.getpid()}"
        )
        temporary.write_text(
            json.dumps(receipt, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.chmod(temporary, 0o600)
        os.replace(temporary, destination)
        self.runtime_receipt_state = (status, contract_sha256, detail)
        if status == "parked":
            self.last_parked_receipt_tuple = getattr(
                self, "parked_geometry_tuples", 0
            )

    def _park_runtime_contract(
        self,
        *,
        status: str,
        contract_sha256: str | None,
        detail: str,
    ) -> None:
        self.observation_contract = None
        state = (status, contract_sha256, detail)
        if self.runtime_receipt_state != state:
            self._write_runtime_receipt(
                status=status,
                contract_sha256=contract_sha256,
                detail=detail,
            )

    def _refresh_runtime_contract(self) -> None:
        if self.runtime_contract_file is None:
            return
        try:
            encoded = self.runtime_contract_file.read_bytes()
        except FileNotFoundError:
            self.runtime_contract_sha256 = None
            self.runtime_rejected_sha256 = None
            self._park_runtime_contract(
                status="parked",
                contract_sha256=None,
                detail="runtime_contract_absent",
            )
            return
        candidate_sha256 = hashlib.sha256(encoded).hexdigest()
        if (
            candidate_sha256 == self.runtime_contract_sha256
            and self.observation_contract is not None
        ):
            return
        if candidate_sha256 == self.runtime_rejected_sha256:
            return
        try:
            candidate = load_runtime_command_contract(
                self.runtime_contract_file,
                robot_id=self.args.robot_id,
                camera_frame=self.args.camera_frame,
                deployment_commit=self.args.deployment_commit,
            )
        except Exception as exc:  # noqa: BLE001 - malformed contracts fail closed
            self.runtime_contract_sha256 = None
            self.runtime_rejected_sha256 = candidate_sha256
            detail = f"{type(exc).__name__}:{exc}"
            self._park_runtime_contract(
                status="rejected",
                contract_sha256=candidate_sha256,
                detail=detail,
            )
            self.get_logger().error(
                f"runtime command contract rejected; sender parked: {detail}"
            )
            return
        try:
            next_sequence = self.transport.last_sequence() + 1
        except Exception as exc:  # noqa: BLE001 - retry transient Hub failures
            self.runtime_contract_sha256 = None
            self.runtime_rejected_sha256 = None
            detail = f"{type(exc).__name__}:{exc}"
            self._park_runtime_contract(
                status="waiting_for_hub_sequence",
                contract_sha256=candidate_sha256,
                detail=detail,
            )
            self.get_logger().warning(
                "validated runtime command contract is waiting for Hub "
                f"sequence resynchronization: {detail}"
            )
            return

        self.observation_contract = candidate
        self.sequence = next_sequence
        self.runtime_contract_sha256 = candidate_sha256
        self.runtime_rejected_sha256 = None
        self.runtime_contract_activations += 1
        self._write_runtime_receipt(
            status="active",
            contract_sha256=candidate_sha256,
            detail="validated_contract_applied_without_dds_restart",
        )
        self.get_logger().info(
            "runtime command contract activated without replacing the DDS "
            f"participant: transform={candidate.transform_version} "
            f"sequence={self.sequence}"
        )

    def on_slam_data(self, msg: String) -> None:
        self.latest_slam_metrics.update(msg.data)

    def on_synced(self, rgb_msg, depth_msg, info_msg, pose_msg) -> None:
        self._handle_synced(rgb_msg, depth_msg, info_msg, pose_msg, None)

    def on_latest_rgb(self, rgb_msg) -> None:
        self.rgb_cache.append(rgb_msg)

    def on_latest_rgb_info(self, rgb_info_msg) -> None:
        self.latest_rgb_info_msg = rgb_info_msg

    def on_synced_with_latest_rgb(
        self, depth_msg, info_msg, pose_msg
    ) -> None:
        self.geometry_tuples_seen += 1
        rgb_info_msg = self.latest_rgb_info_msg
        if not self.rgb_cache or rgb_info_msg is None:
            return
        depth_stamp_ns = stamp_to_ns(depth_msg.header.stamp)
        rgb_msg = min(
            self.rgb_cache,
            key=lambda candidate: abs(
                stamp_to_ns(candidate.header.stamp) - depth_stamp_ns
            ),
        )
        skew_s = abs(
            stamp_to_ns(rgb_msg.header.stamp) - depth_stamp_ns
        ) / 1e9
        self.closest_rgb_skew_ms = skew_s * 1e3
        if skew_s > self.args.latest_rgb_max_skew_s:
            self.rgb_cache_skew_rejections += 1
            return
        self._handle_synced(
            rgb_msg, depth_msg, info_msg, pose_msg, rgb_info_msg
        )

    def on_synced_registered(
        self, rgb_msg, depth_msg, info_msg, pose_msg, rgb_info_msg
    ) -> None:
        self._handle_synced(
            rgb_msg, depth_msg, info_msg, pose_msg, rgb_info_msg
        )

    def _handle_synced(
        self, rgb_msg, depth_msg, info_msg, pose_msg, rgb_info_msg
    ) -> None:
        self.frames_seen += 1
        if self.done:
            return
        if self.runtime_contract_file is not None:
            self._refresh_runtime_contract()
            if self.observation_contract is None:
                self.parked_geometry_tuples += 1
                if (
                    self.runtime_receipt_state is not None
                    and self.runtime_receipt_state[0] == "parked"
                    and (
                        self.parked_geometry_tuples
                        - self.last_parked_receipt_tuple
                        >= PARKED_RECEIPT_TUPLE_INTERVAL
                    )
                ):
                    status, contract_sha256, detail = (
                        self.runtime_receipt_state
                    )
                    self._write_runtime_receipt(
                        status=status,
                        contract_sha256=contract_sha256,
                        detail=detail,
                    )
                return
        now_mono = time.monotonic()
        if now_mono - self.last_upload_monotonic < self.min_period_s:
            return
        self.last_upload_monotonic = now_mono
        try:
            self.process(
                rgb_msg,
                depth_msg,
                info_msg,
                pose_msg,
                rgb_info_msg=rgb_info_msg,
            )
        except Exception as exc:  # noqa: BLE001 - one bad frame must not kill the node
            self.last_frame_error = f"{type(exc).__name__}:{exc}"
            self.last_frame_error_at_ns = time.time_ns()
            if (
                self.runtime_contract_file is not None
                and self.runtime_receipt_state is not None
                and self.runtime_receipt_state[0] == "active"
            ):
                status, contract_sha256, _detail = (
                    self.runtime_receipt_state
                )
                self._write_runtime_receipt(
                    status=status,
                    contract_sha256=contract_sha256,
                    detail="frame_processing_failed",
                )
            self.get_logger().error(f"frame processing failed: {exc}")
            return
        recovered_from_error = self.last_frame_error is not None
        self.last_frame_error = None
        self.last_frame_error_at_ns = None
        if (
            recovered_from_error
            and self.runtime_contract_file is not None
            and self.runtime_receipt_state is not None
            and self.runtime_receipt_state[0] == "active"
        ):
            status, contract_sha256, _detail = self.runtime_receipt_state
            self._write_runtime_receipt(
                status=status,
                contract_sha256=contract_sha256,
                detail="frame_upload_recovered",
            )
        if self.args.max_frames and self.frames_sent >= self.args.max_frames:
            self.get_logger().info(f"reached --max-frames {self.args.max_frames}; shutting down")
            self.done = True

    def _registration_transform(self) -> np.ndarray:
        if self.registration_T_rgb_from_depth is not None:
            return self.registration_T_rgb_from_depth
        if self.registration_tf_buffer is None:
            raise RuntimeError("RGB/depth registration TF buffer is unavailable")
        message = self.registration_tf_buffer.lookup_transform(
            self.args.rgb_optical_frame,
            self.args.depth_optical_frame,
            self.registration_time_zero,
            timeout=self.registration_timeout,
        )
        matrix = transform_message_matrix(message)
        self.registration_T_rgb_from_depth = matrix
        self.get_logger().info(
            "locked observed RealSense static registration "
            f"{self.args.rgb_optical_frame} <- {self.args.depth_optical_frame}: "
            f"translation_m={matrix[:3, 3].round(6).tolist()}"
        )
        return matrix

    def process(
        self,
        rgb_msg,
        depth_msg,
        info_msg,
        pose_msg,
        *,
        rgb_info_msg=None,
    ) -> None:
        contract = self.observation_contract
        if contract is None:
            raise RuntimeError("observation sender is parked")
        t0 = time.perf_counter()
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = depth_msg_to_png16_array(self.bridge, depth_msg)
        registration_coverage = None
        if self.args.register_rgb_to_depth:
            if rgb_info_msg is None:
                raise RuntimeError(
                    "RGB/depth registration requires synchronized RGB CameraInfo"
                )
            rgb, registered_depth_mask, registration_coverage = (
                register_rgb_onto_depth_grid(
                rgb,
                depth.astype(np.float32) * DEPTH_SCALE_M,
                camera_info_matrix(info_msg),
                camera_info_matrix(rgb_info_msg),
                self._registration_transform(),
            )
            )
            if registration_coverage < self.args.registration_min_coverage:
                raise RuntimeError(
                    "RGB/depth calibrated overlap below gate: "
                    f"{registration_coverage:.3f} < "
                    f"{self.args.registration_min_coverage:.3f}"
                )
            # The color imager has a narrower FOV than infra1. Keeping depth
            # outside their calibrated overlap would pair it with fallback
            # display pixels and silently violate the aligned-RGBD contract.
            depth = depth.copy()
            depth[~registered_depth_mask] = 0
        t1 = time.perf_counter()

        ok_rgb, jpeg = cv2.imencode(".jpg", rgb, [int(cv2.IMWRITE_JPEG_QUALITY), self.args.jpeg_quality])
        ok_depth, png = cv2.imencode(".png", depth, [int(cv2.IMWRITE_PNG_COMPRESSION), self.args.png_level])
        if not (ok_rgb and ok_depth):
            raise RuntimeError("JPEG/PNG encoding failed")
        rgb_bytes, depth_bytes = jpeg.tobytes(), png.tobytes()
        t2 = time.perf_counter()

        image_capture_ns = stamp_to_ns(rgb_msg.header.stamp)
        depth_capture_ns = stamp_to_ns(depth_msg.header.stamp)
        geometry_capture_ns = (
            depth_capture_ns
            if self.args.register_rgb_to_depth
            else image_capture_ns
        )
        pose_stamp_ns = stamp_to_ns(pose_msg.header.stamp)
        sync_skew_ns = abs(pose_stamp_ns - geometry_capture_ns)
        rgb_depth_sync_skew_ns = abs(image_capture_ns - depth_capture_ns)
        matrix = odom_to_matrix(pose_msg)
        shared_T_tracking = (
            None
            if contract.shared_tracking_calibration is None
            else contract.shared_tracking_calibration.shared_T_tracking
        )
        if shared_T_tracking is not None:
            matrix = apply_shared_tracking_alignment(
                matrix, shared_T_tracking
            )
        covariance_state, covariance_6x6 = classify_localization_state(
            list(pose_msg.pose.covariance))
        localization_state, localization_detail = self.latest_slam_metrics.apply(
            covariance_state, timeout_s=self.args.slam_health_timeout_s)
        health_snapshot = {
            "safety_state": "UNKNOWN", "localization_state": localization_state,
            "estop_engaged": False,
            "collision_avoidance_ready": False, "motor_controller_ready": False,
            "detail": localization_detail,
        }
        self.latest_health.set(health_snapshot)
        h, w = rgb.shape[:2]
        sent_ns = time.time_ns()
        if self.args.capture_time_source == "wall":
            # Rehearsal mode only: a replayed bag's header.stamp is whatever
            # historical time it was recorded at, which the hub's freshness
            # window would reject outright. Re-stamp to now, exactly like the
            # non-ROS replay sender, and never claim this is a real capture
            # time. Real live-camera operation must use --capture-time-source
            # header (the default) so capture_time_ns is the true ROS stamp.
            capture_time_ns = sent_ns - 50_000_000
        else:
            capture_time_ns = geometry_capture_ns
        metadata = {
            "robot_id": self.args.robot_id,
            "sequence": self.sequence,
            "capture_time_ns": capture_time_ns,
            "sent_time_ns": max(sent_ns, capture_time_ns),
            "pose": {
                "shared_T_camera": {
                    "parent_frame": "shared_world",
                    "child_frame": self.args.camera_frame,
                    "matrix": matrix,
                },
                "covariance_6x6": covariance_6x6,
                "transform_version": contract.transform_version,
            },
            "base_T_camera": (
                None
                if contract.base_camera_calibration is None
                else contract.base_camera_calibration.wire_transform()
            ),
            "intrinsics": {
                "width": w, "height": h,
                "fx": float(info_msg.k[0]), "fy": float(info_msg.k[4]),
                "cx": float(info_msg.k[2]), "cy": float(info_msg.k[5]),
                "distortion_model": info_msg.distortion_model or "none",
                "distortion": [],
            },
            "depth_scale_m": DEPTH_SCALE_M,
            "depth_min_m": 0.3,
            "depth_max_m": 5.0,
            "rgb_encoding": "jpeg",
            "depth_encoding": "png16",
            "rgb_size_bytes": len(rgb_bytes),
            "depth_size_bytes": len(depth_bytes),
            "rgb_sha256": hashlib.sha256(rgb_bytes).hexdigest(),
            "depth_sha256": hashlib.sha256(depth_bytes).hexdigest(),
            "object_goal": {
                "goal_id": "live-ros2-rehearsal",
                "category": contract.goal_category,
            },
            # Same snapshot just written to self.latest_health above. Valid
            # optimizer/IMU telemetry with absent covariance is capped at
            # DEGRADED; a fault forces LOST even if covariance appears plausible.
            "health": dict(health_snapshot),
            "mapping_only": contract.mapping_only,
        }

        def restamp(meta: dict) -> dict:
            now_ns = time.time_ns()
            meta = dict(meta)
            meta["capture_time_ns"] = now_ns - 50_000_000
            meta["sent_time_ns"] = now_ns
            return meta

        t3 = time.perf_counter()
        ack, attempts = self.transport.upload(metadata, rgb_bytes, depth_bytes, restamp)
        t4 = time.perf_counter()

        self.metrics.append({
            "sequence": self.sequence,
            "image_capture_ns": image_capture_ns,
            "depth_capture_ns": depth_capture_ns,
            "pose_sync_skew_ms": round(sync_skew_ns / 1e6, 2),
            "rgb_depth_sync_skew_ms": round(
                rgb_depth_sync_skew_ns / 1e6, 2
            ),
            "rgb_registration": (
                None
                if registration_coverage is None
                else {
                    "method": "depth_reprojection_via_realsense_static_tf",
                    "status": "observed_intrinsics_and_tf_source_derived_projection",
                    "rgb_topic": self.args.rgb_topic,
                    "depth_topic": self.args.depth_topic,
                    "rgb_optical_frame": self.args.rgb_optical_frame,
                    "depth_optical_frame": self.args.depth_optical_frame,
                    "valid_depth_overlap_ratio": round(
                        registration_coverage, 6
                    ),
                }
            ),
            "decode_ms": round((t1 - t0) * 1e3, 1),
            "encode_ms": round((t2 - t1) * 1e3, 1),
            "upload_ms": round((t4 - t3) * 1e3, 1),
            "attempts": attempts,
            "ack_status": ack.get("status"),
            "localization_state": localization_state,
            "localization_detail": localization_detail,
            "rgb_bytes": len(rgb_bytes),
            "depth_bytes": len(depth_bytes),
            "depth_source_encoding": depth_msg.encoding,
        })
        self.sequence += 1
        self.frames_sent += 1
        if self.frames_sent % 10 == 0 or self.frames_sent <= 3:
            self.get_logger().info(
                f"sent {self.frames_sent} (seq={metadata['sequence']}, "
                f"ack={ack.get('status')}, pose_skew={sync_skew_ns / 1e6:.1f}ms, "
                f"upload={round((t4 - t3) * 1e3, 1)}ms)"
            )

    def write_summary(self, path: str) -> None:
        summary = {
            "capture_time_source": self.args.capture_time_source,
            "geometry_tuples_seen": self.geometry_tuples_seen,
            "rgb_cache_skew_rejections": self.rgb_cache_skew_rejections,
            "closest_rgb_skew_ms": self.closest_rgb_skew_ms,
            "frames_seen_by_synchronizer": self.frames_seen,
            "frames_sent": self.frames_sent,
            "retries_total": self.transport.retries_total,
            "http_session_resets_total": self.transport.session_resets_total,
            "runtime_contract_activations": self.runtime_contract_activations,
            "parked_geometry_tuples": self.parked_geometry_tuples,
            "mean_upload_ms": round(float(np.mean([m["upload_ms"] for m in self.metrics])), 1)
            if self.metrics else None,
            "mean_pose_sync_skew_ms": round(float(np.mean([m["pose_sync_skew_ms"] for m in self.metrics])), 1)
            if self.metrics else None,
            "heartbeats_sent": self.heartbeat_thread.beats_sent if self.heartbeat_thread else None,
            "heartbeats_failed": self.heartbeat_thread.beats_failed if self.heartbeat_thread else None,
        }
        with open(path, "w") as f:
            json.dump({"summary": summary, "frames": self.metrics}, f, indent=2)
        print(json.dumps(summary, indent=2))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:18089")
    parser.add_argument("--robot-id", default="robot-0")
    parser.add_argument("--transform-version", default="wsj-live-odom-test-v1")
    parser.add_argument("--rgb-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--info-topic", default="/camera/camera/aligned_depth_to_color/camera_info")
    parser.add_argument("--pose-topic", default="/slam/keyframe_odom",
                        help="Chosen because TinyNav's own build_map_node.py subscribes to exactly "
                             "this topic, exactly-timestamp-paired with /slam/keyframe_image and "
                             "/slam/keyframe_depth, for this same purpose — see the module "
                             "docstring's revised UNVERIFIED section. Never run against wsj "
                             "(offline). Override with /semantic_mapping/camera_pose (the old "
                             "map-relocalized topic, pre-pivot) if this turns out to be wrong.")
    parser.add_argument(
        "--camera-frame",
        default=CAMERA_FRAME,
        help="child-frame label for shared_T_camera; use 'camera' with the "
             "TinyNav-native keyframe image/depth/odom tuple",
    )
    parser.add_argument("--capture-time-source", choices=["header", "wall"], default="header",
                        help="'header' (default) trusts the real ROS image timestamp, correct "
                             "for a live camera. 'wall' re-stamps to now — rehearsal-only, for "
                             "replaying a historical bag whose header stamps would otherwise be "
                             "rejected as stale.")
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument("--max-frames", type=int, default=0, help="0 = unbounded")
    parser.add_argument("--jpeg-quality", type=int, default=92)
    parser.add_argument("--png-level", type=int, default=1)
    parser.add_argument("--sync-queue-size", type=int, default=40)
    parser.add_argument("--sync-slop", type=float, default=0.05)
    parser.add_argument(
        "--register-rgb-to-depth",
        action="store_true",
        help=(
            "calibrated hybrid path: sample the RealSense color stream onto "
            "TinyNav's left-infrared keyframe-depth grid using CameraInfo and "
            "the observed static TF; pose and output intrinsics remain in the "
            "TinyNav depth frame"
        ),
    )
    parser.add_argument(
        "--rgb-info-topic",
        default="/camera/camera/color/camera_info",
    )
    parser.add_argument(
        "--latest-rgb-for-depth",
        action="store_true",
        help=(
            "synchronize only the depth CameraInfo/odometry geometry tuple and "
            "pair it with the closest timestamped frame in a bounded RGB "
            "cache; intended for delayed continuous depth streams whose "
            "independent RGB stream can starve a five-way "
            "ApproximateTimeSynchronizer"
        ),
    )
    parser.add_argument(
        "--rgb-cache-size",
        type=int,
        default=90,
        help="bounded number of recent RGB messages retained for timestamp matching",
    )
    parser.add_argument(
        "--latest-rgb-max-skew-s",
        type=float,
        default=0.05,
        help="maximum accepted absolute timestamp skew for cached RGB",
    )
    parser.add_argument(
        "--rgb-optical-frame",
        default="camera_color_optical_frame",
    )
    parser.add_argument(
        "--depth-optical-frame",
        default="camera_infra1_optical_frame",
    )
    parser.add_argument("--registration-tf-timeout-s", type=float, default=1.0)
    parser.add_argument(
        "--registration-min-coverage",
        type=float,
        default=0.38,
        help=(
            "minimum fraction of valid depth samples that project inside the "
            "color imager; the observed WSJ 848x480 color/640x480 TinyNav "
            "depth profile is stable at 0.412--0.418"
        ),
    )
    parser.add_argument("--goal-category", default="chair")
    parser.add_argument("--metrics-out", default="focus_ros_sender_metrics.json")
    parser.add_argument("--heartbeat-hz", type=float, default=2.0,
                        help="independent liveness/health ping rate, decoupled from the sync "
                             "callback and from upload retries; 0 disables it")
    parser.add_argument("--slam-data-topic", default="/slam/data",
                        help="TinyNav optimizer JSON used as an independent fail-closed pose "
                             "health gate")
    parser.add_argument("--slam-health-timeout-s", type=float, default=5.0,
                        help="mark localization UNKNOWN when /slam/data is older than this")
    parser.add_argument("--base-camera-calibration-file", type=Path)
    parser.add_argument("--shared-tracking-calibration-file", type=Path)
    parser.add_argument("--shared-frame-calibration-id", default="")
    parser.add_argument("--enable-command-capable-observations", action="store_true")
    parser.add_argument("--activation-confirmation", default="")
    parser.add_argument("--runtime-command-contract-file", type=Path)
    parser.add_argument("--runtime-command-receipt-file", type=Path)
    parser.add_argument("--deployment-commit", default="")
    args = parser.parse_args()
    if args.registration_tf_timeout_s <= 0.0:
        parser.error("--registration-tf-timeout-s must be positive")
    if not 0.0 < args.registration_min_coverage <= 1.0:
        parser.error("--registration-min-coverage must be in (0, 1]")
    if args.latest_rgb_max_skew_s <= 0.0:
        parser.error("--latest-rgb-max-skew-s must be positive")
    if args.rgb_cache_size <= 0:
        parser.error("--rgb-cache-size must be positive")
    if args.latest_rgb_for_depth and not args.register_rgb_to_depth:
        parser.error("--latest-rgb-for-depth requires --register-rgb-to-depth")

    args.base_camera_calibration = None
    args.shared_tracking_calibration = None
    if args.runtime_command_contract_file is not None:
        if args.enable_command_capable_observations:
            parser.error(
                "--runtime-command-contract-file cannot be combined with "
                "--enable-command-capable-observations"
            )
        if args.runtime_command_receipt_file is None:
            parser.error(
                "--runtime-command-receipt-file is required with the runtime "
                "command contract"
            )
        if (
            len(args.deployment_commit) != 40
            or any(
                character not in "0123456789abcdef"
                for character in args.deployment_commit
            )
        ):
            parser.error(
                "--deployment-commit must be an explicit 40-character "
                "lowercase Git commit"
            )
        if args.capture_time_source != "header":
            parser.error(
                "runtime command observations require real header timestamps"
            )
        if args.heartbeat_hz != 0:
            parser.error(
                "set --heartbeat-hz 0; the armed v2 receiver owns command "
                "health heartbeats"
            )
        if (
            args.base_camera_calibration_file is not None
            or args.shared_tracking_calibration_file is not None
            or args.shared_frame_calibration_id
        ):
            parser.error(
                "runtime command calibration comes only from the atomic "
                "runtime contract"
            )
        print(
            "runtime-configurable command observation participant enabled; "
            "it stays parked until a checked contract is atomically applied "
            "and still has no planner or actuator output."
        )
    elif args.runtime_command_receipt_file is not None or args.deployment_commit:
        parser.error(
            "runtime receipt/deployment arguments require "
            "--runtime-command-contract-file"
        )
    elif args.enable_command_capable_observations:
        if args.activation_confirmation != "COMMAND_CAPABLE_OBSERVATION_ONLY":
            parser.error(
                "command-capable observations require --activation-confirmation "
                "COMMAND_CAPABLE_OBSERVATION_ONLY"
            )
        if args.base_camera_calibration_file is None:
            parser.error(
                "--base-camera-calibration-file is required for command-capable observations"
            )
        if args.shared_tracking_calibration_file is None:
            parser.error(
                "--shared-tracking-calibration-file is required for "
                "command-capable observations"
            )
        if not args.shared_frame_calibration_id:
            parser.error(
                "--shared-frame-calibration-id is required for "
                "command-capable observations"
            )
        if args.capture_time_source != "header":
            parser.error("command-capable observations require real header timestamps")
        if args.heartbeat_hz != 0:
            parser.error(
                "set --heartbeat-hz 0; the armed v2 receiver owns command health heartbeats"
            )
        args.base_camera_calibration = load_base_camera_calibration(
            args.base_camera_calibration_file,
            expected_robot_id=args.robot_id,
            expected_camera_frame=args.camera_frame,
        )
        args.shared_tracking_calibration = load_shared_tracking_calibration(
            args.shared_tracking_calibration_file,
            robot_id=args.robot_id,
            expected_transform_version=args.transform_version,
            expected_calibration_id=args.shared_frame_calibration_id,
        )
        print(
            "command-capable observation metadata enabled; this sender still has no "
            "planner or actuator output. shared tracking alignment "
            f"sha256={args.shared_tracking_calibration.source_sha256}; "
            "v2 receiver heartbeat remains authoritative."
        )
    elif (
        args.base_camera_calibration_file is not None
        or args.shared_tracking_calibration_file is not None
        or args.shared_frame_calibration_id
    ):
        parser.error(
            "calibration arguments require --enable-command-capable-observations"
        )

    args.token = os.environ.get("FOCUS_ROBOT_TOKEN", "")
    if not args.token:
        print("FOCUS_ROBOT_TOKEN is not set", file=sys.stderr)
        return 2

    rclpy.init()
    node = FocusRosSender(args)

    def handle_signal(_signum, _frame):
        node.done = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        if node.heartbeat_thread is not None:
            node.heartbeat_thread.stop()
            node.heartbeat_thread.join(timeout=2.0)
        node.write_summary(args.metrics_out)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if node.frames_sent > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
