#!/usr/bin/env python3
"""Observation sender for a Yunji WATER delivery-robot chassis (standalone overlay).

Fundamentally different integration shape from the wsj Unitree Go2 sender: this
robot has no ROS 2 topic stream to subscribe to directly. It exposes a
vendor TCP API (port 31001, documented in
`~/workspace/tinynav/yunji-water-robot/docs/vendor/yunji_water_development_guide.md`
on nyush-nuc) for status/pose, and a ROS1 rosbridge WebSocket (port 9090) for
both RGB and raw metric depth (the chassis also has a `web_video_server` HTTP
snapshot endpoint on port 8810, but it turned out to be unreliable from every
HTTP client tried — see the approximations list below — so this sender does
not use it). This sender only ever reads from those two; it never calls
`/api/move`, `/api/joy_control` or any other command endpoint, so it cannot
move the robot.

Two extrinsics that were originally unmeasured placeholders were later
replaced with real values read from the robot's own `/tf` (2026-07-18,
second pass — `/tf_static` never publishes anything on this firmware, but
plain `/tf` carries the same static mount edges as regular messages; see
`audit/YUNJI_WATER_SENDER.md` for the full `/tf` dump):

  - `camera_front_up_depth_optical_frame` and `camera_front_up_color_optical_frame`
    are EXACTLY co-located (confirmed: composing the two frames' published
    edges back to a common parent yields an identity transform — zero
    translation, zero rotation). Depth and RGB therefore differ only in
    intrinsics/FOV, not viewpoint, so per-pixel registration is a proper
    reprojection through each camera's own intrinsics
    (`reproject_rgb_onto_depth_grid()`), not a resize-based approximation.
  - `base_link -> camera_front_up_depth_optical_frame` (composed through
    `camera_front_up_link` and the optical-frame rotation, both read from
    `/tf`) is `MEASURED_T_BASE_LINK_CAMERA` below: translation
    (0.2646, 0, 0.299) m and a real tilt — not the height-only, zero-tilt
    guess this file originally shipped with. `current_pose` (the chassis's
    2D SLAM pose) is composed with this real extrinsic instead of a
    placeholder height.

Third pass (2026-07-19): pose source switched from `/api/robot_status`'s
`current_pose` to rosbridge `/sensors_fusion/odom`, and the reasoning is
worth recording because it reverses an earlier design choice. `current_pose`
is AMCL's pose in the chassis's `map` frame, which only exists once a saved
map has been built and loaded (`/api/map/set_current_map`) — i.e. it
requires "build a map first, then navigate/localize within it." Faithfully
reproducing the original Habitat multi-agent code's semantics instead
requires each robot tracking its own pose from scratch, the way Habitat's
per-episode GPS/Compass sensors are relative to that episode's own start
pose, not a persistent absolute map. `/sensors_fusion/odom` (confirmed live:
`frame_id=odom`, `child_frame_id=base_link`, ~20 Hz fused IMU+wheel+laser+
visual odometry per the vendor README) is exactly that — a live,
map-independent pose that exists from the moment the chassis boots, with no
dependency on any saved map. The shared coordinate frame across robots is
now expected to be established once per exploration session (analogous to
Habitat resetting both agents to the same `episode.start_position`), not
baked into a persistent per-robot map calibration — see
`hub/tools/calibrate_shared_frame.py`.

Trade-off, stated plainly: `odom` drifts over time with no loop-closure
correction (unlike AMCL's `map`-frame pose, which is corrected against the
saved map every cycle, and unlike Habitat's own ground-truth GPS, which
never drifts at all). This is judged acceptable for faithfully reproducing
the source algorithm's semantics; periodic re-calibration or a supplementary
drift-correction mechanism is future work, not solved here.

Known approximation still remaining, stated up front rather than hidden:
  - Depth's raw ROS encoding (`16UC1` on `/camera_front_up/depth/image_raw`,
    the default `--depth-topic` since 2026-07-20) is millimetres, not
    metres, empirically confirmed by cross-checking against
    `/camera_front_up/depth/points` (real metres per REP103). This sender
    divides by 1000 before encoding to the wire's `depth_scale_m`
    convention. (Until 2026-07-20 the default was
    `depth_registered/image_raw`, tagged `32FC1` -- confirmed live that
    topic's pixel values are a bare dtype cast of this same raw millimetre
    data with no actual geometric registration ever applied to it
    (`depth_align=false`, `color_depth_synchronization=false`, no
    `camera_info` published for it) -- switched to the honestly-named raw
    topic; `fetch_depth_frame` dispatches on the message's own `encoding`
    field so either still works.)
  - The `web_video_server` HTTP snapshot endpoint (port 8810) was tried
    first and found unreliable: `requests`, `urllib.request`, `http.client`
    with the request line forced to HTTP/1.0, and even shelling out to
    `curl` all intermittently hit connection resets or empty replies against
    the real device, including with generous delays between requests (tested
    up to 10 s) and regardless of ordering relative to TCP-API calls. Root
    cause not pinned down — most likely a resource limit or bug in this
    embedded server itself, not any one client library. rosbridge (port
    9090), in contrast, was reliable in every trial for the depth topic
    throughout the same investigation, so RGB is fetched over rosbridge too
    instead of HTTP. Every network call to the robot is additionally wrapped
    in `retry_call()` (short exponential backoff) as defense in depth. See
    `audit/YUNJI_WATER_SENDER.md` for the full investigation.

Because of the above, `transform_version` is a distinct, obviously-a-test
label (`yunji-water-robot-test-v1` by default) and every upload is
`mapping_only=true`. Health fields ARE real telemetry from `/api/robot_status`
(battery, e-stop, error code) — better signal than the wsj replay sender ever
had, since this robot's API exposes it directly.

Fourth pass (2026-07-21): added `--camera-source local-realsense` as an
alternative to the rosbridge `camera_front_up` path above. This is a
physically different camera -- an Intel RealSense D405 connected directly to
this sender's own host (nyush-nuc) over USB, not anything on the chassis --
mounted higher up for a better mapping vantage point (the chassis-mounted
camera_front_up sits low). Read via `pyrealsense2` (native SDK, installed
locally on nyush-nuc; udev rules added so the non-root user can open both the
raw USB device and the `/dev/video*` nodes uvcvideo creates for it). Real,
material improvements over the rosbridge path in this mode:
  - Depth-to-color alignment is librealsense's own `rs.align`, using the
    device's real factory-calibrated intrinsics/extrinsics -- not the
    hand-rolled `reproject_rgb_onto_depth_grid()` against K_DEPTH/K_RGB
    constants of undocumented provenance (see the caveat below those
    constants). Output resolution is the color stream's native resolution
    (depth resampled onto it), not depth's tiny 160x120 grid, so RGB reaches
    RedNet at full quality instead of being downsampled to match depth.
  - Intrinsics (fx/fy/cx/cy/width/height) are read live from the device's own
    calibration every run, not hardcoded.
D405 caveat, stated plainly: this model's factory-specified sweet spot is
short range (roughly 7cm-50cm); it was not designed for room-scale distances.
It still reports data further out, but accuracy beyond its spec'd range has
not been characterized here -- `--depth-min-m`/`--depth-max-m` remain the
same metadata hints used for the chassis camera, not a D405-specific
measurement. Pose (`/sensors_fusion/odom`), health/heartbeat
(`/api/robot_status`), and everything else in this file is unchanged and
still comes from the chassis over rosbridge/TCP regardless of which
`--camera-source` is selected.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import socket
import struct
import sys
import threading
import time
import uuid

import cv2
import numpy as np


# --------------------------------------------------------------- WATER TCP API

class WaterTcpClient:
    """Minimal client for the documented WATER TCP API (port 31001).

    Opens a fresh connection per request and closes it immediately after,
    matching the vendor guide's own one-shot example — simple and avoids any
    risk of a stale long-lived connection going bad on this robot's embedded
    server (see audit/YUNJI_WATER_SENDER.md for what else was tried there).
    """

    def __init__(self, host: str, port: int = 31001, timeout_s: float = 4.0) -> None:
        self.host, self.port, self.timeout_s = host, port, timeout_s

    def request(self, path: str, **params) -> dict:
        request_id = uuid.uuid4().hex[:12]
        query = "&".join(f"{k}={v}" for k, v in params.items())
        line = f"{path}?uuid={request_id}" + (f"&{query}" if query else "") + "\n"
        with socket.create_connection((self.host, self.port), timeout=self.timeout_s) as sock:
            sock.settimeout(self.timeout_s)
            sock.sendall(line.encode("utf-8"))
            reader = sock.makefile("rb")
            for _ in range(20):
                raw = reader.readline()
                if not raw:
                    raise ConnectionError("WATER TCP API closed the connection")
                message = json.loads(raw)
                if message.get("type") == "response" and message.get("uuid") == request_id:
                    return message
        raise RuntimeError(f"no matching response for {path} within 20 reads")

    def close(self) -> None:
        pass


# ------------------------------------------------------------------ rosbridge

class _WebSocket:
    """Hardened minimal rosbridge WebSocket client: correct FIN/opcode
    parsing (unlike a naive single-frame-only reader), so it doesn't wedge on
    ping frames or fragmented frames from a large depth payload."""

    def __init__(self, host: str, port: int, timeout_s: float = 6.0) -> None:
        self.sock = socket.create_connection((host, port), timeout=timeout_s)
        self.sock.settimeout(timeout_s)
        key = base64.b64encode(os.urandom(16)).decode()
        request = (
            f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
            f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
        )
        self.sock.sendall(request.encode())
        response = b""
        while b"\r\n\r\n" not in response:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("closed during WebSocket handshake")
            response += chunk
        if b" 101 " not in response.split(b"\r\n", 1)[0]:
            raise RuntimeError(response.decode("latin-1", "replace"))

    def send_json(self, value: dict) -> None:
        payload = json.dumps(value, separators=(",", ":")).encode()
        mask = os.urandom(4)
        length = len(payload)
        header = bytearray([0x81])
        if length < 126:
            header.append(0x80 | length)
        elif length < 65536:
            header.append(0x80 | 126)
            header += struct.pack(">H", length)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", length)
        header += mask
        self.sock.sendall(bytes(header) + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))

    def _recv_exact(self, size: int) -> bytes:
        data = b""
        while len(data) < size:
            chunk = self.sock.recv(size - len(data))
            if not chunk:
                raise ConnectionError("closed")
            data += chunk
        return data

    def _recv_frame(self) -> tuple[int, bytes]:
        header = self._recv_exact(2)
        opcode = header[0] & 0x0F
        masked = bool(header[1] & 0x80)
        length = header[1] & 0x7F
        if length == 126:
            length = struct.unpack(">H", self._recv_exact(2))[0]
        elif length == 127:
            length = struct.unpack(">Q", self._recv_exact(8))[0]
        mask_key = self._recv_exact(4) if masked else None
        payload = self._recv_exact(length)
        if mask_key:
            payload = bytes(b ^ mask_key[i % 4] for i, b in enumerate(payload))
        return opcode, payload

    def _send_pong(self, payload: bytes) -> None:
        mask = os.urandom(4)
        header = bytearray([0x8A, 0x80 | len(payload)]) + mask
        self.sock.sendall(bytes(header) + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))

    def recv_json_message(self, max_control_frames: int = 50) -> dict:
        for _ in range(max_control_frames):
            opcode, payload = self._recv_frame()
            if opcode in (0x1, 0x2):
                return json.loads(payload.decode("utf-8"))
            if opcode == 0x9:
                self._send_pong(payload)
                continue
            if opcode == 0x8:
                raise ConnectionError("server closed the WebSocket")
        raise RuntimeError("too many non-data frames without a message")

    def close(self) -> None:
        self.sock.close()


def fetch_one_topic(host: str, port: int, topic: str, timeout_s: float = 6.0) -> dict:
    """Connect, subscribe, wait for exactly one message, disconnect.

    One-shot per call rather than a persistent subscription: simpler failure
    mode (a stuck depth fetch just times out and retries next keyframe
    instead of wedging a long-lived connection for the rest of the run).
    """
    ws = _WebSocket(host, port, timeout_s=timeout_s)
    try:
        ws.send_json({"op": "subscribe", "topic": topic, "id": "fetch-1", "queue_length": 1})
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            ws.sock.settimeout(max(0.2, deadline - time.monotonic()))
            message = ws.recv_json_message()
            if message.get("op") == "publish" and message.get("topic") == topic:
                return message["msg"]
        raise TimeoutError(f"no message on {topic} within {timeout_s}s")
    finally:
        try:
            ws.send_json({"op": "unsubscribe", "topic": topic, "id": "fetch-1"})
        except OSError:
            pass
        ws.close()


# ------------------------------------------------------------- hub transport

class HubTransport:
    """Same resume/retry contract as the other robot_overlay senders."""

    def __init__(self, base_url, robot_id, token, timeout_s=10.0,
                 max_retries=8, backoff_base_s=0.5, backoff_cap_s=8.0):
        import requests

        self.base_url = base_url.rstrip("/")
        self.robot_id = robot_id
        self.session = requests.Session()
        self.session.headers["X-Robot-Token"] = token
        self.timeout_s = timeout_s
        self.max_retries = max_retries
        self.backoff_base_s = backoff_base_s
        self.backoff_cap_s = backoff_cap_s
        self.retries_total = 0

    def last_sequence(self) -> int:
        response = self.session.get(
            f"{self.base_url}/v1/robots/{self.robot_id}/observations/latest", timeout=self.timeout_s)
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
                    files={"rgb": ("rgb", rgb_bytes, "image/jpeg"),
                           "depth": ("depth", depth_bytes, "image/png")},
                    timeout=self.timeout_s,
                )
                if response.status_code in (200, 201):
                    return response.json(), attempt
                if 400 <= response.status_code < 500 and response.status_code not in (408, 429):
                    raise RuntimeError(
                        f"hub rejected seq {metadata['sequence']}: "
                        f"{response.status_code} {response.text[:300]}")
            except (requests.ConnectionError, requests.Timeout):
                pass
            if attempt > self.max_retries:
                raise RuntimeError(f"giving up on seq {metadata['sequence']} after {attempt} attempts")
            self.retries_total += 1
            delay = min(self.backoff_cap_s, self.backoff_base_s * (2 ** (attempt - 1)))
            time.sleep(delay)
            metadata = restamp(metadata)


class LatestLocalizationState:
    """Thread-safe holder for the main loop's most recently computed
    localization_state, so the independent heartbeat thread can include it
    without itself depending on the (slow, RGBD-coupled) odometry fetch.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value = "UNKNOWN"

    def set(self, value: str) -> None:
        with self._lock:
            self._value = value

    def get(self) -> str:
        with self._lock:
            return self._value


class HeartbeatThread(threading.Thread):
    """Independent 2Hz liveness+health ping, decoupled from the main RGBD
    fetch/encode/upload cycle (which was measured at ~1.8s/cycle, fetch-
    bound — see audit/YUNJI_WATER_SENDER.md soak results) so a stalled or
    slow RGBD path doesn't also blind the hub to health changes.

    Deliberately polls ONLY the fast WATER TCP API (/api/robot_status,
    millisecond-scale) on its own connection — not rosbridge, not the RGBD
    topics — for the fastest-changing safety-relevant fields (e-stop,
    error_code, battery). localization_state is not independently
    re-derived here (that would mean a second concurrent rosbridge
    connection polling /sensors_fusion/odom, doubling this sender's network
    footprint on the robot for a field that changes less abruptly than
    e-stop/battery); it reuses whatever the main loop most recently
    computed via LatestLocalizationState, which lags behind the RGBD cycle
    but is still far better than the previous total silence between
    observations.
    """

    def __init__(self, *, robot_host: str, tcp_port: int, base_url: str, robot_id: str,
                 token: str, localization_state: LatestLocalizationState, period_s: float = 0.5) -> None:
        super().__init__(daemon=True, name="yunji-heartbeat")
        import requests

        self.tcp = WaterTcpClient(robot_host, tcp_port)
        self.session = requests.Session()
        self.session.headers["X-Robot-Token"] = token
        self.base_url = base_url.rstrip("/")
        self.robot_id = robot_id
        self.localization_state = localization_state
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
        import requests

        status_resp = self.tcp.request("/api/robot_status")
        status = status_resp.get("results", {})
        estop = bool(status.get("estop_state") or status.get("hard_estop_state"))
        error_code = str(status.get("error_code", "00000000"))
        healthy = (not estop) and error_code == "00000000"
        health = {
            "safety_state": "READY" if healthy else ("ESTOP" if estop else "HOLD"),
            "localization_state": self.localization_state.get(),
            "estop_engaged": estop,
            "collision_avoidance_ready": healthy,
            "motor_controller_ready": healthy,
            "battery_percent": float(status.get("power_percent", 0.0)),
            "detail": f"heartbeat error_code={error_code} move_status={status.get('move_status')}",
        }
        body = {
            "robot_id": self.robot_id,
            "sent_time_ns": time.time_ns(),
            "health": health,
        }
        self.session.post(
            f"{self.base_url}/v1/robots/{self.robot_id}/heartbeat", json=body, timeout=2.0,
        ).raise_for_status()

    def stop(self) -> None:
        self.stop_event.set()


def retry_call(fn, *, attempts: int = 4, base_delay_s: float = 0.4, label: str = ""):
    """Retry a flaky call to the robot with short exponential backoff.

    Empirically needed: both the TCP API and the HTTP snapshot endpoint
    intermittently raise a connection-reset error that clears up within a
    few seconds. See the module docstring for what was actually observed.
    """
    last_exc = None
    for attempt in range(1, attempts + 1):
        try:
            return fn()
        except Exception as exc:  # noqa: BLE001 - genuinely any transient network error
            last_exc = exc
            if attempt < attempts:
                time.sleep(base_delay_s * (2 ** (attempt - 1)))
    raise RuntimeError(f"{label or fn}: failed after {attempts} attempts: {last_exc}") from last_exc


# --------------------------------------------------------------------- frame

DEPTH_SCALE_M = 0.001
CAMERA_FRAME = "camera_front_up_depth_optical_frame"
ROBOT_MAP_FRAME_NOTE = (
    "/sensors_fusion/odom is the chassis's own live odom-frame pose (no saved "
    "map dependency, drifts uncorrected); still not shared_world without a "
    "session-start calibration (see hub/tools/calibrate_shared_frame.py)"
)

# T_base_link_camera_depth_optical, measured via the real /tf tree on
# 2026-07-18 (base_link -> camera_front_up_link -> camera_front_up_depth_frame
# -> camera_front_up_depth_optical_frame; the middle hop is an identity edge).
# Row-major 4x4. See the module docstring and audit/YUNJI_WATER_SENDER.md.
MEASURED_T_BASE_LINK_CAMERA = np.array([
    [0.0, -0.741297, 0.671178, 0.264600],
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, -0.671178, -0.741297, 0.299000],
    [0.0, 0.0, 0.0, 1.0],
])

# T_base_link_camera for the D405 added 2026-07-21 for --camera-source
# local-realsense (see the module docstring's fourth pass). Physically a
# different camera at a different mount point from camera_front_up above --
# this is NOT a replacement for MEASURED_T_BASE_LINK_CAMERA, both constants
# stay in the file and are selected by --camera-source at the call site.
#
# User-reported measurement (2026-07-21), not independently verified against
# /tf (this camera is bolted to nyush-nuc's mount, off the chassis's own
# published frames, so there is no /tf edge for it to cross-check against):
# translation 12.7cm right / 20.3cm forward / 33.5cm up from base_link's
# origin, mounted level -- no pitch, yaw, or roll (camera's optical axis is
# parallel to the chassis's own forward direction, image horizon level).
# Converted to base_link's REP-103 axes (x-forward, y-left, z-up): right is
# -y, so translation = (0.203, -0.127, 0.335) m. The rotation below is the
# fixed base_link->camera_optical_frame rotation for a body-frame-aligned,
# zero-tilt camera (REP-103 optical convention: z-forward, x-right, y-down),
# not a measurement -- it follows directly from "mounted level, no tilt".
MEASURED_T_BASE_LINK_CAMERA_D405 = np.array([
    [0.0, 0.0, 1.0, 0.203],
    [-1.0, 0.0, 0.0, -0.127],
    [0.0, -1.0, 0.0, 0.335],
    [0.0, 0.0, 0.0, 1.0],
])

# T_base_link_camera for the D455 that replaced the D405 (same 2026-07-21
# session): user-reported as the same mount, moved 10cm higher on Z -- same
# right/forward offset and same "mounted level, no tilt" rotation as the
# D405 above, translation z = 0.335 + 0.10 = 0.435m. Same caveat: not
# independently verified against /tf.
MEASURED_T_BASE_LINK_CAMERA_D455 = np.array([
    [0.0, 0.0, 1.0, 0.203],
    [-1.0, 0.0, 0.0, -0.127],
    [0.0, -1.0, 0.0, 0.435],
    [0.0, 0.0, 0.0, 1.0],
])

# camera_front_up_depth_optical_frame and camera_front_up_color_optical_frame
# were confirmed co-located via the same /tf tree (composing their edges back
# to a common parent yields an exact identity transform) — RGB and depth
# differ only in intrinsics, not viewpoint.
T_DEPTH_OPTICAL_TO_COLOR_OPTICAL = np.eye(4)

# Depth/RGB intrinsics for camera_front_up. Provenance caveat, stated
# plainly: these are NOT live-fetched from a camera_info topic, because
# neither /camera_front_up/depth/camera_info nor .../rgb/camera_info
# publishes anything on this firmware (confirmed by subscribing for 20s and
# getting zero messages on either — checked 2026-07-19; by contrast,
# /camera_front_down/depth/camera_info does publish, so this is specific to
# this camera_front_up driver, not a rosbridge-wide limitation). How these
# exact numbers were originally obtained is not documented anywhere in this
# workspace's audit trail; treat them as inherited, not independently
# re-derived. Since they cannot be cross-checked against a live camera_info
# message, `verify_intrinsics_match_frame_size()` below is a best-effort
# runtime guard instead: it fails loud if a live frame's actual resolution
# ever stops matching the resolution these constants were evidently
# calibrated for (inferred from where their principal point sits), rather
# than silently keeping stale numbers.
K_DEPTH = np.array([[95.5085678100586, 0, 77.49767303466797],
                    [0, 95.5085678100586, 60.664794921875], [0, 0, 1.0]])
K_RGB = np.array([[359.51263427734375, 0, 322.7903137207031],
                  [0, 359.51263427734375, 179.68907165527344], [0, 0, 1.0]])
K_DEPTH_ASSUMED_SIZE = (160, 120)   # (width, height) implied by K_DEPTH's principal point
K_RGB_ASSUMED_SIZE = (640, 360)     # (width, height) implied by K_RGB's principal point


def verify_intrinsics_match_frame_size(depth_shape: tuple[int, int], rgb_shape: tuple[int, int]) -> None:
    """Raises if a live frame's resolution no longer matches what K_DEPTH/K_RGB
    were evidently calibrated for, so a silent firmware/resolution change
    can't quietly misregister RGB onto depth or feed wrong intrinsics
    downstream. Only checked once, on the first frame (resolution does not
    change mid-session on this hardware).
    """
    depth_h, depth_w = depth_shape
    rgb_h, rgb_w = rgb_shape[:2]
    if (depth_w, depth_h) != K_DEPTH_ASSUMED_SIZE:
        raise RuntimeError(
            f"depth frame is {depth_w}x{depth_h} but K_DEPTH was calibrated for "
            f"{K_DEPTH_ASSUMED_SIZE[0]}x{K_DEPTH_ASSUMED_SIZE[1]} — intrinsics are stale, refusing to guess")
    if (rgb_w, rgb_h) != K_RGB_ASSUMED_SIZE:
        raise RuntimeError(
            f"rgb frame is {rgb_w}x{rgb_h} but K_RGB was calibrated for "
            f"{K_RGB_ASSUMED_SIZE[0]}x{K_RGB_ASSUMED_SIZE[1]} — intrinsics are stale, refusing to guess")


def quat_to_matrix(x: float, y: float, z: float, w: float,
                   tx: float, ty: float, tz: float) -> np.ndarray:
    """Full SE(3) from a real quaternion + translation (not a 2D-only theta)."""
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n > 0:
        x, y, z, w = x / n, y / n, z / n, w / n
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def load_shared_frame_transform(path: str | None) -> np.ndarray | None:
    """Loads the ``shared_world_from_other_odom`` matrix written by
    ``hub/tools/calibrate_shared_frame.py``. Returns None if no calibration
    file was given, in which case published poses stay in this robot's own
    local odometry frame (still labelled ``shared_world`` in the wire
    protocol, but only true relative to itself until this is applied).
    """
    if not path:
        return None
    with open(path, encoding="utf-8") as handle:
        calib = json.load(handle)
    matrix = calib["shared_world_from_other_odom"]["matrix"]
    return np.array(matrix, dtype=np.float64).reshape(4, 4)


def pose_to_matrix(odom_pose: dict, T_base_link_camera: np.ndarray) -> list[float]:
    """T_odom_camera = T_odom_baselink(from /sensors_fusion/odom) @ T_base_link_camera.

    Uses the real quaternion + translation reported by the live fused
    odometry (not a 2D x/y/theta-only approximation) — on this chassis
    z stays 0 and roll/pitch stay 0 in practice, but nothing here assumes
    that; it composes whatever SE(3) the odometry actually reports.

    `T_base_link_camera` is caller-supplied (MEASURED_T_BASE_LINK_CAMERA for
    the chassis's own camera_front_up, MEASURED_T_BASE_LINK_CAMERA_D405 for
    the D405 added 2026-07-21) rather than hardcoded here, since which
    camera is actually mounted depends on --camera-source.
    """
    p = odom_pose["position"]
    q = odom_pose["orientation"]
    T_odom_baselink = quat_to_matrix(q["x"], q["y"], q["z"], q["w"], p["x"], p["y"], p["z"])
    T_odom_camera = T_odom_baselink @ T_base_link_camera
    return T_odom_camera.reshape(-1).tolist()


# Thresholds on /sensors_fusion/odom's own reported pose covariance
# (diagonal x/y variance in m^2, yaw variance in rad^2), used to classify
# localization_state instead of the previous placeholder that reported
# TRACKING whenever the TCP API merely responded. Documented on this
# chassis (README section 7.2): sf2 fuses IMU + RF2O laser odometry + wheel
# odometry + mfo_estimator's visual/IMU odometry at ~20 Hz into this exact
# topic, so its covariance reflects real multi-sensor agreement, not a
# placeholder. Caveat, stated plainly: these threshold VALUES are a
# reasonable order-of-magnitude guess, not empirically calibrated against a
# real degraded/lost tracking event — only a stationary, healthy-tracking
# baseline (~4e-6 m^2 / ~1e-5 rad^2) has actually been observed. DEGRADED/
# LOST have never been triggered or verified on this unit.
LOCALIZATION_TRACKING_MAX_POS_VAR_M2 = 0.01     # ~10 cm std
LOCALIZATION_TRACKING_MAX_YAW_VAR_RAD2 = 0.01   # ~5.7 deg std
LOCALIZATION_DEGRADED_MAX_POS_VAR_M2 = 1.0
LOCALIZATION_DEGRADED_MAX_YAW_VAR_RAD2 = 1.0


def classify_localization_state(covariance_6x6: list[float] | None) -> tuple[str, list[float]]:
    """Returns (localization_state, covariance_6x6_for_wire).

    ``covariance_6x6`` here is ROS's 6x6 pose covariance ([x,y,z,roll,pitch,
    yaw] ordering, row-major, 36 elements); the wire protocol's
    ``pose.covariance_6x6`` uses the same convention, so it is passed
    through unchanged rather than re-derived, other than being validated
    and finite-checked.
    """
    if not covariance_6x6 or len(covariance_6x6) != 36:
        return "UNKNOWN", [0.0] * 36
    cov = [float(v) for v in covariance_6x6]
    if not all(math.isfinite(v) for v in cov):
        return "UNKNOWN", [0.0] * 36
    var_x, var_y, var_yaw = cov[0], cov[7], cov[35]
    if var_x < 0 or var_y < 0 or var_yaw < 0:
        return "UNKNOWN", [0.0] * 36
    pos_var = max(var_x, var_y)
    if pos_var <= LOCALIZATION_TRACKING_MAX_POS_VAR_M2 and var_yaw <= LOCALIZATION_TRACKING_MAX_YAW_VAR_RAD2:
        return "TRACKING", cov
    if pos_var <= LOCALIZATION_DEGRADED_MAX_POS_VAR_M2 and var_yaw <= LOCALIZATION_DEGRADED_MAX_YAW_VAR_RAD2:
        return "DEGRADED", cov
    return "LOST", cov


def reproject_rgb_onto_depth_grid(
    rgb_bgr: np.ndarray, depth_m: np.ndarray,
    K_depth: np.ndarray, K_rgb: np.ndarray,
) -> np.ndarray:
    """Per-pixel RGB sample for each depth pixel, via real (identity)
    depth-optical -> color-optical geometry rather than a naive resize.

    For each valid depth pixel: backproject to 3D in the depth optical
    frame, which (co-located frames) is the same 3D point in the color
    optical frame, then project through the RGB camera's own intrinsics to
    sample its color. Pixels whose ray falls outside the RGB image (depth's
    FOV can exceed RGB's at the edges) are left black.
    """
    h, w = depth_m.shape
    out = np.zeros((h, w, 3), dtype=np.uint8)
    valid = depth_m > 0
    if not np.any(valid):
        return out
    vs, us = np.nonzero(valid)
    z = depth_m[vs, us].astype(np.float64)
    x = (us - K_depth[0, 2]) / K_depth[0, 0] * z
    y = (vs - K_depth[1, 2]) / K_depth[1, 1] * z
    # T_DEPTH_OPTICAL_TO_COLOR_OPTICAL is identity, so (x, y, z) is already
    # the point's coordinates in the color optical frame.
    rgb_u = np.round(K_rgb[0, 0] * x / z + K_rgb[0, 2]).astype(np.int64)
    rgb_v = np.round(K_rgb[1, 1] * y / z + K_rgb[1, 2]).astype(np.int64)
    rgb_h, rgb_w = rgb_bgr.shape[:2]
    in_bounds = (rgb_u >= 0) & (rgb_u < rgb_w) & (rgb_v >= 0) & (rgb_v < rgb_h)
    out[vs[in_bounds], us[in_bounds]] = rgb_bgr[rgb_v[in_bounds], rgb_u[in_bounds]]
    return out


def _decode_rgb_msg(msg: dict):
    raw = base64.b64decode(msg["data"])
    encoding = msg.get("encoding", "rgb8")
    height, width = msg["height"], msg["width"]
    channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(encoding, 3)
    array = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, channels)
    bgr = array[:, :, ::-1].copy() if encoding == "rgb8" else array.copy()
    stamp = msg.get("header", {}).get("stamp", {})
    ros_ns = int(stamp.get("secs", 0)) * 1_000_000_000 + int(stamp.get("nsecs", 0))
    return bgr, ros_ns


def fetch_rgb_frame(host: str, ws_port: int, topic: str, timeout_s: float):
    # The HTTP snapshot endpoint (web_video_server, port 8810) turned out to
    # be unreliable from EVERY client tried — requests, urllib, http.client
    # with the request line forced to HTTP/1.0, and even shelling out to
    # curl (rc=52 "empty reply" on a later run after curl had been 100%
    # reliable earlier in the same investigation). That rules out a
    # client-side fix: the flakiness is on the robot's side. rosbridge
    # (port 9090), by contrast, was reliable in every single trial across
    # this whole investigation for the depth topic, so RGB is fetched the
    # same way instead of via HTTP. See audit/YUNJI_WATER_SENDER.md.
    msg = fetch_one_topic(host, ws_port, topic, timeout_s=timeout_s)
    return _decode_rgb_msg(msg)


def fetch_depth_frame(host: str, ws_port: int, topic: str, timeout_s: float):
    msg = fetch_one_topic(host, ws_port, topic, timeout_s=timeout_s)
    raw = base64.b64decode(msg["data"])
    # The real raw depth topic (/camera_front_up/depth/image_raw) is 16UC1
    # (uint16 millimetres, standard depth-camera format). The old default,
    # depth_registered/image_raw, was 32FC1 -- but confirmed live
    # (2026-07-20) to be a bare dtype cast of the same uint16 millimetre
    # values with no real registration applied, so both branches divide by
    # 1000 the same way; only the on-wire dtype differs. Dispatch on the
    # message's own `encoding` field rather than hardcoding one, so this
    # keeps working if pointed at either topic.
    encoding = msg.get("encoding", "16UC1")
    dtype = np.uint16 if encoding == "16UC1" else np.float32
    depth_raw = np.frombuffer(raw, dtype=dtype).reshape(msg["height"], msg["width"])
    depth_m = depth_raw.astype(np.float64) / 1000.0  # empirically confirmed unit; see module docstring
    depth_m[~np.isfinite(depth_m)] = 0.0
    depth_m[depth_m < 0] = 0.0
    stamp = msg.get("header", {}).get("stamp", {})
    ros_ns = int(stamp.get("secs", 0)) * 1_000_000_000 + int(stamp.get("nsecs", 0))
    return depth_m, ros_ns


# --------------------------------------------------------- local RealSense

class LocalRealsenseCamera:
    """Persistent local capture from a RealSense connected directly to this
    host (nyush-nuc), as an alternative to the chassis's rosbridge topics.

    A single background thread owns the `pipeline.wait_for_frames()` call and
    runs it continuously at the camera's native rate (there can only be one
    reader of a given physical device -- unlike the chassis's rosbridge
    topics, a second independent puller here would just fail to open the
    same camera a second time, not merely contend over bandwidth). The main
    sender loop's `read()` is a thread-safe getter of whatever the background
    thread most recently captured, decoupling the 2Hz upload cadence from the
    camera's actual frame rate. The same background thread optionally pushes
    a preview frame to foxglove_relay.py on its own faster, independently
    throttled cadence -- this is NOT the same shape as the reverted
    `RgbStreamThread` experiment for the chassis camera (that one added a
    SECOND rosbridge connection that contended with the main loop's own
    fetch of the same flaky embedded server; here there is exactly one
    reader of the camera, full stop).
    """

    def __init__(self, width: int, height: int, fps: int, timeout_ms: int = 6000, *,
                 preview_url: str | None = None, preview_token: str | None = None,
                 preview_max_rate_hz: float = 10.0, jpeg_quality: int = 80) -> None:
        import pyrealsense2 as rs

        self._rs = rs
        self.timeout_ms = timeout_ms
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale_m = float(depth_sensor.get_depth_scale())
        color_intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.width = color_intrinsics.width
        self.height = color_intrinsics.height
        self.fx = color_intrinsics.fx
        self.fy = color_intrinsics.fy
        self.cx = color_intrinsics.ppx
        self.cy = color_intrinsics.ppy

        self.preview_url = preview_url
        self.preview_token = preview_token
        self.preview_min_period_s = 1.0 / preview_max_rate_hz if preview_max_rate_hz > 0 else None
        self.jpeg_quality = jpeg_quality

        self._lock = threading.Lock()
        self._latest = None  # (bgr, depth_m, capture_time_ns)
        self._error = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True, name="local-realsense-capture")
        self._thread.start()
        deadline = time.monotonic() + (timeout_ms / 1000.0)
        while self._latest is None and self._error is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if self._error is not None:
            raise RuntimeError(f"local RealSense capture thread failed to start: {self._error}")
        if self._latest is None:
            raise TimeoutError("local RealSense produced no frame within the startup timeout")

    def _capture_loop(self) -> None:
        last_preview_push_s = 0.0
        while not self._stop_event.is_set():
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=self.timeout_ms)
                aligned = self.align.process(frames)
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                bgr = np.asanyarray(color_frame.get_data())
                depth_raw = np.asanyarray(depth_frame.get_data())
                depth_m = depth_raw.astype(np.float64) * self.depth_scale_m
                depth_m[~np.isfinite(depth_m)] = 0.0
                depth_m[depth_m < 0] = 0.0
                # get_timestamp() is milliseconds; frames use the default
                # (system) clock unless the device is configured for
                # hardware timestamps.
                capture_time_ns = int(color_frame.get_timestamp() * 1_000_000)
                with self._lock:
                    self._latest = (bgr, depth_m, capture_time_ns)
            except Exception as exc:  # noqa: BLE001 - report once, keep retrying
                self._error = self._error or str(exc)
                continue

            if self.preview_url and self.preview_min_period_s is not None:
                now = time.monotonic()
                if now - last_preview_push_s >= self.preview_min_period_s:
                    last_preview_push_s = now
                    self._push_preview(bgr)

    def _push_preview(self, bgr: np.ndarray) -> None:
        try:
            import requests

            ok, jpeg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok:
                return
            # No requests.Session() -- see the module docstring: a kept-alive
            # connection across an SSH reverse tunnel has been observed to
            # get silently dropped between pushes.
            requests.post(
                self.preview_url,
                headers={"X-Robot-Token": self.preview_token,
                         "Content-Type": "image/jpeg", "Connection": "close"},
                data=jpeg.tobytes(), timeout=3.0,
            ).raise_for_status()
        except Exception:  # noqa: BLE001 - preview push must never break capture
            pass

    def read(self) -> tuple[np.ndarray, np.ndarray, int]:
        """Returns the most recently captured (bgr, depth_m, capture_time_ns),
        with depth aligned onto the color frame's grid/resolution. Never
        blocks on the camera itself -- the background thread already did
        that -- so this decouples the caller's own cadence from the
        camera's actual frame rate."""
        with self._lock:
            if self._latest is None:
                raise RuntimeError("no frame captured yet")
            return self._latest

    def close(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        self.pipeline.stop()


def build_metadata(*, robot_id, sequence, rgb_bytes, depth_bytes, pose_matrix,
                   transform_version, goal_category, status: dict,
                   width: int, height: int, fx, fy, cx, cy,
                   capture_time_ns, localization_state: str, covariance_6x6: list[float]) -> dict:
    now_ns = time.time_ns()
    estop = bool(status.get("estop_state") or status.get("hard_estop_state"))
    error_code = str(status.get("error_code", "00000000"))
    healthy = (not estop) and error_code == "00000000"
    return {
        "robot_id": robot_id,
        "sequence": sequence,
        "capture_time_ns": capture_time_ns,
        "sent_time_ns": max(now_ns, capture_time_ns),
        "pose": {
            "shared_T_camera": {
                "parent_frame": "shared_world",
                "child_frame": CAMERA_FRAME,
                "matrix": pose_matrix,
            },
            "covariance_6x6": covariance_6x6,
            "transform_version": transform_version,
        },
        "base_T_camera": None,
        "intrinsics": {
            "width": width, "height": height,
            "fx": float(fx), "fy": float(fy), "cx": float(cx), "cy": float(cy),
            "distortion_model": "none", "distortion": [],
        },
        "depth_scale_m": DEPTH_SCALE_M,
        "depth_min_m": 0.2,
        "depth_max_m": 3.0,
        "rgb_encoding": "jpeg",
        "depth_encoding": "png16",
        "rgb_size_bytes": len(rgb_bytes),
        "depth_size_bytes": len(depth_bytes),
        "rgb_sha256": __import__("hashlib").sha256(rgb_bytes).hexdigest(),
        "depth_sha256": __import__("hashlib").sha256(depth_bytes).hexdigest(),
        "object_goal": {"goal_id": "yunji-integration-1", "category": goal_category},
        "health": {
            "safety_state": "READY" if healthy else ("ESTOP" if estop else "HOLD"),
            "localization_state": localization_state,
            "estop_engaged": estop,
            "collision_avoidance_ready": healthy,
            "motor_controller_ready": healthy,
            "battery_percent": float(status.get("power_percent", 0.0)),
            "detail": f"error_code={error_code} move_status={status.get('move_status')}",
        },
        "mapping_only": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-host", default="192.168.10.10")
    parser.add_argument("--tcp-port", type=int, default=31001)
    parser.add_argument("--ws-port", type=int, default=9090)
    parser.add_argument("--rgb-topic", default="/camera_front_up/rgb/image_raw")
    parser.add_argument("--depth-topic", default="/camera_front_up/depth/image_raw",
                        help="the genuinely raw depth topic, not depth_registered -- confirmed "
                             "live (2026-07-20) that depth_registered's pixel values are 100% "
                             "identical to this raw topic's, just uint16 cast to float32 with no "
                             "actual geometric registration applied (depth_align=false, "
                             "color_depth_synchronization=false, no camera_info published for "
                             "it). This raw topic's own camera_info matches K_DEPTH below "
                             "byte-for-byte, confirming K_DEPTH was always the real depth "
                             "intrinsics -- switching topics changes no data or math, just stops "
                             "depending on a topic name that lies about what it is.")
    parser.add_argument("--odom-topic", default="/sensors_fusion/odom")
    parser.add_argument("--camera-source", choices=("rosbridge", "local-realsense"), default="rosbridge",
                        help="'rosbridge' (default): RGB+depth from the chassis's camera_front_up "
                             "topics, as before. 'local-realsense': RGB+depth from an Intel "
                             "RealSense connected directly to this host over USB instead -- see "
                             "the module docstring's fourth pass note. Pose/health always come "
                             "from the chassis regardless of this setting.")
    parser.add_argument("--realsense-width", type=int, default=640)
    parser.add_argument("--realsense-height", type=int, default=480)
    parser.add_argument("--realsense-fps", type=int, default=30)
    parser.add_argument("--local-camera-model", choices=("d405", "d455"), default="d455",
                         help="--camera-source local-realsense only: selects which "
                              "MEASURED_T_BASE_LINK_CAMERA_* extrinsic to use, since the D405 "
                              "and D455 were mounted at different points/heights")
    parser.add_argument("--base-url", default="http://127.0.0.1:18089")
    parser.add_argument("--robot-id", default="robot-1")
    parser.add_argument("--transform-version", default="yunji-water-robot-live-odom-test-v1")
    parser.add_argument("--rate-hz", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=0, help="0 = unbounded")
    parser.add_argument("--jpeg-quality", type=int, default=92)
    parser.add_argument("--png-level", type=int, default=1)
    parser.add_argument("--fetch-timeout-s", type=float, default=6.0)
    parser.add_argument("--goal-category", default="water_bottle")
    parser.add_argument("--metrics-out", default="yunji_sender_metrics.json")
    parser.add_argument("--max-consecutive-failures", type=int, default=5,
                        help="abort instead of retrying forever if this many cycles in a row fail")
    parser.add_argument("--shared-frame-transform-file", default=None,
                        help="output of hub/tools/calibrate_shared_frame.py for this robot as "
                             "the 'other' robot; applied to every published pose before upload. "
                             "Omit to keep publishing in this robot's own local odometry frame.")
    parser.add_argument("--heartbeat-hz", type=float, default=2.0,
                        help="independent liveness/health ping rate, decoupled from --rate-hz; "
                             "0 disables it")
    parser.add_argument("--camera-preview-url", default=None,
                        help="if set, POST the already-fetched RGB frame to this "
                             "foxglove_relay.py instance's /camera/{name} endpoint every cycle "
                             "(e.g. http://127.0.0.1:18766/camera/yunji) -- reuses this sender's "
                             "own rosbridge fetch rather than a second independent poller. NOTE "
                             "(2026-07-20): measured directly, this camera only publishes ~once "
                             "per 5s regardless of fetch strategy -- this preview cannot be "
                             "smoother than that; it just avoids adding a second connection.")
    parser.add_argument("--camera-preview-token", default=None,
                        help="X-Robot-Token for --camera-preview-url; required if that's set")
    parser.add_argument("--preview-max-rate-hz", type=float, default=10.0,
                        help="--camera-source local-realsense only: independent preview push rate "
                             "from the background capture thread, decoupled from --rate-hz (the "
                             "chassis rosbridge path has no equivalent -- its preview is tied to "
                             "the main loop's own fetch cadence, same as before)")
    args = parser.parse_args()

    shared_frame_transform = load_shared_frame_transform(args.shared_frame_transform_file)
    if shared_frame_transform is not None:
        print(f"loaded shared-frame calibration from {args.shared_frame_transform_file}")
    else:
        print("no --shared-frame-transform-file given; poses stay in this robot's own local "
              "odometry frame (not yet a real shared_world)")

    token = os.environ.get("FOCUS_ROBOT_TOKEN", "")
    if not token:
        print("FOCUS_ROBOT_TOKEN is not set", file=sys.stderr)
        return 2

    transport = HubTransport(args.base_url, args.robot_id, token)
    try:
        sequence = transport.last_sequence() + 1
        print(f"resume: starting at sequence {sequence} [hub]")
    except Exception as exc:  # noqa: BLE001
        sequence = 0
        print(f"resume: hub unreachable at startup ({exc}); starting at sequence 0")

    latest_localization_state = LatestLocalizationState()
    heartbeat_thread = None
    if args.heartbeat_hz > 0:
        heartbeat_thread = HeartbeatThread(
            robot_host=args.robot_host, tcp_port=args.tcp_port, base_url=args.base_url,
            robot_id=args.robot_id, token=token, localization_state=latest_localization_state,
            period_s=1.0 / args.heartbeat_hz,
        )
        heartbeat_thread.start()
        print(f"heartbeat thread started ({args.heartbeat_hz} Hz, independent of the RGBD cycle)")

    tcp = WaterTcpClient(args.robot_host, args.tcp_port)
    info = tcp.request("/api/robot_info")
    print(f"connected to WATER chassis: {info.get('results', {}).get('product_id')}")

    local_camera = None
    if args.camera_source == "local-realsense":
        local_camera = LocalRealsenseCamera(
            args.realsense_width, args.realsense_height, args.realsense_fps,
            preview_url=args.camera_preview_url, preview_token=args.camera_preview_token,
            preview_max_rate_hz=args.preview_max_rate_hz, jpeg_quality=args.jpeg_quality)
        print(f"local RealSense opened: {local_camera.width}x{local_camera.height} "
              f"fx={local_camera.fx:.2f} fy={local_camera.fy:.2f} "
              f"cx={local_camera.cx:.2f} cy={local_camera.cy:.2f} "
              f"depth_scale_m={local_camera.depth_scale_m}")

    frames_sent = 0
    consecutive_failures = 0
    metrics = []
    intrinsics_verified = False
    period_s = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.0

    def restamp(meta: dict) -> dict:
        now_ns = time.time_ns()
        meta = dict(meta)
        meta["capture_time_ns"] = now_ns - 50_000_000
        meta["sent_time_ns"] = now_ns
        return meta

    try:
        while not args.max_frames or frames_sent < args.max_frames:
            t0 = time.perf_counter()
            try:
                status_resp = retry_call(lambda: tcp.request("/api/robot_status"), label="robot_status")
                status = status_resp.get("results", {})
                odom_msg = retry_call(
                    lambda: fetch_one_topic(
                        args.robot_host, args.ws_port, args.odom_topic, args.fetch_timeout_s),
                    label="odom")

                if local_camera is not None:
                    rgb_registered, depth_m, camera_capture_ns = retry_call(
                        local_camera.read, label="local_realsense_frame")
                    rgb_full = rgb_registered  # already at the color stream's native resolution
                    frame_fx, frame_fy = local_camera.fx, local_camera.fy
                    frame_cx, frame_cy = local_camera.cx, local_camera.cy
                else:
                    rgb_full, rgb_ros_ns = retry_call(
                        lambda: fetch_rgb_frame(
                            args.robot_host, args.ws_port, args.rgb_topic, args.fetch_timeout_s),
                        label="rgb_frame")
                    depth_m, camera_capture_ns = retry_call(
                        lambda: fetch_depth_frame(
                            args.robot_host, args.ws_port, args.depth_topic, args.fetch_timeout_s),
                        label="depth_frame")
                    if not intrinsics_verified:
                        verify_intrinsics_match_frame_size(depth_m.shape, rgb_full.shape)
                        intrinsics_verified = True
                    rgb_registered = reproject_rgb_onto_depth_grid(rgb_full, depth_m, K_DEPTH, K_RGB)
                    frame_fx, frame_fy = K_DEPTH[0, 0], K_DEPTH[1, 1]
                    frame_cx, frame_cy = K_DEPTH[0, 2], K_DEPTH[1, 2]
                t1 = time.perf_counter()

                depth_h, depth_w = depth_m.shape

                ok_rgb, jpeg = cv2.imencode(".jpg", rgb_registered, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality])
                counts = np.clip(np.round(depth_m / DEPTH_SCALE_M), 0, 65535).astype(np.uint16)
                ok_depth, png = cv2.imencode(".png", counts, [int(cv2.IMWRITE_PNG_COMPRESSION), args.png_level])
                if not (ok_rgb and ok_depth):
                    raise RuntimeError("JPEG/PNG encoding failed")
                rgb_bytes, depth_bytes = jpeg.tobytes(), png.tobytes()
                t2 = time.perf_counter()

                if args.camera_preview_url and local_camera is None:
                    # local-realsense already pushes its own preview from the
                    # background capture thread at --preview-max-rate-hz,
                    # decoupled from this loop's --rate-hz.
                    try:
                        import requests
                        # Push rgb_full (native 640x360), NOT rgb_bytes -- rgb_bytes is
                        # rgb_registered reprojected down onto the depth grid's low
                        # 160x120 resolution, which is what the semantic mapping
                        # pipeline needs (pixel-for-pixel correspondence with depth),
                        # but is a needless quality loss for a viewing-only preview.
                        # The mapping upload below is unaffected -- still rgb_bytes.
                        ok_preview, preview_jpeg = cv2.imencode(
                            ".jpg", rgb_full, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality])
                        if ok_preview:
                            # No requests.Session() -- a fresh connection per push, since a
                            # kept-alive connection across an SSH reverse tunnel has been
                            # observed to get silently dropped between the ~1s gaps here.
                            requests.post(
                                args.camera_preview_url,
                                headers={"X-Robot-Token": args.camera_preview_token,
                                         "Content-Type": "image/jpeg", "Connection": "close"},
                                data=preview_jpeg.tobytes(), timeout=3.0,
                            ).raise_for_status()
                    except Exception:  # noqa: BLE001 - preview push must never break the real upload
                        pass

                if local_camera is not None:
                    T_base_link_camera = (
                        MEASURED_T_BASE_LINK_CAMERA_D455 if args.local_camera_model == "d455"
                        else MEASURED_T_BASE_LINK_CAMERA_D405)
                else:
                    T_base_link_camera = MEASURED_T_BASE_LINK_CAMERA
                pose_matrix = pose_to_matrix(odom_msg["pose"]["pose"], T_base_link_camera)
                if shared_frame_transform is not None:
                    T_odom_camera = np.array(pose_matrix, dtype=np.float64).reshape(4, 4)
                    pose_matrix = (shared_frame_transform @ T_odom_camera).reshape(-1).tolist()

                localization_state, covariance_6x6 = classify_localization_state(
                    odom_msg["pose"].get("covariance"))
                latest_localization_state.set(localization_state)

                capture_time_ns = camera_capture_ns if camera_capture_ns else time.time_ns() - 50_000_000
                metadata = build_metadata(
                    robot_id=args.robot_id, sequence=sequence, rgb_bytes=rgb_bytes,
                    depth_bytes=depth_bytes, pose_matrix=pose_matrix,
                    transform_version=args.transform_version, goal_category=args.goal_category,
                    status=status, width=depth_w, height=depth_h,
                    fx=frame_fx, fy=frame_fy, cx=frame_cx, cy=frame_cy,
                    capture_time_ns=capture_time_ns,
                    localization_state=localization_state, covariance_6x6=covariance_6x6)

                t3 = time.perf_counter()
                ack, attempts = transport.upload(metadata, rgb_bytes, depth_bytes, restamp)
                t4 = time.perf_counter()

                metrics.append({
                    "sequence": sequence, "fetch_ms": round((t1 - t0) * 1e3, 1),
                    "encode_ms": round((t2 - t1) * 1e3, 1), "upload_ms": round((t4 - t3) * 1e3, 1),
                    "attempts": attempts, "ack_status": ack.get("status"),
                    "battery_percent": status.get("power_percent"),
                    "estop_state": status.get("estop_state"), "move_status": status.get("move_status"),
                    "depth_valid_fraction": round(float((depth_m > 0).mean()), 3),
                    "localization_state": localization_state,
                })
                print(f"sent {frames_sent + 1} (seq={sequence}, ack={ack.get('status')}, "
                     f"battery={status.get('power_percent')}%, loc={localization_state}, "
                     f"upload={round((t4-t3)*1e3)}ms)")
                sequence += 1
                frames_sent += 1
                consecutive_failures = 0
            except Exception as exc:  # noqa: BLE001 - one bad cycle must not kill the sender
                consecutive_failures += 1
                print(f"cycle failed ({consecutive_failures}/{args.max_consecutive_failures}): {exc}",
                     file=sys.stderr)
                if consecutive_failures >= args.max_consecutive_failures:
                    print("too many consecutive failures; aborting", file=sys.stderr)
                    break

            if period_s:
                remaining = period_s - (time.perf_counter() - t0)
                if remaining > 0:
                    time.sleep(remaining)
    finally:
        tcp.close()
        if local_camera is not None:
            local_camera.close()
        if heartbeat_thread is not None:
            heartbeat_thread.stop()
            heartbeat_thread.join(timeout=2.0)
        summary = {
            "frames_sent": frames_sent, "retries_total": transport.retries_total,
            "mean_upload_ms": round(float(np.mean([m["upload_ms"] for m in metrics])), 1) if metrics else None,
            "mean_fetch_ms": round(float(np.mean([m["fetch_ms"] for m in metrics])), 1) if metrics else None,
            "heartbeats_sent": heartbeat_thread.beats_sent if heartbeat_thread else None,
            "heartbeats_failed": heartbeat_thread.beats_failed if heartbeat_thread else None,
        }
        with open(args.metrics_out, "w") as f:
            json.dump({"summary": summary, "frames": metrics}, f, indent=2)
        print(json.dumps(summary, indent=2))

    return 0 if frames_sent > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
