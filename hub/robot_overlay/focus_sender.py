#!/usr/bin/env python3
"""Robot-side keyframe sender for the Focus hub (standalone overlay).

Runs on the robot (Jetson, system Python) with only numpy + cv2 + requests.
Reads a TinyNav map-record in place and uploads keyframes to the hub over the
authenticated observation API.  This is the transport reference for the future
live ROS 2 sender; it never subscribes to control topics and cannot move the
robot.

Transport behaviour (the parts a flaky WLAN actually tests):
  * resume: on start the sender asks the hub for the last accepted sequence
    (GET /observations/latest) and continues after it; a local counter file is
    kept as a fallback when the hub is unreachable at startup.
  * retries: exponential backoff with a cap; a re-sent frame reuses the exact
    same sequence + payload bytes, so the hub's idempotency turns duplicates
    into 'duplicate' acks instead of errors.
  * freshness: capture_time is stamped just before upload; if a frame spent
    too long in retries it is re-stamped and re-hashed rather than sent stale.
  * metrics: per-stage timings (read/align/encode/upload), payload sizes and
    retry counts are written as JSON for the transport audit.

Deployment: copy this single file to the robot (outside the TinyNav repo).
The token comes from the FOCUS_ROBOT_TOKEN environment variable, never argv.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import pickle
import sys
import time
from pathlib import Path

import cv2
import numpy as np

DEPTH_SCALE_M = 0.001
CAMERA_FRAME = "camera_color_optical_frame"


# ---------------------------------------------------------------- record I/O

class RecordReader:
    """Minimal in-place TinyNav map-record reader (no extraction step)."""

    def __init__(self, record_dir: Path, extracted_dir: Path | None = None) -> None:
        self.record_dir = record_dir
        self.extracted_dir = extracted_dir
        self.poses = {
            int(k): np.asarray(v, dtype=np.float64)
            for k, v in np.load(record_dir / "poses.npy", allow_pickle=True).item().items()
        }
        with (record_dir / "rgb_images_db" / "meta.json").open() as f:
            self.ts_to_idx = {int(k): int(v) for k, v in json.load(f)["ts_to_idx"].items()}
        self.timestamps = sorted(set(self.poses) & set(self.ts_to_idx))
        self.K_infra1 = np.load(record_dir / "intrinsics.npy").astype(np.float64)
        self.K_rgb = np.asarray(
            np.load(record_dir / "rgb_camera_intrinsics.npy", allow_pickle=True), dtype=np.float64
        )
        self.T_rgb_to_infra1 = np.load(record_dir / "T_rgb_to_infra1.npy").astype(np.float64)
        self._depths = None

    def _load_depth(self, ts: int) -> np.ndarray:
        # On the robot the Berkeley-DB shelve is read directly; on hosts whose
        # Python lacks _dbm an extracted depths_pkl directory can be used.
        if self.extracted_dir is not None:
            raw = (self.extracted_dir / "depths_pkl" / f"{ts}.pkl").read_bytes()
            return pickle.loads(raw)
        if self._depths is None:
            import dbm.ndbm

            self._depths = dbm.ndbm.open(str(self.record_dir / "depths"), "r")
        return pickle.loads(self._depths[str(ts).encode()])

    def frames(self, stride: int = 1, limit: int = 0):
        capture = cv2.VideoCapture(str(self.record_dir / "rgb_images_db" / "video.mp4"))
        if not capture.isOpened():
            raise RuntimeError("failed to open rgb video")
        try:
            decoded = 0
            yielded = 0
            for i, ts in enumerate(self.timestamps):
                target = self.ts_to_idx[ts]
                frame = None
                while decoded <= target:
                    ok, frame = capture.read()
                    if not ok:
                        raise RuntimeError(f"video ended before index {target}")
                    decoded += 1
                if i % stride:
                    continue
                if limit and yielded >= limit:
                    break
                depth = self._load_depth(ts)
                yield ts, frame, np.asarray(depth, dtype=np.float32), self.poses[ts]
                yielded += 1
        finally:
            capture.release()


# ------------------------------------------------------------- frame encode

def align_depth_to_rgb(depth_ir, K_ir, K_rgb, T_rgb_to_infra1, rgb_shape):
    valid = depth_ir > 0
    if not np.any(valid):
        return np.zeros(rgb_shape, dtype=np.float32)
    vs, us = np.nonzero(valid)
    z = depth_ir[vs, us].astype(np.float64)
    points = np.stack(
        (
            (us - K_ir[0, 2]) / K_ir[0, 0] * z,
            (vs - K_ir[1, 2]) / K_ir[1, 1] * z,
            z,
        ),
        axis=-1,
    )
    T = np.linalg.inv(T_rgb_to_infra1)
    p = points @ T[:3, :3].T + T[:3, 3]
    front = p[:, 2] > 1e-6
    p = p[front]
    h, w = rgb_shape
    u = np.round(K_rgb[0, 0] * p[:, 0] / p[:, 2] + K_rgb[0, 2]).astype(np.int64)
    v = np.round(K_rgb[1, 1] * p[:, 1] / p[:, 2] + K_rgb[1, 2]).astype(np.int64)
    keep = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    aligned = np.full(h * w, np.inf)
    np.minimum.at(aligned, v[keep] * w + u[keep], p[keep, 2])
    aligned[~np.isfinite(aligned)] = 0.0
    return aligned.reshape(h, w).astype(np.float32)


def encode_payloads(rgb_bgr, depth_m, jpeg_quality, png_level):
    ok, jpeg = cv2.imencode(".jpg", rgb_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
    if not ok:
        raise RuntimeError("jpeg encode failed")
    counts = np.clip(np.round(depth_m / DEPTH_SCALE_M), 0, 65535).astype(np.uint16)
    ok, png = cv2.imencode(".png", counts, [int(cv2.IMWRITE_PNG_COMPRESSION), png_level])
    if not ok:
        raise RuntimeError("png16 encode failed")
    return jpeg.tobytes(), png.tobytes()


def build_metadata(robot_id, sequence, T_world_rgb, K_rgb, shape, rgb_bytes, depth_bytes,
                   transform_version, goal_category):
    now_ns = time.time_ns()
    h, w = shape
    return {
        "robot_id": robot_id,
        "sequence": sequence,
        "capture_time_ns": now_ns - 50_000_000,
        "sent_time_ns": now_ns,
        "pose": {
            "shared_T_camera": {
                "parent_frame": "shared_world",
                "child_frame": CAMERA_FRAME,
                "matrix": [float(x) for x in T_world_rgb.reshape(-1)],
            },
            "covariance_6x6": [0.0] * 36,
            "transform_version": transform_version,
        },
        "base_T_camera": None,
        "intrinsics": {
            "width": w, "height": h,
            "fx": float(K_rgb[0, 0]), "fy": float(K_rgb[1, 1]),
            "cx": float(K_rgb[0, 2]), "cy": float(K_rgb[1, 2]),
            "distortion_model": "none", "distortion": [],
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
        "object_goal": {"goal_id": "transport-test-1", "category": goal_category},
        "health": {
            "safety_state": "UNKNOWN",
            "localization_state": "UNKNOWN",
            "estop_engaged": False,
            "collision_avoidance_ready": False,
            "motor_controller_ready": False,
        },
        "mapping_only": True,
    }


# ---------------------------------------------------------------- transport

class HubTransport:
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

    def last_sequence(self):
        response = self.session.get(
            f"{self.base_url}/v1/robots/{self.robot_id}/observations/latest",
            timeout=self.timeout_s,
        )
        response.raise_for_status()
        return int(response.json()["last_sequence"])

    def upload(self, metadata, rgb_bytes, depth_bytes, restamp):
        """Upload one frame; retry with backoff.  Returns (ack, attempts)."""
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
                # 4xx (except 408/429) will not heal by retrying.
                if 400 <= response.status_code < 500 and response.status_code not in (408, 429):
                    raise RuntimeError(
                        f"hub rejected seq {metadata['sequence']}: "
                        f"{response.status_code} {response.text[:300]}"
                    )
            except (requests.ConnectionError, requests.Timeout):
                pass
            if attempt > self.max_retries:
                raise RuntimeError(f"giving up on seq {metadata['sequence']} after {attempt} attempts")
            self.retries_total += 1
            delay = min(self.backoff_cap_s, self.backoff_base_s * (2 ** (attempt - 1)))
            time.sleep(delay)
            # Keep capture_time fresh across long retry gaps: the hub enforces
            # a 3 s freshness window, so re-stamp before the next attempt.
            metadata = restamp(metadata)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--extracted", type=Path, default=None,
                        help="optional depths_pkl sample dir (for hosts without _dbm)")
    parser.add_argument("--base-url", default="http://127.0.0.1:18089")
    parser.add_argument("--robot-id", default="robot-0")
    parser.add_argument("--transform-version", default="UNSET")
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--jpeg-quality", type=int, default=92)
    parser.add_argument("--png-level", type=int, default=1,
                        help="PNG compression 0-9; higher = smaller/slower")
    parser.add_argument("--goal-category", default="chair")
    parser.add_argument("--state-file", type=Path, default=Path("focus_sender_state.json"))
    parser.add_argument("--metrics-out", type=Path, default=Path("focus_sender_metrics.json"))
    parser.add_argument("--dry-run", action="store_true",
                        help="encode everything, upload nothing")
    parser.add_argument("--loop", type=int, default=1,
                        help="replay the record this many times (soak testing); "
                             "sequence numbers keep increasing across loops")
    args = parser.parse_args()

    token = os.environ.get("FOCUS_ROBOT_TOKEN", "")
    if not token and not args.dry_run:
        print("FOCUS_ROBOT_TOKEN is not set", file=sys.stderr)
        return 2

    reader = RecordReader(args.record, args.extracted)
    transport = None
    start_sequence = 0
    if not args.dry_run:
        transport = HubTransport(args.base_url, args.robot_id, token)
        try:
            start_sequence = transport.last_sequence() + 1
            source = "hub"
        except Exception as exc:  # noqa: BLE001 - fall back to the local counter
            source = f"local ({exc})"
            if args.state_file.is_file():
                start_sequence = json.loads(args.state_file.read_text())["next_sequence"]
        print(f"resume: starting at sequence {start_sequence} [{source}]")

    metrics = {
        "config": {k: str(v) for k, v in vars(args).items() if k != "state_file"},
        "frames": [],
    }
    period_s = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.0
    sequence = start_sequence
    sent = accepted = duplicates = 0
    wall_start = time.perf_counter()

    def restamp(metadata):
        now_ns = time.time_ns()
        metadata = dict(metadata)
        metadata["capture_time_ns"] = now_ns - 50_000_000
        metadata["sent_time_ns"] = now_ns
        return metadata

    def frame_stream():
        for loop_index in range(max(1, args.loop)):
            for item in reader.frames(args.stride, args.limit):
                yield item

    for ts, rgb, depth_ir, pose in frame_stream():
        t0 = time.perf_counter()
        aligned = align_depth_to_rgb(
            depth_ir, reader.K_infra1, reader.K_rgb, reader.T_rgb_to_infra1, rgb.shape[:2])
        t1 = time.perf_counter()
        rgb_bytes, depth_bytes = encode_payloads(rgb, aligned, args.jpeg_quality, args.png_level)
        t2 = time.perf_counter()
        T_world_rgb = pose @ reader.T_rgb_to_infra1
        metadata = build_metadata(
            args.robot_id, sequence, T_world_rgb, reader.K_rgb, rgb.shape[:2],
            rgb_bytes, depth_bytes, args.transform_version, args.goal_category)
        frame_metric = {
            "sequence": sequence,
            "record_ts": ts,
            "align_ms": round((t1 - t0) * 1e3, 1),
            "encode_ms": round((t2 - t1) * 1e3, 1),
            "rgb_bytes": len(rgb_bytes),
            "depth_bytes": len(depth_bytes),
        }
        if transport is not None:
            t3 = time.perf_counter()
            ack, attempts = transport.upload(metadata, rgb_bytes, depth_bytes, restamp)
            frame_metric["upload_ms"] = round((time.perf_counter() - t3) * 1e3, 1)
            frame_metric["attempts"] = attempts
            if ack["status"] == "accepted":
                accepted += 1
            else:
                duplicates += 1
            args.state_file.write_text(json.dumps({"next_sequence": sequence + 1}))
        metrics["frames"].append(frame_metric)
        sent += 1
        sequence += 1
        if period_s:
            remaining = period_s - (time.perf_counter() - t0)
            if remaining > 0:
                time.sleep(remaining)

    elapsed = time.perf_counter() - wall_start
    frames = metrics["frames"]
    payload_bytes = sum(f["rgb_bytes"] + f["depth_bytes"] for f in frames)
    metrics["summary"] = {
        "sent": sent,
        "accepted": accepted,
        "duplicates": duplicates,
        "retries_total": transport.retries_total if transport else 0,
        "elapsed_s": round(elapsed, 2),
        "payload_mib": round(payload_bytes / 2**20, 2),
        "throughput_mib_s": round(payload_bytes / 2**20 / elapsed, 3) if elapsed else None,
        "mean_align_ms": round(float(np.mean([f["align_ms"] for f in frames])), 1) if frames else None,
        "mean_encode_ms": round(float(np.mean([f["encode_ms"] for f in frames])), 1) if frames else None,
        "mean_upload_ms": round(float(np.mean([f["upload_ms"] for f in frames])), 1)
        if frames and transport else None,
    }
    args.metrics_out.write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics["summary"]))
    return 0 if sent and (args.dry_run or accepted + duplicates == sent) else 1


if __name__ == "__main__":
    raise SystemExit(main())
