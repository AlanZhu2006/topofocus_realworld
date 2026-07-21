#!/usr/bin/env python3
"""Replay a TinyNav record into the hub over the real observation HTTP API.

This is the transport rehearsal for the future robot-side ROS sender: it emits
exactly what that sender will emit — aligned RGB-D in the color frame, the
paired camera pose as ``shared_T_camera``, strict metadata with hashes — over
authenticated multipart HTTP.  Capture timestamps are re-stamped to now
(preserving inter-frame spacing at the chosen rate) because the hub enforces a
3 s freshness window; the original record timestamps stay in the replay sample.

Dry-run remains the default posture elsewhere: this tool only *uploads
observations*; it can neither move a robot nor publish decisions.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

from focus_hub.client import HubClient  # noqa: E402
from focus_hub.observation_builder import (  # noqa: E402
    PLACEHOLDER_BASE_T_CAMERA,
    build_metadata,
    encode_frame,
)
from focus_hub.tinynav_replay import TinyNavReplayReader  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--extracted", type=Path, required=True)
    parser.add_argument("--base-url", default="http://127.0.0.1:8088")
    parser.add_argument("--robot-id", default="robot-0")
    parser.add_argument("--token", required=True)
    parser.add_argument("--transform-version", default="UNSET")
    parser.add_argument("--rate-hz", type=float, default=0.0,
                        help="0 = as fast as possible; 1-2 emulates the production keyframe rate")
    parser.add_argument("--limit", type=int, default=0, help="0 = all keyframes")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--goal-category", default="chair")
    parser.add_argument("--command-capable", action="store_true",
                        help="TEST ONLY: mark observations command-capable with a "
                             "placeholder base_T_camera and READY health (dry-run rehearsal)")
    args = parser.parse_args()

    reader = TinyNavReplayReader(args.record, args.extracted)
    base_T_camera = PLACEHOLDER_BASE_T_CAMERA if args.command_capable else None

    sent = accepted = duplicates = 0
    period_s = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.0
    with HubClient(args.base_url, args.robot_id, args.token) as client:
        for i, frame in enumerate(reader.frames()):
            if i % args.stride:
                continue
            if args.limit and sent >= args.limit:
                break
            started = time.perf_counter()
            rgb_bytes, depth_bytes = encode_frame(frame, reader)
            metadata = build_metadata(
                robot_id=args.robot_id,
                sequence=sent,
                frame=frame,
                reader=reader,
                rgb_bytes=rgb_bytes,
                depth_bytes=depth_bytes,
                transform_version=args.transform_version,
                mapping_only=not args.command_capable,
                base_T_camera=base_T_camera,
                health_ready=args.command_capable,
                goal_category=args.goal_category,
            )
            ack = client.upload_bytes(metadata, rgb_bytes, depth_bytes)
            sent += 1
            if ack.status == "accepted":
                accepted += 1
            else:
                duplicates += 1
            if period_s:
                remaining = period_s - (time.perf_counter() - started)
                if remaining > 0:
                    time.sleep(remaining)

    print(f"sent={sent} accepted={accepted} duplicates={duplicates}")
    return 0 if sent and accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
