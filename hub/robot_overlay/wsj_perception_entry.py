#!/usr/bin/env python3
"""Run the pinned WSJ TinyNav perception source with bounded IMU DDS history.

The live-tested TinyNav source deliberately configured a 10,000-sample
best-effort IMU reader.  At 200 Hz that can retain roughly 50 seconds of old
samples.  Once the Python executor falls behind under BuildMap load it can
continue consuming history at approximately the arrival rate and never make
the IMU watermark current again.

This deployment entry point leaves the pinned source file byte-identical.  It
verifies that file's SHA-256, bounds only the module globals used when the ROS
subscriptions are constructed, and then calls the source ``main`` function.
The source's existing timestamp-order, IMU-gap, optimizer, and re-anchor gates
remain authoritative.
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path


PINNED_PERCEPTION_SHA256 = (
    "3a695d5210d60ea1f721549ca7458ba89e7bf32db5178cd1c312c633aef1c3b3"
)
# At the observed 200 Hz IMU rate, 200 samples retain about one second.  That
# covers the 0.12 s executor stalls observed during Scene 03 Formal 07 while
# remaining far below the former 10,000-sample (roughly 50 s) backlog.
DEFAULT_IMU_QOS_DEPTH = 200
MAXIMUM_IMU_QOS_DEPTH = 400
DEFAULT_IMU_BUFFER_SIZE = 4_000


def bounded_integer(
    name: str,
    *,
    default: int,
    minimum: int,
    maximum: int,
) -> int:
    raw = os.environ.get(name)
    value = default if raw is None else int(raw)
    if not minimum <= value <= maximum:
        raise ValueError(
            f"{name} must be in [{minimum}, {maximum}], got {value}"
        )
    return value


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    from tinynav.core import perception_node

    source_path = Path(perception_node.__file__).resolve()
    expected_sha = os.environ.get(
        "FOCUS_WSJ_PINNED_PERCEPTION_SHA256",
        PINNED_PERCEPTION_SHA256,
    )
    actual_sha = sha256_file(source_path)
    if actual_sha != expected_sha:
        raise SystemExit(
            "WSJ TinyNav perception source hash mismatch: "
            f"expected={expected_sha} actual={actual_sha} path={source_path}"
        )

    qos_depth = bounded_integer(
        "FOCUS_WSJ_IMU_QOS_DEPTH",
        default=DEFAULT_IMU_QOS_DEPTH,
        minimum=50,
        maximum=MAXIMUM_IMU_QOS_DEPTH,
    )
    buffer_size = bounded_integer(
        "FOCUS_WSJ_IMU_BUFFER_SIZE",
        default=DEFAULT_IMU_BUFFER_SIZE,
        minimum=qos_depth,
        maximum=20_000,
    )
    perception_node._IMU_QOS_DEPTH = qos_depth
    perception_node._IMU_BUFFER_SIZE = buffer_size
    print(
        "WSJ_PERCEPTION_BOUNDED_IMU_HISTORY: "
        f"qos_depth={qos_depth} buffer_size={buffer_size} "
        f"source_sha256={actual_sha} source={source_path}",
        flush=True,
    )
    perception_node.main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
