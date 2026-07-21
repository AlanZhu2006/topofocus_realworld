#!/usr/bin/env python3
"""Convert a finalized TinyNav BuildMap occupancy volume for Hub/Foxglove.

Read-only with respect to the TinyNav record. This tool sends no network
traffic, publishes no decisions, and has no robot-control interface.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

from focus_hub.tinynav_occupancy import (  # noqa: E402
    load_tinynav_occupancy,
    write_hub_snapshot,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True,
                        help="finalized TinyNav BuildMap output directory")
    parser.add_argument("--out-dir", type=Path, required=True,
                        help="new Hub snapshot directory; existing maps are never overwritten")
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--frame-id", required=True,
                        help="session-local or calibrated frame name published to Foxglove")
    parser.add_argument("--transform-version", required=True,
                        help="version binding for this map session/calibration")
    args = parser.parse_args()

    native = load_tinynav_occupancy(args.record)
    summary = write_hub_snapshot(
        native,
        args.out_dir,
        robot_id=args.robot_id,
        frame_id=args.frame_id,
        transform_version=args.transform_version,
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

