#!/usr/bin/env python3
"""Record an operator-measured physical base_link-to-camera transform.

This tool performs no sensing and issues no robot command. It preserves the
entered mount measurement as a reviewed provenance artifact; it does not
claim an independent survey.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time


WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub/src"))

from focus_hub.base_camera_calibration import load_base_camera_calibration  # noqa: E402


CONFIRMATION = "PHYSICAL_MOUNT_VALUES_REVIEWED"


def rpy_matrix(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> list[float]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Rz(yaw) @ Ry(pitch) @ Rx(roll)
    return [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y,
        -sp, cp * sr, cp * cr, z,
        0.0, 0.0, 0.0, 1.0,
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-id", required=True)
    parser.add_argument("--camera-frame", required=True)
    parser.add_argument("--x-m", type=float, required=True)
    parser.add_argument("--y-m", type=float, required=True)
    parser.add_argument("--z-m", type=float, required=True)
    parser.add_argument("--roll-deg", type=float, required=True)
    parser.add_argument("--pitch-deg", type=float, required=True)
    parser.add_argument("--yaw-deg", type=float, required=True)
    parser.add_argument("--measurement-note", required=True)
    parser.add_argument(
        "--measurement-status",
        choices=("operator_measured_physical_mount", "surveyed_physical_mount"),
        default="operator_measured_physical_mount",
    )
    parser.add_argument("--operator-confirmation", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.operator_confirmation != CONFIRMATION:
        parser.error("confirmation must be " + CONFIRMATION)
    values = (
        args.x_m,
        args.y_m,
        args.z_m,
        args.roll_deg,
        args.pitch_deg,
        args.yaw_deg,
    )
    if not all(math.isfinite(value) for value in values):
        parser.error("mount values must be finite")
    output = args.output.expanduser().resolve()
    if output.exists():
        parser.error(f"refusing to overwrite existing output: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    matrix = rpy_matrix(
        args.x_m,
        args.y_m,
        args.z_m,
        math.radians(args.roll_deg),
        math.radians(args.pitch_deg),
        math.radians(args.yaw_deg),
    )
    artifact = {
        "schema_version": "focus-base-camera-calibration-v1",
        "robot_id": args.robot_id,
        "base_T_camera": {
            "parent_frame": "base_link",
            "child_frame": args.camera_frame,
            "matrix": matrix,
            "convention": "row_major_T_parent_child_rz_ry_rx",
        },
        "measurement": {
            "status": args.measurement_status,
            "note": args.measurement_note,
            "entered_at_ns": time.time_ns(),
            "independently_verified": args.measurement_status == "surveyed_physical_mount",
        },
        "passed": True,
        "safety": {
            "robot_commands_issued": False,
            "robot_interfaces_used": False,
        },
    }
    temporary = output.with_name(f".{output.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(artifact, indent=2, sort_keys=True) + "\n")
    os.replace(temporary, output)
    loaded = load_base_camera_calibration(
        output,
        expected_robot_id=args.robot_id,
        expected_camera_frame=args.camera_frame,
    )
    print(json.dumps({
        "output": str(output),
        "size_bytes": loaded.source_size_bytes,
        "sha256": loaded.source_sha256,
        "measurement_status": loaded.measurement_status,
        "robot_commands_issued": False,
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
