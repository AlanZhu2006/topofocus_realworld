#!/usr/bin/env python3
"""Select one fresh synchronized pair in which both cameras see the board.

This tool reads only append-only Hub spool data.  It is intended for the
interactive calibration wrapper: record a sequence boundary, ask the operator
to place or move the board, then select only observations newer than that
boundary.  No robot, ROS, WATER, TinyNav or command interface is opened.
"""
from __future__ import annotations

import argparse
from contextlib import redirect_stdout
import hashlib
import io
import json
import os
from pathlib import Path
import sys
import time

import numpy as np


WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub/src"))
sys.path.insert(0, str(WORKSPACE / "hub/tools"))

from calibrate_camera_offset_via_board import find_board_pose  # noqa: E402
from focus_hub.models import ObservationMetadata  # noqa: E402


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact(path: Path) -> dict[str, object]:
    return {
        "path": str(path.resolve()),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
        "classification": "observed_hub_spool_input",
    }


def image_path(directory: Path, metadata: ObservationMetadata) -> Path:
    suffix = ".jpg" if metadata.rgb_encoding == "jpeg" else ".png"
    return directory / f"rgb{suffix}"


def candidate_directories(
    spool: Path,
    robot_id: str,
    *,
    after_sequence: int,
    max_candidates: int,
) -> list[Path]:
    root = spool / robot_id
    rows: list[tuple[int, Path]] = []
    if not root.is_dir():
        return []
    for path in root.iterdir():
        if not path.is_dir() or not path.name.isdigit():
            continue
        sequence = int(path.name)
        if sequence > after_sequence:
            rows.append((sequence, path))
    rows.sort(reverse=True)
    return [path for _, path in rows[:max_candidates]]


def detect_candidates(
    spool: Path,
    robot_id: str,
    *,
    after_sequence: int,
    expected_transform_version: str,
    max_candidates: int,
    max_age_s: float,
    rows: int,
    cols: int,
    spacing_m: float,
    now_ns: int,
) -> list[dict[str, object]]:
    detections: list[dict[str, object]] = []
    for directory in candidate_directories(
        spool,
        robot_id,
        after_sequence=after_sequence,
        max_candidates=max_candidates,
    ):
        metadata_path = directory / "metadata.json"
        if not metadata_path.is_file():
            continue
        try:
            metadata = ObservationMetadata.model_validate_json(
                metadata_path.read_text(encoding="utf-8")
            )
        except Exception:
            continue
        if metadata.pose.transform_version != expected_transform_version:
            continue
        age_s = (now_ns - metadata.capture_time_ns) / 1e9
        if age_s < -0.25 or age_s > max_age_s:
            continue
        rgb_path = image_path(directory, metadata)
        if not rgb_path.is_file():
            continue
        intrinsics = metadata.intrinsics
        camera_matrix = np.asarray(
            [
                [intrinsics.fx, 0.0, intrinsics.cx],
                [0.0, intrinsics.fy, intrinsics.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        distortion = np.asarray(
            intrinsics.distortion or [0.0] * 5, dtype=np.float64
        )
        detector_log = io.StringIO()
        try:
            with redirect_stdout(detector_log):
                camera_from_grid = find_board_pose(
                    str(rgb_path),
                    rows,
                    cols,
                    spacing_m,
                    camera_matrix,
                    distortion,
                )
        except SystemExit:
            continue
        grid_from_center = np.eye(4, dtype=np.float64)
        grid_from_center[:3, 3] = [
            (cols - 1) * spacing_m / 2.0,
            (rows - 1) * spacing_m / 2.0,
            0.0,
        ]
        shared_from_camera = np.asarray(
            metadata.pose.shared_T_camera.matrix, dtype=np.float64
        ).reshape(4, 4)
        shared_from_board = (
            shared_from_camera @ camera_from_grid @ grid_from_center
        )
        detections.append(
            {
                "robot_id": robot_id,
                "sequence": metadata.sequence,
                "capture_time_ns": metadata.capture_time_ns,
                "age_s": age_s,
                "transform_version": metadata.pose.transform_version,
                "metadata": artifact(metadata_path),
                "rgb": artifact(rgb_path),
                "shared_from_board_matrix": shared_from_board.reshape(-1).tolist(),
                "detector_log": detector_log.getvalue().strip(),
            }
        )
    return detections


def candidate_records(
    spool: Path,
    robot_id: str,
    *,
    after_sequence: int,
    expected_transform_version: str,
    max_candidates: int,
    max_age_s: float,
    now_ns: int,
) -> list[dict[str, object]]:
    """Load cheap metadata first; image detection happens only after pairing."""

    records: list[dict[str, object]] = []
    for directory in candidate_directories(
        spool,
        robot_id,
        after_sequence=after_sequence,
        max_candidates=max_candidates,
    ):
        metadata_path = directory / "metadata.json"
        if not metadata_path.is_file():
            continue
        try:
            metadata = ObservationMetadata.model_validate_json(
                metadata_path.read_text(encoding="utf-8")
            )
        except Exception:
            continue
        if metadata.pose.transform_version != expected_transform_version:
            continue
        age_s = (now_ns - metadata.capture_time_ns) / 1e9
        if age_s < -0.25 or age_s > max_age_s:
            continue
        rgb_path = image_path(directory, metadata)
        if not rgb_path.is_file():
            continue
        records.append(
            {
                "robot_id": robot_id,
                "sequence": metadata.sequence,
                "capture_time_ns": metadata.capture_time_ns,
                "age_s": age_s,
                "directory": directory,
                "metadata_path": metadata_path,
                "rgb_path": rgb_path,
                "metadata_model": metadata,
            }
        )
    return records


def synchronized_candidate_pairs(
    reference: list[dict[str, object]],
    other: list[dict[str, object]],
    *,
    max_sync_skew_s: float,
) -> list[
    tuple[float, dict[str, object], dict[str, object]]
]:
    """Pair timestamps before running the comparatively expensive detector."""

    pairs: list[
        tuple[float, int, dict[str, object], dict[str, object]]
    ] = []
    for reference_row in reference:
        for other_row in other:
            skew_s = abs(
                int(reference_row["capture_time_ns"])
                - int(other_row["capture_time_ns"])
            ) / 1e9
            if skew_s > max_sync_skew_s:
                continue
            newest_common_ns = min(
                int(reference_row["capture_time_ns"]),
                int(other_row["capture_time_ns"]),
            )
            pairs.append(
                (skew_s, -newest_common_ns, reference_row, other_row)
            )
    pairs.sort(key=lambda item: (item[0], item[1]))
    return [
        (skew_s, reference_row, other_row)
        for skew_s, _, reference_row, other_row in pairs
    ]


def detect_candidate_record(
    record: dict[str, object],
    *,
    rows: int,
    cols: int,
    spacing_m: float,
) -> dict[str, object] | None:
    metadata = record["metadata_model"]
    if not isinstance(metadata, ObservationMetadata):
        raise TypeError("candidate record lacks validated observation metadata")
    rgb_path = record["rgb_path"]
    metadata_path = record["metadata_path"]
    if not isinstance(rgb_path, Path) or not isinstance(metadata_path, Path):
        raise TypeError("candidate record lacks artifact paths")
    intrinsics = metadata.intrinsics
    camera_matrix = np.asarray(
        [
            [intrinsics.fx, 0.0, intrinsics.cx],
            [0.0, intrinsics.fy, intrinsics.cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    distortion = np.asarray(
        intrinsics.distortion or [0.0] * 5, dtype=np.float64
    )
    detector_log = io.StringIO()
    try:
        with redirect_stdout(detector_log):
            camera_from_grid = find_board_pose(
                str(rgb_path),
                rows,
                cols,
                spacing_m,
                camera_matrix,
                distortion,
            )
    except SystemExit:
        return None
    grid_from_center = np.eye(4, dtype=np.float64)
    grid_from_center[:3, 3] = [
        (cols - 1) * spacing_m / 2.0,
        (rows - 1) * spacing_m / 2.0,
        0.0,
    ]
    shared_from_camera = np.asarray(
        metadata.pose.shared_T_camera.matrix, dtype=np.float64
    ).reshape(4, 4)
    shared_from_board = (
        shared_from_camera @ camera_from_grid @ grid_from_center
    )
    return {
        "robot_id": metadata.robot_id,
        "sequence": metadata.sequence,
        "capture_time_ns": metadata.capture_time_ns,
        "age_s": record["age_s"],
        "transform_version": metadata.pose.transform_version,
        "metadata": artifact(metadata_path),
        "rgb": artifact(rgb_path),
        "shared_from_board_matrix": shared_from_board.reshape(-1).tolist(),
        "detector_log": detector_log.getvalue().strip(),
    }


def detect_synchronized_pair(
    reference: list[dict[str, object]],
    other: list[dict[str, object]],
    *,
    max_sync_skew_s: float,
    rows: int,
    cols: int,
    spacing_m: float,
) -> tuple[
    dict[str, object],
    dict[str, object],
    float,
    dict[str, int],
]:
    pairs = synchronized_candidate_pairs(
        reference, other, max_sync_skew_s=max_sync_skew_s
    )
    if not pairs:
        raise ValueError(
            "no time-synchronized candidate observations; both cameras must "
            "continue uploading while the board is visible"
        )
    caches: dict[str, dict[int, dict[str, object] | None]] = {
        "reference": {},
        "other": {},
    }
    for skew_s, reference_record, other_record in pairs:
        reference_sequence = int(reference_record["sequence"])
        other_sequence = int(other_record["sequence"])
        if reference_sequence not in caches["reference"]:
            caches["reference"][reference_sequence] = detect_candidate_record(
                reference_record,
                rows=rows,
                cols=cols,
                spacing_m=spacing_m,
            )
        if other_sequence not in caches["other"]:
            caches["other"][other_sequence] = detect_candidate_record(
                other_record,
                rows=rows,
                cols=cols,
                spacing_m=spacing_m,
            )
        reference_detection = caches["reference"][reference_sequence]
        other_detection = caches["other"][other_sequence]
        if reference_detection is not None and other_detection is not None:
            return (
                reference_detection,
                other_detection,
                skew_s,
                {
                    "synchronized_pairs": len(pairs),
                    "reference_images_detected": sum(
                        value is not None
                        for value in caches["reference"].values()
                    ),
                    "other_images_detected": sum(
                        value is not None
                        for value in caches["other"].values()
                    ),
                },
            )
    raise ValueError(
        "synchronized observations arrived, but the complete board was not "
        "detected in both views"
    )


def choose_pair(
    reference: list[dict[str, object]],
    other: list[dict[str, object]],
    *,
    max_sync_skew_s: float,
) -> tuple[dict[str, object], dict[str, object], float]:
    candidates: list[
        tuple[float, int, dict[str, object], dict[str, object]]
    ] = []
    for reference_row in reference:
        for other_row in other:
            skew_s = abs(
                int(reference_row["capture_time_ns"])
                - int(other_row["capture_time_ns"])
            ) / 1e9
            if skew_s <= max_sync_skew_s:
                newest_common_ns = min(
                    int(reference_row["capture_time_ns"]),
                    int(other_row["capture_time_ns"]),
                )
                candidates.append(
                    (skew_s, -newest_common_ns, reference_row, other_row)
                )
    if not candidates:
        raise ValueError(
            "no synchronized pair with a detected board; keep the full board "
            "visible to both cameras and capture again"
        )
    skew_s, _, reference_row, other_row = min(
        candidates, key=lambda item: (item[0], item[1])
    )
    return reference_row, other_row, skew_s


def atomic_write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--spool", type=Path, default=WORKSPACE / "hub/runtime/spool"
    )
    parser.add_argument("--reference-robot", default="robot-0")
    parser.add_argument("--other-robot", default="robot-1")
    parser.add_argument("--reference-after-sequence", type=int, required=True)
    parser.add_argument("--other-after-sequence", type=int, required=True)
    parser.add_argument("--reference-transform-version", required=True)
    parser.add_argument("--other-transform-version", required=True)
    parser.add_argument("--rows", type=int, default=7)
    parser.add_argument("--cols", type=int, default=10)
    parser.add_argument("--spacing-m", type=float, default=0.04)
    parser.add_argument("--max-sync-skew-s", type=float, default=0.25)
    parser.add_argument("--max-age-s", type=float, default=30.0)
    parser.add_argument("--max-candidates", type=int, default=500)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.max_sync_skew_s <= 0.0 or args.max_age_s <= 0.0:
        parser.error("time thresholds must be positive")
    if not 1 <= args.max_candidates <= 2000:
        parser.error("--max-candidates must be between 1 and 2000")
    now_ns = time.time_ns()
    reference = candidate_records(
        args.spool,
        args.reference_robot,
        after_sequence=args.reference_after_sequence,
        expected_transform_version=args.reference_transform_version,
        max_candidates=args.max_candidates,
        max_age_s=args.max_age_s,
        now_ns=now_ns,
    )
    other = candidate_records(
        args.spool,
        args.other_robot,
        after_sequence=args.other_after_sequence,
        expected_transform_version=args.other_transform_version,
        max_candidates=args.max_candidates,
        max_age_s=args.max_age_s,
        now_ns=now_ns,
    )
    try:
        reference_row, other_row, skew_s, diagnostics = (
            detect_synchronized_pair(
                reference,
                other,
                max_sync_skew_s=args.max_sync_skew_s,
                rows=args.rows,
                cols=args.cols,
                spacing_m=args.spacing_m,
            )
        )
    except ValueError as exc:
        parser.error(str(exc))
    payload = {
        "schema_version": "focus-live-board-pair-v1",
        "selected_at_ns": now_ns,
        "reference": reference_row,
        "other": other_row,
        "capture_skew_s": skew_s,
        "candidate_counts": {
            args.reference_robot: len(reference),
            args.other_robot: len(other),
        },
        "selection_diagnostics": diagnostics,
        "safety": {
            "robot_interfaces_used": False,
            "robot_commands_issued": False,
            "spool_read_only": True,
        },
    }
    if args.output is not None:
        if args.output.exists():
            parser.error(f"refusing to overwrite output: {args.output}")
        atomic_write_json(args.output, payload)
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
