#!/usr/bin/env python3
"""Computes a --offset-file for calibrate_shared_frame.py from a shared
calibration board seen by both robots' cameras from different positions.

calibrate_shared_frame.py's default (no --offset-file) assumes the two
robots were physically coincident (same position and orientation) at the
sync instant. That assumption does not hold when the robots are placed at
different spots around a shared calibration target instead -- this script
covers that case: each robot's own RGB frame is used to solve the board's
pose in that camera's own optical frame via PnP (a symmetric circle grid,
detected with cv2.findCirclesGrid), and the two per-camera board poses are
composed into the one fixed transform calibrate_shared_frame.py's
--offset-file expects: T_referenceCamera_otherCamera (the reference robot's
camera frame to the other robot's camera frame, at the instant both images
were captured).

Caveat, stated plainly: both PnP solves use each robot's own COLOR optical
frame intrinsics (matching what was used to capture the calibration image),
and the resulting offset is between the two cameras' COLOR optical frames.
This is only exactly correct if each robot's published `pose.shared_T_camera`
also represents its color optical frame's pose -- confirmed for the yunji
D405 path (see MEASURED_T_BASE_LINK_CAMERA_D405's derivation), NOT
independently re-verified for wsj's TinyNav sender here. If wsj's pose
stream actually represents a different frame (e.g. depth optical), this
offset carries a fixed, uncorrected error equal to that frame's difference
from the color optical frame.

The two images must be captured close enough in time that neither robot
moved meaningfully between them -- this script does not check that; the
caller is responsible for capturing them together, the same operational
requirement calibrate_shared_frame.py itself has for its two pose readings.
"""
from __future__ import annotations

import argparse
import json
import sys

import cv2
import numpy as np


def build_object_points(rows: int, cols: int, spacing_m: float) -> np.ndarray:
    """Object points for a symmetric circle grid, board frame at Z=0,
    origin at the first (top-left) circle, X increasing along a row."""
    points = np.zeros((rows * cols, 3), dtype=np.float64)
    for r in range(rows):
        for c in range(cols):
            points[r * cols + c] = (c * spacing_m, r * spacing_m, 0.0)
    return points


def canonicalize_grid_centers(centers: np.ndarray) -> tuple[np.ndarray, bool]:
    """Make the image's upper-left diagonal endpoint the grid origin.

    A symmetric circle grid is unchanged by a 180-degree rotation, and
    ``findCirclesGrid`` can therefore reverse all 70 centers between otherwise
    identical frames.  The robots observe the same face of an upright board,
    so anchoring the ordering to the image diagonal removes that frame-to-frame
    flip without operator-selected points.
    """
    points = np.asarray(centers, dtype=np.float32).reshape(-1, 2)
    first_key = (float(points[0].sum()), float(points[0, 1]), float(points[0, 0]))
    last_key = (float(points[-1].sum()), float(points[-1, 1]), float(points[-1, 0]))
    reversed_order = first_key > last_key
    if reversed_order:
        points = points[::-1].copy()
    return points.reshape(-1, 1, 2), reversed_order


def find_board_pose(
    image_path: str, rows: int, cols: int, spacing_m: float,
    K: np.ndarray, dist: np.ndarray,
) -> np.ndarray:
    """Returns T_camera_board (4x4) by detecting a symmetric circle grid and
    solving PnP. Tries both (cols, rows) and (rows, cols) as OpenCV's
    patternSize, since "7x10" from a verbal description doesn't pin down
    which axis is rows vs. columns in image space.
    """
    image = cv2.imread(image_path)
    if image is None:
        raise SystemExit(f"could not read image: {image_path}")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Keep the original 4x/default attempt first so previously accepted
    # evidence is solved exactly as before.  A closer board can produce many
    # carpet/IR blob candidates at 4x, while Odin's wider image needs OpenCV's
    # clustering pass to isolate the board.  The 2x fallbacks cover both cases
    # without cropping or operator-selected points.
    detector_attempts = (
        (4.0, cv2.CALIB_CB_SYMMETRIC_GRID, "default"),
        (2.0, cv2.CALIB_CB_SYMMETRIC_GRID, "default"),
        (
            4.0,
            cv2.CALIB_CB_SYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING,
            "clustering",
        ),
        (
            2.0,
            cv2.CALIB_CB_SYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING,
            "clustering",
        ),
    )
    scaled_images: dict[float, np.ndarray] = {}
    for upscale, flags, detector_name in detector_attempts:
        upscaled = scaled_images.setdefault(
            upscale,
            cv2.resize(
                gray,
                None,
                fx=upscale,
                fy=upscale,
                interpolation=cv2.INTER_CUBIC,
            ),
        )
        for pattern_size in ((cols, rows), (rows, cols)):
            found, centers = cv2.findCirclesGrid(
                upscaled, pattern_size, flags=flags
            )
            if not found:
                continue
            centers = centers / upscale
            centers, reversed_order = canonicalize_grid_centers(centers)
            actual_cols, actual_rows = pattern_size
            object_points = build_object_points(actual_rows, actual_cols, spacing_m)
            ok, rvec, tvec = cv2.solvePnP(object_points, centers, K, dist)
            if not ok:
                raise SystemExit(f"{image_path}: circle grid found but solvePnP failed")
            R, _ = cv2.Rodrigues(rvec)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = tvec.reshape(-1)
            print(
                f"{image_path}: detected {pattern_size[0]}x{pattern_size[1]} grid "
                f"with {detector_name}/{upscale:g}x, board is "
                f"{np.linalg.norm(tvec):.3f}m from the camera, "
                f"ordering_reversed={str(reversed_order).lower()}"
            )
            return T
    raise SystemExit(
        f"{image_path}: could not detect a {rows}x{cols} symmetric circle grid "
        f"(tried both axis orderings) -- is the full board visible and undistorted "
        f"enough for detection?")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-image", required=True,
                         help="RGB frame from the reference robot's camera (e.g. wsj)")
    parser.add_argument("--other-image", required=True,
                         help="RGB frame from the other robot's camera (e.g. yunji), "
                              "captured at the same time as --reference-image")
    parser.add_argument("--rows", type=int, required=True)
    parser.add_argument("--cols", type=int, required=True)
    parser.add_argument("--spacing-m", type=float, required=True,
                         help="distance between adjacent circle centers, in metres")
    parser.add_argument("--reference-fx", type=float, required=True)
    parser.add_argument("--reference-fy", type=float, required=True)
    parser.add_argument("--reference-cx", type=float, required=True)
    parser.add_argument("--reference-cy", type=float, required=True)
    parser.add_argument("--reference-dist", type=float, nargs="*", default=[0.0] * 5,
                         help="reference camera's distortion coefficients (k1 k2 p1 p2 k3); "
                              "default zero (already-rectified stream)")
    parser.add_argument("--other-fx", type=float, required=True)
    parser.add_argument("--other-fy", type=float, required=True)
    parser.add_argument("--other-cx", type=float, required=True)
    parser.add_argument("--other-cy", type=float, required=True)
    parser.add_argument("--other-dist", type=float, nargs="*", default=[0.0] * 5)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    K_ref = np.array([[args.reference_fx, 0, args.reference_cx],
                       [0, args.reference_fy, args.reference_cy], [0, 0, 1.0]])
    K_other = np.array([[args.other_fx, 0, args.other_cx],
                         [0, args.other_fy, args.other_cy], [0, 0, 1.0]])
    dist_ref = np.array(args.reference_dist, dtype=np.float64)
    dist_other = np.array(args.other_dist, dtype=np.float64)

    T_ref_board = find_board_pose(
        args.reference_image, args.rows, args.cols, args.spacing_m, K_ref, dist_ref)
    T_other_board = find_board_pose(
        args.other_image, args.rows, args.cols, args.spacing_m, K_other, dist_other)

    T_ref_other = T_ref_board @ np.linalg.inv(T_other_board)

    translation = T_ref_other[:3, 3]
    print(f"\nT_reference_camera_other_camera: translation={np.round(translation, 4).tolist()} m, "
          f"distance={np.linalg.norm(translation):.3f}m")

    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump({"matrix": T_ref_other.reshape(-1).tolist()}, handle, indent=2)
    print(f"wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
