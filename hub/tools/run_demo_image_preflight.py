#!/usr/bin/env python3
"""Verify and optionally repeat Perception-VLM calls on historical images.

This tool has no Hub client, no decision publisher and no robot command path.
Its output is explicitly ineligible for navigation SR/SPL.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from statistics import fmean, pstdev
import sys
import time

import cv2
import httpx


WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

from focus_hub.demo_image_preflight import (  # noqa: E402
    ImagePreflightManifest,
    validate_historical_inputs,
)
from focus_hub.shadow_coordination import sha256_file  # noqa: E402
from focus_hub.vlm_decision import choose_scene_worth_exploring_glm  # noqa: E402
from focus_hub.yolo_detector import YoloDetector  # noqa: E402


DEFAULT_MANIFEST = (
    WORKSPACE
    / "hub"
    / "config"
    / "experiments"
    / "triple_ai_demo_image_preflight_v1.json"
)
DEFAULT_YOLO_WEIGHTS = WORKSPACE / "artifacts" / "vision" / "yolov10m.pt"


def atomic_write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Verify the four historical image cases and optionally run the "
            "source Perception-VLM stage five times per robot image."
        )
    )
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--run-perception",
        action="store_true",
        help="call local YOLO once per image and GLM for every planned repetition",
    )
    parser.add_argument(
        "--glm-url", default="http://127.0.0.1:31511/v1"
    )
    parser.add_argument("--yolo-weights", type=Path, default=DEFAULT_YOLO_WEIGHTS)
    parser.add_argument("--timeout-s", type=float, default=120.0)
    return parser.parse_args()


def load_manifest(path: Path) -> ImagePreflightManifest:
    return ImagePreflightManifest.model_validate_json(
        path.read_text(encoding="utf-8")
    )


def run_perception_repetitions(
    manifest: ImagePreflightManifest,
    *,
    glm_url: str,
    yolo_weights: Path,
    timeout_s: float,
) -> dict[str, object]:
    model_response = httpx.get(f"{glm_url}/models", timeout=10.0)
    model_response.raise_for_status()
    detector = YoloDetector(yolo_weights, conf=0.2)
    cases: list[dict[str, object]] = []
    errors: list[str] = []
    call_count = 0

    for case in manifest.cases:
        observations: list[dict[str, object]] = []
        for observation in case.observations:
            rgb_path = WORKSPACE / observation.rgb.path
            image = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError(f"failed to decode {rgb_path}")
            detections = detector.detect(image)
            repetitions: list[dict[str, object]] = []
            for repetition_index in range(1, manifest.repetitions_per_case + 1):
                started = time.monotonic()
                try:
                    yes, no = choose_scene_worth_exploring_glm(
                        image,
                        target=case.target_category,
                        detections=detections,
                        base_url=glm_url,
                        timeout_s=timeout_s,
                    )
                    repetition = {
                        "repetition": repetition_index,
                        "p_yes": yes,
                        "p_no": no,
                        "latency_s": time.monotonic() - started,
                        "status": "model_inference_observed",
                    }
                except Exception as exc:  # keep the complete preflight record
                    message = (
                        f"{case.case_id}/{observation.robot_id}/"
                        f"{repetition_index}: {type(exc).__name__}: {exc}"
                    )
                    errors.append(message)
                    repetition = {
                        "repetition": repetition_index,
                        "latency_s": time.monotonic() - started,
                        "status": "model_inference_failed",
                        "error": message,
                    }
                call_count += 1
                repetitions.append(repetition)

            valid = [item for item in repetitions if "p_yes" in item]
            yes_values = [float(item["p_yes"]) for item in valid]
            no_values = [float(item["p_no"]) for item in valid]
            observations.append(
                {
                    "robot_id": observation.robot_id,
                    "sequence": observation.sequence,
                    "rgb_path": observation.rgb.path,
                    "target_category": case.target_category,
                    "recorded_wire_goal_category": (
                        observation.wire_object_goal_category
                    ),
                    "detections": detections,
                    "repetitions": repetitions,
                    "successful_calls": len(valid),
                    "p_yes_mean": fmean(yes_values) if yes_values else None,
                    "p_yes_population_std": (
                        pstdev(yes_values) if yes_values else None
                    ),
                    "p_no_mean": fmean(no_values) if no_values else None,
                }
            )
        cases.append(
            {
                "case_id": case.case_id,
                "target_category": case.target_category,
                "observations": observations,
            }
        )

    return {
        "status": "completed" if not errors else "completed_with_errors",
        "classification": "model-derived historical-image perception preflight",
        "official_navigation_metrics_eligible": False,
        "model_endpoint_response": model_response.json(),
        "yolo_provenance": detector.provenance,
        "yolo_policy": "one deterministic detection pass per historical image",
        "glm_call_count": call_count,
        "errors": errors,
        "cases": cases,
    }


def main() -> int:
    args = parse_args()
    manifest_path = args.manifest.expanduser().resolve()
    manifest = load_manifest(manifest_path)
    started_at_ns = time.time_ns()
    report: dict[str, object] = {
        "started_at_ns": started_at_ns,
        "manifest": {
            "path": str(manifest_path),
            "size_bytes": manifest_path.stat().st_size,
            "sha256": sha256_file(manifest_path),
            "classification": "source-derived tracked experiment configuration",
        },
        "input_validation": validate_historical_inputs(
            manifest,
            workspace=WORKSPACE,
        ),
        "robot_commands_sent": False,
        "hub_publications": 0,
    }
    exit_code = 0
    if args.run_perception:
        perception = run_perception_repetitions(
            manifest,
            glm_url=args.glm_url,
            yolo_weights=args.yolo_weights.expanduser().resolve(),
            timeout_s=args.timeout_s,
        )
        report["perception_preflight"] = perception
        if perception["status"] != "completed":
            exit_code = 1
    else:
        report["perception_preflight"] = {
            "status": "not_requested",
            "official_navigation_metrics_eligible": False,
        }
    report["completed_at_ns"] = time.time_ns()
    report["duration_s"] = (report["completed_at_ns"] - started_at_ns) / 1e9

    if args.output is not None:
        atomic_write_json(args.output.expanduser().resolve(), report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
