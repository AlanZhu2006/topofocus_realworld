#!/usr/bin/env python3
"""Build a strict v2 batch from one frozen VLM round without publishing it."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

from focus_hub.v2_scene_batch import build_batch_from_shadow_manifest  # noqa: E402


def atomic_write_json(path: Path, payload: object) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument(
        "--registry-state",
        type=Path,
        default=WORKSPACE / "hub/runtime/state/registry_state.json",
    )
    parser.add_argument(
        "--robot-config",
        type=Path,
        default=WORKSPACE / "hub/config/robots.json",
    )
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--episode-id", required=True)
    parser.add_argument("--execution-epoch", type=int, default=0)
    parser.add_argument("--lease-s", type=float, default=8.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.execution_epoch < 0:
        parser.error("--execution-epoch must be non-negative")
    if not 0.1 <= args.lease_s <= 10.0:
        parser.error("--lease-s must be between 0.1 and 10.0")
    output = args.output.expanduser().resolve()
    if output.exists():
        print(f"refusing to overwrite existing output: {output}", file=sys.stderr)
        return 2
    output.mkdir(parents=True)

    built = build_batch_from_shadow_manifest(
        args.manifest,
        args.registry_state,
        scene_id=args.scene_id,
        episode_id=args.episode_id,
        execution_epoch=args.execution_epoch,
        now_ns=time.time_ns(),
        robot_config_path=args.robot_config,
        lease_duration_ns=int(args.lease_s * 1e9),
    )
    batch_path = output / "decision_batch.json"
    report_path = output / "preflight_report.json"
    atomic_write_json(batch_path, built.batch.model_dump(mode="json"))
    atomic_write_json(report_path, built.report)
    print(json.dumps({
        "status": built.report["status"],
        "preflight_ready": built.report["preflight_ready"],
        "active_robot_ids": built.report["active_robot_ids"],
        "robot_commands_sent": False,
        "batch": str(batch_path),
        "report": str(report_path),
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
