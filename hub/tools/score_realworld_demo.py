#!/usr/bin/env python3
"""Validate recorded supervised-autonomy episodes and report SR/SPL."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys


WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

from focus_hub.realworld_eval import (  # noqa: E402
    RealworldExperimentResults,
    score_experiment,
    validate_result_evidence,
)
from focus_hub.shadow_coordination import sha256_file  # noqa: E402


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
            "Score four-scene/five-trial supervised real-world results. "
            "Static image preflights are not accepted by this schema."
        )
    )
    parser.add_argument("--records", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--expected-scenes", type=int, default=4)
    parser.add_argument("--expected-trials-per-scene", type=int, default=5)
    parser.add_argument(
        "--allow-incomplete",
        action="store_true",
        help="produce a progress report while retaining explicit shape errors",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    records_path = args.records.expanduser().resolve()
    results = RealworldExperimentResults.model_validate_json(
        records_path.read_text(encoding="utf-8")
    )
    report = score_experiment(
        results,
        expected_scenes=args.expected_scenes,
        expected_trials_per_scene=args.expected_trials_per_scene,
        allow_incomplete=args.allow_incomplete,
    )
    report["records_provenance"] = {
        "path": str(records_path),
        "size_bytes": records_path.stat().st_size,
        "sha256": sha256_file(records_path),
        "classification": "observed supervised-autonomy episode record",
    }
    report["evidence_validation"] = validate_result_evidence(
        results,
        workspace=WORKSPACE,
    )
    if args.output is not None:
        atomic_write_json(args.output.expanduser().resolve(), report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
