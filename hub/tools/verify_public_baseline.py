#!/usr/bin/env python3
"""Run the read-only public deployment-baseline verifier."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
VALIDATOR_PATH = (
    REPOSITORY_ROOT / "hub" / "src" / "focus_hub" / "public_baseline.py"
)


def _load_validator():
    """Load the standard-library verifier without importing Hub dependencies."""

    spec = importlib.util.spec_from_file_location(
        "_focus_public_baseline", VALIDATOR_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load baseline validator: {VALIDATOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    validator = _load_validator()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        type=Path,
        default=REPOSITORY_ROOT,
        help="repository root (default: inferred from this script)",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        help="optional manifest path; file contracts remain workspace-relative",
    )
    args = parser.parse_args()
    try:
        summary = validator.validate_public_baseline(
            args.workspace, args.manifest
        )
    except validator.BaselineValidationError as exc:
        print(f"PUBLIC_BASELINE_INVALID: {exc}", file=sys.stderr)
        return 1
    print(
        "PUBLIC_BASELINE_OK "
        + json.dumps(
            {
                "baseline_id": summary.baseline_id,
                "file_count": summary.file_count,
                "total_bytes": summary.total_bytes,
                "physical_commands_sent": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
