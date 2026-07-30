#!/usr/bin/env python3
"""Fail-closed preflight for the executable-source semantic stack."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys


WORKSPACE = Path(__file__).resolve().parents[2]
HUB_SRC = WORKSPACE / "hub" / "src"
DEPENDENCIES = WORKSPACE / "dependencies"
for path in (HUB_SRC, DEPENDENCIES):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from focus_hub.source_semantics import (  # noqa: E402
    verify_source_semantic_stack,
)


def write_json_atomic(path: Path, payload: object) -> None:
    destination = path.expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(
        f".{destination.name}.{os.getpid()}.tmp"
    )
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, destination)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    result = verify_source_semantic_stack(args.workspace)
    if args.output is not None:
        write_json_atomic(args.output, result)
    print(
        json.dumps(
            {
                "schema_version": result["schema_version"],
                "status": "ready",
                "record": (
                    None
                    if args.output is None
                    else str(args.output.expanduser().resolve())
                ),
                "cuda_device_name": result["python_runtime"][
                    "cuda_device_name"
                ],
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
