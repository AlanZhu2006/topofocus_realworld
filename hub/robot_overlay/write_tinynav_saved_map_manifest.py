#!/usr/bin/env python3
"""Hash a finalized TinyNav map into an atomic provenance manifest."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys


OVERLAY = Path(__file__).resolve().parent
HUB_SRC = OVERLAY.parent / "src"
if HUB_SRC.is_dir():
    sys.path.insert(0, str(HUB_SRC))

from focus_hub.tinynav_map_contract import (  # noqa: E402
    build_saved_map_manifest,
    validate_saved_map_manifest,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-directory", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--source-file",
        type=Path,
        action="append",
        default=[],
        help="runtime source file to include with size and SHA-256 provenance",
    )
    args = parser.parse_args()
    directory = args.map_directory.expanduser().resolve()
    output = (
        directory / "focus_saved_map_manifest.json"
        if args.output is None
        else args.output.expanduser().resolve()
    )
    if output.parent != directory:
        parser.error("--output must be directly inside --map-directory")
    if output.exists():
        parser.error(f"refusing to overwrite existing manifest: {output}")
    try:
        manifest = build_saved_map_manifest(
            directory,
            source_files=args.source_file,
        )
    except ValueError as exc:
        parser.error(str(exc))
    encoded = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode()
    temporary = output.with_name(f".{output.name}.tmp.{os.getpid()}")
    try:
        with temporary.open("xb") as handle:
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temporary, 0o444)
        os.replace(temporary, output)
    finally:
        if temporary.exists():
            temporary.unlink()
    try:
        validate_saved_map_manifest(
            output,
            map_directory=directory,
            verify_hashes=False,
        )
    except ValueError as exc:
        output.unlink(missing_ok=True)
        parser.error(f"written manifest failed validation: {exc}")
    print(
        json.dumps(
            {
                "manifest": str(output),
                "size_bytes": output.stat().st_size,
                "map_id": manifest["map_id"],
                "map_snapshot_sha256": manifest["map_snapshot_sha256"],
                "result_status": manifest["result_status"],
                "robot_commands_issued": False,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
