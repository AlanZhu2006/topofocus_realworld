#!/usr/bin/env python3
"""Extract a TinyNav map_record directory into a portable immutable replay sample.

TinyNav stores per-keyframe depth (and features/embeddings) in Python ``shelve``
databases whose on-disk format is Berkeley DB Hash.  The hub's conda-based
environment cannot open that format, so this tool must run with the *system*
interpreter (``/usr/bin/python3.10`` on Ubuntu links ``dbm.ndbm`` against
libdb-5.3).  It deliberately uses only the standard library: values are copied
as raw pickle bytes without unpickling, so numpy is not required here and the
bytes stay byte-identical to what the robot wrote.

Outputs, under --output:
  depths_pkl/<timestamp_ns>.pkl   raw pickled depth value per keyframe
  manifest.json                   SHA-256 of every source file and every
                                  extracted file, plus counts and key lists

The source record directory is opened read-only and never modified.  The tool
refuses to overwrite an existing output directory.
"""
from __future__ import annotations

import argparse
import dbm.ndbm
import hashlib
import json
import os
import sys
from pathlib import Path


def sha256_file(path: Path, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        while True:
            block = f.read(chunk)
            if not block:
                break
            h.update(block)
    return h.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True,
                        help="TinyNav map_record directory (read-only source)")
    parser.add_argument("--output", type=Path, required=True,
                        help="Output directory for the portable sample (must not exist)")
    args = parser.parse_args()

    record = args.record.resolve()
    output = args.output.resolve()
    if not record.is_dir():
        print(f"record directory not found: {record}", file=sys.stderr)
        return 2
    if output.exists():
        print(f"refusing to overwrite existing output: {output}", file=sys.stderr)
        return 2

    source_files = [
        "poses.npy",
        "intrinsics.npy",
        "rgb_camera_intrinsics.npy",
        "T_rgb_to_infra1.npy",
        "baseline.npy",
        "tf_messages.npy",
        "rgb_images_db/video.mp4",
        "rgb_images_db/meta.json",
        "depths.db",
    ]
    missing = [f for f in source_files if not (record / f).is_file()]
    if missing:
        print(f"record is missing required files: {missing}", file=sys.stderr)
        return 2

    manifest = {
        "tool": "extract_tinynav_record.py",
        "record_dir": str(record),
        "source_sha256": {},
        "depth_keys": [],
        "extracted_sha256": {},
    }
    for rel in source_files:
        manifest["source_sha256"][rel] = {
            "sha256": sha256_file(record / rel),
            "bytes": (record / rel).stat().st_size,
        }

    depths_out = output / "depths_pkl"
    depths_out.mkdir(parents=True, exist_ok=False)

    # dbm.ndbm.open appends the .db suffix itself.
    db = dbm.ndbm.open(str(record / "depths"), "r")
    try:
        keys = sorted(int(k.decode("ascii")) for k in db.keys())
        for ts in keys:
            raw = db[str(ts).encode("ascii")]
            dest = depths_out / f"{ts}.pkl"
            with dest.open("wb") as f:
                f.write(raw)
            manifest["extracted_sha256"][f"depths_pkl/{ts}.pkl"] = {
                "sha256": hashlib.sha256(raw).hexdigest(),
                "bytes": len(raw),
            }
        manifest["depth_keys"] = keys
    finally:
        db.close()

    with (output / "manifest.json").open("w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)

    # Extracted sample is evidence: drop write permission on the payload files.
    for p in depths_out.iterdir():
        os.chmod(p, 0o444)

    print(f"extracted {len(manifest['depth_keys'])} depth keyframes to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
