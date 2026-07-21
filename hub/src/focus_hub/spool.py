from __future__ import annotations

import json
import os
import shutil
import tempfile
from pathlib import Path

from .models import ObservationMetadata


class SpoolError(RuntimeError):
    pass


class ObservationSpool:
    """Append-only replay spool. It never deletes old robot data automatically."""

    def __init__(self, root: Path, *, min_free_bytes: int = 20 * 1024**3) -> None:
        self.root = root
        self.min_free_bytes = min_free_bytes

    def write(self, metadata: ObservationMetadata, rgb: bytes, depth: bytes) -> Path:
        self.root.mkdir(parents=True, exist_ok=True)
        free = shutil.disk_usage(self.root).free
        required = len(rgb) + len(depth) + self.min_free_bytes
        if free < required:
            raise SpoolError(
                f"spool has {free} free bytes, needs at least {required}; refusing instead of evicting data"
            )

        robot_root = self.root / metadata.robot_id
        robot_root.mkdir(parents=True, exist_ok=True)
        final = robot_root / f"{metadata.sequence:020d}"
        if final.exists():
            return final

        temp = Path(tempfile.mkdtemp(prefix=f".{metadata.sequence:020d}-", dir=robot_root))
        try:
            rgb_suffix = ".jpg" if metadata.rgb_encoding == "jpeg" else ".png"
            (temp / f"rgb{rgb_suffix}").write_bytes(rgb)
            (temp / "depth.png").write_bytes(depth)
            payload = metadata.model_dump(mode="json")
            (temp / "metadata.json").write_text(
                json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            for path in temp.iterdir():
                with path.open("rb") as handle:
                    os.fsync(handle.fileno())
            os.replace(temp, final)
            return final
        except Exception:
            shutil.rmtree(temp, ignore_errors=True)
            raise

