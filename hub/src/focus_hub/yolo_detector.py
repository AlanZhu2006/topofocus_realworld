"""Real YOLOv10 object detection for the Perception VLM stage, ported from
`main.py`'s `yolo = YOLO(args.yolo_weights); yolo_output = yolo(source=rgb, conf=0.2)`.

Confirmed against `running_inference.md`/`run_cmd.txt` on the real HPC
source (`ssh alantorch:/scratch/jl9356/Focus_realworld`): real baseline
experiments run with `--yolo yolov10` (the default), not a placeholder —
the Perception VLM's "Scene Object (Object Detection)" input is genuinely
real YOLO output in the actual upstream pipeline. The weights
(`artifacts/vision/yolov10m.pt`) and the `ultralytics` package were already
present in this workspace (from the original G0 artifact transfer), so this
is a real port, not a stand-in.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class YoloDetection:
    """One image-space detection retained for optional depth projection."""

    class_name: str
    confidence: float
    xyxy: tuple[float, float, float, float]


class YoloDetector:
    """Thin wrapper matching upstream's exact call shape: `model(source=rgb,
    conf=0.2)` -> {class_name: confidence} for the Perception VLM prompt.
    """

    def __init__(self, weights_path: Path | str, *, conf: float = 0.2) -> None:
        from ultralytics import YOLO

        resolved = Path(weights_path).expanduser().resolve()
        if not resolved.is_file():
            raise FileNotFoundError(f"YOLO weights not found: {resolved}")
        digest = hashlib.sha256()
        with resolved.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        self.weights_path = resolved
        self.weights_size_bytes = resolved.stat().st_size
        self.weights_sha256 = digest.hexdigest()
        self.model = YOLO(str(resolved))
        self.conf = conf

    @property
    def provenance(self) -> dict[str, str | int]:
        """Immutable model provenance for runtime map snapshots."""

        return {
            "source_path": str(self.weights_path),
            "size_bytes": self.weights_size_bytes,
            "sha256": self.weights_sha256,
            "status": "source_artifact_model_inference_unverified",
        }

    def detect_boxes(self, rgb_bgr: np.ndarray) -> list[YoloDetection]:
        """Return class, confidence and image-space box for every detection."""

        results = self.model(source=rgb_bgr, conf=self.conf, verbose=False)
        names = results[0].names
        boxes = results[0].boxes
        return [
            YoloDetection(
                class_name=str(names[int(class_id)]),
                confidence=float(confidence),
                xyxy=tuple(float(value) for value in xyxy),
            )
            for xyxy, class_id, confidence in zip(
                boxes.xyxy.tolist(), boxes.cls.tolist(), boxes.conf.tolist()
            )
        ]

    def detect(self, rgb_bgr: np.ndarray) -> dict[str, float]:
        """Returns {class_name: confidence}, ported from the
        `yolo_mapping = [names[int(c)] for c in cls]` / `zip(yolo_mapping,
        confs)` pattern in main.py. Last detection wins if a class appears
        more than once, matching upstream's own dict-comprehension
        behavior (`{k: v for k, v in zip(...)}`).
        """
        return {
            detection.class_name: detection.confidence
            for detection in self.detect_boxes(rgb_bgr)
        }
