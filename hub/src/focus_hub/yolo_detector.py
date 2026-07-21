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

from pathlib import Path

import numpy as np


class YoloDetector:
    """Thin wrapper matching upstream's exact call shape: `model(source=rgb,
    conf=0.2)` -> {class_name: confidence} for the Perception VLM prompt.
    """

    def __init__(self, weights_path: Path | str, *, conf: float = 0.2) -> None:
        from ultralytics import YOLO

        self.model = YOLO(str(weights_path))
        self.conf = conf

    def detect(self, rgb_bgr: np.ndarray) -> dict[str, float]:
        """Returns {class_name: confidence}, ported from the
        `yolo_mapping = [names[int(c)] for c in cls]` / `zip(yolo_mapping,
        confs)` pattern in main.py. Last detection wins if a class appears
        more than once, matching upstream's own dict-comprehension
        behavior (`{k: v for k, v in zip(...)}`).
        """
        results = self.model(source=rgb_bgr, conf=self.conf, verbose=False)
        names = results[0].names
        classes = results[0].boxes.cls
        confidences = results[0].boxes.conf
        return {
            names[int(c)]: float(conf)
            for c, conf in zip(classes.tolist(), confidences.tolist())
        }
