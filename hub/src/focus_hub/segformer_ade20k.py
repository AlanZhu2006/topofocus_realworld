"""Real-camera pixel semantics from the validated SegFormer ADE20K model.

The upstream Focus_realworld executable uses MP3D-40 RedNet labels for its
semantic BEV.  That model is retained as the default backend.  Real WSJ RGB
observations expose a repeatable domain gap, however: a visible chair produces
no production-thresholded RedNet chair pixels.  This deployment adapter uses
the same SegFormer-B0/ADE20K family and 0.35 confidence gate previously
validated by TinyNav's isolated real-camera semantic-mapping package, then
collapses its source labels into the MP3D IDs already consumed by
``CentralMapper``.

This is model inference without real-world pixel ground truth.  It is not
reported as an upstream/source-identical RedNet result and it does not change
the YOLOv10 evidence supplied to the Perception VLM.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Mapping

import cv2
import numpy as np


MODEL_REPOSITORY = "nvidia/segformer-b0-finetuned-ade-512-512"
MODEL_REVISION = "489d5cd81a0b59fab9b7ea758d3548ebe99677da"

# ADE20K names are resolved from the checkpoint's own id2label table.  Values
# are MP3D-40 IDs used by source/Focus_realworld/constants.py.  Only direct
# category equivalents are mapped; floor/wall/door and broad "other furniture"
# labels deliberately remain unknown because the HPC BEV exposes object-goal
# channels, not a general ADE20K scene parser.
ADE20K_NAME_TO_MP3D_ID: Mapping[str, int] = {
    "chair": 4,
    "armchair": 4,
    "seat": 4,
    "bench": 4,
    "swivel chair": 4,
    "ottoman": 4,
    "stool": 4,
    "sofa": 11,
    "plant": 15,
    "flower": 15,
    "pot": 15,
    "bed": 12,
    "toilet": 19,
    "television receiver": 23,
    "crt screen": 23,
    "bathtub": 26,
    "shower": 24,
    "fireplace": 28,
    "refrigerator": 38,
    "stove": 38,
    "oven": 38,
    "microwave": 38,
    "dishwasher": 38,
    "towel": 21,
    "chest of drawers": 14,
    "table": 6,
    "desk": 6,
    "pool table": 6,
    "coffee table": 6,
}

EXPECTED_MODEL_FILES: Mapping[str, tuple[int, str]] = {
    "config.json": (
        6884,
        "209caa9091e4632f7c8883c11170cd08ad29af68b23c09590aa4a5befb1a2a7f",
    ),
    "preprocessor_config.json": (
        271,
        "8039d1d210abaa7117ad78e58cdfd6141a2ec72c03dae891b3cd76737e422c6c",
    ),
    "model.safetensors": (
        15036944,
        "6ae39addd01de6b1b8bde2cf677d43a5cd733424b8d186de3f95d1c51fee23f9",
    ),
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_model_directory(model_dir: Path | str) -> list[dict[str, object]]:
    """Fail closed unless every pinned model file has the observed identity."""

    root = Path(model_dir).expanduser().resolve()
    provenance: list[dict[str, object]] = []
    for name, (expected_size, expected_sha256) in EXPECTED_MODEL_FILES.items():
        path = root / name
        if not path.is_file():
            raise FileNotFoundError(f"missing pinned SegFormer file: {path}")
        size = path.stat().st_size
        checksum = _sha256(path)
        if size != expected_size or checksum != expected_sha256:
            raise ValueError(
                "SegFormer artifact identity mismatch: "
                f"path={path}, size={size}/{expected_size}, "
                f"sha256={checksum}/{expected_sha256}"
            )
        provenance.append(
            {
                "path": str(path),
                "size_bytes": size,
                "sha256": checksum,
                "status": "observed_and_checksum_verified",
            }
        )
    return provenance


def build_source_to_mp3d_lookup(
    id2label: Mapping[int | str, str],
    *,
    allowed_categories: tuple[str, ...] | None = None,
) -> np.ndarray:
    """Resolve the checkpoint label table into a dense MP3D-ID lookup."""

    labels = {int(class_id): str(name).strip().casefold() for class_id, name in id2label.items()}
    if set(labels) != set(range(len(labels))):
        raise ValueError("SegFormer id2label IDs must be contiguous from zero")
    allowed_mp3d_ids = None
    if allowed_categories is not None:
        category_to_id = {
            "chair": 4,
            "sofa": 11,
            "plant": 15,
            "bed": 12,
            "toilet": 19,
            "tv": 23,
            "bathtub": 26,
            "shower": 24,
            "fireplace": 28,
            "appliances": 38,
            "towel": 21,
            "chest_of_drawers": 14,
            "table": 6,
        }
        unknown = sorted(set(allowed_categories).difference(category_to_id))
        if unknown:
            raise ValueError(f"unsupported SegFormer map categories: {unknown}")
        allowed_mp3d_ids = {category_to_id[name] for name in allowed_categories}

    # MP3D class 1 is the source RedNet wrapper's unknown/background fallback.
    lookup = np.ones(len(labels), dtype=np.int16)
    for source_id, source_name in labels.items():
        mp3d_id = ADE20K_NAME_TO_MP3D_ID.get(source_name)
        if mp3d_id is not None and (
            allowed_mp3d_ids is None or mp3d_id in allowed_mp3d_ids
        ):
            lookup[source_id] = mp3d_id
    return lookup


def collapse_ade20k_prediction(
    source_labels: np.ndarray,
    confidence: np.ndarray,
    source_to_mp3d: np.ndarray,
    output_shape: tuple[int, int],
    *,
    min_confidence: float,
) -> np.ndarray:
    """Map low-resolution ADE20K argmax labels to full-size MP3D IDs.

    This intentionally follows the already-validated TinyNav postprocessing:
    threshold at the native 128x128 logit resolution, then restore the camera
    resolution with nearest-neighbour sampling.  Bilinear interpolation of
    categorical IDs would invent boundary labels.
    """

    labels = np.asarray(source_labels)
    scores = np.asarray(confidence)
    lookup = np.asarray(source_to_mp3d)
    if labels.ndim != 2 or not np.issubdtype(labels.dtype, np.integer):
        raise ValueError("source_labels must be a 2-D integer array")
    if scores.shape != labels.shape or not np.issubdtype(scores.dtype, np.floating):
        raise ValueError("confidence must be a floating array matching source_labels")
    if lookup.ndim != 1 or not np.issubdtype(lookup.dtype, np.integer):
        raise ValueError("source_to_mp3d must be a one-dimensional integer lookup")
    if labels.size and (int(labels.min()) < 0 or int(labels.max()) >= lookup.size):
        raise ValueError("source_labels contains an out-of-range class ID")
    if not 0.0 <= min_confidence <= 1.0:
        raise ValueError("min_confidence must be in [0, 1]")
    height, width = (int(value) for value in output_shape)
    if height <= 0 or width <= 0:
        raise ValueError("output_shape dimensions must be positive")

    mapped = lookup[labels]
    mapped = np.where(scores >= min_confidence, mapped, 1).astype(np.int16)
    if mapped.shape != (height, width):
        mapped = cv2.resize(
            mapped,
            (width, height),
            interpolation=cv2.INTER_NEAREST,
        )
    return np.ascontiguousarray(mapped, dtype=np.int16)


class SegformerAde20kSegmenter:
    """Pinned local Hugging Face SegFormer model with MP3D-compatible output."""

    def __init__(
        self,
        model_dir: Path | str,
        *,
        device: str = "cuda",
        min_confidence: float = 0.35,
        allowed_categories: tuple[str, ...] | None = None,
    ) -> None:
        if not 0.0 <= min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        from transformers import SegformerForSemanticSegmentation, SegformerImageProcessor
        import torch

        self.model_dir = Path(model_dir).expanduser().resolve()
        artifacts = verify_model_directory(self.model_dir)
        self.device = torch.device(device)
        self.min_confidence = float(min_confidence)
        self.allowed_categories = allowed_categories
        self._torch = torch
        self.processor = SegformerImageProcessor.from_pretrained(
            self.model_dir,
            local_files_only=True,
        )
        self.model = SegformerForSemanticSegmentation.from_pretrained(
            self.model_dir,
            local_files_only=True,
        ).to(self.device)
        self.model.eval()
        self.source_to_mp3d = build_source_to_mp3d_lookup(
            self.model.config.id2label,
            allowed_categories=allowed_categories,
        )
        self.provenance = {
            "backend": "segformer_b0_ade20k_to_mp3d40",
            "status": "deployment_adapter_model_inference_unverified",
            "source_repository": MODEL_REPOSITORY,
            "source_revision": MODEL_REVISION,
            "min_confidence": self.min_confidence,
            "allowed_map_categories": (
                None if allowed_categories is None else list(allowed_categories)
            ),
            "postprocessing": (
                "native_logit_argmax_then_confidence_gate_then_nearest_resize"
            ),
            "artifacts": artifacts,
        }

    def segment(self, rgb_bgr: np.ndarray, depth_m: np.ndarray) -> np.ndarray:
        del depth_m  # Geometry remains authoritative in CentralMapper.
        bgr = np.asarray(rgb_bgr)
        if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            raise ValueError("rgb_bgr must be HxWx3 uint8")
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        inputs = self.processor(images=rgb, return_tensors="pt").to(self.device)
        with self._torch.inference_mode():
            logits = self.model(**inputs).logits
            probabilities = logits.softmax(dim=1)
            confidence, source_labels = probabilities.max(dim=1)
        return collapse_ade20k_prediction(
            source_labels[0].cpu().numpy(),
            confidence[0].cpu().numpy(),
            self.source_to_mp3d,
            bgr.shape[:2],
            min_confidence=self.min_confidence,
        )
