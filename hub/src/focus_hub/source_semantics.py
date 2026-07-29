"""Exact executable-source RedNet + Detectron2 semantic pixel pipeline.

The active HM3D path in ``source/Focus_realworld/agents/vlm_agents.py`` does
not choose between RedNet and Detectron2.  It first converts RedNet's MP3D-40
labels to fifteen HM3D channels, then applies six class-specific Mask R-CNN
operations:

* chair, sofa and bed pixels unsupported by Mask R-CNN are cleared;
* plant, toilet and TV Mask R-CNN pixels are added.

This module reproduces that order and its exact ``== 0``/``== 1`` mask-count
tests.  It deliberately returns a multi-hot ``H x W x 15`` tensor: collapsing
to one class ID would lose overlapping source channels before BEV projection.
"""

from __future__ import annotations

import hashlib
import importlib.metadata
from pathlib import Path
from typing import Mapping

import numpy as np

from .central_mapping import (
    HM3D_CATEGORY_NAMES,
    MP_CATEGORIES_MAPPING,
    RedNetSegmenter,
)

SOURCE_MASKRCNN_CONFIDENCE = 0.9
SOURCE_DETECTRON2_COMMIT = "b4a4a3bd136852dae5fb1de37978dee412653e31"
SOURCE_MASKRCNN_WEIGHT_URL = (
    "https://dl.fbaipublicfiles.com/detectron2/"
    "COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/"
    "model_final_f10217.pkl"
)
SOURCE_MASKRCNN_WEIGHT_SIZE = 177_841_981
SOURCE_MASKRCNN_WEIGHT_SHA256 = (
    "9a737e290372f1f70994ebcbd89d8004dbb3ae30a605fd915a190fa4a782dd66"
)
SOURCE_REDNET_WEIGHT_SIZE = 656_550_984
SOURCE_REDNET_WEIGHT_SHA256 = (
    "f94d1c62a73bc05690ae29200d3dbd033ff243e7ce91755d1cd928bde844f995"
)
SOURCE_MASKRCNN_CONFIG_SIZE = 192
SOURCE_MASKRCNN_CONFIG_SHA256 = (
    "e51a639dea109a372822f1b38a603fdeab7f470119d9dcb8d8ad53bf533a46ba"
)
SOURCE_MASKRCNN_BASE_CONFIG_SIZE = 1_318
SOURCE_MASKRCNN_BASE_CONFIG_SHA256 = (
    "95306a7335903880e35b8c919a7daad1d8ce4b2c926a69506be1c7116a0799d7"
)
DETECTRON2_RUNTIME_SENTINELS: Mapping[str, tuple[int, str]] = {
    "__init__.py": (
        258,
        "13ad28f1c53e186f8c26bab8e3afeb26834659316e2260c4f64e8a539b35091a",
    ),
    "modeling/meta_arch/rcnn.py": (
        13_896,
        "9b9b8b178292bb5e5953901ae0a6101a3e52e7b96c9ced9584beb113d2a5460c",
    ),
    "modeling/roi_heads/mask_head.py": (
        12_185,
        "4370a06a7fef79ba00780f7e5d0f9854cccf70b26bf8168e36a21be09ac1cf2d",
    ),
    "modeling/postprocessing.py": (
        4_045,
        "364549a6a53c84366f9047a0e1b8dc77758d04cf543806264b0231c8813ce09a",
    ),
    "config/defaults.py": (
        30_045,
        "3f0dc374811b83ab81c4d5bf0b8d7d6afb86c683a8a67aa9d46ea5a39cacd7f5",
    ),
}

# Detectron2 COCO contiguous class IDs -> source semantic_input channel.
# This is copied from the immutable ``constants.py::coco_categories_mapping``.
COCO_CLASS_TO_HM3D_CHANNEL: Mapping[int, int] = {
    56: 0,  # chair
    57: 1,  # couch / source HM3D sofa
    58: 2,  # potted plant
    59: 3,  # bed
    61: 4,  # toilet
    62: 5,  # tv
    60: 6,  # dining table
    69: 7,  # oven
    71: 8,  # sink
    72: 9,  # refrigerator
    73: 10,  # book
    74: 11,  # clock
    75: 12,  # vase
    41: 13,  # cup
    39: 14,  # bottle
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_artifact(
    path: Path | str,
    *,
    expected_size: int,
    expected_sha256: str,
    label: str,
) -> dict[str, object]:
    """Fail closed unless a model artifact has the reviewed identity."""

    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"missing {label}: {resolved}")
    observed_size = resolved.stat().st_size
    observed_sha256 = sha256_file(resolved)
    if observed_size != expected_size or observed_sha256 != expected_sha256:
        raise ValueError(
            f"{label} identity mismatch: path={resolved}, "
            f"size={observed_size}/{expected_size}, "
            f"sha256={observed_sha256}/{expected_sha256}"
        )
    return {
        "path": str(resolved),
        "size_bytes": observed_size,
        "sha256": observed_sha256,
        "status": "observed_and_checksum_verified",
    }


def verify_detectron2_runtime() -> dict[str, object]:
    """Bind the installed Python runtime to sentinel files from the pin."""

    import detectron2
    import detectron2._C

    version = importlib.metadata.version("detectron2")
    if version != "0.6":
        raise RuntimeError(
            f"source semantic backend requires Detectron2 0.6, got {version}"
        )
    package_root = Path(detectron2.__file__).resolve().parent
    artifacts = []
    for relative_path, (size, digest) in DETECTRON2_RUNTIME_SENTINELS.items():
        artifacts.append(
            verify_artifact(
                package_root / relative_path,
                expected_size=size,
                expected_sha256=digest,
                label=f"pinned Detectron2 runtime file {relative_path}",
            )
        )
    extension_path = Path(detectron2._C.__file__).resolve()
    return {
        "version": version,
        "pinned_upstream_commit": SOURCE_DETECTRON2_COMMIT,
        "status": "locally_built_from_pinned_upstream_commit",
        "python_sentinels": artifacts,
        "cuda_extension": {
            "path": str(extension_path),
            "size_bytes": extension_path.stat().st_size,
            "sha256": sha256_file(extension_path),
            "status": "observed_local_compatibility_build",
        },
    }


def rednet_labels_to_hm3d_channels(rednet_labels: np.ndarray) -> np.ndarray:
    """Apply the source MP3D-40 -> HM3D-15 one-hot conversion."""

    labels = np.asarray(rednet_labels)
    if labels.ndim != 2 or not np.issubdtype(labels.dtype, np.number):
        raise ValueError("RedNet labels must be a two-dimensional numeric array")
    channels = np.zeros(
        (*labels.shape, len(HM3D_CATEGORY_NAMES)),
        dtype=np.float32,
    )
    for channel, rednet_id in enumerate(MP_CATEGORIES_MAPPING):
        channels[:, :, channel][labels == rednet_id] = 1.0
    return channels


def accumulate_maskrcnn_channels(
    pred_classes: np.ndarray,
    pred_masks: np.ndarray,
    image_shape: tuple[int, int],
) -> np.ndarray:
    """Reproduce ``SemanticPredMaskRCNN.get_prediction`` accumulation."""

    classes = np.asarray(pred_classes)
    masks = np.asarray(pred_masks)
    height, width = (int(value) for value in image_shape)
    if height <= 0 or width <= 0:
        raise ValueError("image_shape dimensions must be positive")
    if classes.ndim != 1 or not np.issubdtype(classes.dtype, np.integer):
        raise ValueError("pred_classes must be a one-dimensional integer array")
    if masks.shape != (classes.size, height, width):
        raise ValueError(
            "pred_masks must have shape (instances, image height, image width)"
        )
    if not (
        np.issubdtype(masks.dtype, np.bool_) or np.issubdtype(masks.dtype, np.number)
    ):
        raise ValueError("pred_masks must be boolean or numeric")

    semantic_input = np.zeros(
        (height, width, len(HM3D_CATEGORY_NAMES)),
        dtype=np.float32,
    )
    for class_id, mask in zip(classes, masks, strict=True):
        channel = COCO_CLASS_TO_HM3D_CHANNEL.get(int(class_id))
        if channel is not None:
            semantic_input[:, :, channel] += mask.astype(
                np.float32,
                copy=False,
            )
    return semantic_input


def apply_source_maskrcnn_override(
    rednet_hm3d: np.ndarray,
    maskrcnn_hm3d: np.ndarray,
) -> np.ndarray:
    """Apply the six override statements in source order, without cleanup."""

    rednet = np.asarray(rednet_hm3d)
    maskrcnn = np.asarray(maskrcnn_hm3d)
    expected_channels = len(HM3D_CATEGORY_NAMES)
    if (
        rednet.ndim != 3
        or rednet.shape[2] != expected_channels
        or maskrcnn.shape != rednet.shape
    ):
        raise ValueError("RedNet and Mask R-CNN tensors must match H x W x HM3D-15")
    if not np.all(np.isfinite(rednet)) or not np.all(np.isfinite(maskrcnn)):
        raise ValueError("semantic tensors must contain only finite values")

    result = rednet.astype(np.float32, copy=True)
    result[:, :, 0][maskrcnn[:, :, 0] == 0] = 0
    result[:, :, 1][maskrcnn[:, :, 1] == 0] = 0
    result[:, :, 2][maskrcnn[:, :, 2] == 1] = 1
    result[:, :, 3][maskrcnn[:, :, 3] == 0] = 0
    result[:, :, 4][maskrcnn[:, :, 4] == 1] = 1
    result[:, :, 5][maskrcnn[:, :, 5] == 1] = 1
    return result


class SourceMaskRCNNPredictor:
    """Source BatchPredictor contract with a local, checksum-pinned weight."""

    def __init__(
        self,
        weight_path: Path | str,
        config_path: Path | str,
        *,
        device: str = "cuda:0",
    ) -> None:
        artifact = verify_artifact(
            weight_path,
            expected_size=SOURCE_MASKRCNN_WEIGHT_SIZE,
            expected_sha256=SOURCE_MASKRCNN_WEIGHT_SHA256,
            label="source Detectron2 Mask R-CNN weight",
        )
        resolved_config = Path(config_path).expanduser().resolve()
        config_artifact = verify_artifact(
            resolved_config,
            expected_size=SOURCE_MASKRCNN_CONFIG_SIZE,
            expected_sha256=SOURCE_MASKRCNN_CONFIG_SHA256,
            label="source Detectron2 Mask R-CNN config",
        )
        base_config = resolved_config.parent.parent / "Base-RCNN-FPN.yaml"
        base_config_artifact = verify_artifact(
            base_config,
            expected_size=SOURCE_MASKRCNN_BASE_CONFIG_SIZE,
            expected_sha256=SOURCE_MASKRCNN_BASE_CONFIG_SHA256,
            label="source Detectron2 Mask R-CNN base config",
        )

        import torch
        from detectron2.checkpoint import DetectionCheckpointer
        from detectron2.config import get_cfg
        from detectron2.modeling import build_model

        runtime_provenance = verify_detectron2_runtime()
        cfg = get_cfg()
        cfg.merge_from_file(str(resolved_config))
        cfg.merge_from_list(
            [
                "MODEL.WEIGHTS",
                str(Path(weight_path).expanduser().resolve()),
                "MODEL.DEVICE",
                device,
            ]
        )
        cfg.MODEL.RETINANET.SCORE_THRESH_TEST = SOURCE_MASKRCNN_CONFIDENCE
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = SOURCE_MASKRCNN_CONFIDENCE
        cfg.MODEL.PANOPTIC_FPN.COMBINE.INSTANCES_CONFIDENCE_THRESH = (
            SOURCE_MASKRCNN_CONFIDENCE
        )
        cfg.freeze()

        self._torch = torch
        self.model = build_model(cfg)
        self.model.eval()
        DetectionCheckpointer(self.model).load(cfg.MODEL.WEIGHTS)
        self.input_format = cfg.INPUT.FORMAT
        if self.input_format not in {"RGB", "BGR"}:
            raise ValueError(
                f"unsupported Detectron2 input format: {self.input_format}"
            )
        self.provenance = {
            "model": "Mask R-CNN R50-FPN 3x COCO",
            "weight": artifact,
            "weight_source_url": SOURCE_MASKRCNN_WEIGHT_URL,
            "confidence_threshold": SOURCE_MASKRCNN_CONFIDENCE,
            "config": config_artifact,
            "base_config": base_config_artifact,
            "detectron2": runtime_provenance,
            "preprocessing": ("source_batch_predictor_original_resolution_no_resize"),
            "input_format": self.input_format,
        }

    def predict_channels(self, rgb_bgr: np.ndarray) -> np.ndarray:
        """Return the source Mask R-CNN accumulator at camera resolution."""

        image = np.asarray(rgb_bgr)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("rgb_bgr must have shape H x W x 3")
        if image.dtype != np.uint8:
            raise ValueError("rgb_bgr must be uint8")
        original = image
        if self.input_format == "RGB":
            original = original[:, :, ::-1]
        height, width = original.shape[:2]
        tensor = self._torch.as_tensor(
            np.ascontiguousarray(original.astype("float32").transpose(2, 0, 1))
        )
        with self._torch.no_grad():
            prediction = self.model(
                [{"image": tensor, "height": height, "width": width}]
            )[0]
        instances = prediction["instances"]
        return accumulate_maskrcnn_channels(
            instances.pred_classes.detach().cpu().numpy(),
            instances.pred_masks.detach().cpu().numpy(),
            (height, width),
        )


class SourceRedNetDetectron2Segmenter:
    """Complete source pixel backend, retaining all fifteen HM3D channels."""

    def __init__(
        self,
        rednet_checkpoint: Path | str,
        maskrcnn_weight: Path | str,
        maskrcnn_config: Path | str,
        *,
        device: str = "cuda:0",
    ) -> None:
        rednet_artifact = verify_artifact(
            rednet_checkpoint,
            expected_size=SOURCE_REDNET_WEIGHT_SIZE,
            expected_sha256=SOURCE_REDNET_WEIGHT_SHA256,
            label="source RedNet weight",
        )
        self.rednet = RedNetSegmenter(rednet_checkpoint, device=device)
        self.maskrcnn = SourceMaskRCNNPredictor(
            maskrcnn_weight,
            maskrcnn_config,
            device=device,
        )
        self.provenance = {
            "backend": "source_rednet_detectron2_hm3d15",
            "status": "source_exact_model_inference_unverified",
            "method": (
                "mp3d40_rednet_rgbd_then_detectron2_maskrcnn_six_class_override"
            ),
            "source_maskrcnn_override_applied": True,
            "source_compatibility": "executable_source_pixel_pipeline",
            "hm3d_category_order": list(HM3D_CATEGORY_NAMES),
            "override_order": [
                "chair_clear",
                "sofa_clear",
                "plant_add",
                "bed_clear",
                "toilet_add",
                "tv_add",
            ],
            "rednet_weight": rednet_artifact,
            "maskrcnn": self.maskrcnn.provenance,
        }

    def segment(self, rgb_bgr: np.ndarray, depth_m: np.ndarray) -> np.ndarray:
        rednet_labels = self.rednet.segment(rgb_bgr, depth_m)
        rednet_hm3d = rednet_labels_to_hm3d_channels(rednet_labels)
        maskrcnn_hm3d = self.maskrcnn.predict_channels(rgb_bgr)
        return apply_source_maskrcnn_override(rednet_hm3d, maskrcnn_hm3d)
