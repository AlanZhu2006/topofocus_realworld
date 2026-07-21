"""TensorRT SegFormer closed-set semantic perception backend."""

from __future__ import annotations

from pathlib import Path
from typing import Protocol

from numpy.typing import NDArray

from semantic_mapping.segformer_processing import (
    SegformerProcessorConfig,
    navigation_semantics_from_logits,
    prepare_segformer_input,
)
from semantic_mapping.semantic_backend import SemanticBackend, SemanticFrame
from semantic_mapping.semantic_label_mapping import (
    SemanticLabelMapping,
    load_huggingface_id2label,
)
from semantic_mapping.semantic_schema import SemanticClassSchema
from semantic_mapping.tensorrt_runner import TensorRtEngineRunner


class InferenceRunner(Protocol):
    """Minimal engine runner contract used for testability."""

    @property
    def input_shapes(self) -> dict[str, tuple[int, ...]]: ...

    @property
    def output_shapes(self) -> dict[str, tuple[int, ...]]: ...

    def infer(self, inputs: dict[str, NDArray]) -> dict[str, NDArray]: ...

    def close(self) -> None: ...


class SegformerTensorRtBackend(SemanticBackend):
    """Run SegFormer TensorRT and collapse ADE20K labels to navigation classes."""

    def __init__(
        self,
        engine_path: str | Path,
        model_config_path: str | Path,
        preprocessor_config_path: str | Path,
        label_mapping_path: str | Path,
        schema: SemanticClassSchema,
        *,
        min_confidence: float = 0.35,
        input_name: str = "pixel_values",
        output_name: str = "logits",
        runner: InferenceRunner | None = None,
    ) -> None:
        if not 0.0 <= min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        self.schema = schema
        self.processor = SegformerProcessorConfig.from_json(
            preprocessor_config_path
        )
        source_labels = load_huggingface_id2label(model_config_path)
        self.label_mapping = SemanticLabelMapping.from_yaml(
            label_mapping_path, schema, source_labels
        )
        self.min_confidence = float(min_confidence)
        self.input_name = input_name
        self.output_name = output_name
        expected_input_shape = (
            1,
            3,
            self.processor.input_height,
            self.processor.input_width,
        )
        self.runner = runner or TensorRtEngineRunner(
            engine_path, {input_name: expected_input_shape}
        )
        if self.runner.input_shapes.get(input_name) != expected_input_shape:
            raise ValueError("TensorRT input shape does not match preprocessor config")
        output_shape = self.runner.output_shapes.get(output_name)
        if output_shape is None or len(output_shape) != 4:
            raise ValueError(f"TensorRT output {output_name!r} is missing or invalid")
        if output_shape[0] != 1 or output_shape[1] != len(source_labels):
            raise ValueError("TensorRT output class count does not match model config")

    @property
    def class_names(self) -> dict[int, str]:
        return self.schema.class_names

    def infer(self, rgb_image: NDArray, timestamp_ns: int) -> SemanticFrame:
        model_input = prepare_segformer_input(rgb_image, self.processor)
        outputs = self.runner.infer({self.input_name: model_input})
        labels, confidence = navigation_semantics_from_logits(
            outputs[self.output_name],
            self.label_mapping.source_to_target,
            rgb_image.shape[:2],
            unknown_id=self.schema.unknown_id,
            min_confidence=self.min_confidence,
        )
        return SemanticFrame(
            label_image=labels,
            confidence_image=confidence,
            class_names=self.class_names,
            timestamp_ns=int(timestamp_ns),
            source_timestamp_ns=int(timestamp_ns),
        )

    def close(self) -> None:
        self.runner.close()
