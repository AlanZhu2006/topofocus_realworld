import json
from pathlib import Path

import numpy as np
import yaml

from semantic_mapping.segformer_tensorrt_backend import SegformerTensorRtBackend
from semantic_mapping.semantic_schema import SemanticClassSchema


CLASSES_PATH = Path(__file__).parents[1] / "config" / "semantic_classes.yaml"


class FakeRunner:
    def __init__(self) -> None:
        self.input_shapes = {"pixel_values": (1, 3, 1, 1)}
        self.output_shapes = {"logits": (1, 4, 1, 1)}
        self.closed = False

    def infer(self, inputs: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        assert inputs["pixel_values"].shape == (1, 3, 1, 1)
        logits = np.array([[[[-3.0]], [[4.0]], [[-3.0]], [[-3.0]]]])
        return {"logits": logits.astype(np.float32)}

    def close(self) -> None:
        self.closed = True


def test_segformer_backend_produces_navigation_semantic_frame(tmp_path: Path) -> None:
    model_config = tmp_path / "config.json"
    model_config.write_text(
        json.dumps(
            {"id2label": {"0": "wall", "1": "floor", "2": "person", "3": "sky"}}
        ),
        encoding="utf-8",
    )
    preprocessor = tmp_path / "preprocessor.json"
    preprocessor.write_text(
        json.dumps(
            {
                "size": {"height": 1, "width": 1},
                "image_mean": [0.0, 0.0, 0.0],
                "image_std": [1.0, 1.0, 1.0],
                "rescale_factor": 1.0 / 255.0,
            }
        ),
        encoding="utf-8",
    )
    mapping = tmp_path / "mapping.yaml"
    mapping.write_text(
        yaml.safe_dump(
            {
                "version": 1,
                "source_dataset": "test",
                "semantic_classes_version": 1,
                "mappings": {
                    "wall": ["wall"],
                    "floor": ["floor"],
                    "dynamic_object": ["person"],
                },
            }
        ),
        encoding="utf-8",
    )
    schema = SemanticClassSchema.from_yaml(CLASSES_PATH)
    runner = FakeRunner()
    backend = SegformerTensorRtBackend(
        "unused.engine",
        model_config,
        preprocessor,
        mapping,
        schema,
        runner=runner,
    )

    frame = backend.infer(np.zeros((2, 2, 3), dtype=np.uint8), 123)

    np.testing.assert_array_equal(frame.label_image, np.ones((2, 2), np.uint8))
    assert np.all(frame.confidence_image > 0.99)
    assert frame.timestamp_ns == 123
    backend.close()
    assert runner.closed
