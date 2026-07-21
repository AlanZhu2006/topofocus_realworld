from pathlib import Path

import numpy as np

from semantic_mapping.semantic_bev_projector import (
    SemanticBEVGrid,
    SemanticBEVProjectionConfig,
    project_semantic_to_bev,
)
from semantic_mapping.semantic_map_serializer import (
    load_semantic_voxel_map,
    save_semantic_voxel_map,
)
from semantic_mapping.semantic_schema import SemanticClass, SemanticClassSchema
from semantic_mapping.semantic_voxel_map import (
    SemanticVoxelConfig,
    SparseSemanticVoxelMap,
)


def test_semantic_map_save_load(tmp_path: Path) -> None:
    schema = SemanticClassSchema(
        1,
        (
            SemanticClass(0, "unknown", (0, 0, 0)),
            SemanticClass(1, "floor", (0, 255, 0)),
        ),
    )
    config = SemanticVoxelConfig(
        resolution_m=0.1,
        class_count=2,
        valid_class_ids=(0, 1),
        dynamic_class_ids=(),
        min_observations=1,
    )
    semantic_map = SparseSemanticVoxelMap(config)
    semantic_map.integrate_observations(
        np.asarray([[0.01, 0.01, 0.01]], dtype=np.float32),
        np.asarray([1], dtype=np.uint8),
        np.asarray([0.8], dtype=np.float32),
        123,
    )
    bev = project_semantic_to_bev(
        semantic_map,
        SemanticBEVProjectionConfig(resolution_m=0.1, padding_cells=0),
        grid=SemanticBEVGrid((0.0, 0.0), 0.1, 1, 1),
        floor_class_id=1,
    )
    save_semantic_voxel_map(
        tmp_path, semantic_map, schema, frame_id="map", timestamp_ns=123, bev=bev
    )
    restored, metadata = load_semantic_voxel_map(tmp_path)
    assert metadata["frame_id"] == "map"
    assert metadata["semantic_schema"]["version"] == 1
    assert metadata["bev"]["width"] == 1
    assert metadata["bev"]["ground_z"] == 0.0
    assert (tmp_path / "semantic_bev_tensor.npz").is_file()
    with np.load(tmp_path / "semantic_bev_tensor.npz", allow_pickle=False) as saved:
        assert saved["semantic_label"].tolist() == [[1]]
        assert float(saved["ground_z"]) == 0.0
    assert restored.label_and_confidence((0, 0, 0))[0] == 1
