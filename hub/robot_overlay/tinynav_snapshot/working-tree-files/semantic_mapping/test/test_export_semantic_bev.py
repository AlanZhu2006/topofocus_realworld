from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

from semantic_mapping.semantic_bev_projector import (
    SemanticBEVGrid,
    SemanticBEVProjectionConfig,
    project_semantic_to_bev,
)
from semantic_mapping.semantic_map_serializer import save_semantic_voxel_map
from semantic_mapping.semantic_schema import SemanticClass, SemanticClassSchema
from semantic_mapping.semantic_voxel_map import SemanticVoxelConfig, SparseSemanticVoxelMap


ROOT_DIR = Path(__file__).resolve().parents[2]


def test_export_uses_final_occupancy_grid_and_ground_height(tmp_path: Path) -> None:
    semantic_directory = tmp_path / "semantic_mapping"
    schema = SemanticClassSchema(
        1,
        (
            SemanticClass(0, "unknown", (0, 0, 0)),
            SemanticClass(1, "floor", (0, 255, 0)),
        ),
    )
    semantic_map = SparseSemanticVoxelMap(
        SemanticVoxelConfig(
            resolution_m=0.1,
            class_count=2,
            valid_class_ids=(0, 1),
            dynamic_class_ids=(),
            min_observations=1,
        )
    )
    semantic_map.integrate_observations(
        np.asarray([[0.01, 0.01, -0.35]], dtype=np.float32),
        np.asarray([1], dtype=np.uint8),
        np.asarray([1.0], dtype=np.float32),
        123,
    )
    initial_bev = project_semantic_to_bev(
        semantic_map,
        SemanticBEVProjectionConfig(resolution_m=0.1, ground_z=-0.4),
        grid=SemanticBEVGrid((0.0, 0.0), 0.1, 1, 1),
        floor_class_id=1,
    )
    save_semantic_voxel_map(
        semantic_directory,
        semantic_map,
        schema,
        frame_id="map",
        timestamp_ns=123,
        bev=initial_bev,
    )
    occupancy_metadata = {
        "format_version": 1,
        "map_type": "sparse_occupancy_phase2",
        "frame_id": "map",
        "timestamp_ns": 456,
        "bev": {
            "resolution_m": 0.1,
            "origin_xy": [-0.1, -0.1],
            "width": 3,
            "height": 2,
            "ground_z": -0.4,
        },
    }
    (semantic_directory / "metadata.yaml").write_text(
        yaml.safe_dump(occupancy_metadata), encoding="utf-8"
    )

    result = subprocess.run(
        [
            sys.executable,
            str(ROOT_DIR / "scripts" / "export_semantic_bev.py"),
            str(tmp_path),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    with (semantic_directory / "semantic_metadata.yaml").open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    assert metadata["timestamp_ns"] == 456
    assert metadata["bev"]["width"] == 3
    assert metadata["bev"]["height"] == 2
    assert metadata["bev"]["ground_z"] == -0.4
    with np.load(semantic_directory / "semantic_bev_tensor.npz", allow_pickle=False) as data:
        assert data["semantic_label"].shape == (2, 3)
        assert float(data["ground_z"]) == -0.4
