from pathlib import Path

import numpy as np

from semantic_mapping.bev_projector import (
    BEVProjectionConfig,
    project_occupancy_to_bev,
)
from semantic_mapping.map_serializer import (
    load_occupancy_voxel_map,
    save_occupancy_map,
)
from semantic_mapping.occupancy_voxel_map import (
    OccupancyVoxelConfig,
    SparseOccupancyVoxelMap,
)


def test_phase2_map_save_and_load_round_trip(tmp_path: Path) -> None:
    voxel_map = SparseOccupancyVoxelMap(
        OccupancyVoxelConfig(
            resolution_m=0.1,
            origin_xyz=(-1.0, 2.0, 0.0),
            truncation_distance_m=0.05,
        )
    )
    voxel_map.integrate_points(
        [-0.95, 2.05, 0.45],
        np.array([[-0.55, 2.05, 0.25]], dtype=np.float32),
        timestamp_ns=123,
    )
    bev = project_occupancy_to_bev(
        voxel_map,
        BEVProjectionConfig(resolution_m=0.1, padding_cells=0),
    )

    output = save_occupancy_map(
        tmp_path / "map",
        voxel_map,
        bev,
        frame_id="map",
        timestamp_ns=123,
        ground_z=0.0,
    )
    restored, metadata = load_occupancy_voxel_map(output)

    assert metadata["frame_id"] == "map"
    assert metadata["timestamp_ns"] == 123
    assert (output / "planner_tensor.npz").is_file()
    for name, expected in voxel_map.to_arrays().items():
        np.testing.assert_array_equal(restored.to_arrays()[name], expected)
