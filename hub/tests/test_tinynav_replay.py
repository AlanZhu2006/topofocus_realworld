from __future__ import annotations

import json
import pickle

import cv2
import numpy as np
import pytest

from focus_hub.central_mapping import CentralMapper, MapperConfig
from focus_hub.tinynav_replay import TinyNavReplayReader

# Optical camera frame (x right, y down, z forward) to a z-up world where the
# camera looks along world +x from 0.5 m above the floor.
T_WORLD_CAM = np.array(
    [
        [0.0, 0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.5],
        [0.0, 0.0, 0.0, 1.0],
    ]
)

WIDTH, HEIGHT = 64, 48
K = np.array([[40.0, 0.0, 32.0], [0.0, 40.0, 24.0], [0.0, 0.0, 1.0]])


@pytest.fixture
def synthetic_record(tmp_path):
    record = tmp_path / "record"
    extracted = tmp_path / "extracted"
    (record / "rgb_images_db").mkdir(parents=True)
    (extracted / "depths_pkl").mkdir(parents=True)

    timestamps = [1_000_000_000, 2_000_000_000, 3_000_000_000]
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]

    writer = cv2.VideoWriter(
        str(record / "rgb_images_db" / "video.mp4"),
        cv2.VideoWriter_fourcc(*"mp4v"),
        30,
        (WIDTH, HEIGHT),
    )
    assert writer.isOpened()
    for color in colors:
        frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        frame[:] = color
        writer.write(frame)
    writer.release()
    with (record / "rgb_images_db" / "meta.json").open("w") as f:
        json.dump({"ts_to_idx": {str(t): i for i, t in enumerate(timestamps)}}, f)

    poses = {t: T_WORLD_CAM.copy() for t in timestamps}
    np.save(record / "poses.npy", poses, allow_pickle=True)
    np.save(record / "intrinsics.npy", K)
    np.save(record / "rgb_camera_intrinsics.npy", K)
    np.save(record / "T_rgb_to_infra1.npy", np.eye(4))
    np.save(record / "baseline.npy", np.float64(0.05))
    np.save(record / "tf_messages.npy", {timestamps[0]: {}}, allow_pickle=True)

    manifest = {"depth_keys": timestamps, "source_sha256": {}, "extracted_sha256": {}}
    for i, t in enumerate(timestamps):
        depth = np.full((HEIGHT, WIDTH), 2.0 + 0.1 * i, dtype=np.float32)
        depth[0, 0] = 0.0  # one invalid pixel
        with (extracted / "depths_pkl" / f"{t}.pkl").open("wb") as f:
            f.write(pickle.dumps(depth))
    with (extracted / "manifest.json").open("w") as f:
        json.dump(manifest, f)

    return record, extracted, timestamps, colors


def test_reader_pairs_streams_in_timestamp_order(synthetic_record):
    record, extracted, timestamps, colors = synthetic_record
    reader = TinyNavReplayReader(record, extracted)
    assert len(reader) == 3

    frames = list(reader.frames())
    assert [f.timestamp_ns for f in frames] == timestamps
    for i, frame in enumerate(frames):
        assert frame.rgb_bgr.shape == (HEIGHT, WIDTH, 3)
        # mp4v is lossy; the dominant channel must still win by a wide margin.
        mean_bgr = frame.rgb_bgr.reshape(-1, 3).mean(axis=0)
        assert int(np.argmax(mean_bgr)) == int(np.argmax(colors[i]))
        assert frame.depth_m[1, 1] == pytest.approx(2.0 + 0.1 * i)
        assert frame.depth_m[0, 0] == 0.0
        np.testing.assert_allclose(frame.T_world_rgb, frame.T_world_infra1)


def test_reader_rejects_mismatched_keyframe_sets(synthetic_record):
    record, extracted, timestamps, _ = synthetic_record
    manifest = {"depth_keys": timestamps[:-1], "source_sha256": {}, "extracted_sha256": {}}
    with (extracted / "manifest.json").open("w") as f:
        json.dump(manifest, f)
    with pytest.raises(ValueError, match="disagree"):
        TinyNavReplayReader(record, extracted)


class FakeFrame:
    def __init__(self, depth: np.ndarray, pose: np.ndarray):
        self.timestamp_ns = 0
        self.index = 0
        self.rgb_bgr = np.zeros((*depth.shape, 3), dtype=np.uint8)
        self.depth_m = depth
        self.T_world_infra1 = pose
        self.T_world_rgb = pose


def make_mapper() -> CentralMapper:
    config = MapperConfig(map_size_m=8.0)
    return CentralMapper(
        config=config,
        K_infra1=K,
        K_rgb=K,
        T_rgb_to_infra1=np.eye(4),
        origin_xy_m=(-4.0, -4.0),
        floor_z_m=0.0,
    )


def test_mapper_places_wall_at_expected_world_cell():
    mapper = make_mapper()
    depth = np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32)
    # Every pixel is labelled chair (RedNet MP3D id 4 -> semantic channel 2).
    semantic = np.full((HEIGHT, WIDTH), 4, dtype=np.int16)
    mapper.integrate(FakeFrame(depth, T_WORLD_CAM), semantic)

    grid = mapper.map.grid
    # The optical axis hits world (x=2, y=0, z=0.5): row/col from origin -4 m.
    row, col = mapper.map.world_to_cell(np.array([2.0]), np.array([0.0]))
    assert grid[1, row[0], col[0]] == 1.0            # explored
    assert grid[0, row[0], col[0]] == 1.0            # obstacle in height band
    assert grid[2, row[0], col[0]] == 1.0            # chair channel
    # Nothing behind the wall was observed.
    far_row, far_col = mapper.map.world_to_cell(np.array([3.5]), np.array([0.0]))
    assert grid[1, far_row[0], far_col[0]] == 0.0


def test_mapper_ignores_out_of_band_heights_for_obstacles():
    mapper = make_mapper()
    depth = np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32)
    semantic = np.ones((HEIGHT, WIDTH), dtype=np.int16)
    # Raise the camera so every hit lands above the obstacle band.
    lifted = T_WORLD_CAM.copy()
    lifted[2, 3] = 4.0
    mapper.integrate(FakeFrame(depth, lifted), semantic)
    assert mapper.map.grid[0].sum() == 0.0
    assert mapper.map.grid[1].sum() > 0.0


def test_mapper_max_fusion_is_deterministic_and_monotonic():
    mapper_a = make_mapper()
    mapper_b = make_mapper()
    depth = np.full((HEIGHT, WIDTH), 2.0, dtype=np.float32)
    semantic = np.full((HEIGHT, WIDTH), 4, dtype=np.int16)
    for mapper in (mapper_a, mapper_b):
        mapper.integrate(FakeFrame(depth, T_WORLD_CAM), semantic)
        before = mapper.map.grid.copy()
        mapper.integrate(FakeFrame(depth, T_WORLD_CAM), semantic)
        assert np.all(mapper.map.grid >= before)
    assert mapper_a.map.grid.tobytes() == mapper_b.map.grid.tobytes()
    assert mapper_a.map.frames_fused == 2
