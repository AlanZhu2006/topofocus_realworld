"""Central per-robot semantic BEV mapping from replayed real RGB-D keyframes.

This adapts the source-derived mapping rules of
``source/Focus_realworld/semantic_mapping.py`` to real SE(3) camera poses:

  * RedNet (``dependencies/RedNet``, MP3D-40 checkpoint) segments each RGB
    keyframe; the 15 HM3D categories are selected with the upstream
    ``mp_categories_mapping`` table.
  * Depth pixels are lifted to 3-D in the rectified infra1 frame, transformed
    into the gravity-aligned TinyNav world (+z up), and splatted into a 2-D
    grid at the upstream 5 cm resolution.
  * Channel semantics follow upstream: channel 0 obstacle (points inside a
    height band above the floor), channel 1 explored (any valid return),
    channels 2..16 the semantic categories. Semantic/explored evidence keeps
    the upstream element-wise-max rule. Live deployments may opt channel 0
    into reversible log-odds evidence, derived from TinyNav BuildMap's use of
    both free-ray and occupied-endpoint evidence, so a single noisy endpoint
    is not necessarily permanent.

Deviations from upstream, recorded rather than hidden:
  * poses come from TinyNav SLAM instead of Habitat ground truth, so no
    (dx, dy, dtheta) dead-reckoning integration or grid_sample warping is
    needed; points are transformed directly into the world frame.
  * upstream gates six semantic channels with a second Grounded-SAM
    predictor; the replay mapper uses RedNet alone (mapping_only scope).
  * the floor plane is estimated from real depth because the real camera
    height is not a Habitat constant. Live mapping keeps all three plane
    coefficients; collapsing a tilted floor to one scalar z creates false
    obstacles away from the camera position used to obtain that scalar.
  * added 2026-07-21: each valid depth return also marks the "explored"
    channel along the ray from the camera to that point (not just the
    endpoint cell). Upstream doesn't need this -- Habitat's simulated depth
    is dense and noise-free, so upstream's endpoint-only "explored" marking
    already produces a filled sector. A real depth camera's valid-return
    pattern is sparser and less uniform (textureless surfaces, range
    limits), so endpoint-only marking left visible gaps between rays
    radiating from the camera instead of a filled swept area -- reported by
    a user as "black ray" artifacts on Yunji's map. This is a practical
    real-sensor addition, not an upstream-documented rule; only the
    "explored" channel is affected by ray tracing, obstacle/semantic
    channels still only come from the true endpoint.

Everything here is deterministic given identical inputs.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# Upstream HM3D 15-category selection over MP3D-40 RedNet ids
# (source/Focus_realworld/constants.py, mp_categories_mapping).
MP_CATEGORIES_MAPPING: tuple[int, ...] = (4, 11, 15, 12, 19, 23, 26, 24, 28, 38, 21, 16, 14, 6, 16)
HM3D_CATEGORY_NAMES: tuple[str, ...] = (
    "chair", "sofa", "plant", "bed", "toilet", "tv", "bathtub", "shower",
    "fireplace", "appliances", "towel", "sink", "chest_of_drawers", "table", "stairs",
)

# Upstream depth normalisation for RedNet (Habitat convention).
DEPTH_MIN_M = 0.5
DEPTH_MAX_M = 5.0


@dataclass(frozen=True)
class MapperConfig:
    resolution_m: float = 0.05          # upstream map_resolution = 5 cm
    map_size_m: float = 24.0            # upstream map_size_cm = 2400
    obstacle_band_low_m: float = 0.25   # upstream min_z = 25 cm above floor
    obstacle_band_high_m: float = 1.5   # upstream agent band top, generalised
    semantic_band_low_m: float = 0.25   # keep object evidence independent from
    semantic_band_high_m: float = 1.5   # the planner collision-height band
    map_pred_threshold: float = 1.0     # upstream defaults
    exp_pred_threshold: float = 1.0
    cat_pred_threshold: float = 5.0
    max_range_m: float = DEPTH_MAX_M    # ignore returns beyond RedNet's clip
    min_range_m: float = 0.3
    depth_stride: int = 1
    ray_trace_steps: int = 40           # samples along each camera->point ray for
                                         # free-space "explored" marking; higher is
                                         # more complete but costs more per frame
    ray_trace_chunk_points: int = 8192  # bound temporary arrays for dense RGB-D
    obstacle_fusion_mode: str = "max"   # "max" preserves upstream replay behavior;
                                         # "log_odds" is the live sensor mode
    obstacle_free_update: float = -0.40
    obstacle_occupied_update: float = 0.85
    obstacle_min_log_odds: float = -4.0
    obstacle_max_log_odds: float = 4.0
    obstacle_probability_threshold: float = 0.70
    obstacle_min_hits: int = 1
    semantic_fusion_mode: str = "max"  # "max" preserves upstream behavior;
                                        # "multi_view" requires repeated cells
    semantic_min_hits: int = 1
    semantic_winner_margin_hits: int = 0

    def __post_init__(self) -> None:
        if self.obstacle_fusion_mode not in {"max", "log_odds"}:
            raise ValueError("obstacle_fusion_mode must be 'max' or 'log_odds'")
        if self.semantic_fusion_mode not in {"max", "multi_view"}:
            raise ValueError("semantic_fusion_mode must be 'max' or 'multi_view'")
        if self.obstacle_band_low_m >= self.obstacle_band_high_m:
            raise ValueError("obstacle height band is inverted")
        if self.semantic_band_low_m >= self.semantic_band_high_m:
            raise ValueError("semantic height band is inverted")
        if self.obstacle_free_update >= 0.0 or self.obstacle_occupied_update <= 0.0:
            raise ValueError("expected obstacle_free_update < 0 and occupied_update > 0")
        if self.obstacle_min_log_odds >= self.obstacle_max_log_odds:
            raise ValueError("obstacle log-odds bounds are inverted")
        if not 0.5 < self.obstacle_probability_threshold < 1.0:
            raise ValueError("obstacle probability threshold must be between 0.5 and 1")
        if self.obstacle_min_hits < 1:
            raise ValueError("obstacle_min_hits must be positive")
        if self.semantic_min_hits < 1:
            raise ValueError("semantic_min_hits must be positive")
        if self.semantic_winner_margin_hits < 0:
            raise ValueError("semantic_winner_margin_hits must be non-negative")


class RedNetSegmenter:
    """Wraps the upstream RedNet loader with the upstream input contract."""

    def __init__(self, checkpoint: Path | str, device: str = "cuda") -> None:
        import torch
        from RedNet.RedNet_model import load_rednet

        self._torch = torch
        self.device = torch.device(device)
        self.model = load_rednet(self.device, ckpt=str(checkpoint), resize=True)
        self.model.eval()

    def segment(self, rgb_bgr: np.ndarray, depth_m: np.ndarray) -> np.ndarray:
        """Return the MP3D-40 class id per pixel (1..40) as int16 (H, W)."""
        torch = self._torch
        rgb = np.ascontiguousarray(rgb_bgr[:, :, ::-1], dtype=np.float32)
        depth_norm = (depth_m - DEPTH_MIN_M) / (DEPTH_MAX_M - DEPTH_MIN_M)
        depth_norm = np.clip(depth_norm, 0.0, 1.0).astype(np.float32)
        # Upstream marks invalid returns as far rather than near.
        depth_norm[depth_m <= 0.0] = 1.0
        with torch.no_grad():
            image_t = torch.from_numpy(rgb).to(self.device).unsqueeze(0)
            depth_t = torch.from_numpy(depth_norm[..., None]).to(self.device).unsqueeze(0)
            pred = self.model(image_t, depth_t)
        return pred.squeeze(0).cpu().numpy().astype(np.int16)


@dataclass
class CentralSemanticMap:
    """Accumulated world-frame BEV map for one robot."""

    config: MapperConfig
    origin_xy_m: tuple[float, float]
    floor_z_m: float
    floor_plane_coefficients: tuple[float, float, float] | None = None
    grid: np.ndarray = field(init=False)   # (2 + 15, N, N) float32 in [0, 1]
    frames_fused: int = field(init=False, default=0)

    def __post_init__(self) -> None:
        if not np.isfinite(self.floor_z_m):
            raise ValueError("floor_z_m must be finite")
        if self.floor_plane_coefficients is None:
            self.floor_plane_coefficients = (0.0, 0.0, float(self.floor_z_m))
        coefficients = np.asarray(self.floor_plane_coefficients, dtype=np.float64)
        if coefficients.shape != (3,) or not np.all(np.isfinite(coefficients)):
            raise ValueError("floor_plane_coefficients must contain three finite values")
        self.floor_plane_coefficients = tuple(float(value) for value in coefficients)
        cells = int(round(self.config.map_size_m / self.config.resolution_m))
        self.grid = np.zeros((2 + len(HM3D_CATEGORY_NAMES), cells, cells), dtype=np.float32)

    @property
    def cells(self) -> int:
        return self.grid.shape[1]

    def world_to_cell(self, x_m: np.ndarray, y_m: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        col = np.floor((x_m - self.origin_xy_m[0]) / self.config.resolution_m).astype(np.int64)
        row = np.floor((y_m - self.origin_xy_m[1]) / self.config.resolution_m).astype(np.int64)
        return row, col


class CentralMapper:
    """Fuses replayed keyframes into a CentralSemanticMap."""

    def __init__(
        self,
        config: MapperConfig,
        K_infra1: np.ndarray,
        K_rgb: np.ndarray,
        T_rgb_to_infra1: np.ndarray,
        origin_xy_m: tuple[float, float],
        floor_z_m: float,
        floor_plane_coefficients: tuple[float, float, float] | None = None,
    ) -> None:
        self.config = config
        self.K_infra1 = np.asarray(K_infra1, dtype=np.float64)
        self.K_rgb = np.asarray(K_rgb, dtype=np.float64)
        # p_rgb = T_infra1_to_rgb @ p_infra1
        self.T_infra1_to_rgb = np.linalg.inv(np.asarray(T_rgb_to_infra1, dtype=np.float64))
        self.map = CentralSemanticMap(
            config=config,
            origin_xy_m=origin_xy_m,
            floor_z_m=floor_z_m,
            floor_plane_coefficients=floor_plane_coefficients,
        )
        self._pixel_rays: np.ndarray | None = None
        self._pixel_shape: tuple[int, int] | None = None
        self._obstacle_log_odds = np.zeros(
            (self.map.cells, self.map.cells), dtype=np.float32
        )
        self._obstacle_hits = np.zeros(
            (self.map.cells, self.map.cells), dtype=np.uint32
        )
        self._semantic_hits = np.zeros(
            (len(HM3D_CATEGORY_NAMES), self.map.cells, self.map.cells),
            dtype=np.uint16,
        )

    def _rays(self, shape: tuple[int, int]) -> np.ndarray:
        if self._pixel_shape != shape:
            h, w = shape
            stride = self.config.depth_stride
            vs, us = np.meshgrid(
                np.arange(0, h, stride, dtype=np.float64),
                np.arange(0, w, stride, dtype=np.float64),
                indexing="ij",
            )
            fx, fy = self.K_infra1[0, 0], self.K_infra1[1, 1]
            cx, cy = self.K_infra1[0, 2], self.K_infra1[1, 2]
            rays = np.stack(((us - cx) / fx, (vs - cy) / fy, np.ones_like(us)), axis=-1)
            self._pixel_rays = rays
            self._pixel_shape = shape
        return self._pixel_rays

    def integrate(
        self,
        frame,
        semantic_pred: np.ndarray,
        *,
        floor_plane_coefficients: tuple[float, float, float] | None = None,
    ) -> None:
        """Fuse one ReplayFrame with its RedNet prediction into the map."""
        cfg = self.config
        depth = frame.depth_m[:: cfg.depth_stride, :: cfg.depth_stride].astype(np.float64)
        rays = self._rays(frame.depth_m.shape)
        valid = (depth >= cfg.min_range_m) & (depth <= cfg.max_range_m)
        if not np.any(valid):
            self.map.frames_fused += 1
            return

        points_infra1 = rays[valid] * depth[valid][:, None]

        # World coordinates (TinyNav world, +z up).
        R = frame.T_world_infra1[:3, :3]
        t = frame.T_world_infra1[:3, 3]
        points_world = points_infra1 @ R.T + t

        # Sample the RedNet class by projecting each depth point into the RGB
        # image (depth stays in its native rectified frame; no depth warping).
        p_rgb = points_infra1 @ self.T_infra1_to_rgb[:3, :3].T + self.T_infra1_to_rgb[:3, 3]
        in_front = p_rgb[:, 2] > 1e-6
        u = np.full(len(p_rgb), -1.0)
        v = np.full(len(p_rgb), -1.0)
        u[in_front] = self.K_rgb[0, 0] * p_rgb[in_front, 0] / p_rgb[in_front, 2] + self.K_rgb[0, 2]
        v[in_front] = self.K_rgb[1, 1] * p_rgb[in_front, 1] / p_rgb[in_front, 2] + self.K_rgb[1, 2]
        h_rgb, w_rgb = semantic_pred.shape
        ui = np.round(u).astype(np.int64)
        vi = np.round(v).astype(np.int64)
        in_rgb = (ui >= 0) & (ui < w_rgb) & (vi >= 0) & (vi < h_rgb)
        labels = np.zeros(len(p_rgb), dtype=np.int16)
        labels[in_rgb] = semantic_pred[vi[in_rgb], ui[in_rgb]]

        grid = self.map.grid
        cells = self.map.cells
        frame_counts = np.zeros((grid.shape[0], cells * cells), dtype=np.float64)

        # Free-space ray marking: sample points strictly between the camera
        # and each (unfiltered, i.e. including out-of-map) endpoint, so a ray
        # to a far or out-of-bounds point still fills in the in-map cells it
        # actually swept through. Only the "explored" channel is touched --
        # obstacle/semantic labels only ever come from the true endpoint below.
        # Process dense RGB-D in bounded chunks: constructing all
        # (ray_trace_steps x valid_pixels) samples at once reached ~900 MiB RSS
        # at 848x480 in the 2026-07-21 integration benchmark.
        n_steps = self.config.ray_trace_steps
        if n_steps > 1 and len(points_world):
            origin_xy = t[:2]
            frac = (np.arange(1, n_steps, dtype=np.float64) / n_steps)  # excludes 0 and 1
            chunk_size = max(1, int(self.config.ray_trace_chunk_points))
            for start in range(0, len(points_world), chunk_size):
                endpoints = points_world[start : start + chunk_size]
                ray_x = origin_xy[0] + frac[:, None] * (endpoints[:, 0][None, :] - origin_xy[0])
                ray_y = origin_xy[1] + frac[:, None] * (endpoints[:, 1][None, :] - origin_xy[1])
                ray_row, ray_col = self.map.world_to_cell(ray_x.reshape(-1), ray_y.reshape(-1))
                ray_in_map = (ray_row >= 0) & (ray_row < cells) & (ray_col >= 0) & (ray_col < cells)
                ray_flat = ray_row[ray_in_map] * cells + ray_col[ray_in_map]
                frame_counts[1] += np.bincount(ray_flat, minlength=cells * cells)

        row, col = self.map.world_to_cell(points_world[:, 0], points_world[:, 1])
        in_map = (row >= 0) & (row < cells) & (col >= 0) & (col < cells)
        row, col = row[in_map], col[in_map]
        map_points = points_world[in_map]
        active_floor_plane = (
            self.map.floor_plane_coefficients
            if floor_plane_coefficients is None
            else floor_plane_coefficients
        )
        coefficients = np.asarray(active_floor_plane, dtype=np.float64)
        if coefficients.shape != (3,) or not np.all(np.isfinite(coefficients)):
            raise ValueError("floor plane override must contain three finite values")
        floor_z = (
            coefficients[0] * map_points[:, 0]
            + coefficients[1] * map_points[:, 1]
            + coefficients[2]
        )
        z_rel = map_points[:, 2] - floor_z
        labels = labels[in_map]

        flat = row * cells + col
        np.add.at(frame_counts[1], flat, 1.0)  # explored: every in-map valid return

        obstacle_band = (
            (z_rel >= self.config.obstacle_band_low_m)
            & (z_rel <= self.config.obstacle_band_high_m)
        )
        semantic_band = (
            (z_rel >= self.config.semantic_band_low_m)
            & (z_rel <= self.config.semantic_band_high_m)
        )
        np.add.at(frame_counts[0], flat[obstacle_band], 1.0)

        for channel, rednet_id in enumerate(MP_CATEGORIES_MAPPING, start=2):
            chosen = semantic_band & (labels == rednet_id)
            if np.any(chosen):
                np.add.at(frame_counts[channel], flat[chosen], 1.0)

        frame_map = np.zeros_like(grid)
        frame_map[0] = np.clip(
            frame_counts[0].reshape(cells, cells) / cfg.map_pred_threshold, 0.0, 1.0
        )
        frame_map[1] = np.clip(
            frame_counts[1].reshape(cells, cells) / cfg.exp_pred_threshold, 0.0, 1.0
        )
        frame_map[2:] = np.clip(
            frame_counts[2:].reshape(-1, cells, cells) / cfg.cat_pred_threshold, 0.0, 1.0
        )

        # Explored retains upstream's deterministic max fusion. Semantic
        # channels can retain the exact source max rule or, for an explicitly
        # selected real-camera deployment adapter, require repeated keyframe
        # support and a unique winning category. Geometry can retain the exact
        # replay mode too, or use the
        # live sensor mode: one free/occupied update per XY cell per frame,
        # with occupied winning an intra-frame conflict. This mirrors the
        # evidence semantics of TinyNav's native occupancy builder and, unlike
        # max fusion, lets later free rays clear a noisy obstacle endpoint.
        np.maximum(grid[1], frame_map[1], out=grid[1])
        if cfg.semantic_fusion_mode == "max":
            np.maximum(grid[2:], frame_map[2:], out=grid[2:])
        else:
            # Count at most one vote per class/cell/keyframe. A dense patch in
            # one image must not satisfy the multi-view confirmation gate by
            # itself. Saturation avoids uint16 wraparound in long experiments.
            supported = (
                frame_counts[2:].reshape(
                    len(HM3D_CATEGORY_NAMES), cells, cells
                )
                >= cfg.cat_pred_threshold
            )
            np.add(
                self._semantic_hits,
                supported,
                out=self._semantic_hits,
                casting="unsafe",
                where=self._semantic_hits < np.iinfo(np.uint16).max,
            )
            winners = np.argmax(self._semantic_hits, axis=0)
            winner_hits = np.max(self._semantic_hits, axis=0)
            if len(HM3D_CATEGORY_NAMES) > 1:
                runner_up_hits = np.partition(
                    self._semantic_hits, -2, axis=0
                )[-2]
            else:
                runner_up_hits = np.zeros_like(winner_hits)
            confirmed = (
                (winner_hits >= cfg.semantic_min_hits)
                & (
                    winner_hits.astype(np.int32)
                    - runner_up_hits.astype(np.int32)
                    >= cfg.semantic_winner_margin_hits
                )
            )
            grid[2:] = 0.0
            confirmed_rows, confirmed_cols = np.nonzero(confirmed)
            grid[
                2 + winners[confirmed_rows, confirmed_cols],
                confirmed_rows,
                confirmed_cols,
            ] = 1.0
        if cfg.obstacle_fusion_mode == "max":
            np.maximum(grid[0], frame_map[0], out=grid[0])
        else:
            occupied = (
                frame_counts[0].reshape(cells, cells) >= cfg.map_pred_threshold
            )
            free = (frame_counts[1].reshape(cells, cells) > 0.0) & ~occupied
            self._obstacle_log_odds[free] += cfg.obstacle_free_update
            self._obstacle_log_odds[occupied] += cfg.obstacle_occupied_update
            np.clip(
                self._obstacle_log_odds,
                cfg.obstacle_min_log_odds,
                cfg.obstacle_max_log_odds,
                out=self._obstacle_log_odds,
            )
            self._obstacle_hits[occupied] += 1
            threshold_log_odds = np.log(
                cfg.obstacle_probability_threshold
                / (1.0 - cfg.obstacle_probability_threshold)
            )
            grid[0] = (
                (self._obstacle_log_odds > threshold_log_odds)
                & (self._obstacle_hits >= cfg.obstacle_min_hits)
            )
        self.map.frames_fused += 1


def estimate_floor_z(
    frames,
    K_infra1: np.ndarray,
    config: MapperConfig,
    sample_frames: int = 20,
    percentile: float = 5.0,
) -> float:
    """Estimate the world-frame floor height from the first few keyframes.

    Deterministic: fixed frame budget, fixed percentile over pooled world z.
    """
    fx, fy = K_infra1[0, 0], K_infra1[1, 1]
    cx, cy = K_infra1[0, 2], K_infra1[1, 2]
    pooled: list[np.ndarray] = []
    for i, frame in enumerate(frames):
        if i >= sample_frames:
            break
        depth = frame.depth_m[::4, ::4].astype(np.float64)
        h, w = depth.shape
        vs, us = np.meshgrid(
            np.arange(0, frame.depth_m.shape[0], 4, dtype=np.float64),
            np.arange(0, frame.depth_m.shape[1], 4, dtype=np.float64),
            indexing="ij",
        )
        valid = (depth >= config.min_range_m) & (depth <= config.max_range_m)
        if not np.any(valid):
            continue
        z = depth[valid]
        x = (us[valid] - cx) / fx * z
        y = (vs[valid] - cy) / fy * z
        pts = np.stack((x, y, z), axis=-1)
        R = frame.T_world_infra1[:3, :3]
        t = frame.T_world_infra1[:3, 3]
        pooled.append((pts @ R.T + t)[:, 2])
    if not pooled:
        raise ValueError("no valid depth returns to estimate the floor height")
    return float(np.percentile(np.concatenate(pooled), percentile))
