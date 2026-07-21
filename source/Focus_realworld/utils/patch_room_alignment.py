"""
Step 3: Patch ↔ Room Alignment
================================
Maps each LLM vision token from GLM-4V-9B to a room_id from the room
segmentation map, enabling room-level attention routing and token pruning.

Model: GLM-4V-9B (THUDM/glm-4v-9b)
  - Vision encoder: EVA2-CLIP-E
  - ViT: 1120x1120 input, 14x14 patch → 80x80 = 6400 ViT patches
  - Stride-2 Conv2d in EVA2CLIPModel downsamples to 40x40 = 1600 LLM tokens
  - These 1600 tokens are what the LLM transformer layers actually see

Rendered map: 480x480 (sem_map_frontier from Decision_Generation_Vis)

Each LLM token covers a 12x12 pixel region in the 480x480 map
(480 / 40 = 12).

Goal:
  - patch_room[i]        -> room_id for LLM token i  (-1 = wall/unknown)
  - room_to_patches[r]   -> list of LLM token indices belonging to room r
"""

import numpy as np
import cv2
import logging
from collections import defaultdict


# -- GLM-4V-9B LLM vision token grid --
# ViT produces 80x80=6400 patches, but stride-2 conv in EVA2CLIPModel
# (visual.py:156) downsamples to 40x40=1600 tokens fed to the LLM.
LLM_GRID_SIDE = 40          # tokens per side after stride-2 conv
LLM_N_TOKENS = LLM_GRID_SIDE * LLM_GRID_SIDE  # 1600
MAP_SIZE = 480               # rendered map resolution
CELL_SIZE = MAP_SIZE / LLM_GRID_SIDE  # 12.0 pixels per token


def build_patch_room_mapping(room_map, map_size=MAP_SIZE, grid_side=LLM_GRID_SIDE):
    """
    Assign each LLM vision token a room_id via majority vote on room_map pixels.

    The 480x480 map is divided into a 40x40 grid. Each cell is 12x12 pixels.
    For each cell, we look at the room_map pixels it covers and pick the
    most common room_id (ignoring background 0 and unknown -1).

    Args:
        room_map:    np.ndarray [H, W] with integer room IDs (0=background, >0=room)
        map_size:    rendered map image size (default 480)
        grid_side:   LLM token grid dimension (default 40)

    Returns:
        patch_room:        np.ndarray [1600], room_id per LLM token (-1 if wall/unknown)
        room_to_patches:   dict {room_id: [token_indices]}
    """
    cell = map_size / grid_side  # 12.0

    n_tokens = grid_side * grid_side
    patch_room = np.full(n_tokens, -1, dtype=np.int32)
    room_to_patches = defaultdict(list)

    for idx in range(n_tokens):
        row, col = divmod(idx, grid_side)

        # pixel range this token covers in the 480x480 map
        y0 = int(row * cell)
        y1 = int((row + 1) * cell)
        x0 = int(col * cell)
        x1 = int((col + 1) * cell)

        # clamp to map bounds
        y0, y1 = max(0, y0), min(map_size, y1)
        x0, x1 = max(0, x0), min(map_size, x1)
        if y1 <= y0 or x1 <= x0:
            continue

        region = room_map[y0:y1, x0:x1].flatten()
        if len(region) == 0:
            continue

        # majority vote, ignoring background (0) and unknown (-1)
        valid = region[(region > 0)]
        if len(valid) == 0:
            continue

        vals, cnts = np.unique(valid, return_counts=True)
        winner = vals[np.argmax(cnts)]
        patch_room[idx] = winner
        room_to_patches[int(winner)].append(idx)

    return patch_room, dict(room_to_patches)


def compute_coverage(patch_room):
    """Fraction of LLM tokens assigned to a valid room (not -1)."""
    return float((patch_room != -1).mean())


def visualize_patch_room_overlay(room_map, room_info, room_to_patches,
                                  room_segmentation,
                                  map_size=MAP_SIZE,
                                  grid_side=LLM_GRID_SIDE,
                                  top_n=3,
                                  alpha=0.5):
    """
    Side-by-side: [room map] | [LLM token grid colored by room assignment].

    Each cell (12x12 pixels) is filled with its room's color, with grid lines
    showing token boundaries. Used for DoD verification: token cells at room
    borders should align with the room map within ~1 cell width.

    Args:
        room_map:           room segmentation map
        room_info:          dict {room_id: {area, ...}}
        room_to_patches:    dict {room_id: [token_indices]}
        room_segmentation:  RoomSegmentation instance (for visualize_rooms)
        map_size:           rendered map size (default 480)
        grid_side:          LLM token grid dimension (default 40)
        top_n:              number of rooms to label
        alpha:              blend factor for token overlay

    Returns:
        vis: np.ndarray [H, W*2, 3] BGR side-by-side image
    """
    cell = map_size / grid_side  # 12.0

    # Left panel: original room segmentation
    room_vis = room_segmentation.visualize_rooms(room_map, room_info).copy()

    # Right panel: token grid colored by room assignment
    patch_layer = np.zeros((map_size, map_size, 3), dtype=np.uint8)

    # Reproduce the exact same colors as visualize_rooms():
    #   np.random.seed(42), then randint(50,255,3) per room_id in room_info order
    np.random.seed(42)
    room_colors = {}
    for rid in room_info.keys():
        room_colors[rid] = tuple(np.random.randint(50, 255, 3).tolist())

    # Fill each token cell with its room color
    for rid, patches in room_to_patches.items():
        if rid <= 0:
            continue
        color = room_colors.get(rid, (128, 128, 128))
        for idx in patches:
            row, col = divmod(idx, grid_side)
            y0 = int(row * cell)
            y1 = int((row + 1) * cell)
            x0 = int(col * cell)
            x1 = int((col + 1) * cell)
            y0, y1 = max(0, y0), min(map_size, y1)
            x0, x1 = max(0, x0), min(map_size, x1)
            patch_layer[y0:y1, x0:x1] = color

    # Draw grid lines to show token boundaries
    for i in range(grid_side + 1):
        pos = int(i * cell)
        pos = min(pos, map_size - 1)
        cv2.line(patch_layer, (pos, 0), (pos, map_size - 1), (80, 80, 80), 1)
        cv2.line(patch_layer, (0, pos), (map_size - 1, pos), (80, 80, 80), 1)

    # Blend token grid on top of room map for right panel
    right = cv2.addWeighted(room_vis, 1.0 - alpha, patch_layer, alpha, 0)

    # Add labels for top-N rooms on right panel
    top_rooms = sorted(room_info.keys(),
                       key=lambda r: room_info[r]['area'],
                       reverse=True)[:top_n]
    for rid in top_rooms:
        patches = room_to_patches.get(rid, [])
        if not patches:
            continue
        # Find centroid of this room's tokens
        ys, xs = [], []
        for idx in patches:
            row, col = divmod(idx, grid_side)
            ys.append(int((row + 0.5) * cell))
            xs.append(int((col + 0.5) * cell))
        cy, cx = int(np.mean(ys)), int(np.mean(xs))
        cv2.putText(right, f"R{rid}", (cx - 10, cy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # Concatenate side-by-side: [room map | token overlay]
    vis = np.hstack([room_vis, right])

    return vis


def log_patch_room_stats(patch_room, room_to_patches, room_info, step):
    """Log coverage and per-room token counts (excludes background room 0)."""
    coverage = compute_coverage(patch_room)
    total = len(patch_room)
    assigned = int(coverage * total)
    logging.info(f"LLM token coverage: {coverage*100:.1f}% ({assigned}/{total} tokens, {LLM_GRID_SIDE}x{LLM_GRID_SIDE} grid)")
    for rid in sorted(room_to_patches.keys()):
        if rid <= 0:
            continue
        n_patches = len(room_to_patches[rid])
        logging.info(f"  Room {rid}: {n_patches} tokens")
    logging.info(f"==========================================")
    logging.info("")
