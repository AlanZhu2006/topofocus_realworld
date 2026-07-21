"""
Step 6: Active Patches Construction (No Attention Dependency)
==============================================================
Deterministic selection of which LLM vision tokens (patches) matter
for the current navigation step, based on:
  - Chosen frontier -> its room
  - Neighboring rooms (spatially adjacent)
  - Delta patches (map changes since last step)

No reliance on noisy A_patch from Step 5.

Output:
  ActivePatches  <= 256 token indices (out of 1600)
"""

import os
import json
import logging
import numpy as np
import cv2
from pathlib import Path
from datetime import datetime
from collections import defaultdict

# Must match patch_room_alignment.py
LLM_GRID_SIDE = 40
LLM_N_TOKENS = LLM_GRID_SIDE * LLM_GRID_SIDE  # 1600
MAP_SIZE = 480
CELL_SIZE = MAP_SIZE / LLM_GRID_SIDE  # 12.0

BUDGET = 256


# ===================================================================
# 1. Frontier -> Room
# ===================================================================

def frontier_to_room(centroid_x, centroid_y, patch_room,
                     map_size=MAP_SIZE, grid_side=LLM_GRID_SIDE):
    """
    Map a frontier centroid (in unflipped map coords) to a room_id
    via the patch_room array.

    The sem_map_frontier image is flipud'd before the VLM sees it,
    so image_row = map_size - centroid_x.  But patch_room is built
    from the raw (unflipped) room_map, so we index directly.

    Args:
        centroid_x: row in unflipped map space
        centroid_y: column in map space
        patch_room: np.array of shape (1600,) mapping token idx -> room_id

    Returns:
        room_id (int), or -1 if unassigned
    """
    cell = map_size / grid_side
    patch_row = int(centroid_x / cell)
    patch_col = int(centroid_y / cell)

    # Clamp
    patch_row = max(0, min(grid_side - 1, patch_row))
    patch_col = max(0, min(grid_side - 1, patch_col))

    patch_idx = patch_row * grid_side + patch_col
    return int(patch_room[patch_idx])


# ===================================================================
# 2. Neighbor Rooms
# ===================================================================

def find_neighbor_rooms(room_id, patch_room, grid_side=LLM_GRID_SIDE):
    """
    Find rooms spatially adjacent to the given room in the patch grid.

    Two rooms are neighbors if any of their patches are 4-connected
    adjacent in the 40x40 grid.

    Args:
        room_id:    the room to find neighbors for
        patch_room: np.array (1600,) token idx -> room_id
        grid_side:  grid dimension (40)

    Returns:
        set of neighboring room_ids (excludes room_id itself and -1)
    """
    grid = patch_room.reshape(grid_side, grid_side)
    room_mask = (grid == room_id)
    neighbors = set()

    rows, cols = np.where(room_mask)
    for r, c in zip(rows, cols):
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < grid_side and 0 <= nc < grid_side:
                nid = int(grid[nr, nc])
                if nid != room_id and nid != -1:
                    neighbors.add(nid)

    return neighbors


# ===================================================================
# 3. Delta Patches (map changes)
# ===================================================================

def compute_delta_patches(map_cur, map_prev,
                          map_size=MAP_SIZE, grid_side=LLM_GRID_SIDE):
    """
    Compare current and previous 480x480 semantic maps to find
    which patches (12x12 pixel regions) have changed.

    Args:
        map_cur:  current map as np.array, shape (C, H, W) or (H, W)
        map_prev: previous map as np.array, same shape
        map_size: pixel dimension (480)
        grid_side: patch grid dimension (40)

    Returns:
        set of patch indices that changed
    """
    # Handle multi-channel maps: take the max across channels
    if map_cur.ndim == 3:
        cur = map_cur.max(axis=0).astype(np.float32)
    else:
        cur = map_cur.astype(np.float32)

    if map_prev.ndim == 3:
        prev = map_prev.max(axis=0).astype(np.float32)
    else:
        prev = map_prev.astype(np.float32)

    diff = np.abs(cur - prev)
    cell = map_size / grid_side

    delta_patches = set()
    for idx in range(grid_side * grid_side):
        row, col = divmod(idx, grid_side)
        y0 = int(row * cell)
        y1 = int((row + 1) * cell)
        x0 = int(col * cell)
        x1 = int((col + 1) * cell)
        y0, y1 = max(0, y0), min(map_size, y1)
        x0, x1 = max(0, x0), min(map_size, x1)

        if diff[y0:y1, x0:x1].sum() > 0:
            delta_patches.add(idx)

    return delta_patches


# ===================================================================
# 4. Delta Rooms
# ===================================================================

def compute_delta_rooms(delta_patches, patch_room):
    """
    Find rooms that contain any delta patches.

    Args:
        delta_patches: set of changed patch indices
        patch_room: np.array (1600,) token idx -> room_id

    Returns:
        set of room_ids with changes (excludes -1)
    """
    rooms = set()
    for idx in delta_patches:
        rid = int(patch_room[idx])
        if rid != -1:
            rooms.add(rid)
    return rooms


# ===================================================================
# 5. Main Entry Point
# ===================================================================

def compute_active_patches(prev_state, patch_room, room_to_patches,
                           sem_map_frontier, chosen_frontier_centroid,
                           full_map_pred_np, budget=BUDGET):
    """
    Compute the set of active patches for the current step.

    First step of episode (prev_state is None):
        ActivePatches = all 1600 (no previous data).

    Subsequent steps:
        TopRooms_prev = {chosen_room, neighbor_rooms} from PREVIOUS step
        DeltaRooms    = rooms with map changes this step
        ActiveRooms   = TopRooms_prev | DeltaRooms
        ActivePatches = DeltaPatches | patches(ActiveRooms)  [budget cap]

    Args:
        prev_state:     dict from previous step, or None for first step.
                        Keys: 'chosen_room', 'neighbor_rooms', 'map_snapshot'
        patch_room:     np.array (1600,) token idx -> room_id
        room_to_patches: dict {room_id: [patch indices]}
        sem_map_frontier: the 480x480 map image (used for delta if needed)
        chosen_frontier_centroid: (cx, cy) centroid of chosen frontier in map coords
        full_map_pred_np: np.array of full_map_pred for delta computation
        budget:         max active patches (default 256)

    Returns:
        active_patches: sorted list of active patch indices
        new_state:      dict to pass as prev_state next step
        debug_info:     dict with diagnostic details
    """
    cx, cy = chosen_frontier_centroid

    # Current step's chosen room and neighbors
    chosen_room = frontier_to_room(cx, cy, patch_room)
    neighbor_rooms = find_neighbor_rooms(chosen_room, patch_room) if chosen_room != -1 else set()

    # Build new state for next step
    new_state = {
        'chosen_room': chosen_room,
        'neighbor_rooms': neighbor_rooms,
        'map_snapshot': full_map_pred_np.copy(),
    }

    # First step: return all patches
    if prev_state is None:
        all_patches = sorted(range(LLM_N_TOKENS))
        debug_info = {
            'is_first_step': True,
            'chosen_room': chosen_room,
            'neighbor_rooms': sorted(neighbor_rooms),
            'n_active': LLM_N_TOKENS,
            'budget': budget,
        }
        return all_patches, new_state, debug_info

    # --- Subsequent steps ---

    # TopRooms from PREVIOUS step
    prev_chosen = prev_state['chosen_room']
    prev_neighbors = prev_state['neighbor_rooms']
    top_rooms_prev = {prev_chosen} | prev_neighbors
    top_rooms_prev.discard(-1)

    # Delta patches and rooms
    map_prev = prev_state['map_snapshot']
    delta_patches = compute_delta_patches(full_map_pred_np, map_prev)
    delta_rooms = compute_delta_rooms(delta_patches, patch_room)

    # ActiveRooms = TopRooms_prev | DeltaRooms
    active_rooms = top_rooms_prev | delta_rooms

    # ActivePatches = DeltaPatches | patches(ActiveRooms)
    active_set = set(delta_patches)  # always include delta
    for rid in active_rooms:
        if rid in room_to_patches:
            active_set.update(room_to_patches[rid])

    # Budget cap
    active_set, trimmed_rooms = _trim_to_budget(
        active_set, delta_patches, active_rooms,
        room_to_patches, patch_room, budget
    )

    active_patches = sorted(active_set)

    debug_info = {
        'is_first_step': False,
        'chosen_room': chosen_room,
        'neighbor_rooms': sorted(neighbor_rooms),
        'prev_chosen_room': prev_chosen,
        'prev_neighbor_rooms': sorted(prev_neighbors),
        'top_rooms_prev': sorted(top_rooms_prev),
        'delta_rooms': sorted(delta_rooms),
        'active_rooms': sorted(active_rooms),
        'n_delta_patches': len(delta_patches),
        'n_active': len(active_patches),
        'budget': budget,
        'trimmed_rooms': sorted(trimmed_rooms),
    }

    return active_patches, new_state, debug_info




# ===================================================================
# 5b. Pruning-only entry point (no frontier needed)
# ===================================================================

def compute_pruning_set(prev_state, patch_room, room_to_patches,
                        full_map_pred_np, budget=BUDGET):
    """
    Compute active patches for VLM pruning, BEFORE the Decision VLM call.

    Unlike compute_active_patches(), this does NOT need the chosen frontier
    centroid (which is only used for building next step's state).

    Returns:
        sorted list of active patch indices, or None if pruning should be
        skipped (first step or no reduction).
    """
    if prev_state is None:
        return None  # First step — no pruning

    if 'chosen_room' not in prev_state or 'map_snapshot' not in prev_state:
        return None  # Incomplete state

    # TopRooms from PREVIOUS step
    prev_chosen = prev_state['chosen_room']
    prev_neighbors = prev_state.get('neighbor_rooms', set())
    top_rooms_prev = {prev_chosen} | prev_neighbors
    top_rooms_prev.discard(-1)

    # Delta patches and rooms
    map_prev = prev_state['map_snapshot']
    delta_patches = compute_delta_patches(full_map_pred_np, map_prev)
    delta_rooms = compute_delta_rooms(delta_patches, patch_room)

    # ActiveRooms = TopRooms_prev | DeltaRooms
    active_rooms = top_rooms_prev | delta_rooms

    # ActivePatches = DeltaPatches | patches(ActiveRooms)
    active_set = set(delta_patches)
    for rid in active_rooms:
        if rid in room_to_patches:
            active_set.update(room_to_patches[rid])

    # Budget cap
    active_set, _ = _trim_to_budget(
        active_set, delta_patches, active_rooms,
        room_to_patches, patch_room, budget
    )

    active_patches = sorted(active_set)

    if len(active_patches) >= LLM_N_TOKENS:
        return None  # No actual reduction — skip pruning

    return active_patches

# ===================================================================
# 6. Budget Trimming
# ===================================================================

def _trim_to_budget(active_set, delta_patches, active_rooms,
                    room_to_patches, patch_room, budget):
    """
    Cap active_set at budget by dropping patches from smallest rooms first.
    Delta patches are never dropped.

    Args:
        active_set:     set of patch indices (will be modified in place)
        delta_patches:  set of patch indices that must be kept
        active_rooms:   set of room_ids contributing patches
        room_to_patches: dict {room_id: [patch indices]}
        patch_room:     np.array (1600,)
        budget:         max number of patches

    Returns:
        (trimmed_active_set, trimmed_rooms): the final set and which rooms were dropped
    """
    trimmed_rooms = set()

    if len(active_set) <= budget:
        return active_set, trimmed_rooms

    # Sort active rooms by number of patches (smallest first for dropping)
    room_sizes = []
    for rid in active_rooms:
        patches_in_room = set(room_to_patches.get(rid, []))
        room_sizes.append((rid, len(patches_in_room)))
    room_sizes.sort(key=lambda x: x[1])

    # Drop smallest rooms until within budget
    for rid, size in room_sizes:
        if len(active_set) <= budget:
            break
        # Remove this room's patches (but keep delta patches)
        patches_to_remove = set(room_to_patches.get(rid, [])) - delta_patches
        active_set -= patches_to_remove
        trimmed_rooms.add(rid)

    # If still over budget after dropping rooms (many delta patches),
    # just cap by taking first `budget` indices
    if len(active_set) > budget:
        keep = set(sorted(active_set)[:budget])
        active_set = keep

    return active_set, trimmed_rooms


# ===================================================================
# 7. Logger
# ===================================================================

class ActivePatchesLogger:
    """
    Writes active patches DoD metrics to dedicated log files.

    Output files:
      - {log_dir}/active_patches/agent{id}.jsonl  (machine-readable)
      - {log_dir}/active_patches/agent{id}.log    (human-readable)
    """

    def __init__(self, log_dir, agent_id=0):
        self.agent_id = agent_id
        self.dir = Path(log_dir) / "active_patches"
        self.dir.mkdir(parents=True, exist_ok=True)

        self.jsonl_path = self.dir / f"agent{agent_id}.jsonl"
        self.txt_path = self.dir / f"agent{agent_id}.log"

        self._jsonl = open(self.jsonl_path, "a")
        self._txt = open(self.txt_path, "a")

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        header = f"[ActivePatches] agent={agent_id} started at {ts}"
        self._txt.write(header + "\n" + "=" * 70 + "\n")
        self._txt.flush()

        self._entries = []

    def log(self, step, active_patches, debug_info):
        """
        Log one step's active patches info.

        Args:
            step:           int, navigation step number
            active_patches: list of active patch indices
            debug_info:     dict from compute_active_patches()
        """
        entry = {
            "step": step,
            "agent_id": self.agent_id,
            "n_active": len(active_patches),
            **debug_info,
        }
        self._entries.append(entry)

        # JSONL
        self._jsonl.write(json.dumps(entry, default=str) + "\n")
        self._jsonl.flush()

        # Human-readable
        is_first = debug_info.get('is_first_step', False)
        n = len(active_patches)
        budget = debug_info.get('budget', BUDGET)
        budget_ok = "OK" if n <= budget else "FAIL"

        if is_first:
            lines = [
                f"\n[Step {step}] FIRST STEP — all {n} patches active",
                f"  Chosen room: {debug_info.get('chosen_room')}",
                f"  Neighbor rooms: {debug_info.get('neighbor_rooms')}",
            ]
        else:
            delta_ok = "OK"
            n_delta = debug_info.get('n_delta_patches', 0)
            lines = [
                f"\n[Step {step}] Active:{n}/{budget} Budget:{budget_ok}",
                f"  TopRooms_prev: {debug_info.get('top_rooms_prev')}",
                f"  DeltaRooms: {debug_info.get('delta_rooms')}",
                f"  ActiveRooms: {debug_info.get('active_rooms')}",
                f"  DeltaPatches: {n_delta}",
                f"  Chosen room (current): {debug_info.get('chosen_room')}",
                f"  Neighbor rooms (current): {debug_info.get('neighbor_rooms')}",
            ]
            trimmed = debug_info.get('trimmed_rooms', [])
            if trimmed:
                lines.append(f"  Trimmed rooms: {trimmed}")

        self._txt.write("\n".join(lines) + "\n")
        self._txt.flush()

    def write_summary(self):
        """Write DoD summary at end of episode."""
        if not self._entries:
            return {"status": "NO_DATA"}

        total = len(self._entries)
        budget_passes = sum(1 for e in self._entries if e['n_active'] <= BUDGET
                           or e.get('is_first_step', False))
        non_first = [e for e in self._entries if not e.get('is_first_step', False)]

        summary = {
            "total_steps": total,
            "budget_pass_rate": round(budget_passes / total, 4) if total > 0 else 0,
            "avg_active": round(np.mean([e['n_active'] for e in non_first]), 1) if non_first else 0,
            "max_active": max(e['n_active'] for e in non_first) if non_first else 0,
            "DOD": "PASS" if budget_passes == total else "FAIL",
        }

        self._txt.write("\n" + "=" * 70 + "\n")
        self._txt.write("=== ActivePatches DoD SUMMARY ===\n")
        for k, v in summary.items():
            self._txt.write(f"  {k}: {v}\n")
        self._txt.write("=" * 70 + "\n")
        self._txt.flush()

        self._jsonl.write(json.dumps({"summary": summary}) + "\n")
        self._jsonl.flush()

        return summary

    def close(self):
        self._jsonl.close()
        self._txt.close()


# ===================================================================
# 8. Visualization
# ===================================================================

def save_active_patches_overlay(active_patches, patch_room, room_to_patches,
                                base_image, save_path,
                                chosen_frontier_centroid=None,
                                debug_info=None,
                                map_size=MAP_SIZE, grid_side=LLM_GRID_SIDE):
    """
    Save a visualization showing active patches overlaid on the map.

    Active patches are highlighted in green, inactive in dark.
    Delta patches (if available) are shown in yellow.

    Args:
        active_patches:  list of active patch indices
        patch_room:      np.array (1600,)
        room_to_patches: dict {room_id: [patch indices]}
        base_image:      480x480 BGR map image (sem_map_frontier, already flipud'd)
        save_path:       full path to save image
        chosen_frontier_centroid: (cx, cy) for marking
        debug_info:      dict with delta info
        map_size:        480
        grid_side:       40
    """
    cell = map_size / grid_side
    active_set = set(active_patches)

    # Start with dimmed base image
    if base_image is not None:
        overlay = (base_image.astype(np.float32) * 0.3).astype(np.uint8)
    else:
        overlay = np.zeros((map_size, map_size, 3), dtype=np.uint8)

    # Brighten active patches
    # patch_room is in raw (unflipped) space, but base_image (sem_map_frontier)
    # is flipud'd. Flip the row: image_row = (grid_side - 1 - row).
    for idx in active_set:
        row, col = divmod(idx, grid_side)
        flipped_row = grid_side - 1 - row
        y0 = int(flipped_row * cell)
        y1 = int((flipped_row + 1) * cell)
        x0 = int(col * cell)
        x1 = int((col + 1) * cell)
        y0, y1 = max(0, y0), min(map_size, y1)
        x0, x1 = max(0, x0), min(map_size, x1)

        if base_image is not None:
            overlay[y0:y1, x0:x1] = base_image[y0:y1, x0:x1]
        # Green tint for active
        green_layer = np.zeros((y1 - y0, x1 - x0, 3), dtype=np.uint8)
        green_layer[:, :, 1] = 40  # subtle green tint
        overlay[y0:y1, x0:x1] = cv2.add(overlay[y0:y1, x0:x1], green_layer)

    # Draw grid lines
    for i in range(grid_side + 1):
        pos = int(i * cell)
        pos = min(pos, map_size - 1)
        cv2.line(overlay, (pos, 0), (pos, map_size - 1), (60, 60, 60), 1)
        cv2.line(overlay, (0, pos), (map_size - 1, pos), (60, 60, 60), 1)

    # Mark chosen frontier centroid
    if chosen_frontier_centroid is not None:
        cx, cy = chosen_frontier_centroid
        # Apply flipud for drawing: image_row = map_size - cx
        draw_x = cy  # column -> x in cv2
        draw_y = map_size - cx  # flipped row -> y in cv2
        draw_x = max(0, min(map_size - 1, draw_x))
        draw_y = max(0, min(map_size - 1, draw_y))
        cv2.circle(overlay, (draw_x, draw_y), 8, (0, 255, 0), 2)
        cv2.putText(overlay, "F", (draw_x + 10, draw_y + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # Add text info
    n = len(active_patches)
    cv2.putText(overlay, f"Active: {n}/{BUDGET}",
                (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    if debug_info:
        rooms_text = f"Rooms: {debug_info.get('active_rooms', [])}"
        cv2.putText(overlay, rooms_text,
                    (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    cv2.imwrite(save_path, overlay)
