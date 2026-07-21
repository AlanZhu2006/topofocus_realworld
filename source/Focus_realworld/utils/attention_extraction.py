"""
Step 5: Attention Extraction — Client-side Metrics, Visualization, and DoD Logging
====================================================================================
Processes A_patch (1600-dim saliency vector) received from the VLM server.

Responsibilities:
  1. Verify A_patch sanity (sum≈1, all>=0)
  2. Compute entropy and compare to uniform
  3. Compute frontier hit-mass and compare to random baseline
  4. Save heatmap visualization as separate images
  5. Write dedicated DoD log files (separate from DecisionLogger)
"""

import os
import json
import math
import logging
import numpy as np
import cv2
from pathlib import Path
from datetime import datetime

# Must match the LLM token grid from patch_room_alignment.py
LLM_GRID_SIDE = 40
LLM_N_TOKENS = LLM_GRID_SIDE * LLM_GRID_SIDE  # 1600
MAP_SIZE = 480
CELL_SIZE = MAP_SIZE / LLM_GRID_SIDE  # 12.0




# ═══════════════════════════════════════════════════════════
# 1b. Reconstruct Full A_patch from Pruned (Step 7)
# ═══════════════════════════════════════════════════════════

def reconstruct_full_a_patch(a_patch_pruned, active_to_grid):
    """
    Map a K-length pruned attention vector back to a sparse 1600-length
    vector for visualization and DoD metrics.

    Args:
        a_patch_pruned: list of K floats (attention weights over active patches)
        active_to_grid: dict mapping position (int) -> original grid index (0-1599)

    Returns:
        list of 1600 floats (sparse, zeros for inactive patches)
    """
    full = np.zeros(LLM_N_TOKENS, dtype=np.float64)
    for pos, grid_idx in active_to_grid.items():
        pos_int = int(pos)
        if pos_int < len(a_patch_pruned):
            full[grid_idx] = a_patch_pruned[pos_int]

    # Re-normalize to sum=1 if any mass exists
    total = full.sum()
    if total > 0:
        full = full / total

    return full.tolist()

# ═══════════════════════════════════════════════════════════
# 1. A_patch Sanity Checks
# ═══════════════════════════════════════════════════════════

def verify_a_patch(a_patch):
    """
    Check basic invariants: sum≈1, all>=0, correct length.

    Returns:
        dict with keys: valid, length_ok, sum, sum_ok, all_nonneg
    """
    arr = np.array(a_patch, dtype=np.float64)
    length_ok = len(arr) == LLM_N_TOKENS
    s = float(arr.sum())
    sum_ok = abs(s - 1.0) < 1e-4
    all_nonneg = bool(np.all(arr >= 0))

    return {
        "valid": length_ok and sum_ok and all_nonneg,
        "length_ok": length_ok,
        "length": len(arr),
        "sum": round(s, 6),
        "sum_ok": sum_ok,
        "all_nonneg": all_nonneg,
    }


# ═══════════════════════════════════════════════════════════
# 2. Entropy
# ═══════════════════════════════════════════════════════════

def compute_entropy(a_patch):
    """
    Compute Shannon entropy of A_patch distribution.

    Returns:
        dict with keys: entropy, uniform_entropy, ratio
    """
    arr = np.array(a_patch, dtype=np.float64)
    arr = arr[arr > 0]  # only positive entries
    entropy = -float(np.sum(arr * np.log(arr)))
    uniform_entropy = math.log(LLM_N_TOKENS)  # log(1600) ≈ 7.38
    ratio = entropy / uniform_entropy if uniform_entropy > 0 else 0

    return {
        "entropy": round(entropy, 4),
        "uniform_entropy": round(uniform_entropy, 4),
        "ratio": round(ratio, 4),
        "pass": ratio < 0.85,  # entropy should be clearly below uniform
    }


# ═══════════════════════════════════════════════════════════
# 3. Frontier Hit-Mass
# ═══════════════════════════════════════════════════════════

def _frontier_centroid_to_patch(centroid_x, centroid_y, map_size=MAP_SIZE, grid_side=LLM_GRID_SIDE):
    """
    Convert frontier centroid (in unflipped map coords) to patch grid coords.

    The VLM sees a flipud'd image. So map row R becomes image row (map_size - R).
    The patch grid row = int(image_row / cell_size).

    Args:
        centroid_x: row in unflipped map space
        centroid_y: column in map space
    Returns:
        (patch_row, patch_col) in the 40x40 grid
    """
    # Apply flipud: image_row = map_size - centroid_x
    image_row = map_size - centroid_x
    image_col = centroid_y

    cell = map_size / grid_side
    patch_row = int(image_row / cell)
    patch_col = int(image_col / cell)

    # Clamp to grid
    patch_row = max(0, min(grid_side - 1, patch_row))
    patch_col = max(0, min(grid_side - 1, patch_col))

    return patch_row, patch_col


def compute_frontier_neighborhoods(frontiers_dict, radius=2,
                                    map_size=MAP_SIZE, grid_side=LLM_GRID_SIDE):
    """
    For each frontier, find the set of patch indices within a radius.

    Args:
        frontiers_dict: dict like {'frontier_0': '<centroid: (x, y), number: N>', ...}
        radius: neighborhood radius in patch cells
    Returns:
        dict {frontier_key: set of patch indices}
    """
    import re
    pattern = r'<centroid: (.*?), (.*?), number: (.*?)>'
    neighborhoods = {}

    for key, value in frontiers_dict.items():
        match = re.match(pattern, value)
        if not match:
            continue
        cx = int(match.group(1)[1:])
        cy = int(match.group(2)[:-1])

        pr, pc = _frontier_centroid_to_patch(cx, cy, map_size, grid_side)

        # Collect all patches within radius
        nbh = set()
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                r, c = pr + dr, pc + dc
                if 0 <= r < grid_side and 0 <= c < grid_side:
                    nbh.add(r * grid_side + c)
        neighborhoods[key] = nbh

    return neighborhoods


def compute_hit_mass(a_patch, neighborhoods):
    """
    Compute attention mass in each frontier's neighborhood.

    Returns:
        dict {frontier_key: float hit_mass}
    """
    arr = np.array(a_patch, dtype=np.float64)
    result = {}
    for key, nbh in neighborhoods.items():
        indices = list(nbh)
        result[key] = float(arr[indices].sum())
    return result


def compute_random_baseline(neighborhood_size, a_patch=None, n_trials=1000):
    """
    Expected hit-mass and std for random neighborhoods of given size.

    If a_patch is provided, sample random neighborhoods against the actual
    attention distribution (correct null hypothesis: "is the chosen frontier
    special, or would any random region have similar mass?").
    Falls back to uniform baseline if a_patch is None.
    """
    if a_patch is not None:
        arr = np.array(a_patch, dtype=np.float64)
    else:
        arr = np.ones(LLM_N_TOKENS) / LLM_N_TOKENS

    masses = []
    for _ in range(n_trials):
        random_indices = np.random.choice(LLM_N_TOKENS, neighborhood_size, replace=False)
        masses.append(float(arr[random_indices].sum()))
    mean = float(np.mean(masses))
    std = float(np.std(masses))

    return {"mean": round(mean, 6), "std": round(std, 6)}


def compute_dod_metrics(a_patch, frontiers_dict, chosen_frontier_idx, radius=2):
    """
    Full DoD evaluation for one step.

    Args:
        a_patch: list of 1600 floats
        frontiers_dict: frontier dict (same format as full_Frontiers_dict)
        chosen_frontier_idx: int index of chosen frontier (0-based)
        radius: neighborhood radius

    Returns:
        dict with all metrics for logging
    """
    sanity = verify_a_patch(a_patch)
    entropy_info = compute_entropy(a_patch)
    neighborhoods = compute_frontier_neighborhoods(frontiers_dict, radius=radius)

    hit_masses = compute_hit_mass(a_patch, neighborhoods)

    # Find chosen frontier key
    frontier_keys = sorted(frontiers_dict.keys())
    chosen_key = frontier_keys[chosen_frontier_idx] if chosen_frontier_idx < len(frontier_keys) else None

    chosen_hit_mass = hit_masses.get(chosen_key, 0.0) if chosen_key else 0.0

    # Random baseline for the chosen frontier's neighborhood size
    nbh_size = len(neighborhoods.get(chosen_key, set())) if chosen_key else 0
    baseline = compute_random_baseline(nbh_size, a_patch=a_patch) if nbh_size > 0 else {"mean": 0, "std": 0}

    threshold = baseline["mean"] + 2 * baseline["std"]
    hit_mass_pass = chosen_hit_mass > threshold

    return {
        "sanity": sanity,
        "entropy": entropy_info,
        "chosen_frontier": chosen_key,
        "chosen_hit_mass": round(chosen_hit_mass, 6),
        "all_hit_masses": {k: round(v, 6) for k, v in hit_masses.items()},
        "neighborhood_size": nbh_size,
        "random_baseline_mean": baseline["mean"],
        "random_baseline_std": baseline["std"],
        "threshold": round(threshold, 6),
        "hit_mass_pass": hit_mass_pass,
    }


# ═══════════════════════════════════════════════════════════
# 4. Visualization — Separate Heatmap Images
# ═══════════════════════════════════════════════════════════

def save_attention_heatmap(a_patch, frontiers_dict, chosen_frontier_idx,
                            base_image, save_path,
                            map_size=MAP_SIZE, grid_side=LLM_GRID_SIDE):
    """
    Save a heatmap overlay of A_patch on the map image.

    Args:
        a_patch:             list of 1600 floats
        frontiers_dict:      frontier dict
        chosen_frontier_idx: index of chosen frontier
        base_image:          the 480x480 sem_map_frontier BGR image (already flipud'd)
        save_path:           full path to save the image
    """
    import re

    arr = np.array(a_patch, dtype=np.float64).reshape(grid_side, grid_side)

    # Percentile-clipped normalization: clip at p2/p98 so the color
    # range covers the bulk of the variation instead of being dominated
    # by a few outlier peaks (which push everything else to blue).
    vmin = np.percentile(arr, 2)
    vmax = np.percentile(arr, 98)
    if vmax - vmin < 1e-12:          # degenerate case: fall back to min/max
        vmin, vmax = arr.min(), arr.max()
    arr_clipped = np.clip(arr, vmin, vmax)
    arr_norm = (arr_clipped - vmin) / (vmax - vmin + 1e-10)
    arr_uint8 = (arr_norm * 255).astype(np.uint8)

    # Upscale to map size — INTER_NEAREST keeps crisp per-patch squares
    # (each patch = 12x12 pixels).  INTER_LINEAR smears them into blobs.
    heatmap_small = cv2.applyColorMap(arr_uint8, cv2.COLORMAP_JET)
    heatmap = cv2.resize(heatmap_small, (map_size, map_size), interpolation=cv2.INTER_NEAREST)

    # Blend with base image
    if base_image is not None:
        overlay = cv2.addWeighted(base_image, 0.5, heatmap, 0.5, 0)
    else:
        overlay = heatmap

    # Draw frontier markers
    pattern = r'<centroid: (.*?), (.*?), number: (.*?)>'
    alpha_labels = [chr(65 + i) for i in range(26)]
    frontier_keys = sorted(frontiers_dict.keys())

    for fi, key in enumerate(frontier_keys):
        value = frontiers_dict[key]
        match = re.match(pattern, value)
        if not match:
            continue
        cx = int(match.group(1)[1:])
        cy = int(match.group(2)[:-1])

        # Apply same flipud transform as Decision_Generation_Vis
        def d240(x):
            if x < 240:
                return x + 2 * (240 - x)
            else:
                return x - 2 * (x - 240)

        draw_x = cy  # column = x in cv2
        draw_y = d240(cx)  # flipped row = y in cv2

        # Chosen frontier: green circle, others: white
        if fi == chosen_frontier_idx:
            color = (0, 255, 0)
            thickness = 2
        else:
            color = (255, 255, 255)
            thickness = 1

        cv2.circle(overlay, (draw_x, draw_y), 7, color, thickness)
        label = alpha_labels[fi] if fi < len(alpha_labels) else str(fi)
        cv2.putText(overlay, label, (draw_x + 8, draw_y + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    # Save
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    cv2.imwrite(save_path, overlay)


# ═══════════════════════════════════════════════════════════
# 5. Dedicated DoD Logger — Separate from DecisionLogger
# ═══════════════════════════════════════════════════════════

class AttentionDoDLogger:
    """
    Writes attention DoD metrics to dedicated log files,
    separate from the frontier decision logs.

    Output files:
      - {log_dir}/attention_dod/agent{id}.jsonl  (machine-readable)
      - {log_dir}/attention_dod/agent{id}.log    (human-readable)
    """

    def __init__(self, log_dir, agent_id=0):
        self.agent_id = agent_id
        self.dir = Path(log_dir) / "attention_dod"
        self.dir.mkdir(parents=True, exist_ok=True)

        self.jsonl_path = self.dir / f"agent{agent_id}.jsonl"
        self.txt_path = self.dir / f"agent{agent_id}.log"

        self._jsonl = open(self.jsonl_path, "a")
        self._txt = open(self.txt_path, "a")

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        header = f"[AttentionDoD] agent={agent_id} started at {ts}"
        self._txt.write(header + "\n" + "=" * 70 + "\n")
        self._txt.flush()

        self._entries = []

    def log(self, step, metrics):
        """
        Log one step's DoD metrics.

        Args:
            step: int, navigation step number
            metrics: dict from compute_dod_metrics()
        """
        entry = {"step": step, "agent_id": self.agent_id, **metrics}
        self._entries.append(entry)

        # JSONL
        self._jsonl.write(json.dumps(entry, default=str) + "\n")
        self._jsonl.flush()

        # Human-readable
        sanity = metrics["sanity"]
        entropy = metrics["entropy"]
        s_status = "OK" if sanity["valid"] else "FAIL"
        e_status = "OK" if entropy["pass"] else "FAIL"
        h_status = "OK" if metrics["hit_mass_pass"] else "FAIL"

        lines = [
            f"\n[Step {step}] Sanity:{s_status} Entropy:{e_status} HitMass:{h_status}",
            f"  A_patch: sum={sanity['sum']}, nonneg={sanity['all_nonneg']}, len={sanity['length']}",
            f"  Entropy: {entropy['entropy']:.3f} / {entropy['uniform_entropy']:.3f} (ratio={entropy['ratio']:.3f})",
            f"  Chosen: {metrics['chosen_frontier']} hit_mass={metrics['chosen_hit_mass']:.4f}",
            f"  Random baseline: mean={metrics['random_baseline_mean']:.4f} std={metrics['random_baseline_std']:.4f} threshold={metrics['threshold']:.4f}",
        ]
        self._txt.write("\n".join(lines) + "\n")
        self._txt.flush()

    def write_summary(self):
        """
        Write DoD summary at end of episode.
        """
        if not self._entries:
            return {"status": "NO_DATA"}

        total = len(self._entries)
        sanity_pass = sum(1 for e in self._entries if e["sanity"]["valid"])
        entropy_pass = sum(1 for e in self._entries if e["entropy"]["pass"])
        hit_mass_pass = sum(1 for e in self._entries if e["hit_mass_pass"])

        summary = {
            "total_steps": total,
            "sanity_pass_rate": round(sanity_pass / total, 4),
            "entropy_pass_rate": round(entropy_pass / total, 4),
            "hit_mass_pass_rate": round(hit_mass_pass / total, 4),
            "hit_mass_passes": hit_mass_pass,
            "DOD": "PASS" if hit_mass_pass / total > 0.5 else "FAIL",
        }

        self._txt.write("\n" + "=" * 70 + "\n")
        self._txt.write("=== DoD SUMMARY ===\n")
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
