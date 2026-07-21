import torch
import clip
import numpy as np
import logging
import os
from typing import Dict, List


class RoomSemantics:
    def __init__(self, device="cuda", clip_model="ViT-B/32"):
        self.device = device
        self.model, _ = clip.load(clip_model, device=device)
        self.model.eval()

    @torch.no_grad()
    def encode_text(self, text: str) -> torch.Tensor:
        tokens = clip.tokenize([text]).to(self.device)
        embedding = self.model.encode_text(tokens)
        embedding = embedding / embedding.norm(dim=-1, keepdim=True)
        return embedding

    def extract_objects_in_room(
        self,
        room_id: int,
        room_mask: np.ndarray,
        semantic_map: np.ndarray,
        object_categories: List[str],
        min_pixels: int = 10,
    ) -> List[str]:
        """Find which object categories have sufficient pixels inside a room."""
        room_pixels = (room_mask == room_id)

        objects = []
        for idx, cat_name in enumerate(object_categories):
            if idx >= len(semantic_map):
                continue
            pixels_in_room = semantic_map[idx][room_pixels].sum()
            if pixels_in_room > min_pixels:
                objects.append(cat_name)

        return objects

    def compute_room_semantics(
        self,
        room_mask: np.ndarray,
        room_info: Dict,
        full_map_pred: torch.Tensor,
        object_categories: List[str],
        target_object: str,
    ) -> Dict:
        """Compute objects and CLIP similarity score for every detected room.

        Returns:
            dict mapping room_id -> {'objects': [...], 'text': str, 'sim_room': float}
        """
        semantic_map = full_map_pred[4:]
        if isinstance(semantic_map, torch.Tensor):
            semantic_map = semantic_map.cpu().numpy()

        # Encode target description once
        target_text = f"a room that likely contains {target_object}"
        target_embedding = self.encode_text(target_text)

        room_semantics = {}

        for room_id in room_info.keys():
            objects = self.extract_objects_in_room(
                room_id, room_mask, semantic_map, object_categories
            )

            if len(objects) == 0:
                text = "an empty room"
            else:
                text = "room contains: " + ", ".join(sorted(objects))

            room_embedding = self.encode_text(text)

            sim_room = (room_embedding @ target_embedding.T).item()

            room_semantics[room_id] = {
                "objects": objects,
                "text": text,
                "sim_room": sim_room,
            }

        return room_semantics

    @staticmethod
    def save_room_semantics_log(
        room_semantics: Dict,
        target_object: str,
        episode_n: int,
        step: int,
        dump_dir: str,
    ):
        """Append room-semantic results to a per-episode text file."""
        episode_dir = os.path.join(dump_dir, "episodes", f"eps_{episode_n}")
        os.makedirs(episode_dir, exist_ok=True)
        log_path = os.path.join(episode_dir, "room_semantics.txt")

        sorted_ids = sorted(
            room_semantics.keys(),
            key=lambda r: room_semantics[r]["sim_room"],
            reverse=True,
        )

        lines = []
        lines.append(f"--- step {step}  target: {target_object} ---")
        for room_id in sorted_ids:
            info = room_semantics[room_id]
            objs = ", ".join(info["objects"]) if info["objects"] else "(none)"
            lines.append(
                f"  Room {room_id}: sim={info['sim_room']:.4f}  objects=[{objs}]"
            )
        lines.append("")

        with open(log_path, "a") as f:
            f.write("\n".join(lines))
