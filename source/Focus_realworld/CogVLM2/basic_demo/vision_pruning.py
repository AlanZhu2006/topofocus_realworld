"""
Step 7: Vision Token Pruning
==============================
Replaces 1600 ViT output tokens with [R room tokens] + [K active patches]
between ViT output and LLM input.  ViT always processes the full image;
pruning happens via a forward hook on model.transformer.vision.

Room tokens are mean-pooled from the active patches belonging to each room.
Active patch tokens are gathered directly from the ViT output.
"""

import torch
from loguru import logger
from typing import List, Dict, Optional

# ---------------------------------------------------------------------------
# Global pruning state — set before model.generate(), cleared after.
# ---------------------------------------------------------------------------

_pruning_state = {
    "active_patch_indices": None,   # list[int], K patch indices (0-1599)
    "room_patch_groups": None,      # dict[str, list[int]], room_id -> patch indices
    "num_pruned_patches": None,     # int, R + K
    "index_mapping": None,          # dict with R, K, active_to_grid mapping
}


def set_pruning_params(active_patch_indices: List[int],
                       room_patch_groups: Dict[str, List[int]]):
    """
    Set pruning parameters before model.generate().

    Args:
        active_patch_indices: sorted list of K active patch indices (0-1599)
        room_patch_groups:    dict mapping room_id (str) to list of patch
                              indices belonging to that room.  Only rooms
                              whose patches overlap active_patch_indices
                              contribute room tokens.
    """
    active_set = set(active_patch_indices)

    # Build room tokens only from rooms that have active patches
    room_groups = {}
    for rid, patches in room_patch_groups.items():
        overlap = [p for p in patches if p in active_set]
        if overlap:
            room_groups[rid] = overlap

    R = len(room_groups)
    K = len(active_patch_indices)

    # Build index mapping: position -> original grid index
    # Positions 0..R-1 are room tokens (no single grid index)
    # Positions R..R+K-1 are individual active patches
    active_to_grid = {i: idx for i, idx in enumerate(active_patch_indices)}

    _pruning_state["active_patch_indices"] = active_patch_indices
    _pruning_state["room_patch_groups"] = room_groups
    _pruning_state["num_pruned_patches"] = R + K
    _pruning_state["index_mapping"] = {
        "R": R,
        "K": K,
        "room_ids": list(room_groups.keys()),
        "room_patch_counts": {rid: len(ps) for rid, ps in room_groups.items()},
        "active_to_grid": active_to_grid,
    }

    logger.info(f"[VisionPruning] set: R={R} rooms, K={K} active, total={R+K}")


def clear_pruning_params():
    """Clear pruning state after model.generate()."""
    _pruning_state["active_patch_indices"] = None
    _pruning_state["room_patch_groups"] = None
    _pruning_state["num_pruned_patches"] = None
    _pruning_state["index_mapping"] = None


def get_pruning_state():
    """Return current pruning state (read-only access)."""
    return _pruning_state


# ---------------------------------------------------------------------------
# Core pruning function
# ---------------------------------------------------------------------------

def prune_vision_features(images_features: torch.Tensor) -> torch.Tensor:
    """
    Prune ViT output from [B, 1600, D] to [B, R+K, D].

    First R tokens: mean-pooled room representations.
    Next  K tokens: gathered individual active patch features.

    Args:
        images_features: [B, 1600, D] tensor from ViT

    Returns:
        [B, R+K, D] tensor
    """
    api = _pruning_state["active_patch_indices"]
    rpg = _pruning_state["room_patch_groups"]

    if api is None or rpg is None:
        return images_features  # no pruning

    B, N, D = images_features.shape
    device = images_features.device
    dtype = images_features.dtype

    pruned_parts = []

    # 1. Room tokens (mean-pooled)
    for rid in sorted(rpg.keys()):
        patch_indices = rpg[rid]
        idx_tensor = torch.tensor(patch_indices, device=device, dtype=torch.long)
        room_feats = images_features[:, idx_tensor, :]  # [B, n_patches, D]
        room_token = room_feats.mean(dim=1, keepdim=True)  # [B, 1, D]
        pruned_parts.append(room_token)

    # 2. Active patch tokens (gathered)
    active_idx = torch.tensor(api, device=device, dtype=torch.long)
    active_feats = images_features[:, active_idx, :]  # [B, K, D]
    pruned_parts.append(active_feats)

    pruned = torch.cat(pruned_parts, dim=1)  # [B, R+K, D]

    logger.info(f"[VisionPruning] pruned: {N} -> {pruned.shape[1]} "
                f"(R={len(rpg)}, K={len(api)})")

    return pruned


# ---------------------------------------------------------------------------
# Installation: hooks + monkey-patches
# ---------------------------------------------------------------------------

def install_pruning(model):
    """
    One-time setup at server startup.  Installs:
      1. A forward hook on model.transformer.vision to prune output
      2. Monkey-patch on model.transformer.forward for num_patches override
      3. Monkey-patch on model.prepare_inputs_for_generation for same

    Args:
        model: the ChatGLMForConditionalGeneration model instance
    """
    transformer = model.transformer

    # --- 1. Vision forward hook ---
    def vision_hook(module, input, output):
        """Post-forward hook: prune ViT output if pruning is active."""
        if _pruning_state["active_patch_indices"] is not None:
            return prune_vision_features(output)
        return output

    transformer.vision.register_forward_hook(vision_hook)
    logger.info("[VisionPruning] Registered vision forward hook")

    # --- 2. Monkey-patch transformer.forward ---
    original_forward = transformer.forward

    def patched_forward(self_transformer, **kwargs):
        # If pruning is active, we need to intercept the num_patches calculation
        # The original code computes:
        #   num_patches = (image_size // patch_size // 2) ** 2  = 1600
        # But after our vision hook, images_features has R+K tokens.
        # We patch this by temporarily modifying the vision config.
        if _pruning_state["num_pruned_patches"] is not None:
            # Save original values
            orig_image_size = self_transformer.config.vision_config['image_size']
            orig_patch_size = self_transformer.config.vision_config['patch_size']

            # We need: (image_size // patch_size // 2) ** 2 = num_pruned
            # Trick: set image_size and patch_size so the formula yields our value.
            # But this is fragile. Instead, let's directly monkey-patch the forward.
            pass

        return original_forward(**kwargs)

    # Actually, a cleaner approach: wrap the entire forward to fix up
    # the splicing after it happens.  But the simplest approach is to
    # directly patch the two places that use num_patches.
    #
    # Since the vision hook already changes the output shape, the
    # splicing in forward() will work correctly IF we override num_patches.
    # Let's do it by wrapping.

    _original_transformer_forward = transformer.forward.__func__ if hasattr(transformer.forward, '__func__') else transformer.forward

    def _patched_transformer_forward(
        self,
        input_ids=None,
        images=None,
        position_ids=None,
        attention_mask=None,
        full_attention_mask=None,
        past_key_values=None,
        inputs_embeds=None,
        use_cache=None,
        output_hidden_states=None,
        return_dict=None,
    ):
        """Patched forward that uses actual vision feature count instead of hardcoded 1600."""
        from transformers.modeling_outputs import BaseModelOutputWithPast

        if past_key_values is None:
            assert input_ids is not None and inputs_embeds is None

            def is_empty(images_list):
                if images_list is None or len(images_list) == 0:
                    return True
                for image_list in images_list:
                    if isinstance(image_list, torch.Tensor) and image_list.numel() > 0:
                        return False
                    if isinstance(image_list, list) and len(image_list) > 0:
                        return False
                return True

            if not is_empty(images):
                # Use actual pruned count if pruning is active
                if _pruning_state["num_pruned_patches"] is not None:
                    num_patches = _pruning_state["num_pruned_patches"]
                else:
                    image_size = self.config.vision_config['image_size']
                    patch_size = self.config.vision_config['patch_size']
                    num_patches = (image_size // patch_size // 2) ** 2

                assert len(input_ids) == len(images)
                inputs_embeds = self.embedding(input_ids)

                images = images.to(dtype=inputs_embeds.dtype)
                images_features = self.vision(images)
                # After our hook, images_features is [B, R+K, D] if pruning active

                if position_ids is None:
                    position_ids = self.get_position_ids(input_ids, device=inputs_embeds.device)
                new_input_embeds, new_position_ids = [], []

                for i in range(len(input_ids)):
                    input_id = input_ids[i].tolist()
                    boi_token_pos = input_id.index(self.config.boi_token_id)
                    eoi_token_pos = input_id.index(self.config.eoi_token_id)
                    assert eoi_token_pos - boi_token_pos == 2
                    new_input_embeds.append(torch.cat(
                        (inputs_embeds[i, :boi_token_pos],
                         images_features[i].to(inputs_embeds.device),
                         inputs_embeds[i, eoi_token_pos + 1:])))
                    new_position_ids.append(torch.cat(
                        (position_ids[i, :boi_token_pos + 1],
                         position_ids[i, boi_token_pos + 1].repeat(num_patches),
                         position_ids[i, eoi_token_pos:])
                    ))
                inputs_embeds = torch.stack(new_input_embeds, dim=0)
                position_ids = torch.stack(new_position_ids, dim=0)

        output_hidden_states = (
            output_hidden_states if output_hidden_states is not None else self.config.output_hidden_states
        )
        use_cache = use_cache if use_cache is not None else self.config.use_cache
        return_dict = return_dict if return_dict is not None else self.config.use_return_dict

        batch_size, seq_length = input_ids.shape

        if inputs_embeds is None:
            inputs_embeds = self.embedding(input_ids)

        if self.pre_seq_len is not None:
            if past_key_values is None:
                past_key_values = self.get_prompt(batch_size=batch_size, device=input_ids.device,
                                                  dtype=inputs_embeds.dtype)
            if attention_mask is not None:
                attention_mask = torch.cat([attention_mask.new_ones((batch_size, self.pre_seq_len)),
                                            attention_mask], dim=-1)

        if full_attention_mask is None:
            if (attention_mask is not None and not attention_mask.all()) or (past_key_values and seq_length != 1):
                if self.training:
                    # Use actual num_patches for attention mask construction
                    if _pruning_state["num_pruned_patches"] is not None:
                        num_patches_mask = _pruning_state["num_pruned_patches"]
                    else:
                        image_size = self.config.vision_config['image_size']
                        patch_size = self.config.vision_config['patch_size']
                        num_patches_mask = (image_size // patch_size // 2) ** 2

                    new_input_ids, new_attention_mask = [], []
                    for i in range(len(input_ids)):
                        input_id = input_ids[i].tolist()
                        boi_token_pos = input_id.index(self.config.boi_token_id)
                        eoi_token_pos = input_id.index(self.config.eoi_token_id)
                        assert eoi_token_pos - boi_token_pos == 2
                        new_attention_mask.append(torch.cat(
                            (attention_mask[i, :boi_token_pos + 1],
                             torch.ones(num_patches_mask).to(attention_mask.device),
                             attention_mask[i, eoi_token_pos:])))
                        new_input_ids.append(torch.cat(
                            (input_ids[i, :boi_token_pos + 1],
                             input_ids[i, -1].repeat(num_patches_mask),
                             input_ids[i, eoi_token_pos:])))
                    attention_mask = torch.stack(new_attention_mask, dim=0)
                    input_ids = torch.stack(new_input_ids, dim=0)
                    inputs_embeds = self.embedding(input_ids)

                full_attention_mask = self.get_masks(inputs_embeds, past_key_values, padding_mask=attention_mask)

        rotary_pos_emb = self.rotary_pos_emb(self.seq_length)
        if position_ids is not None:
            rotary_pos_emb = rotary_pos_emb[position_ids]
        else:
            rotary_pos_emb = rotary_pos_emb[None, :seq_length]

        hidden_states, presents, all_hidden_states, all_self_attentions = self.encoder(
            inputs_embeds, full_attention_mask, rotary_pos_emb=rotary_pos_emb,
            kv_caches=past_key_values, use_cache=use_cache, output_hidden_states=output_hidden_states
        )

        if not return_dict:
            return tuple(v for v in [hidden_states, presents, all_hidden_states, all_self_attentions] if v is not None)

        return BaseModelOutputWithPast(
            last_hidden_state=hidden_states,
            past_key_values=presents,
            hidden_states=all_hidden_states,
            attentions=all_self_attentions,
        )

    import types
    transformer.forward = types.MethodType(_patched_transformer_forward, transformer)
    logger.info("[VisionPruning] Monkey-patched transformer.forward")

    # --- 3. Monkey-patch prepare_inputs_for_generation ---
    _original_prepare = model.prepare_inputs_for_generation

    def _patched_prepare_inputs(
        input_ids,
        images=None,
        past_key_values=None,
        attention_mask=None,
        position_ids=None,
        use_cache=None,
        is_first_forward=True,
        **kwargs
    ):
        if position_ids is None:
            position_ids = model.get_position_ids(input_ids, device=input_ids.device)

        if attention_mask is not None:
            # Use actual pruned count if active
            if _pruning_state["num_pruned_patches"] is not None:
                num_patches = _pruning_state["num_pruned_patches"]
            else:
                image_size = model.config.vision_config['image_size']
                patch_size = model.config.vision_config['patch_size']
                num_patches = (image_size // patch_size // 2) ** 2

            new_attention_masks = []
            eoi_token_pos = 6
            boi_token_pos = 4

            def is_empty(images_list):
                if images_list is None or len(images_list) == 0:
                    return True
                for image_list in images_list:
                    if isinstance(image_list, torch.Tensor) and image_list.numel() > 0:
                        return False
                    if isinstance(image_list, list) and len(image_list) > 0:
                        return False
                return True

            for i in range(len(input_ids)):
                input_id = input_ids[i].tolist()
                if not is_empty(images):
                    boi_token_pos = input_id.index(model.config.boi_token_id)
                    eoi_token_pos = input_id.index(model.config.eoi_token_id)
                assert eoi_token_pos - boi_token_pos == 2
                new_attention_masks.append(torch.cat(
                    (attention_mask[i, :boi_token_pos + 1],
                     attention_mask.new_ones(num_patches),
                     attention_mask[i, eoi_token_pos:])
                ))
            attention_mask = torch.stack(new_attention_masks, dim=0)

        if not is_first_forward:
            if past_key_values is not None:
                position_ids = position_ids[..., -1:]
                input_ids = input_ids[:, -1:]

        return {
            "input_ids": input_ids,
            "images": images,
            "past_key_values": past_key_values,
            "position_ids": position_ids,
            "attention_mask": attention_mask,
            "return_last_logit": True,
            "use_cache": use_cache
        }

    model.prepare_inputs_for_generation = _patched_prepare_inputs
    logger.info("[VisionPruning] Monkey-patched prepare_inputs_for_generation")
    logger.info("[VisionPruning] Installation complete")
