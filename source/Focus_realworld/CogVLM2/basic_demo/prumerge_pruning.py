"""
LLaVA-PruMerge baseline (ablation) — adapted to GLM-4V / EVA2-CLIP
==================================================================
Training-free visual-token reduction from LLaVA-PruMerge
(https://arxiv.org/abs/2403.15388), ported to GLM-4V-9B.

PruMerge's original code targets a HuggingFace ``CLIPVisionModel`` whose patch
features feed the LLM directly. GLM-4V differs in two ways:
  * its vision tower (``EVA2CLIPModel``) uses a *fused* ``query_key_value``
    linear + SDPA (no exposed attention), and
  * a 2x2 stride-2 Conv2d + GLU projector sits between the ViT patches (6400 on
    an 80x80 grid) and the 1600 tokens the LLM consumes.
PruMerge's native insertion point (CLIP patches -> LLM) therefore does not
exist verbatim. We keep PruMerge's three defining elements and apply them at
the point that maps cleanly onto GLM-4V:

  1. CLS->patch attention:  capture the fused QKV of a late vision layer and
     recompute the [CLS] token's attention over the 6400 patches.
  2. Grid alignment:        the conv is non-overlapping 2x2, so each of the
     1600 LLM tokens corresponds *exactly* to one 2x2 block of the 80x80 patch
     grid -> 2x2 average-pool the CLS attention (and the keys) to 1600.
  3. Adaptive IQR count:    outlier detection on the pooled attention picks how
     many tokens to keep (PruMerge's headline feature; data-dependent).
  4. topk select + merge:   keep the top tokens, then merge the rest into them
     by cosine similarity of (pooled) keys, weighted by attention — exactly
     PruMerge's recipe — plus PruMerge's single mean "extra" token.

This operates on the vision tower OUTPUT (the [boi, 1600 patches, eoi] tensor),
reusing the same splice path the repo already uses for vision pruning. The only
monkey-patch is a *dynamic-count* ``ChatGLMModel.forward`` (the stock one
hard-codes 1600), kept entirely in this file.

Inert unless ``PRUNE_METHOD=prumerge``. Env knobs:
  PRUMERGE_VARIANT   base | plus     (default base)
  PRUMERGE_IQR_SCALE float           (default 1.0; scales the kept ratio)
  PRUMERGE_MERGE_K   int             (default 32; neighbours merged per token)
  PRUMERGE_LAYER     int             (default -1; vision layer for CLS attn)
  PRUMERGE_MIN_KEEP  int             (default 320; safety floor for kept tokens)
"""

import os

import numpy as np
import torch
import torch.nn.functional as F
from loguru import logger

try:
    from transformers.modeling_outputs import BaseModelOutputWithPast
except Exception:  # pragma: no cover
    BaseModelOutputWithPast = None


_state = {
    "enabled": False,
    "layer": -1,            # vision layer index used for CLS attention
    "iqr_scale": 1.0,       # multiplies the IQR-derived keep ratio
    "merge_k": 32,          # neighbours merged into each kept token
    "variant": "base",      # "base" | "plus"
    "min_keep": 320,       # conservative floor to avoid over-pruning collapse
    "captured_qkv": None,   # fused QKV of the chosen vision layer (prefill)
}


def prumerge_enabled() -> bool:
    return bool(_state["enabled"])


def prumerge_status() -> str:
    if not _state["enabled"]:
        return "PruMerge: DISABLED"
    return (f"PruMerge: ENABLED (variant={_state['variant']}, "
            f"iqr_scale={_state['iqr_scale']}, merge_k={_state['merge_k']}, "
            f"layer={_state['layer']})")


def configure_prumerge_from_env():
    method = os.environ.get("PRUNE_METHOD", "").strip().lower()
    if method != "prumerge":
        _state["enabled"] = False
        logger.info("[PruMerge] PRUNE_METHOD != 'prumerge' -> disabled")
        return
    _state["enabled"] = True
    _state["variant"] = os.environ.get("PRUMERGE_VARIANT", "base").strip().lower()
    _state["iqr_scale"] = float(os.environ.get("PRUMERGE_IQR_SCALE", "1.0"))
    _state["merge_k"] = int(os.environ.get("PRUMERGE_MERGE_K", "32"))
    _state["layer"] = int(os.environ.get("PRUMERGE_LAYER", "-1"))
    _state["min_keep"] = int(os.environ.get("PRUMERGE_MIN_KEEP", "320"))
    logger.info(f"[PruMerge] {prumerge_status()}")


# ---------------------------------------------------------------------------
# PruMerge selection helpers
# ---------------------------------------------------------------------------

def _outlier_ratio(attn_1d: torch.Tensor) -> float:
    """PruMerge IQR outlier detection -> fraction of tokens to keep."""
    a = attn_1d.to(torch.float32).cpu().numpy().flatten()
    q1, q3 = np.percentile(a, 25), np.percentile(a, 75)
    upper = q3 + 1.5 * (q3 - q1)
    n_out = int(np.count_nonzero(a > upper))
    return n_out / max(1, len(a))


def _prumerge_select_merge(patch_feats, pooled_attn, pooled_keys):
    """Run PruMerge on a single image.

    Args:
        patch_feats: [N, D]  the N=1600 vision-output patch tokens (LLM dim)
        pooled_attn: [N]     CLS attention per token (pooled to N)
        pooled_keys: [N, dk] per-token keys (pooled to N) for merge similarity
    Returns:
        [m_kept, D] merged patch tokens (m_kept = m + 1 extra token)
    """
    N, D = patch_feats.shape
    device = patch_feats.device

    ratio = _outlier_ratio(pooled_attn) * _state["iqr_scale"]
    m = int(N * ratio)
    min_keep = max(1, min(int(_state["min_keep"]), N - 1))
    m = max(min_keep, min(m, N - 1))

    imp = pooled_attn.to(torch.float32)
    topk_idx = torch.topk(imp, m).indices                       # [m]
    keep_mask = torch.zeros(N, dtype=torch.bool, device=device)
    keep_mask[topk_idx] = True
    compl_idx = torch.arange(N, device=device)[~keep_mask]      # [N-m]

    keys = F.normalize(pooled_keys.to(torch.float32), dim=-1)   # [N, dk]
    feats = patch_feats.to(torch.float32)

    # Merge: for each kept token, pull in its merge_k nearest tokens (by key
    # cosine sim), weighted by attention, and add them in (PruMerge eq.).
    keep_keys = keys[topk_idx]                                  # [m, dk]
    cos = keep_keys @ keys.t()                                  # [m, N]
    cos[torch.arange(m, device=device), topk_idx] = -1e9        # exclude self
    kk = int(min(_state["merge_k"], N - 1))
    nbr_idx = torch.topk(cos, kk, dim=1).indices                # [m, kk]
    nbr_feats = feats[nbr_idx]                                  # [m, kk, D]
    nbr_w = imp[nbr_idx].unsqueeze(-1)                          # [m, kk, 1]
    nbr_w = nbr_w / nbr_w.sum(dim=1, keepdim=True).clamp_min(1e-6)
    merged = feats[topk_idx] + (nbr_feats * nbr_w).sum(dim=1)   # [m, D]

    # PruMerge appends one extra token: attention-weighted sum of pruned tokens.
    if compl_idx.numel() > 0:
        non_w = imp[compl_idx].unsqueeze(-1)                    # [N-m, 1]
        extra = (feats[compl_idx] * non_w).sum(dim=0, keepdim=True)
        extra = extra / non_w.sum().clamp_min(1e-6)             # [1, D]
        merged = torch.cat([merged, extra], dim=0)             # [m+1, D]

    # PruMerge+: supplement with uniformly spaced (spatial) tokens.
    if _state["variant"] == "plus" and ratio > 0:
        step = max(1, int(1.0 / ratio) // 3)
        spatial = torch.arange(0, N, step, device=device)
        extra_mask = torch.zeros(N, dtype=torch.bool, device=device)
        extra_mask[spatial] = True
        extra_mask &= ~keep_mask                                # avoid dups
        extra_idx = torch.arange(N, device=device)[extra_mask]
        if extra_idx.numel() > 0:
            merged = torch.cat([merged, feats[extra_idx]], dim=0)

    # Safety: never hand NaN/Inf vision features to the LLM.
    merged = torch.nan_to_num(merged, nan=0.0, posinf=0.0, neginf=0.0)
    return merged.to(patch_feats.dtype)


# ---------------------------------------------------------------------------
# Hooks
# ---------------------------------------------------------------------------

def _qkv_capture_hook(module, inp, out):
    """Forward hook on a vision layer's fused query_key_value: stash output."""
    if not _state["enabled"]:
        return
    _state["captured_qkv"] = out.detach()


def _vision_output_hook(module, inp, out):
    """Forward hook on EVA2CLIPModel: replace [boi,1600,eoi] with PruMerge set."""
    if not _state["enabled"]:
        return out
    qkv = _state["captured_qkv"]
    if qkv is None:
        return out

    if out.dim() != 3 or out.shape[1] < 3:
        return out
    B = out.shape[0]
    if B != 1:
        logger.warning(f"[PruMerge] batch={B} unsupported, skipping prune")
        _state["captured_qkv"] = None
        return out

    # Fail-open: any error in selection/merge -> log + return UNPRUNED output.
    # This guarantees PruMerge can never 500 the server; worst case the call
    # runs with full vision tokens. The traceback is logged for diagnosis.
    try:
        boi = out[:, 0:1, :]
        eoi = out[:, -1:, :]
        patch_feats = out[:, 1:-1, :]                  # [B, 1600, D]
        n_out = patch_feats.shape[1]

        # ViT internal QKV -> CLS attention + keys over the patches.
        # PruMerge computes the CLS attention from the FULL q/k vectors (it does
        # NOT split into heads) and scales by the full hidden dim. We match that
        # exactly: slice the fused QKV into full-dim q, k.
        Bv, Lv, three_h = qkv.shape
        hid = three_h // 3
        q = qkv[:, :, :hid]                              # [B, Lv, hid]
        k = qkv[:, :, hid:2 * hid]                       # [B, Lv, hid]

        scale = hid ** -0.5
        cls_scores = (q[:, 0:1, :] @ k.transpose(-1, -2)) * scale        # [B, 1, Lv]
        cls_attn = F.softmax(cls_scores.float(), dim=-1)[:, 0, 1:]       # [B, n_patch]
        n_patch = cls_attn.shape[1]
        g = int(round(n_patch ** 0.5))

        # The 2x2 stride-2 conv maps each output token to one 2x2 patch block.
        if g * g != n_patch or (g // 2) * (g // 2) != n_out:
            logger.warning(f"[PruMerge] grid mismatch (g={g}, n_patch={n_patch}, "
                           f"n_out={n_out}); skipping prune")
            _state["captured_qkv"] = None
            return out

        cls_map = cls_attn.reshape(B, 1, g, g)
        pooled_attn = F.avg_pool2d(cls_map, 2).reshape(B, -1)            # [B, n_out]

        keys_patch = k[:, 1:, :].float()                                # [B, n_patch, hid]
        kg = keys_patch.reshape(B, g, g, hid).permute(0, 3, 1, 2)       # [B, hid, g, g]
        pooled_keys = F.avg_pool2d(kg, 2).reshape(B, hid, -1).transpose(1, 2)  # [B, n_out, hid]

        merged = _prumerge_select_merge(patch_feats[0], pooled_attn[0], pooled_keys[0])
        new_out = torch.cat([boi, merged.unsqueeze(0), eoi], dim=1)      # [1, m+2(+), D]

        _state["captured_qkv"] = None
        logger.info(f"[PruMerge] {n_out} -> {merged.shape[0]} patch tokens "
                    f"(variant={_state['variant']})")
        return new_out
    except Exception:
        logger.exception("[PruMerge] prune/merge FAILED — returning unpruned output")
        _state["captured_qkv"] = None
        return out


# ---------------------------------------------------------------------------
# Dynamic-count ChatGLMModel.forward (stock one hard-codes num_patches=1600)
# ---------------------------------------------------------------------------

def _is_empty(images_list):
    if images_list is None or len(images_list) == 0:
        return True
    for image_list in images_list:
        if isinstance(image_list, torch.Tensor) and image_list.numel() > 0:
            return False
        if isinstance(image_list, list) and len(image_list) > 0:
            return False
    return True


def _make_dynamic_transformer_forward(transformer):

    def forward(self, input_ids=None, images=None, position_ids=None,
                attention_mask=None, full_attention_mask=None,
                past_key_values=None, inputs_embeds=None, use_cache=None,
                output_hidden_states=None, return_dict=None):
        if past_key_values is None:
            assert input_ids is not None and inputs_embeds is None
            if not _is_empty(images):
                assert len(input_ids) == len(images)
                inputs_embeds = self.embedding(input_ids)
                images = images.to(dtype=inputs_embeds.dtype)
                images_features = self.vision(images)
                # PruMerge hook may have changed the token count; derive it.
                num_patches = images_features.shape[1] - 2

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
                         position_ids[i, eoi_token_pos:])))
                inputs_embeds = torch.stack(new_input_embeds, dim=0)
                position_ids = torch.stack(new_position_ids, dim=0)

        output_hidden_states = (
            output_hidden_states if output_hidden_states is not None
            else self.config.output_hidden_states)
        use_cache = use_cache if use_cache is not None else self.config.use_cache
        return_dict = return_dict if return_dict is not None else self.config.use_return_dict

        batch_size, seq_length = input_ids.shape
        if inputs_embeds is None:
            inputs_embeds = self.embedding(input_ids)

        if self.pre_seq_len is not None:
            if past_key_values is None:
                past_key_values = self.get_prompt(batch_size=batch_size,
                                                  device=input_ids.device,
                                                  dtype=inputs_embeds.dtype)
            if attention_mask is not None:
                attention_mask = torch.cat(
                    [attention_mask.new_ones((batch_size, self.pre_seq_len)),
                     attention_mask], dim=-1)

        if full_attention_mask is None:
            if (attention_mask is not None and not attention_mask.all()) or \
                    (past_key_values and seq_length != 1):
                full_attention_mask = self.get_masks(
                    inputs_embeds, past_key_values, padding_mask=attention_mask)

        rotary_pos_emb = self.rotary_pos_emb(self.seq_length)
        if position_ids is not None:
            rotary_pos_emb = rotary_pos_emb[position_ids]
        else:
            rotary_pos_emb = rotary_pos_emb[None, :seq_length]

        hidden_states, presents, all_hidden_states, all_self_attentions = self.encoder(
            inputs_embeds, full_attention_mask, rotary_pos_emb=rotary_pos_emb,
            kv_caches=past_key_values, use_cache=use_cache,
            output_hidden_states=output_hidden_states)

        if not return_dict:
            return tuple(v for v in [hidden_states, presents, all_hidden_states,
                                     all_self_attentions] if v is not None)

        return BaseModelOutputWithPast(
            last_hidden_state=hidden_states,
            past_key_values=presents,
            hidden_states=all_hidden_states,
            attentions=all_self_attentions,
        )

    return forward


def install_prumerge(model):
    """Install PruMerge hooks/patch. Safe to call unconditionally (inert if off)."""
    if not _state["enabled"]:
        logger.info("[PruMerge] install_prumerge: disabled, nothing installed")
        return
    if BaseModelOutputWithPast is None:
        raise RuntimeError("[PruMerge] could not import BaseModelOutputWithPast")

    transformer = model.transformer            # ChatGLMModel
    vision = transformer.vision                # EVA2CLIPModel
    vis_layers = vision.transformer.layers
    L = _state["layer"] if _state["layer"] >= 0 else len(vis_layers) + _state["layer"]
    if not (0 <= L < len(vis_layers)):
        raise ValueError(f"[PruMerge] vision layer {L} out of range "
                         f"(0..{len(vis_layers)-1})")

    attn = vis_layers[L].attention
    attn.query_key_value.register_forward_hook(_qkv_capture_hook)
    logger.info(f"[PruMerge] QKV capture hook on vision layer {L}")

    # Clear stale capture before each vision forward; this avoids accidentally
    # reusing a previous request's QKV if a capture hook is skipped.
    def _clear_capture_pre_hook(*_):
        _state["captured_qkv"] = None
    vision.register_forward_pre_hook(_clear_capture_pre_hook)
    vision.register_forward_hook(_vision_output_hook)
    logger.info("[PruMerge] output hook on EVA2CLIPModel")

    # Replace ChatGLMModel.forward with the dynamic-count version. (install
    # order: this runs after install_pruning, so it supersedes it; room
    # pruning is unused for this baseline.)
    import types
    transformer.forward = types.MethodType(
        _make_dynamic_transformer_forward(transformer), transformer)
    logger.info(f"[PruMerge] patched ChatGLMModel.forward — {prumerge_status()}")
