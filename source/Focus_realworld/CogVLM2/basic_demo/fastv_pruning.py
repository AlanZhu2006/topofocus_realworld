"""
FastV baseline (ablation) — ported to GLM-4V / ChatGLM
======================================================
This is a *self-contained* re-implementation of FastV
(https://arxiv.org/abs/2403.06764) for the GLM-4V-9B (ChatGLM) backbone.

FastV's original code lives inside a *forked* `transformers` and is wired into
`LlamaModel.forward` (LLaVA only).  GLM-4V is a different architecture, so we
copy the *algorithm* (not the package) onto the GLM decoder loop:

  * At decoder layer K (``agg_layer``), rank the image tokens by the average
    attention the LAST query token pays to them, keep the top-``budget`` and
    drop the rest for every layer >= K.
  * GLM-4V runs SDPA (no attention weights returned), so — exactly like the
    repo's existing ``attention_hooks.py`` — we capture Q/K via a pre-hook on
    one ``core_attention`` module and recompute the last-token -> image
    attention manually.

We use FastV's *attention-mask* variant (token length unchanged, dropped image
columns are masked out for layers >= K).  This is KV-cache / FlashAttention
safe and matches the methodology FastV uses for its accuracy ablations.

Everything here is **inert** unless ``PRUNE_METHOD=fastv`` is set in the
environment, so importing/installing it never changes default server behaviour.

Default configuration mirrors FastV's shipped demo config (k=3, r=0.75):
  FASTV_K=3, FASTV_RATIO=0.75  ->  keep 25% of the 1600 image tokens (=400).
"""

import os
import math

import torch
from loguru import logger

# GLM-4V begin-of-image token id (boi). The boi position is where the 1600
# image features start in the (embedding-expanded) sequence.
BOI_TOKEN_ID = 151339

# ---------------------------------------------------------------------------
# Global FastV state. Set once from env at startup; per-request fields
# (image_start / keep / drop / captured) are refreshed on every prefill.
# ---------------------------------------------------------------------------
_fastv_state = {
    "enabled": False,
    "agg_layer": 3,        # K: pruning applies for layers with index >= K
    "ratio": 0.75,         # FastV "r": fraction of image tokens to DROP
    "image_len": 1600,     # number of image tokens GLM-4V emits per image
    "budget": 400,         # number of image tokens to KEEP (= round(N*(1-r)))
    # per-request (refreshed each prefill)
    "image_start": None,   # boi position in the expanded sequence
    "keep": None,          # LongTensor of kept absolute positions
    "drop": None,          # LongTensor of dropped absolute positions
    "captured": None,      # (q_last, k_full) from layer K-1 during prefill
}

def fastv_enabled() -> bool:
    return bool(_fastv_state["enabled"])


def fastv_status() -> str:
    s = _fastv_state
    if not s["enabled"]:
        return "FastV: DISABLED"
    return (f"FastV: ENABLED (K={s['agg_layer']}, ratio={s['ratio']}, "
            f"keep={s['budget']}/{s['image_len']})")


def configure_fastv_from_env():
    """Read env vars and arm FastV if requested. Call once at startup."""
    method = os.environ.get("PRUNE_METHOD", "").strip().lower()
    if method != "fastv":
        _fastv_state["enabled"] = False
        logger.info("[FastV] PRUNE_METHOD != 'fastv' -> FastV disabled")
        return

    image_len = int(os.environ.get("FASTV_IMAGE_LEN", "1600"))
    k = int(os.environ.get("FASTV_K", "3"))
    ratio = float(os.environ.get("FASTV_RATIO", "0.75"))

    # budget = #tokens kept. FASTV_BUDGET overrides the ratio if provided.
    if os.environ.get("FASTV_BUDGET"):
        budget = int(os.environ["FASTV_BUDGET"])
    else:
        budget = int(round(image_len * (1.0 - ratio)))
    budget = max(1, min(budget, image_len))

    _fastv_state.update(
        enabled=True, agg_layer=max(1, k), ratio=ratio,
        image_len=image_len, budget=budget,
        image_start=None, keep=None, drop=None, captured=None,
    )
    logger.info(f"[FastV] {fastv_status()}")


# ---------------------------------------------------------------------------
# Token-ranking from captured attention (FastV's selection criterion)
# ---------------------------------------------------------------------------

def _compute_keep_drop(device):
    """From the captured layer-(K-1) attention, return (keep, drop) abs indices."""
    st = _fastv_state
    cap = st["captured"]
    if cap is None:
        return None, None

    q_last, k_full = cap                  # [B,H,1,hd], [B,H,S,hd]
    img_start = st["image_start"]
    img_len = st["image_len"]
    budget = st["budget"]

    q = q_last[0]                          # [H,1,hd]
    k = k_full[0]                          # [H,S,hd]
    if img_start is None or img_start + img_len > k.shape[1]:
        return None, None

    k_img = k[:, img_start:img_start + img_len, :]            # [H,img_len,hd]
    d_k = q.shape[-1]
    scores = torch.matmul(q, k_img.transpose(-1, -2)) / math.sqrt(d_k)  # [H,1,img_len]
    probs = torch.softmax(scores, dim=-1)
    avg = probs.mean(dim=0)[0]             # [img_len]  (mean over heads)

    keep_rel = torch.topk(avg, budget).indices
    keep_mask = torch.zeros(img_len, dtype=torch.bool, device=avg.device)
    keep_mask[keep_rel] = True

    all_rel = torch.arange(img_len, device=avg.device)
    keep_abs = (img_start + all_rel[keep_mask]).to(device).long()
    drop_abs = (img_start + all_rel[~keep_mask]).to(device).long()
    return keep_abs, drop_abs


def _build_fastv_mask(drop, bsz, q_len, k_len, device, is_prefill):
    """Boolean mask (True = blocked) shaped [B,1,q_len,k_len].

    GLM CoreAttention applies ``~mask`` before SDPA, so True must mean blocked.
    """
    if is_prefill:
        idx = torch.arange(q_len, device=device)
        # causal: block future keys (j > i)
        mask = idx[None, :] > idx[:, None]      # [q_len,k_len] bool
        mask[:, drop] = True                    # block dropped image columns
    else:
        mask = torch.zeros(q_len, k_len, dtype=torch.bool, device=device)
        mask[:, drop] = True
    return mask[None, None].expand(bsz, 1, q_len, k_len)


# ---------------------------------------------------------------------------
# Patched GLMTransformer.forward
# ---------------------------------------------------------------------------

def _make_patched_encoder_forward(encoder, genuine_forward):
    """Build the FastV-aware replacement for GLMTransformer.forward.

    ``genuine_forward`` is the original (bound) forward; ``encoder`` is the
    GLMTransformer module. We use a closure (not a bound method) so this works
    whether or not the model is wrapped by accelerate hooks.
    """

    def patched(hidden_states, attention_mask, rotary_pos_emb,
                kv_caches=None, use_cache=True, output_hidden_states=False):
        st = _fastv_state

        # Fall back to the original behaviour whenever FastV is not applicable.
        if (not st["enabled"]) or st["image_start"] is None \
                or st["budget"] >= st["image_len"] or encoder.training:
            return genuine_forward(
                hidden_states, attention_mask, rotary_pos_emb,
                kv_caches=kv_caches, use_cache=use_cache,
                output_hidden_states=output_hidden_states,
            )

        if not kv_caches:
            kv_caches = [None for _ in range(encoder.num_layers)]
        presents = () if use_cache else None
        all_self_attentions = None
        all_hidden_states = () if output_hidden_states else None

        K = st["agg_layer"]
        bsz, S = hidden_states.shape[0], hidden_states.shape[1]
        device = hidden_states.device
        is_prefill = S > 1

        if is_prefill:
            # New prefill -> recompute selection for this request.
            st["captured"] = None
            st["keep"] = None
            st["drop"] = None

        fastv_mask = None  # built lazily, reused across all layers >= K this call

        for index in range(encoder.num_layers):
            if output_hidden_states:
                all_hidden_states = all_hidden_states + (hidden_states,)

            layer = encoder._get_layer(index)

            if st["drop"] is not None and index >= K:
                if fastv_mask is None:
                    if is_prefill:
                        fastv_mask = _build_fastv_mask(st["drop"], bsz, S, S, device, True)
                    else:
                        past = kv_caches[index][0].shape[2] if kv_caches[index] is not None else 0
                        fastv_mask = _build_fastv_mask(st["drop"], bsz, 1, past + 1, device, False)
                cur_mask = fastv_mask
            else:
                cur_mask = attention_mask

            layer_ret = layer(
                hidden_states, cur_mask, rotary_pos_emb,
                kv_cache=kv_caches[index], use_cache=use_cache,
            )
            hidden_states, kv_cache = layer_ret
            if use_cache:
                presents = presents + (kv_cache,)

            # FastV ranks tokens using the attention of layer K-1, then prunes
            # from layer K onward. Derive keep/drop right after layer K-1.
            if is_prefill and index == (K - 1) and st["drop"] is None:
                keep, drop = _compute_keep_drop(device)
                if keep is not None and drop is not None and drop.numel() > 0:
                    st["keep"], st["drop"] = keep, drop

        if output_hidden_states:
            all_hidden_states = all_hidden_states + (hidden_states,)

        if encoder.post_layer_norm:
            hidden_states = encoder.final_layernorm(hidden_states)

        return hidden_states, presents, all_hidden_states, all_self_attentions

    return patched


# ---------------------------------------------------------------------------
# Hooks
# ---------------------------------------------------------------------------

def _make_capture_hook():
    """Pre-hook on layer (K-1) core_attention: stash Q_last and K during prefill."""
    def hook_fn(module, args):
        if not _fastv_state["enabled"]:
            return
        query_layer = args[0]   # [B,H,S,hd]
        key_layer = args[1]     # [B,H,S,hd]  (already GQA-expanded)
        if query_layer.shape[2] <= 1:
            return              # decode step, not prefill
        _fastv_state["captured"] = (
            query_layer[:, :, -1:, :].detach().float(),
            key_layer.detach().float(),
        )
    return hook_fn


def _transformer_pre_hook(module, args, kwargs):
    """Pre-hook on ChatGLMModel.forward: locate image start at each prefill."""
    if not _fastv_state["enabled"]:
        return
    input_ids = kwargs.get("input_ids", None)
    if input_ids is None and len(args) > 0:
        input_ids = args[0]
    past = kwargs.get("past_key_values", None)
    if input_ids is None or past is not None:
        return
    if input_ids.dim() != 2 or input_ids.shape[1] <= 1:
        return
    ids = input_ids[0].tolist()
    # Reset per-request state on every prefill; only arm if this call has an image.
    _fastv_state["keep"] = None
    _fastv_state["drop"] = None
    _fastv_state["captured"] = None
    _fastv_state["image_start"] = ids.index(BOI_TOKEN_ID) if BOI_TOKEN_ID in ids else None


def install_fastv(model):
    """Install FastV hooks/patch. Safe to call unconditionally (inert if disabled)."""
    if not _fastv_state["enabled"]:
        logger.info("[FastV] install_fastv: disabled, nothing installed")
        return

    transformer = model.transformer          # ChatGLMModel
    encoder = transformer.encoder             # GLMTransformer
    K = _fastv_state["agg_layer"]
    if K - 1 >= encoder.num_layers:
        raise ValueError(f"[FastV] agg_layer K={K} too large for "
                         f"{encoder.num_layers}-layer model")

    # 1) capture hook on layer (K-1) core_attention
    core_attn = encoder.layers[K - 1].self_attention.core_attention
    core_attn.register_forward_pre_hook(_make_capture_hook())
    logger.info(f"[FastV] capture hook on encoder.layers[{K-1}].core_attention")

    # 2) image-start detector on the ChatGLMModel
    try:
        transformer.register_forward_pre_hook(_transformer_pre_hook, with_kwargs=True)
    except TypeError:
        # very old torch without with_kwargs support
        def _legacy(mod, args):
            return _transformer_pre_hook(mod, args, {})
        transformer.register_forward_pre_hook(_legacy)
    logger.info("[FastV] image-start pre-hook on transformer")

    # 3) monkey-patch the encoder loop.
    #    Under accelerate (e.g. 4-bit load) the real forward is stashed in
    #    ``_old_forward`` while ``forward`` is a functools.partial wrapper that
    #    handles device alignment. Patch ``_old_forward`` in that case so the
    #    accelerate wrapper still runs; otherwise patch ``forward`` directly.
    if hasattr(encoder, "_old_forward"):
        attr = "_old_forward"
    else:
        attr = "forward"
    genuine_forward = getattr(encoder, attr)
    setattr(encoder, attr, _make_patched_encoder_forward(encoder, genuine_forward))
    logger.info(f"[FastV] patched GLMTransformer.{attr} — {fastv_status()}")
