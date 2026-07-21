"""
Step 5: Attention Extraction — Server-side Hooks
=================================================
Hooks into the last 4 transformer layers of GLM-4V-9B to capture
attention weights from the decision token (the one generating the
frontier letter) to all 1600 vision tokens.

Hook path: model.transformer.encoder.layers[i].self_attention.core_attention

CoreAttention.forward() receives:
  query_layer  [B, 32, S, 128]   (32 query heads, already GQA-expanded)
  key_layer    [B, 32, S, 128]   (K expanded from 2 KV groups to 32)
  value_layer  [B, 32, S, 128]
  attention_mask

We register forward_pre_hooks to intercept Q and K before SDPA
(which doesn't return attention weights), then compute attention
weights manually for the last query position only.
"""

import math
import torch
import numpy as np
from loguru import logger


class AttentionCaptureHooks:
    """
    Captures attention weights from decision token → vision tokens
    during the prefill pass of model.generate().
    """

    def __init__(self, model, layer_indices=None, top_m_heads=4):
        """
        Args:
            model:         GLM-4V-9B model instance
            layer_indices: which transformer layers to hook (default: last 4)
            top_m_heads:   number of lowest-entropy heads to select
        """
        if layer_indices is None:
            layer_indices = [36, 37, 38, 39]

        self.layer_indices = layer_indices
        self.top_m_heads = top_m_heads
        self.model = model

        # State
        self._enabled = False
        self._image_start = 0
        self._image_count = 1600
        self._pruning_map = None  # Step 7: pruning index mapping
        self._captured = {}       # layer_idx -> (Q_last, K)
        self._hooks = []

        # Register hooks
        for li in self.layer_indices:
            layer = model.transformer.encoder.layers[li]
            core_attn = layer.self_attention.core_attention
            hook = core_attn.register_forward_pre_hook(self._make_hook(li))
            self._hooks.append(hook)

        logger.info(f"[AttentionHooks] Registered on layers {layer_indices}, top_m={top_m_heads}")

    def set_image_token_range(self, start, count=1600, pruning_index_mapping=None):
        """
        Set the position range of vision tokens in the sequence.

        Args:
            start: index of first vision token (= boi_token_pos in input_ids,
                   since boi itself is replaced by the first vision feature)
            count: number of vision tokens (1600 normally, R+K when pruned)
            pruning_index_mapping: dict with R, K, active_to_grid when pruning
                                   is active, or None for unpruned mode.
        """
        self._image_start = start
        self._image_count = count
        self._pruning_map = pruning_index_mapping

    def enable(self):
        """Enable capture for the next forward pass."""
        self._enabled = True
        self._captured = {}

    def disable(self):
        """Disable capture."""
        self._enabled = False
        self._captured = {}

    def _make_hook(self, layer_idx):
        """
        Create a forward_pre_hook closure for a specific layer.

        During prefill (Q.shape[2] > 1, meaning we're processing the full
        input sequence, not a single decode token), capture:
          - Q_last: the last query position [B, 32, 1, 128]
          - K: all key positions [B, 32, S, 128]
        """
        def hook_fn(module, args):
            if not self._enabled:
                return
            query_layer = args[0]  # [B, 32, S_q, 128]
            key_layer = args[1]    # [B, 32, S_k, 128]

            # Only capture during prefill (full sequence, not single-token decode)
            if query_layer.shape[2] <= 1:
                return

            # Already captured this layer for this pass
            if layer_idx in self._captured:
                return

            # Store only Q_last and K (detached, float32 for precision)
            q_last = query_layer[:, :, -1:, :].detach().float()  # [B, 32, 1, 128]
            k_full = key_layer.detach().float()                   # [B, 32, S, 128]
            self._captured[layer_idx] = (q_last, k_full)

        return hook_fn

    def compute_attention_weights(self):
        """
        After model.generate() completes, compute A_patch from captured Q, K.

        When pruning is active (self._pruning_map is set), the vision token
        count is R+K instead of 1600.  We extract attention over only the
        K active-patch positions (skipping the R room tokens at the start),
        returning a K-length list.

        Returns:
            list of floats (1600 when unpruned, K when pruned), or None.
        """
        if not self._captured:
            logger.warning("[AttentionHooks] No attention captured")
            return None

        captured_layers = sorted(self._captured.keys())
        if len(captured_layers) == 0:
            return None

        img_start = self._image_start
        img_end = img_start + self._image_count

        # Determine if pruned and where active patches start
        pruned = self._pruning_map is not None
        if pruned:
            R = self._pruning_map["R"]
            K = self._pruning_map["K"]
            # Vision tokens in sequence: [room_0..room_{R-1}, active_0..active_{K-1}]
            # We want attention over the K active positions only
            active_start = img_start + R
            active_end = img_start + R + K
            n_target = K
            logger.info(f"[AttentionHooks] Pruned mode: R={R}, K={K}, "
                        f"active_start={active_start}, active_end={active_end}")
        else:
            active_start = img_start
            active_end = img_end
            n_target = self._image_count

        all_head_attn = []

        for li in captured_layers:
            q_last, k_full = self._captured[li]
            seq_len = k_full.shape[2]
            if active_end > seq_len:
                logger.warning(
                    f"[AttentionHooks] Layer {li}: active_end={active_end} > seq_len={seq_len}, skipping"
                )
                continue

            d_k = q_last.shape[-1]  # 128

            # Slice keys for active patches only (skip room tokens if pruned)
            k_vis = k_full[:, :, active_start:active_end, :]  # [B, 32, n_target, 128]

            # Q_last @ K_vis^T / sqrt(d_k) -> [B, 32, 1, n_target]
            attn_scores = torch.matmul(q_last, k_vis.transpose(-2, -1)) / math.sqrt(d_k)
            attn_probs = torch.softmax(attn_scores, dim=-1)
            vis_attn = attn_probs[0, :, 0, :]  # [32, n_target]

            all_head_attn.append(vis_attn)

        if len(all_head_attn) == 0:
            return None

        stacked = torch.stack(all_head_attn, dim=0)
        n_layers, n_heads, n_tokens = stacked.shape
        flat = stacked.reshape(n_layers * n_heads, n_tokens)

        logger.info(
            f"[AttentionHooks] Averaging ALL {n_layers * n_heads} heads "
            f"(no entropy filter), n_tokens={n_tokens}"
        )

        a_patch = flat.mean(dim=0)

        logger.info(
            f"[AttentionHooks] A_patch stats: "
            f"min={a_patch.min():.8f} max={a_patch.max():.8f} "
            f"sum={a_patch.sum():.8f} peak_patch={int(a_patch.argmax())}"
        )

        total = a_patch.sum()
        if total > 0:
            a_patch = a_patch / total

        result = a_patch.cpu().numpy().tolist()
        self._captured = {}

        return result

    def remove_hooks(self):
        """Remove all registered hooks."""
        for h in self._hooks:
            h.remove()
        self._hooks = []
        logger.info("[AttentionHooks] All hooks removed")
