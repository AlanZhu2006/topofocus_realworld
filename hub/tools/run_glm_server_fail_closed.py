#!/usr/bin/env python3
"""Launch the immutable upstream GLM server with one fail-closed score patch.

The upstream server silently substitutes random scores when every requested
candidate token underflows to zero.  Its two-candidate fallback also assumes
the labels are always ``Yes``/``No``; a Decision request such as ``[A, C]``
can therefore receive Yes/No probabilities mislabeled as A/C.  Both cases
break the ABCD image/prompt/score/target contract.

This deployment launcher keeps ``source/`` byte-for-byte immutable.  It
checks the reviewed source SHA-256, replaces that one exact score block in
memory, combines each requested token's unspaced and leading-space forms,
and raises on zero/non-finite candidate mass.  Normal inference performs no
extra model call.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import types


WORKSPACE = Path(__file__).resolve().parents[2]
UPSTREAM_SERVER = (
    WORKSPACE
    / "source"
    / "Focus_realworld"
    / "CogVLM2"
    / "basic_demo"
    / "glm4_openai_api_demo_1gpu.py"
)
UPSTREAM_SHA256 = (
    "991bb7a288a69c36f2ad3999f63e5908d8044b9d6bc2adef71c8f432a58526f6"
)
PATCH_CONTRACT = "focus-glm-candidate-scores-fail-closed-v1"
MODEL_CARD_ID = "cogvlm2-19b-focus-score-contract-v1"

_UPSTREAM_SCORE_BLOCK = """\
            token_probs = torch.softmax(full_out_dict.scores[0][0], dim=0)
            torch.set_printoptions(profile="full")
            torch.set_printoptions(profile="default") # reset
            slice_idxs = torch.tensor([string2idx[s] for s in params['return_string_probabilities']])
            string_probs_unnormalized = token_probs[slice_idxs]
            if string_probs_unnormalized.sum() == 0.0 and len(slice_idxs) == 2:
                slice_idxs = torch.tensor([string2idx[s] for s in [" Yes", " No"]])
                string_probs_unnormalized = token_probs[slice_idxs]
            elif string_probs_unnormalized.sum() == 0.0 and len(slice_idxs) == 4:
                slice_idxs = torch.tensor([string2idx[s] for s in [" A", " B", " C", " D"]])
                string_probs_unnormalized = token_probs[slice_idxs]
            elif string_probs_unnormalized.sum() == 0.0 and len(slice_idxs) > 4:
                slice_idxs = torch.tensor([string2idx[s] for s in ALL_nums_])
                string_probs_unnormalized = token_probs[slice_idxs]


            if string_probs_unnormalized.sum() == 0.0:
                rand_list = [random.random() for _ in range(len(slice_idxs))]
                total_sum = sum(rand_list)
                gen_probabilities = [num/total_sum for num in rand_list]
            else:
                string_probs = string_probs_unnormalized / string_probs_unnormalized.sum()
                gen_probabilities = string_probs.to(torch.float).cpu().numpy().tolist()
"""

_FAIL_CLOSED_SCORE_BLOCK = """\
            token_probs = torch.softmax(full_out_dict.scores[0][0], dim=0)
            requested_candidates = params['return_string_probabilities']
            candidate_masses = []
            for candidate in requested_candidates:
                variants = [candidate]
                spaced_candidate = f" {candidate}"
                if spaced_candidate in string2idx:
                    variants.append(spaced_candidate)
                token_ids = sorted({string2idx[value] for value in variants})
                candidate_masses.append(token_probs[token_ids].sum())
            string_probs_unnormalized = torch.stack(candidate_masses)
            probability_mass = string_probs_unnormalized.sum()
            if (
                not bool(torch.isfinite(probability_mass).item())
                or float(probability_mass.item()) <= 0.0
            ):
                raise RuntimeError(
                    "VLM candidate token probability mass is zero/non-finite; "
                    "refusing upstream random or label-mismatched fallback"
                )
            string_probs = string_probs_unnormalized / probability_mass
            gen_probabilities = (
                string_probs.to(torch.float).cpu().numpy().tolist()
            )
"""


def patched_source_text(source: str) -> str:
    occurrences = source.count(_UPSTREAM_SCORE_BLOCK)
    if occurrences != 1:
        raise RuntimeError(
            "reviewed upstream GLM score block changed: "
            f"expected one occurrence, found {occurrences}"
        )
    patched = source.replace(
        _UPSTREAM_SCORE_BLOCK,
        _FAIL_CLOSED_SCORE_BLOCK,
        1,
    )
    model_card = '    model_card = ModelCard(id="cogvlm2-19b")'
    if patched.count(model_card) != 1:
        raise RuntimeError("reviewed upstream GLM model-card block changed")
    return patched.replace(
        model_card,
        f'    model_card = ModelCard(id="{MODEL_CARD_ID}")',
        1,
    )


def runtime_main_module(source_path: Path) -> types.ModuleType:
    """Create the real ``__main__`` module used by the patched upstream app.

    The upstream file enables postponed annotations and defines Pydantic
    models.  Executing it in a detached dictionary leaves those models with
    ``__module__ == "__main__"`` while ``sys.modules["__main__"]`` still
    points at this wrapper.  Pydantic then cannot resolve names such as
    ``Optional`` from the upstream namespace.  A registered module preserves
    normal Python execution semantics without writing a patched source file.
    """

    module = types.ModuleType("__main__")
    module.__file__ = str(source_path)
    module.__package__ = None
    module.__dict__["__builtins__"] = __builtins__
    return module


def main() -> int:
    source_bytes = UPSTREAM_SERVER.read_bytes()
    observed_sha256 = hashlib.sha256(source_bytes).hexdigest()
    if observed_sha256 != UPSTREAM_SHA256:
        raise RuntimeError(
            "immutable upstream GLM server checksum changed; review the "
            "deployment score patch before launching"
        )
    patched = patched_source_text(source_bytes.decode("utf-8"))
    compile(patched, str(UPSTREAM_SERVER), "exec")
    print(
        json.dumps(
            {
                "status": "launching_reviewed_upstream_with_runtime_patch",
                "source_path": str(UPSTREAM_SERVER),
                "source_size_bytes": len(source_bytes),
                "source_sha256": observed_sha256,
                "patch_contract": PATCH_CONTRACT,
                "model_card_id": MODEL_CARD_ID,
                "candidate_score_policy": (
                    "sum unspaced+leading-space token mass per exact "
                    "requested label; zero/non-finite mass raises"
                ),
            },
            sort_keys=True,
        ),
        flush=True,
    )
    sys.path.insert(0, str(UPSTREAM_SERVER.parent))
    sys.argv[0] = str(UPSTREAM_SERVER)
    runtime_module = runtime_main_module(UPSTREAM_SERVER)
    # The upstream uvicorn call blocks until shutdown, so this replacement
    # remains authoritative for the complete server lifetime.
    sys.modules["__main__"] = runtime_module
    exec(
        compile(patched, str(UPSTREAM_SERVER), "exec"),
        runtime_module.__dict__,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
