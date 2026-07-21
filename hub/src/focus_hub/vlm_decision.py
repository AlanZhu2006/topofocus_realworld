"""The real 3-stage VLM decision cascade, over the local offline
OpenAI-compatible GLM-4V server.

Mirrors the upstream decision pattern exactly (see `vlm_prompts.py`'s
module docstring for the full story of why this exists): Perception VLM
("is this scene worth exploring?") -> Judgment VLM ("explore a new
frontier or revisit a historical point?") -> gate -> Decision VLM (pick a
lettered frontier). Each stage reads the local server's deterministic
string-probability extension (temperature 0, one token, softmax sliced to
the candidate strings) — this module owns the request/parse mechanics;
`vlm_prompts.py` owns the prompt text and pure parsing/scoring helpers.

If the VLM server is unreachable and the caller explicitly allowed a
fallback, the largest frontier is chosen and the result is labelled
accordingly — the selection source is always recorded, never implied.
"""
from __future__ import annotations

import base64
from dataclasses import dataclass, field

import cv2
import numpy as np

from .directional_memory import DirectionalMemory
from .frontiers import Frontier
from .vlm_prompts import (
    build_decision_prompt,
    build_judgment_prompt,
    build_perception_prompt,
    perception_weight_decision,
)


@dataclass(frozen=True)
class FrontierChoice:
    frontier: Frontier
    source: str                     # "glm-4v" or "largest-frontier-fallback"
    probabilities: dict[str, float]
    raw_content: str


def _call_glm(
    image_bgr: np.ndarray,
    prompt: str,
    candidates: list[str],
    *,
    base_url: str,
    timeout_s: float,
) -> tuple[dict[str, float], str]:
    """Shared request/parse mechanics for all three stages: encode the
    image, ask for `return_string_probabilities` over `candidates`, return
    (probabilities, raw_text_content).
    """
    import httpx

    ok, jpeg = cv2.imencode(".jpg", image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    if not ok:
        raise RuntimeError("failed to encode image")
    encoded = base64.b64encode(jpeg.tobytes()).decode("ascii")
    payload = {
        "model": "THUDM/glm-4v-9b",
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{encoded}"}},
                ],
                "return_string_probabilities": "[" + ", ".join(candidates) + "]",
            }
        ],
        "temperature": 0.0,
        "max_tokens": 1,
        "stream": False,
    }
    with httpx.Client(timeout=timeout_s) as client:
        response = client.post(f"{base_url}/chat/completions", json=payload)
        response.raise_for_status()
    message = response.json()["choices"][0]["message"]
    scores = message.get("scores")
    content = str(message.get("content", "")).strip()
    if scores and len(scores) == len(candidates):
        probabilities = {c: float(s) for c, s in zip(candidates, scores)}
    else:
        probabilities = {}
    return probabilities, content


# =============================================================================
# Stage 1: Perception VLM
# =============================================================================

def choose_scene_worth_exploring_glm(
    rgb_bgr: np.ndarray,
    *,
    target: str,
    detections: dict[str, float] | None,
    base_url: str,
    timeout_s: float = 120.0,
) -> tuple[float, float]:
    """Returns (P_yes, P_no) — "is the current scene worth exploring for
    `target`", combining the raw logits with the emitted text label via
    `perception_weight_decision` (matches upstream's `Perception_PR`
    exactly, including its "Neither" -> raw-logit fallback).
    """
    prompt = build_perception_prompt(target, detections)
    probabilities, content = _call_glm(rgb_bgr, prompt, ["Yes", "No"], base_url=base_url, timeout_s=timeout_s)
    if probabilities:
        rel = (probabilities.get("Yes", 0.0), probabilities.get("No", 0.0))
    else:
        rel = (0.5, 0.5)
    weighted = perception_weight_decision(rel, content)
    if weighted == "Neither":
        return rel
    return weighted


# =============================================================================
# Stage 2: Judgment VLM ("FN")
# =============================================================================

def choose_explore_or_revisit_glm(
    sem_map_bgr: np.ndarray,
    *,
    target: str,
    scene_objects: str,
    frontiers_results: str,
    history_results: str,
    cur_location: tuple[int, int],
    pre_goal_point,
    perception_pr_yes: float,
    base_url: str,
    timeout_s: float = 120.0,
) -> tuple[float, float]:
    """Returns (P_yes, P_no) — "explore a new frontier (Yes) or revisit a
    historical point (No)" — matches upstream's `FN_PR`.
    """
    prompt = build_judgment_prompt(
        target=target, scene_objects=scene_objects, frontiers_results=frontiers_results,
        history_results=history_results, cur_location=cur_location, pre_goal_point=pre_goal_point,
        perception_pr_yes=perception_pr_yes,
    )
    probabilities, content = _call_glm(sem_map_bgr, prompt, ["Yes", "No"], base_url=base_url, timeout_s=timeout_s)
    if probabilities:
        rel = (probabilities.get("Yes", 0.0), probabilities.get("No", 0.0))
    else:
        rel = (0.5, 0.5)
    weighted = perception_weight_decision(rel, content)
    if weighted == "Neither":
        return rel
    return weighted


# =============================================================================
# Stage 3: Decision VLM (frontier choice)
# =============================================================================

def choose_frontier_glm(
    bev_bgr: np.ndarray,
    frontiers: list[Frontier],
    *,
    base_url: str,
    goal_category: str,
    scene_objects: str = "",
    scene_information: str = "",
    cur_location: tuple[int, int] = (0, 0),
    pre_goal_point=None,
    timeout_s: float = 120.0,
) -> FrontierChoice:
    if not frontiers:
        raise ValueError("no frontier candidates to choose from")
    letters = [f.frontier_id for f in frontiers]
    frontiers_results = "".join(f"{f.frontier_id}: [Coordinates: ({f.row}, {f.col})]\n" for f in frontiers)
    prompt = build_decision_prompt(
        scene_information=scene_information, scene_objects=scene_objects,
        frontiers_results=frontiers_results, target=goal_category,
        cur_location=cur_location, pre_goal_point=pre_goal_point, valid_candidates=letters,
    )
    probabilities, content = _call_glm(bev_bgr, prompt, letters, base_url=base_url, timeout_s=timeout_s)
    if probabilities:
        chosen_letter = max(probabilities, key=probabilities.get)
    else:
        chosen_letter = content if content in letters else letters[0]
    chosen = next(f for f in frontiers if f.frontier_id == chosen_letter)
    return FrontierChoice(frontier=chosen, source="glm-4v", probabilities=probabilities, raw_content=content)


def choose_frontier_fallback(frontiers: list[Frontier]) -> FrontierChoice:
    if not frontiers:
        raise ValueError("no frontier candidates to choose from")
    chosen = max(frontiers, key=lambda f: f.size_cells)
    return FrontierChoice(frontier=chosen, source="largest-frontier-fallback", probabilities={}, raw_content="")


# =============================================================================
# Full cascade orchestration
# =============================================================================

@dataclass
class CascadeResult:
    perception_pr: tuple[float, float] | None = None
    judgment_pr: tuple[float, float] | None = None
    gate_passed: bool = False
    gate_reason: str = ""
    frontier_choice: FrontierChoice | None = None
    history_index: int | None = None
    errors: list[str] = field(default_factory=list)


def run_decision_cascade(
    *,
    rgb_bgr: np.ndarray,
    judgment_map_bgr: np.ndarray,
    decision_map_bgr: np.ndarray,
    frontiers: list[Frontier],
    target: str,
    detections: dict[str, float] | None,
    scene_objects: str,
    cur_location_rc: tuple[int, int],
    heading_deg: float,
    pre_goal_point,
    step: int,
    early_episode_step_threshold: int,
    memory: DirectionalMemory,
    base_url: str,
    timeout_s: float = 120.0,
) -> CascadeResult:
    """Full Perception -> Judgment -> gate -> Decision cascade, ported from
    the block in `main.py` immediately following the two VLM calls (the
    `angle_score`/`history_state` update, then
    `if FN_PR[0] >= 0.5 or agent[j].l_step <= 125:`).

    Not ported (explicitly, not silently): when the gate fails, upstream
    picks among historical observation points instead
    (`form_prompt_for_DecisionVLM_History` / `Single_Agent_Decision_Prompt_History`).
    This cascade just reports gate_passed=False and returns no frontier
    choice in that case — the caller should treat it the same as "no
    decision this cycle" (safe, fail-closed, but not the full upstream
    behavior for that branch).
    """
    result = CascadeResult()
    if not frontiers:
        result.gate_reason = "no frontier candidates"
        return result

    try:
        result.perception_pr = choose_scene_worth_exploring_glm(
            rgb_bgr, target=target, detections=detections, base_url=base_url, timeout_s=timeout_s)
    except Exception as exc:  # noqa: BLE001 - one bad VLM call must not kill the cycle
        result.errors.append(f"perception: {exc}")
        result.perception_pr = (0.5, 0.5)

    history_letters = [chr(ord("a") + i) for i in range(26)] + [chr(ord("A") + i) for i in range(26)]
    history_results = "".join(
        f"{history_letters[i]}: [Coordinates: {node}]\n" for i, node in enumerate(memory.history_nodes)
    ) or "No historical observation points"
    frontiers_results = "".join(f"{f.frontier_id}: [Coordinates: ({f.row}, {f.col})]\n" for f in frontiers)

    try:
        result.judgment_pr = choose_explore_or_revisit_glm(
            judgment_map_bgr, target=target, scene_objects=scene_objects,
            frontiers_results=frontiers_results, history_results=history_results,
            cur_location=cur_location_rc, pre_goal_point=pre_goal_point,
            perception_pr_yes=result.perception_pr[0], base_url=base_url, timeout_s=timeout_s)
    except Exception as exc:  # noqa: BLE001
        result.errors.append(f"judgment: {exc}")
        result.judgment_pr = (0.5, 0.5)

    angle_score = result.perception_pr[0] * 2 + result.judgment_pr[0]
    result.history_index = memory.update(cur_location_rc, heading_deg, angle_score)

    result.gate_passed = result.judgment_pr[0] >= 0.5 or step <= early_episode_step_threshold
    result.gate_reason = (
        f"judgment_pr_yes={result.judgment_pr[0]:.3f}>=0.5" if result.judgment_pr[0] >= 0.5
        else (f"early episode (step {step}<={early_episode_step_threshold})" if result.gate_passed
              else f"gated: judgment_pr_yes={result.judgment_pr[0]:.3f}<0.5 and step {step} past early window")
    )
    if not result.gate_passed:
        return result

    try:
        result.frontier_choice = choose_frontier_glm(
            decision_map_bgr, frontiers, base_url=base_url, goal_category=target,
            scene_objects=scene_objects, cur_location=cur_location_rc,
            pre_goal_point=pre_goal_point, timeout_s=timeout_s)
    except Exception as exc:  # noqa: BLE001
        result.errors.append(f"decision: {exc}")
    return result
