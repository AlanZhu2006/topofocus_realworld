"""Upstream's real 3-stage VLM decision cascade, ported verbatim.

Source: `source/Focus_realworld/src/SystemPrompt.py` and
`source/Focus_realworld/src/frontier_parser.py` (both read-only, never
executed directly — this project always ports the algorithm/prompts and
re-implements the surrounding plumbing against real wire-protocol data,
exactly as already done for RedNet segmentation, frontier extraction, and
max-fusion).

Discovered 2026-07-19 while explaining the VLM side to the user: the hub's
first VLM port (`vlm_decision.py`, from the very early E2E-wiring phase)
only implemented the LAST of three VLM calls upstream actually makes per
agent per step:

  1. Perception VLM  — "is this scene worth exploring?" (Yes/No), given the
     raw RGB frame + real YOLO object-detection results.
  2. Judgment VLM ("FN") — "explore a new frontier or revisit a historical
     point?" (Yes/No), given the rendered semantic map with frontier points,
     historical observation points, and the robot's pose/heading marked.
  3. Decision VLM — pick a lettered frontier (what `vlm_decision.py`
     already did) — gated on stage 2's answer (or an early-episode override).

The executable ``main.py`` gate-fail branch does *not* invoke the History
Decision prompt that is also defined in ``SystemPrompt.py``. It assigns
``Final_PR = history_score_copy`` and takes the first maximum. The deployment
adapter follows that executed branch and keeps the unused prompt only as
source provenance, not as a fourth model call.

Verified directly against the original HPC source
(`ssh alantorch:/scratch/jl9356/Focus_realworld`, `running_inference.md` +
`run_cmd.txt`): the real baseline experiments run with real YOLOv10
detections (`--yolo yolov10`, default enabled, NOT a placeholder) and
without vision-token pruning (`--enable_pruning` is opt-in, off by default).
This module ports stages 1 and 2's prompts and the shared helpers; stage 3's
prompt lives in `vlm_decision.py` alongside the request/parse mechanics.
"""
from __future__ import annotations

import re
from dataclasses import dataclass

import cv2
import numpy as np

# =============================================================================
# Stage 1: Perception VLM — verbatim from SystemPrompt.py
# =============================================================================

PERCEPTION_SYSTEM_PROMPT = """
Based on your description, you need to determine whether the current scene is worth exploring for the robot, based on the provided information. Follow the guidelines below to make your decision clearly and explicitly.

**Information Format:**

- **Target of Navigation:** Format - <Name>
- **Scene Object (Object Detection):** Format - <Name, Confidence Score>

**Decision Criteria:**

1. **Examine the Relationship Between the Target Object and the Scene:**
   - If the object detection model predicts the presence of the target object with a high probability (e.g., >85%), determine if the scene is worth exploring.
2. **Analyze the Scene Context:**
   - Assess the context of the scene, including ceilings, walls, floors, or windows.
   - Use your knowledge of typical object locations to evaluate the likelihood of the target object being present (e.g., beds are usually found in bedrooms, TVs in living rooms).
3. **Considering Object Proximity and Context:**
   - Evaluate the proximity of the target object to the scene in the image.
   - Ensure your judgment does not violate the high probability criterion from point (1). For instance, a bathtub is unlikely in a bedroom, but the presence of a door could indicate the target object might be nearby.
4. **Disregard Generic Objects:**
   - Ignore objects commonly found in various rooms (like light switches and doors) as they do not provide strong evidence for the target object's presence.

------
**Output Format:**
 Your output should be a simple "[Yes, No]" statement indicating whether the scene is worth exploring based on the given criteria.


"""

_PERCEPTION_TEMPLATE = """
- **Target of navigation:** {TARGET}

- Scene object (Object Detection):
{OBJECT_DETECTION}

**Decision:**
"""


def build_perception_prompt(target: str, detections: dict[str, float] | None) -> str:
    """Ported from `form_prompt_for_PerceptionVLM`.

    ``detections``: {class_name: confidence}, from a real YOLO detector run
    on the same RGB frame. Empty/None renders "No Detections" — this is
    upstream's own defined fallback for zero detections, not something
    invented here, but real experiments run with real YOLO by default (see
    module docstring) so an empty dict here is a real degraded state, not
    the norm.
    """
    if not detections:
        object_detection = "No Detections"
    else:
        object_detection = "".join(f"  {name}, {conf}\n" for name, conf in detections.items())
    body = _PERCEPTION_TEMPLATE.format(TARGET=target, OBJECT_DETECTION=object_detection)
    return PERCEPTION_SYSTEM_PROMPT + body


# =============================================================================
# Stage 2: Judgment VLM ("FN") — verbatim from SystemPrompt.py
# =============================================================================

FN_SYSTEM_PROMPT = '''
You are a knowledgeable and skilled expert in indoor navigation planning. Based on the current top-down semantic map of an indoor environment provided, you will see various points marked as either 'Frontier Points' or 'Historical Observation Points'. Frontier points represent unexplored areas that the robot has yet to navigate, while Historical Observation Points signify areas the robot has previously explored or observed.

**Information Format:**
- **Your Navigation Target:** Format - `<Name>`
- **Scene Objects:** Format - `<Name: [Coordinates: <(x1, y1), (x2, y2)...>]>`
- **Frontier Points (The black dots and corresponding black uppercase letters on the image):** Format - `<Name: [Coordinates: <(x1, y1), (x2, y2)...>]>`
- **Historical Observation Points (The green dots and corresponding green lowercase letters on the image):** Format - `<Name: [Coordinates: <[x1, y1], [x2, y2]...>]>`
- **Your location (The red arrow):** Format - `<(x, y)>`
- **Previous Movement:** Format - `<(x, y)>`

Your goal is to guide the robot for exploration purposes based on the relationship between different objects, the structure of the explored area, the robot's position, the proximity of exploration points, and the direction of previous movement. Consider the following factors when making your decision:

1. **Level of Exploration:**
   - If there are a very small number of Historical Observation Points and a large number of gaps in the semantic map, prefer exploring Frontier Points.

2. **Explorability Worthiness:**
   - Scenes that are worth exploring ({ISWORTH}) are usually more likely to be explored by choosing Frontier Points.

3. **Proximity and Accessibility:**
   - Evaluate how Your location relates to surrounding obstacles. Frontier Points or Historical Observation Points that are close and free of obstacles tend to have higher exploration priority.

4. **Relationship Between Location and Previous Movement:**
   - If Your location is too close to Previous Movement, it may indicate a collision trap. In such cases, prefer to explore Historical Observation Points that are close to Your location.

**Decision Format:**
Your recommendation should be a simple "[Yes, No]" statement:
- **Yes:** Explore a Frontier Point.
- **No:** Revisit a Historical Observation Point.

**Example:**

---

**Your Navigation Target**: `TargetObject`

**Scene Objects**:
- `Chair: [Coordinates: (3, 4), (2, 3)]`
- `Table: [Coordinates: (5, 6), (6, 7)]`

**Frontier Points (The black dots and corresponding black uppercase letters on the image)**:
- `A: [Coordinates: (8, 9)]`
- `B: [Coordinates: (10, 11)]`

**Historical Observation Points (The green dots and corresponding green lowercase letters on the image)**:
- `a: [Coordinates: (1, 2)]`
- `b: [Coordinates: (3, 5)]`

**Your location (The red dot)**: `(4, 4)`

**Previous Movement**: `(2, 2)`


**Decision:** Yes

---

Now, begin your analysis with the provided scene image information.

- **Your Navigation Target:** `{TARGET}`
- **Scene Objects:** `{SCENE_OBJECTS}`
- **Frontier Points (The black dots and corresponding black uppercase letters on the image):** `{FRONTIERS_RESULTS}`
- **Historical Observation Points (The green dots and corresponding green lowercase letters on the image):** `{HISTORY_NODES}`
- **Your location (The red arrow):** `{CUR_LOCATION}`
- **Previous Movement:** `{PRE}`

Would you recommend:
Yes) Exploring a frontier point? If so, answer ONLY Yes.
No) Revisiting a historical observation point? If so, answer ONLY No.

---
'''


def build_judgment_prompt(
    *,
    target: str,
    scene_objects: str,
    frontiers_results: str,
    history_results: str,
    cur_location: tuple[int, int],
    pre_goal_point,
    perception_pr_yes: float,
) -> str:
    """Ported from `form_prompt_for_FN`."""
    pre = "No Movements" if not pre_goal_point else pre_goal_point
    isworth = ("The current scene is worth exploring." if perception_pr_yes >= 0.50
               else "The current scene is not worth exploring.")
    return FN_SYSTEM_PROMPT.format(
        SCENE_OBJECTS=scene_objects,
        TARGET=target,
        ISWORTH=isworth,
        FRONTIERS_RESULTS=frontiers_results,
        HISTORY_NODES=history_results,
        CUR_LOCATION=cur_location,
        PRE=pre,
    )


# =============================================================================
# Shared helpers — verbatim logic from SystemPrompt.py
# =============================================================================

def format_frontiers_for_prompt(frontier_rowcols: list[tuple[str, int, int]]) -> str:
    """``[(letter, row, col), ...]`` -> the `A: [Coordinates: (r, c)]\\n...` block."""
    return "".join(f"{letter}: [Coordinates: {(row, col)}]\n" for letter, row, col in frontier_rowcols)


def format_history_for_prompt(history_nodes: list[tuple[int, int]]) -> str:
    """Ported from the History-block-building loop in `form_prompt_for_FN`."""
    if not history_nodes:
        return "No historical observation points"
    letters = [chr(ord("a") + i) for i in range(26)] + [chr(ord("A") + i) for i in range(26)]
    return "".join(f"{letters[i]}: [Coordinates: {node}]\n" for i, node in enumerate(history_nodes))


def contains_yes_or_no(vlm_pred: str) -> str:
    """Ported from `contains_yes_or_no`."""
    if "Yes" in vlm_pred:
        return "Yes"
    if "No" in vlm_pred:
        return "No"
    return "Neither"


def perception_weight_decision(vlm_rel: tuple[float, float], vlm_pred: str) -> tuple[float, float] | str:
    """Ported verbatim from `Perception_weight_decision`.

    Combines the raw [P(Yes), P(No)] logits with which text label the model
    actually emitted, producing a self-consistent renormalized (P(Yes),
    P(No)) pair — or "Neither" if the model's text didn't say Yes or No.
    """
    b_decision = contains_yes_or_no(vlm_pred)
    if b_decision == "Neither":
        return b_decision
    x, y = vlm_rel
    if b_decision == "Yes":
        weighted_yes_prob = x
        weighted_no_prob = y * (1 - x)
    else:
        weighted_yes_prob = x * (1 - y)
        weighted_no_prob = y
    total = weighted_yes_prob + weighted_no_prob
    if total == 0:
        return b_decision
    return weighted_yes_prob / total, weighted_no_prob / total


# =============================================================================
# Scene-object extraction — ported from `Objects_Extract`, operating on our
# own grid's category channels (grid[2:2+len(HM3D_CATEGORY_NAMES)]) instead
# of upstream's differently-offset full_map_pred.
# =============================================================================

@dataclass(frozen=True)
class SceneObject:
    category: str
    polygon_rowcol: np.ndarray  # (N, 1, 2) int32, cv2.approxPolyDP output


def extract_scene_objects(
    category_grid: np.ndarray,
    category_names: tuple[str, ...],
    *,
    threshold: float = 0.1,
    min_contour_points: int = 30,
) -> dict[str, list[SceneObject]]:
    """Ported from `Objects_Extract` (main.py). One entry per category with
    at least one large-enough connected blob; each blob simplified to a
    polygon via the same close-then-contour-then-approxPolyDP pipeline.
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    result: dict[str, list[SceneObject]] = {}
    for i, name in enumerate(category_names):
        channel = category_grid[i]
        if channel.sum() == 0:
            continue
        mask = (channel > threshold).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            cv2.inRange(mask, 1, 1), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        objects = []
        for cnt in contours:
            if len(cnt) > min_contour_points:
                epsilon = 0.05 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                objects.append(SceneObject(category=name, polygon_rowcol=approx))
        if objects:
            result[name] = objects
    return result


def format_scene_objects_for_prompt(scene_objects: dict[str, list[SceneObject]]) -> str:
    """Ported from the `semantic_segmentation` string-building expression
    inlined in both `form_prompt_for_FN` and `form_prompt_for_DecisionVLM_Frontier`.
    """
    lines = []
    for name, objects in scene_objects.items():
        polys = ", ".join(
            "<" + ", ".join(f"{int(pt[0][0])}, {int(pt[0][1])}" for pt in obj.polygon_rowcol) + ">"
            for obj in objects
        )
        lines.append(f"{name}: {polys}")
    return "\n".join(lines) + "\n"


# =============================================================================
# Stage 3: Decision VLM — verbatim from SystemPrompt.py, then patched
# decision-first (see patch_frontier_prompt below).
# =============================================================================

SINGLE_AGENT_DECISION_PROMPT_FRONTIER1 = '''
You are a knowledgeable and skilled expert in indoor navigation planning. Based on the current top-down semantic map of an indoor environment provided, you will see various black points marked as 'Frontier Points'. Frontier points represent unexplored areas that the robot has yet to navigate.

**Information Format:**

- **Your Navigation Target:** <{TARGET}>
- **Scene Objects:** <{SCENE_OBJECTS}>
- **Frontier Points (The black dots and corresponding black uppercase letters on the image):** <{FRONTIERS_RESULTS}>
- **Your location (The red arrow):** <{CUR_LOCATION}>
- **Previous Movement:** <{PRE}>
- **Your location facing (the direction indicated by the red arrow) scene information:** <{SCENE_INFORMATION}>

Your task is to guide the robot for exploration purposes based on the relationship between different objects, the structure of the explored area, the robot's position, the proximity of the exploration point, and the direction of previous movement. Consider the following factors when making your decision:

1. **Proximity and Accessibility:**
   - Evaluate how Your location relates to surrounding obstacles. Frontier Points that are close and free of obstacles tend to have higher exploration priority.
2. **Relationship Between Location and Previous Movement:**
   - If Your location is too close to Previous Movement, it may indicate the robot is entering a collision trap. Prefer to explore Frontier Points that are farther from Your location.
3. **Exploration Consistency:**
   - Minimize frequent switches between Frontier Points. The robot should maintain its exploration direction unless an efficient switch is evident.
4. **Target-Oriented Exploration:**
   - If there are accessible Frontier Points in the direction of Your location (red arrow), and the scenario information includes Your Navigation Target, give the highest priority to exploring these Frontier Points without violating point (1).

**Your Recommendation:**
 Describe the information around each Frontier Point in the diagram and indicate whether you are likely to choose it for exploration based on the provided criteria.

------

**Example:**

**Your Navigation Target:** TargetObject

**Scene Objects:**

- Chair: [Coordinates: (3, 4), (2, 3)]
- Table: [Coordinates: (5, 6), (6, 7)]

**Frontier Points (The black dots and corresponding black uppercase letters on the image):**

- A: [Coordinates: (8, 9)]
- B: [Coordinates: (10, 11)]

**Your location (The red arrow):** (4, 4)

**Previous Movement:** (2, 2)

**Analysis:**

1. **Proximity and Accessibility:** Frontier Point A at (8, 9) is relatively close and free of obstacles.
2. **Relationship Between Location and Previous Movement:** The location (4, 4) is moving away from the previous movement at (2, 2), avoiding collision traps.
3. **Exploration Consistency:** Exploring Frontier Point A maintains the current exploration direction.
4. **Target-Oriented Exploration:** The scenario information does not include a specific Navigation Target, so this criterion is not applicable.

**Recommendation:** I am likely to choose Frontier Point A for exploration.

------

Now, begin your analysis with the provided scene information:

- **Your Navigation Target:** <{TARGET}>
- **Scene Objects:** <{SCENE_OBJECTS}>
- **Frontier Points (The black dots and corresponding black uppercase letters on the image):** <{FRONTIERS_RESULTS}>
- **Your location (The red arrow):** <{CUR_LOCATION}>
- **Previous Movement:** <{PRE}>
- **Your location facing (the direction indicated by the red arrow) scene information:** <{SCENE_INFORMATION}>

Describe the information around each Frontier Point and indicate whether you are likely to choose it for exploration.

------

Explanation Ends.
**Output Format:**
Your choice MUST in 'A', 'B', 'C', 'D'(if exist) **WITHOUT ANY OTHER DESCRIPTION**. You don't need to add punctuation at the end. You don't need to add space at the beginning.
'''


def build_decision_prompt(
    *,
    scene_information: str,
    scene_objects: str,
    frontiers_results: str,
    target: str,
    cur_location: tuple[int, int],
    pre_goal_point,
    valid_candidates: list[str],
) -> str:
    """Ported from `form_prompt_for_DecisionVLM_Frontier`, including the
    unconditional decision-first patch (upstream applies this regardless of
    run_mode — confirmed against `run_cmd.txt` on the real HPC source)."""
    pre = "No Movements" if not pre_goal_point else pre_goal_point
    prompt = SINGLE_AGENT_DECISION_PROMPT_FRONTIER1.format(
        SCENE_INFORMATION=scene_information,
        SCENE_OBJECTS=scene_objects,
        FRONTIERS_RESULTS=frontiers_results,
        TARGET=target,
        CUR_LOCATION=cur_location,
        PRE=pre,
    )
    return patch_frontier_prompt(prompt, valid_candidates)


# =============================================================================
# Stage 3 support: decision-first prompt patch + robust parser, ported
# verbatim from `frontier_parser.py`.
# =============================================================================

def _build_decision_suffix(valid_candidates: list[str]) -> str:
    if valid_candidates:
        letters = ", ".join(valid_candidates[:-1]) + (
            f", or {valid_candidates[-1]}" if len(valid_candidates) > 1 else valid_candidates[0]
        )
    else:
        letters = "A, B, C, or D"
    example = valid_candidates[0] if valid_candidates else "B"
    return f"""
**CRITICAL OUTPUT FORMAT — follow EXACTLY:**
Line 1: Your choice letter ONLY ({letters}).
Line 2: REASON=<one short sentence>

Example:
{example}
REASON=closest frontier to kitchen, unobstructed path

Output now:
"""


def patch_frontier_prompt(original_prompt: str, valid_candidates: list[str] | None = None) -> str:
    """Ported verbatim from `patch_frontier_prompt`: cuts the free-text
    chain-of-thought instruction block and replaces it with a decision-
    first format, so the FIRST generated token is the decision letter
    (required for the server's single-token softmax extraction to mean
    anything — see vlm_decision.py's module docstring).
    """
    if valid_candidates is None:
        valid_candidates = ["A", "B", "C", "D"]
    cut_marker = "Explanation Ends."
    idx = original_prompt.find(cut_marker)
    if idx != -1:
        base = original_prompt[:idx + len(cut_marker)]
    else:
        base = re.sub(r"\*\*Output Format:\*\*.*", "", original_prompt, flags=re.DOTALL)
    return base.rstrip() + "\n\n" + _build_decision_suffix(valid_candidates)


@dataclass
class FrontierParseResult:
    success: bool
    chosen: str | None = None
    reason: str | None = None
    raw_output: str = ""
    error: str | None = None
    fell_back: bool = False


_DECISION_FIRST_RE = re.compile(r"^\s*([A-Z])\b", re.MULTILINE)
_DECISION_ANYWHERE_RE = re.compile(r"\b([A-Z])\b")
_REASON_RE = re.compile(r"REASON\s*=\s*(.+)", re.IGNORECASE)


def parse_frontier_decision(
    raw_output: str, valid_candidates: list[str], *, fallback: str = "first_valid",
) -> FrontierParseResult:
    """Ported verbatim from `parse_frontier_decision` (diagnostics/logging
    only in the real cascade — the actual decision uses logit scores, same
    as upstream).
    """
    raw = raw_output.strip()
    valid_set = set(c.upper() for c in valid_candidates)
    if not valid_set:
        return FrontierParseResult(success=False, raw_output=raw, error="valid_candidates is empty")

    reason_match = _REASON_RE.search(raw)
    reason = reason_match.group(1).strip() if reason_match else None

    m = _DECISION_FIRST_RE.search(raw)
    if m and m.group(1) in valid_set:
        return FrontierParseResult(success=True, chosen=m.group(1), reason=reason, raw_output=raw)

    for m in _DECISION_ANYWHERE_RE.finditer(raw):
        if m.group(1) in valid_set:
            return FrontierParseResult(
                success=True, chosen=m.group(1), reason=reason, raw_output=raw,
                error=f"letter not on first line, found '{m.group(1)}' at pos {m.start()}")

    if fallback == "first_valid" and valid_candidates:
        pick = valid_candidates[0]
        return FrontierParseResult(
            success=True, chosen=pick, reason=reason, raw_output=raw,
            error=f"no valid letter found, fell back to '{pick}'", fell_back=True)

    return FrontierParseResult(
        success=False, reason=reason, raw_output=raw,
        error=f"no valid letter found in output, valid={sorted(valid_set)}")
