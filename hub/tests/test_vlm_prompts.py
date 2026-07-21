from __future__ import annotations

import numpy as np

from focus_hub.vlm_prompts import (
    build_decision_prompt,
    build_judgment_prompt,
    build_perception_prompt,
    contains_yes_or_no,
    extract_scene_objects,
    format_history_for_prompt,
    format_scene_objects_for_prompt,
    parse_frontier_decision,
    patch_frontier_prompt,
    perception_weight_decision,
)


def test_perception_prompt_no_detections():
    prompt = build_perception_prompt("chair", None)
    assert "No Detections" in prompt
    assert "chair" in prompt


def test_perception_prompt_with_detections():
    prompt = build_perception_prompt("chair", {"chair": 0.92, "table": 0.3})
    assert "chair, 0.92" in prompt
    assert "table, 0.3" in prompt


def test_judgment_prompt_reflects_perception_pr():
    worth = build_judgment_prompt(
        target="chair", scene_objects="", frontiers_results="A: [Coordinates: (1, 2)]\n",
        history_results="No historical observation points", cur_location=(5, 5),
        pre_goal_point=None, perception_pr_yes=0.9)
    assert "worth exploring" in worth
    not_worth = build_judgment_prompt(
        target="chair", scene_objects="", frontiers_results="", history_results="",
        cur_location=(5, 5), pre_goal_point=None, perception_pr_yes=0.1)
    assert "not worth exploring" in not_worth


def test_decision_prompt_is_patched_decision_first():
    prompt = build_decision_prompt(
        scene_information="", scene_objects="", frontiers_results="A: [Coordinates: (1, 2)]\n",
        target="chair", cur_location=(5, 5), pre_goal_point=None, valid_candidates=["A", "B"])
    # The free-text CoT instruction block should be cut, replaced by the
    # decision-first suffix.
    assert "CRITICAL OUTPUT FORMAT" in prompt
    assert prompt.rstrip().endswith("Output now:")
    assert "Describe the information around each Frontier Point" in prompt  # body kept


def test_patch_frontier_prompt_no_marker_falls_back_to_output_format_strip():
    original = "Some analysis text.\n**Output Format:**\nOld instructions here."
    patched = patch_frontier_prompt(original, ["A", "B", "C"])
    assert "Old instructions here" not in patched
    assert "Some analysis text." in patched
    assert "A, B, or C" in patched


def test_contains_yes_or_no():
    assert contains_yes_or_no("Yes") == "Yes"
    assert contains_yes_or_no("No, definitely not") == "No"
    assert contains_yes_or_no("maybe") == "Neither"


def test_perception_weight_decision_yes_and_no():
    result_yes = perception_weight_decision((0.8, 0.2), "Yes")
    assert result_yes[0] > result_yes[1]
    result_no = perception_weight_decision((0.2, 0.8), "No")
    assert result_no[1] > result_no[0]
    assert perception_weight_decision((0.5, 0.5), "unparseable") == "Neither"


def test_perception_weight_decision_zero_total_falls_back_to_label():
    # x=0 and b_decision='No' makes weighted_yes=x*(1-y)=0, weighted_no=y=0
    # -> genuinely zero total -> falls back to the raw text label.
    assert perception_weight_decision((0.0, 0.0), "No") == "No"


def test_parse_frontier_decision_decision_first():
    result = parse_frontier_decision("B\nREASON=closest unobstructed frontier", ["A", "B", "C"])
    assert result.success
    assert result.chosen == "B"
    assert result.reason == "closest unobstructed frontier"
    assert not result.fell_back


def test_parse_frontier_decision_letter_anywhere():
    result = parse_frontier_decision("I think the answer is C here", ["A", "B", "C"])
    assert result.success
    assert result.chosen == "C"
    assert "not on first line" in result.error


def test_parse_frontier_decision_fallback():
    result = parse_frontier_decision("no idea", ["A", "B"])
    assert result.success
    assert result.chosen == "A"
    assert result.fell_back


def test_format_history_for_prompt_empty_and_nonempty():
    assert format_history_for_prompt([]) == "No historical observation points"
    formatted = format_history_for_prompt([(1, 2), (3, 4)])
    assert "a: [Coordinates: (1, 2)]" in formatted
    assert "b: [Coordinates: (3, 4)]" in formatted


def test_extract_scene_objects_and_format():
    names = ("chair", "sofa")
    grid = np.zeros((2, 40, 40), dtype=np.float32)
    grid[0, 10:20, 10:20] = 1.0  # a real, large-enough chair blob
    objects = extract_scene_objects(grid, names)
    assert "chair" in objects
    assert "sofa" not in objects
    text = format_scene_objects_for_prompt(objects)
    assert text.startswith("chair:")


def test_extract_scene_objects_ignores_tiny_noise():
    names = ("chair",)
    grid = np.zeros((1, 40, 40), dtype=np.float32)
    grid[0, 5, 5] = 1.0  # single pixel, contour too small
    objects = extract_scene_objects(grid, names)
    assert objects == {}
