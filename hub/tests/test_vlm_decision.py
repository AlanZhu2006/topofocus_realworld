from __future__ import annotations

import numpy as np
import pytest

from focus_hub import vlm_decision as vd
from focus_hub.directional_memory import DirectionalMemory
from focus_hub.frontiers import Frontier

_IMG = np.zeros((10, 10, 3), dtype=np.uint8)


def _frontier(letter="A", row=5, col=5, size=10):
    return Frontier(frontier_id=letter, row=row, col=col, x_m=0.0, y_m=0.0, size_cells=size)


def _patch_call_glm(monkeypatch, responses):
    """responses: list of (probabilities, content) consumed in call order."""
    calls = []

    def fake(image, prompt, candidates, *, base_url, timeout_s):
        calls.append((prompt, candidates))
        return responses[len(calls) - 1]

    monkeypatch.setattr(vd, "_call_glm", fake)
    return calls


def test_choose_scene_worth_exploring_glm_uses_weighted_decision(monkeypatch):
    _patch_call_glm(monkeypatch, [({"Yes": 0.8, "No": 0.2}, "Yes")])
    result = vd.choose_scene_worth_exploring_glm(
        _IMG, target="chair", detections=None, base_url="http://x")
    assert result[0] > result[1]


def test_choose_scene_worth_exploring_glm_no_scores_falls_back_neutral(monkeypatch):
    _patch_call_glm(monkeypatch, [({}, "garbage")])
    result = vd.choose_scene_worth_exploring_glm(
        _IMG, target="chair", detections=None, base_url="http://x")
    assert result == (0.5, 0.5)


def test_choose_frontier_glm_picks_argmax_probability(monkeypatch):
    _patch_call_glm(monkeypatch, [({"A": 0.1, "B": 0.7, "C": 0.2}, "B")])
    frontiers = [_frontier("A"), _frontier("B", row=6, col=6), _frontier("C", row=7, col=7)]
    choice = vd.choose_frontier_glm(_IMG, frontiers, base_url="http://x", goal_category="chair")
    assert choice.frontier.frontier_id == "B"
    assert choice.source == "glm-4v"


def test_choose_frontier_glm_no_scores_falls_back_to_content_or_first(monkeypatch):
    _patch_call_glm(monkeypatch, [({}, "C")])
    frontiers = [_frontier("A"), _frontier("B"), _frontier("C")]
    choice = vd.choose_frontier_glm(_IMG, frontiers, base_url="http://x", goal_category="chair")
    assert choice.frontier.frontier_id == "C"


def test_choose_frontier_glm_raises_on_no_frontiers():
    with pytest.raises(ValueError):
        vd.choose_frontier_glm(_IMG, [], base_url="http://x", goal_category="chair")


def test_choose_frontier_fallback_picks_largest():
    frontiers = [_frontier("A", size=5), _frontier("B", size=50), _frontier("C", size=10)]
    choice = vd.choose_frontier_fallback(frontiers)
    assert choice.frontier.frontier_id == "B"
    assert choice.source == "largest-frontier-fallback"


def test_run_decision_cascade_gate_passes_runs_decision_stage(monkeypatch):
    # perception Yes/No, judgment Yes/No, decision A/B
    _patch_call_glm(monkeypatch, [
        ({"Yes": 0.9, "No": 0.1}, "Yes"),   # perception
        ({"Yes": 0.9, "No": 0.1}, "Yes"),   # judgment
        ({"A": 0.9, "B": 0.1}, "A"),        # decision
    ])
    memory = DirectionalMemory()
    frontiers = [_frontier("A"), _frontier("B", row=6, col=6)]
    result = vd.run_decision_cascade(
        rgb_bgr=_IMG, judgment_map_bgr=_IMG, decision_map_bgr=_IMG, frontiers=frontiers,
        target="chair", detections=None, scene_objects="", cur_location_rc=(5, 5),
        heading_deg=90.0, pre_goal_point=None, step=200, early_episode_step_threshold=125,
        memory=memory, base_url="http://x")
    assert result.gate_passed
    assert result.frontier_choice is not None
    assert result.frontier_choice.frontier.frontier_id == "A"
    assert result.errors == []
    assert memory.history_nodes == [(5, 5)]


def test_run_decision_cascade_gate_fails_skips_decision_stage(monkeypatch):
    calls = _patch_call_glm(monkeypatch, [
        ({"Yes": 0.1, "No": 0.9}, "No"),   # perception: not worth exploring
        ({"Yes": 0.1, "No": 0.9}, "No"),   # judgment: revisit, not explore
    ])
    memory = DirectionalMemory()
    frontiers = [_frontier("A")]
    result = vd.run_decision_cascade(
        rgb_bgr=_IMG, judgment_map_bgr=_IMG, decision_map_bgr=_IMG, frontiers=frontiers,
        target="chair", detections=None, scene_objects="", cur_location_rc=(5, 5),
        heading_deg=0.0, pre_goal_point=None, step=200, early_episode_step_threshold=125,
        memory=memory, base_url="http://x")
    assert not result.gate_passed
    assert result.frontier_choice is None
    assert "gated" in result.gate_reason
    assert len(calls) == 2  # decision stage never called


def test_run_decision_cascade_early_episode_forces_gate_open(monkeypatch):
    _patch_call_glm(monkeypatch, [
        ({"Yes": 0.1, "No": 0.9}, "No"),   # perception
        ({"Yes": 0.1, "No": 0.9}, "No"),   # judgment: would gate closed...
        ({"A": 1.0}, "A"),                  # ...but early-episode step forces decision anyway
    ])
    memory = DirectionalMemory()
    frontiers = [_frontier("A")]
    result = vd.run_decision_cascade(
        rgb_bgr=_IMG, judgment_map_bgr=_IMG, decision_map_bgr=_IMG, frontiers=frontiers,
        target="chair", detections=None, scene_objects="", cur_location_rc=(5, 5),
        heading_deg=0.0, pre_goal_point=None, step=10, early_episode_step_threshold=125,
        memory=memory, base_url="http://x")
    assert result.gate_passed
    assert "early episode" in result.gate_reason
    assert result.frontier_choice is not None


def test_run_decision_cascade_no_frontiers_short_circuits():
    memory = DirectionalMemory()
    result = vd.run_decision_cascade(
        rgb_bgr=_IMG, judgment_map_bgr=_IMG, decision_map_bgr=_IMG, frontiers=[],
        target="chair", detections=None, scene_objects="", cur_location_rc=(5, 5),
        heading_deg=0.0, pre_goal_point=None, step=10, early_episode_step_threshold=125,
        memory=memory, base_url="http://x")
    assert result.perception_pr is None
    assert not result.gate_passed
    assert "no frontier candidates" in result.gate_reason


def test_run_decision_cascade_stage_error_is_recorded_not_raised(monkeypatch):
    def fake(image, prompt, candidates, *, base_url, timeout_s):
        raise RuntimeError("server unreachable")

    monkeypatch.setattr(vd, "_call_glm", fake)
    memory = DirectionalMemory()
    frontiers = [_frontier("A")]
    result = vd.run_decision_cascade(
        rgb_bgr=_IMG, judgment_map_bgr=_IMG, decision_map_bgr=_IMG, frontiers=frontiers,
        target="chair", detections=None, scene_objects="", cur_location_rc=(5, 5),
        heading_deg=0.0, pre_goal_point=None, step=10, early_episode_step_threshold=125,
        memory=memory, base_url="http://x")
    # perception and judgment both fail -> neutral (0.5, 0.5) each; gate passes
    # via early-episode step regardless, then decision also fails.
    assert result.perception_pr == (0.5, 0.5)
    assert result.judgment_pr == (0.5, 0.5)
    assert result.gate_passed
    assert result.frontier_choice is None
    assert len(result.errors) == 3
    assert any("perception" in e for e in result.errors)
    assert any("judgment" in e for e in result.errors)
    assert any("decision" in e for e in result.errors)
