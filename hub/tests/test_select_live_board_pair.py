from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


TOOLS = Path(__file__).resolve().parents[1] / "tools"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "select_live_board_pair",
        TOOLS / "select_live_board_pair.py",
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def row(robot_id: str, sequence: int, capture_time_ns: int) -> dict:
    return {
        "robot_id": robot_id,
        "sequence": sequence,
        "capture_time_ns": capture_time_ns,
    }


def test_choose_pair_prefers_smallest_skew_then_newest():
    module = load_module()
    reference = [
        row("robot-0", 10, 10_000_000_000),
        row("robot-0", 11, 11_000_000_000),
    ]
    other = [
        row("robot-1", 20, 10_100_000_000),
        row("robot-1", 21, 11_100_000_000),
    ]

    selected_reference, selected_other, skew_s = module.choose_pair(
        reference, other, max_sync_skew_s=0.25
    )

    assert selected_reference["sequence"] == 11
    assert selected_other["sequence"] == 21
    assert skew_s == pytest.approx(0.1)


def test_choose_pair_rejects_unsynchronized_detections():
    module = load_module()

    with pytest.raises(ValueError, match="no synchronized pair"):
        module.choose_pair(
            [row("robot-0", 1, 1_000_000_000)],
            [row("robot-1", 2, 2_000_000_000)],
            max_sync_skew_s=0.25,
        )


def test_timestamp_pairing_handles_asymmetric_camera_rates():
    module = load_module()
    reference = [row("robot-0", 10, 100_000_000_000)]
    other = [
        row("robot-1", sequence, 90_000_000_000 + sequence * 100_000_000)
        for sequence in range(1, 201)
    ]

    # The latest twelve 10 Hz frames span only 1.1 seconds and cannot match
    # the newest slow reference keyframe. The metadata-first full window can.
    assert not module.synchronized_candidate_pairs(
        reference,
        other[-12:],
        max_sync_skew_s=0.25,
    )
    pairs = module.synchronized_candidate_pairs(
        reference,
        other,
        max_sync_skew_s=0.25,
    )
    assert pairs
    assert pairs[0][0] == pytest.approx(0.0)
    assert pairs[0][1]["sequence"] == 10
    assert pairs[0][2]["sequence"] == 100
