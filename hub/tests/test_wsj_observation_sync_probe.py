from __future__ import annotations

from types import SimpleNamespace

import pytest

from hub.robot_overlay.probe_wsj_observation_sync import (
    nearest_stamp_skew_s,
    stamp_ns,
)


def message(sec: int, nanosec: int) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec)
        )
    )


def test_stamp_ns_uses_ros_header_time() -> None:
    assert stamp_ns(message(12, 345)) == 12_000_000_345


def test_nearest_stamp_skew_selects_closest_cached_rgb() -> None:
    cache = [message(1, 0), message(1, 40_000_000), message(2, 0)]
    assert nearest_stamp_skew_s(cache, 1_050_000_000) == pytest.approx(0.01)


def test_nearest_stamp_skew_is_infinite_without_rgb() -> None:
    assert nearest_stamp_skew_s([], 1_000_000_000) == float("inf")
