from __future__ import annotations

import importlib.util
from pathlib import Path


OVERLAY = Path(__file__).parents[1] / "robot_overlay"


def load_module():
    path = OVERLAY / "run_tinynav_saved_map_localization.py"
    spec = importlib.util.spec_from_file_location(
        "test_tinynav_saved_map_localization_overlay_module", path
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_saved_map_wrapper_pins_source_and_isolates_legacy_navigation() -> None:
    module = load_module()

    assert module.PINNED_MAP_NODE_SIZE == 40_254
    assert len(module.PINNED_MAP_NODE_SHA256) == 64
    assert "/map/relocalization" not in module.PRIVATE_REMAPS
    assert "/mapping/cmd_pois" in module.PRIVATE_REMAPS
    assert "/control/target_pose" in module.PRIVATE_REMAPS
    assert "/mapping/nav_done" in module.PRIVATE_REMAPS
    assert "/benchmark/stop" in module.PRIVATE_REMAPS
    assert all(
        topic.startswith("/focus/maploc/source/")
        for topic in module.PRIVATE_REMAPS.values()
    )


def test_saved_map_wrapper_never_imports_robot_sdk() -> None:
    source = (
        OVERLAY / "run_tinynav_saved_map_localization.py"
    ).read_text(encoding="utf-8")

    assert "unitree" not in source.lower()
    assert "go2" not in source.lower()
    assert "cmd_vel" not in source
