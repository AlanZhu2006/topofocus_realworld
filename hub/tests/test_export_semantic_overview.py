from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


def load_module():
    path = (
        Path(__file__).resolve().parents[1]
        / "tools"
        / "export_semantic_overview.py"
    )
    name = "focus_test_export_semantic_overview"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def test_overlay_prefers_calibrated_base_pose():
    module = load_module()

    overlay, pose_source = module.overlay_from_status(
        "yunji",
        {
            "last_robot_xy_m": [1.0, 2.0],
            "last_robot_heading_deg": 45.0,
            "robot_trajectory_xy_m": [[0.5, 2.0], [1.0, 2.0]],
            "last_camera_xy_m": [9.0, 9.0],
        },
    )

    assert pose_source == "calibrated_base_link"
    assert overlay.pose_xy_m == (1.0, 2.0)
    assert overlay.heading_deg == 45.0
    assert overlay.trajectory_xy_m == ((0.5, 2.0), (1.0, 2.0))


def test_semantic_stats_preserve_component_evidence():
    module = load_module()
    grid = np.zeros((17, 8, 8), dtype=np.float32)
    grid[1, 1:7, 1:7] = 1.0
    grid[2, 1:3, 1:3] = 1.0
    grid[2, 6, 6] = 1.0

    stats = module.semantic_stats(grid)

    assert stats["explored_cells"] == 36
    assert stats["categories"]["chair"] == {
        "cells": 5,
        "components": 2,
        "component_areas_desc": [4, 1],
    }


def test_parse_robot_requires_name_and_snapshot_directory():
    module = load_module()

    with pytest.raises(Exception, match="expected NAME:SNAPSHOT_DIR"):
        module.parse_robot("wsj")

    parsed = module.parse_robot("wsj:hub/runtime/map")
    assert parsed.name == "wsj"
    assert parsed.directory == Path("hub/runtime/map")
