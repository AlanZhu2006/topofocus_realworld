from __future__ import annotations

import json
from pathlib import Path


LAYOUT_PATH = Path(__file__).resolve().parents[1] / "foxglove" / "dual_robot_dashboard.json"


def panel_ids(node):
    if isinstance(node, str):
        yield node
        return
    yield from panel_ids(node["first"])
    yield from panel_ids(node["second"])


def test_dashboard_defaults_to_fused_geometry_and_references_every_panel():
    dashboard = json.loads(LAYOUT_PATH.read_text())
    configs = dashboard["configById"]
    fused_topics = configs["3D!fusedmap"]["topics"]

    assert fused_topics["/fused/geometry_map"]["visible"] is True
    assert fused_topics["/fused/semantic_map"]["visible"] is False
    assert fused_topics["/wsj/map_pose"]["visible"] is True
    assert fused_topics["/yunji/map_pose"]["visible"] is True
    assert set(panel_ids(dashboard["layout"])) == set(configs)
