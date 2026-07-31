from __future__ import annotations

import importlib.util
from pathlib import Path


def load_launcher_module():
    path = (
        Path(__file__).resolve().parents[1]
        / "robot_overlay"
        / "run_tinynav_buildmap_online_mapping.py"
    )
    spec = importlib.util.spec_from_file_location("tinynav_online_mapping", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_ros_parameter_overrides_omit_empty_optional_values():
    launcher = load_launcher_module()

    arguments = launcher.ros_parameter_override_arguments(
        {
            "input.directory": "",
            "unset.value": None,
            "frames.target_frame": "world",
            "input.allow_frame_id_override": False,
        }
    )

    assert arguments == [
        "-p",
        "frames.target_frame:=world",
        "-p",
        "input.allow_frame_id_override:=false",
    ]
