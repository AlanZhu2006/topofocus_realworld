from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import time


TOOL = (
    Path(__file__).resolve().parents[1]
    / "robot_overlay"
    / "probe_tracking_epoch.py"
)


def load_tool():
    spec = importlib.util.spec_from_file_location("tracking_epoch_probe", TOOL)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_process_identity_uses_observed_proc_start_time():
    tool = load_tool()
    before_ns = time.time_ns()
    identity = tool.process_identity(os.getpid())
    after_ns = time.time_ns()

    assert identity["pid"] == os.getpid()
    assert 0 < identity["process_start_time_ns"] <= after_ns
    assert identity["process_start_time_ns"] <= before_ns
    assert len(identity["command_line_sha256"]) == 64
    assert Path(identity["executable"]).is_absolute()
