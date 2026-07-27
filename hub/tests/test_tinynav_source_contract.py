from __future__ import annotations

import importlib.util
import hashlib
from pathlib import Path
import sys

import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "robot_overlay"
    / "tinynav_source_contract.py"
)
SPEC = importlib.util.spec_from_file_location(
    "tinynav_source_contract_test", MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_dual_robot_source_contracts_are_explicit_and_distinct():
    wsj = MODULE.SOURCE_CONTRACTS["source-default"]
    yunji = MODULE.SOURCE_CONTRACTS["yunji-water"]

    for component in ("planner", "controller"):
        assert wsj[component].commit != yunji[component].commit
        assert wsj[component].sha256 != yunji[component].sha256
        assert len(wsj[component].sha256) == 64
        assert len(yunji[component].sha256) == 64


def test_source_contract_verifier_records_path_size_and_checksum(
    tmp_path, monkeypatch
):
    source = tmp_path / "planning_node.py"
    payload = b"pinned planner\n"
    source.write_bytes(payload)
    contract = MODULE.TinyNavSourceFileContract(
        repository="https://example.invalid/tinynav.git",
        commit="1" * 40,
        relative_path="tinynav/core/planning_node.py",
        size_bytes=len(payload),
        sha256=hashlib.sha256(payload).hexdigest(),
    )
    monkeypatch.setitem(
        MODULE.SOURCE_CONTRACTS,
        "source-default",
        {"planner": contract},
    )

    provenance = MODULE.verify_tinynav_source(
        source,
        robot_profile="source-default",
        component="planner",
    )

    assert provenance["source_path"] == str(source.resolve())
    assert provenance["size_bytes"] == len(payload)
    assert provenance["sha256"] == contract.sha256
    assert provenance["classification"].startswith("observed_runtime")


def test_source_contract_verifier_fails_closed_on_drift(
    tmp_path, monkeypatch
):
    source = tmp_path / "cmd_vel_control.py"
    source.write_bytes(b"drifted")
    contract = MODULE.TinyNavSourceFileContract(
        repository="https://example.invalid/tinynav.git",
        commit="2" * 40,
        relative_path="tinynav/platforms/cmd_vel_control.py",
        size_bytes=8,
        sha256="0" * 64,
    )
    monkeypatch.setitem(
        MODULE.SOURCE_CONTRACTS,
        "yunji-water",
        {"controller": contract},
    )

    with pytest.raises(RuntimeError, match="source contract mismatch"):
        MODULE.verify_tinynav_source(
            source,
            robot_profile="yunji-water",
            component="controller",
        )
