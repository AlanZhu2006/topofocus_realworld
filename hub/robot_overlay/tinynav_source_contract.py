#!/usr/bin/env python3
"""Fail-closed contracts for the two immutable TinyNav runtime baselines.

WSJ and Yunji intentionally use different upstream TinyNav histories.  The
deployment wrappers may adapt those sources, but they must never silently bind
to whichever ``tinynav`` happens to be first on ``PYTHONPATH``.  These
contracts were observed directly on both robots on 2026-07-27 and turn that
otherwise implicit difference into a checked, provenance-bearing interface.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path


ROBOT_PROFILES = ("source-default", "yunji-water")


@dataclass(frozen=True)
class TinyNavSourceFileContract:
    repository: str
    commit: str
    relative_path: str
    size_bytes: int
    sha256: str


SOURCE_CONTRACTS: dict[
    str, dict[str, TinyNavSourceFileContract]
] = {
    "source-default": {
        "planner": TinyNavSourceFileContract(
            repository="https://github.com/UniflexAI/tinynav.git",
            commit="933fce54ae65e775a1262c346180341f5657c0e4",
            relative_path="tinynav/core/planning_node.py",
            size_bytes=29_580,
            sha256=(
                "18645e43f105a1c8265c30733a57b7eb"
                "24e6d34c039b631cf9b7de559ebee03f"
            ),
        ),
        "controller": TinyNavSourceFileContract(
            repository="https://github.com/UniflexAI/tinynav.git",
            commit="933fce54ae65e775a1262c346180341f5657c0e4",
            relative_path="tinynav/platforms/cmd_vel_control.py",
            size_bytes=10_478,
            sha256=(
                "ea67c986934232b6ae42ffaca239dce2"
                "1e3136efa7f133defb3037addde5350d"
            ),
        ),
    },
    "yunji-water": {
        "planner": TinyNavSourceFileContract(
            repository="git@github.com:AlanZhu2006/go2_tinynav.git",
            commit="5705bb61dafb407594970ab2bc85c63fc71e0a24",
            relative_path="tinynav/core/planning_node.py",
            size_bytes=32_331,
            sha256=(
                "1d78d6204508a3cec880eb6899980fc7"
                "7850fc5b262bf1266f0e15ba43c7dc0e"
            ),
        ),
        "controller": TinyNavSourceFileContract(
            repository="git@github.com:AlanZhu2006/go2_tinynav.git",
            commit="5705bb61dafb407594970ab2bc85c63fc71e0a24",
            relative_path="tinynav/platforms/cmd_vel_control.py",
            size_bytes=15_083,
            sha256=(
                "40519ebb1c9845e0a112f55f0a1ef579"
                "0280153ebaf198ff5122103e1372c50b"
            ),
        ),
    },
}


def verify_tinynav_source(
    source_path: str | Path,
    *,
    robot_profile: str,
    component: str,
) -> dict[str, object]:
    """Verify one imported source file and return complete provenance."""

    if robot_profile not in SOURCE_CONTRACTS:
        raise ValueError(f"unsupported TinyNav robot profile: {robot_profile}")
    try:
        contract = SOURCE_CONTRACTS[robot_profile][component]
    except KeyError as exc:
        raise ValueError(
            f"unsupported TinyNav source component: {component}"
        ) from exc

    path = Path(source_path).resolve()
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise RuntimeError(
            f"TinyNav {component} source is unreadable: {path}"
        ) from exc
    actual_sha256 = hashlib.sha256(payload).hexdigest()
    if (
        len(payload) != contract.size_bytes
        or actual_sha256 != contract.sha256
    ):
        raise RuntimeError(
            f"TinyNav {component} source contract mismatch for "
            f"{robot_profile}: expected_size={contract.size_bytes} "
            f"actual_size={len(payload)} expected_sha256={contract.sha256} "
            f"actual_sha256={actual_sha256} source_path={path}"
        )

    return {
        "classification": "observed_runtime_source_matches_pinned_contract",
        "contract_observation_date": "2026-07-27",
        "robot_profile": robot_profile,
        "component": component,
        "repository": contract.repository,
        "commit": contract.commit,
        "relative_path": contract.relative_path,
        "source_path": str(path),
        "size_bytes": len(payload),
        "sha256": actual_sha256,
    }
