"""Pure helpers for independent leases in one concurrent v2 episode."""
from __future__ import annotations

import hashlib
from typing import Iterable

from .transport_v2 import DecisionBatchV2, HighLevelDecisionV2


def _bounded_id(value: str) -> str:
    if len(value) <= 128:
        return value
    suffix = hashlib.sha256(value.encode("utf-8")).hexdigest()[:16]
    return f"{value[:111]}-{suffix}"


def next_coordination_batch(
    current: DecisionBatchV2,
    *,
    active_robot_ids: Iterable[str],
    execution_epoch: int,
    issued_at_ns: int,
    expires_at_ns: int,
    identity_token: str,
) -> DecisionBatchV2:
    """Renew active GOAL legs and move every inactive robot to a new HOLD.

    Keeping the active robot's ``leg_id`` and incrementing only its lease
    avoids restarting its local planner when the other robot arrives first.
    Inactive HOLDs use a new leg on every atomic pair because HOLD needs no
    renewal authority or feedback dependency.
    """

    if execution_epoch < 0:
        raise ValueError("execution_epoch must be non-negative")
    if expires_at_ns <= issued_at_ns:
        raise ValueError("batch expiry must follow issue time")
    active = tuple(active_robot_ids)
    active_set = set(active)
    current_by_robot = {decision.robot_id: decision for decision in current.decisions}
    if not active_set.issubset(current_by_robot):
        raise ValueError("active robot is outside the current batch")
    if len(active_set) != len(active):
        raise ValueError("active robot IDs contain duplicates")

    decisions: list[HighLevelDecisionV2] = []
    for previous in current.decisions:
        raw = previous.model_dump(mode="json")
        raw["issued_at_ns"] = issued_at_ns
        raw["expires_at_ns"] = expires_at_ns
        raw["coordination"] = {
            "execution_epoch": execution_epoch,
            "active_robot_ids": list(active),
        }
        if previous.robot_id in active_set:
            if previous.mode.value != "GOAL" or previous.target is None:
                raise ValueError("only an existing GOAL leg can be renewed")
            raw["lease_sequence"] = previous.lease_sequence + 1
            raw["decision_id"] = _bounded_id(
                f"{previous.leg_id}-lease-{previous.lease_sequence + 1}-{identity_token}"
            )
        else:
            raw["mode"] = "HOLD"
            raw["target"] = None
            raw["lease_sequence"] = 0
            raw["leg_id"] = _bounded_id(
                f"{previous.decision_batch_id}-{previous.robot_id}-hold-"
                f"e{execution_epoch}-{identity_token}"
            )
            raw["decision_id"] = _bounded_id(f"{raw['leg_id']}-lease-0")
            raw["reason"] = "supervised episode inactive robot HOLD"
        decisions.append(HighLevelDecisionV2.model_validate(raw))
    return DecisionBatchV2(decisions=tuple(decisions))
