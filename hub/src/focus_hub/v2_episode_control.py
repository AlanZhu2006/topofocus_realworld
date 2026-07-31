"""Pure helpers for independent leases in one concurrent v2 episode."""
from __future__ import annotations

import hashlib
from typing import Iterable, Mapping

from .transport_v2 import DecisionBatchV2, HighLevelDecisionV2


RECOVERABLE_LOCAL_PATH_REJECTIONS = frozenset(
    {
        "LOCAL_GOAL_UNREACHABLE",
        "LOCAL_PATH_REVERSE_REQUIRED",
        "LOCAL_PLANNER_TURN_STALLED",
        "LOCAL_PLANNER_NO_PROGRESS",
        "LOCAL_PLANNER_PATH_STALE",
        "LOCAL_ROUTER_HOLD_TIMEOUT",
        "UNREACHABLE",
    }
)


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


def scope_initial_coordination_batch(
    candidate: DecisionBatchV2,
    *,
    active_robot_ids: Iterable[str],
    identity_token: str,
) -> DecisionBatchV2:
    """Isolate an unready robot before the candidate batch is first published.

    Unlike :func:`next_coordination_batch`, this helper operates only on an
    unpublished lease-zero candidate.  Ready GOAL legs therefore retain their
    original identity and lease sequence.  Every inactive robot becomes an
    explicit HOLD in the same atomic batch, so one robot-local readiness fault
    cannot weaken its own gate or abort an independently ready peer.
    """

    active = tuple(active_robot_ids)
    active_set = set(active)
    current_by_robot = {
        decision.robot_id: decision for decision in candidate.decisions
    }
    if not active_set.issubset(current_by_robot):
        raise ValueError("active robot is outside the candidate batch")
    if len(active_set) != len(active):
        raise ValueError("active robot IDs contain duplicates")
    if not identity_token:
        raise ValueError("initial coordination identity token is empty")
    if any(item.lease_sequence != 0 for item in candidate.decisions):
        raise ValueError(
            "initial coordination scoping requires unpublished lease-zero decisions"
        )
    for robot_id in active:
        decision = current_by_robot[robot_id]
        if decision.mode.value != "GOAL" or decision.target is None:
            raise ValueError("only a candidate GOAL may remain active")

    decisions: list[HighLevelDecisionV2] = []
    for previous in candidate.decisions:
        raw = previous.model_dump(mode="json")
        raw["coordination"] = {
            "execution_epoch": previous.coordination.execution_epoch,
            "active_robot_ids": list(active),
        }
        if previous.robot_id not in active_set:
            raw["mode"] = "HOLD"
            raw["target"] = None
            raw["leg_id"] = _bounded_id(
                f"{previous.decision_batch_id}-{previous.robot_id}-"
                f"readiness-hold-{identity_token}"
            )
            raw["decision_id"] = _bounded_id(f"{raw['leg_id']}-lease-0")
            raw["reason"] = (
                "robot-local runtime readiness isolation HOLD; preserved "
                "source/VLM selection remains in the frozen candidate artifact"
            )
        decisions.append(HighLevelDecisionV2.model_validate(raw))
    return DecisionBatchV2(decisions=tuple(decisions))


def recoverable_local_path_failure(
    decision: HighLevelDecisionV2,
    event: Mapping[str, object],
) -> bool:
    """Return whether a failed goal leg may be safely replanned.

    Transform, localization, e-stop, operator and health failures remain
    episode-wide fail-closed conditions. Only an explicit robot-local
    ``REJECTED`` for path/progress feasibility is recoverable. Frontier legs
    may be isolated while their peer continues. A semantic leg instead causes
    a coordinated HOLD before a fresh source/VLM round chooses an approach;
    it can never be promoted to ARRIVED by this helper.
    """

    target = decision.target
    return bool(
        target is not None
        and str(event.get("status", "")) == "REJECTED"
        and str(event.get("reason_code", ""))
        in RECOVERABLE_LOCAL_PATH_REJECTIONS
    )
