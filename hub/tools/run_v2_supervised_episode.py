#!/usr/bin/env python3
"""Run one supervised v2 episode with independent leases and fail-closed HOLD."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time
import uuid

import httpx

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

from focus_hub.transport_v2 import DecisionBatchV2  # noqa: E402
from focus_hub.v2_episode_control import next_coordination_batch  # noqa: E402
from focus_hub.v2_scene_batch import build_batch_from_shadow_manifest  # noqa: E402


LIVE_CONFIRMATION = "OPERATOR_PRESENT_AND_ROBOTS_CLEAR"
ACTIVE_FEEDBACK = {"RECEIVED", "ACCEPTED", "NAVIGATING"}
FAILURE_FEEDBACK = {
    "REJECTED",
    "OPERATOR_INTERVENTION",
    "LOCAL_ESTOP",
    "STOPPED",
    "HOLDING",
}


def atomic_write_json(path: Path, payload: object) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


class EpisodeClient:
    def __init__(self, base_url: str, admin_token: str) -> None:
        self.client = httpx.Client(
            base_url=base_url.rstrip("/"),
            headers={"X-Admin-Token": admin_token},
            timeout=5.0,
        )

    def close(self) -> None:
        self.client.close()

    def publish(self, batch: DecisionBatchV2) -> dict[str, object]:
        response = self.client.post(
            "/v2/admin/decision-batches",
            json=batch.model_dump(mode="json"),
        )
        response.raise_for_status()
        payload = response.json()
        if not isinstance(payload, dict):
            raise RuntimeError("Hub returned a malformed publish response")
        return payload

    def state(self, robot_id: str) -> dict[str, object]:
        response = self.client.get(
            f"/v2/admin/robots/{robot_id}/navigation-state"
        )
        response.raise_for_status()
        payload = response.json()
        if not isinstance(payload, dict):
            raise RuntimeError("Hub returned a malformed navigation state")
        return payload

    def readiness(self, robot_id: str) -> dict[str, object]:
        response = self.client.get(
            f"/v2/admin/robots/{robot_id}/runtime-readiness"
        )
        response.raise_for_status()
        payload = response.json()
        if not isinstance(payload, dict):
            raise RuntimeError("Hub returned malformed runtime readiness")
        return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument(
        "--registry-state",
        type=Path,
        default=WORKSPACE / "hub/runtime/state/registry_state.json",
    )
    parser.add_argument(
        "--robot-config",
        type=Path,
        default=WORKSPACE / "hub/config/robots.json",
    )
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--episode-id", required=True)
    parser.add_argument("--execution-epoch", type=int, default=0)
    parser.add_argument("--lease-s", type=float, default=8.0)
    parser.add_argument("--renew-before-s", type=float, default=3.0)
    parser.add_argument("--poll-s", type=float, default=0.5)
    parser.add_argument("--max-runtime-s", type=float, default=180.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hub-url", default="http://127.0.0.1:18089")
    parser.add_argument("--admin-token-file", type=Path)
    parser.add_argument("--enable-live-goal-publication", action="store_true")
    parser.add_argument("--operator-confirmation", default="")
    args = parser.parse_args()
    if not 1.0 <= args.lease_s <= 10.0:
        parser.error("--lease-s must be between 1 and 10 seconds")
    if not 0.5 <= args.renew_before_s < args.lease_s:
        parser.error("--renew-before-s must be positive and shorter than the lease")
    if args.poll_s <= 0.0 or args.max_runtime_s <= 0.0:
        parser.error("poll and runtime limits must be positive")
    output = args.output.expanduser().resolve()
    if output.exists():
        print(f"refusing to overwrite existing output: {output}", file=sys.stderr)
        return 2
    output.mkdir(parents=True)

    now_ns = time.time_ns()
    built = build_batch_from_shadow_manifest(
        args.manifest,
        args.registry_state,
        scene_id=args.scene_id,
        episode_id=args.episode_id,
        execution_epoch=args.execution_epoch,
        now_ns=now_ns,
        robot_config_path=args.robot_config,
        lease_duration_ns=int(args.lease_s * 1e9),
    )
    atomic_write_json(output / "preflight_report.json", built.report)
    atomic_write_json(
        output / "batch_000_initial.json", built.batch.model_dump(mode="json")
    )
    if not args.enable_live_goal_publication:
        print(json.dumps({
            "status": "preflight_only_no_network_no_motion",
            "preflight_ready": built.report["preflight_ready"],
            "robot_commands_sent": False,
            "output": str(output),
        }, indent=2, sort_keys=True))
        return 0

    if args.operator_confirmation != LIVE_CONFIRMATION:
        print(
            "live publication requires --operator-confirmation " + LIVE_CONFIRMATION,
            file=sys.stderr,
        )
        return 2
    if not bool(built.report["preflight_ready"]):
        print("v2 scene preflight has blockers; refusing live publication", file=sys.stderr)
        return 3
    if args.admin_token_file is None:
        print("--admin-token-file is required for live publication", file=sys.stderr)
        return 2
    admin_token = args.admin_token_file.expanduser().read_text(encoding="utf-8").strip()
    if not admin_token:
        print("admin token file is empty", file=sys.stderr)
        return 2

    event_log_path = output / "controller_events.jsonl"
    log = event_log_path.open("a", encoding="utf-8", buffering=1)

    def emit(event: str, **fields: object) -> None:
        log.write(json.dumps({"t_ns": time.time_ns(), "event": event, **fields}) + "\n")
        log.flush()
        os.fsync(log.fileno())

    current = built.batch
    active = set(current.decisions[0].coordination.active_robot_ids)
    client = EpisodeClient(args.hub_url, admin_token)
    try:
        runtime_readiness = {
            robot_id: client.readiness(robot_id)
            for robot_id in sorted(active)
        }
    except Exception as exc:  # noqa: BLE001 - missing readiness is fail-closed
        client.close()
        log.close()
        print(f"runtime readiness query failed: {exc}", file=sys.stderr)
        return 3
    atomic_write_json(output / "runtime_readiness.json", runtime_readiness)
    not_ready = {
        robot_id: report
        for robot_id, report in runtime_readiness.items()
        if report.get("ready_for_goal") is not True
    }
    if not_ready:
        client.close()
        log.close()
        print(
            "robot-local runtime readiness blocked live publication: "
            + json.dumps(not_ready, sort_keys=True),
            file=sys.stderr,
        )
        return 3
    epoch = args.execution_epoch
    publish_count = 0
    live_started = False
    outcome = "aborted_before_publish"
    final_states: dict[str, object] = {}

    def publish(batch: DecisionBatchV2, reason: str) -> None:
        nonlocal current, publish_count, live_started
        response = client.publish(batch)
        current = batch
        publish_count += 1
        live_started = True
        atomic_write_json(
            output / f"batch_{publish_count:03d}_{reason}.json",
            batch.model_dump(mode="json"),
        )
        emit(
            "batch_published",
            reason=reason,
            decision_batch_id=batch.decisions[0].decision_batch_id,
            active_robot_ids=list(batch.decisions[0].coordination.active_robot_ids),
            decision_ids=[decision.decision_id for decision in batch.decisions],
            response=response,
        )

    def transition(next_active: set[str], reason: str) -> None:
        nonlocal epoch
        epoch += 1
        issued = time.time_ns()
        ordered_active = tuple(
            decision.robot_id
            for decision in current.decisions
            if decision.robot_id in next_active
        )
        next_batch = next_coordination_batch(
            current,
            active_robot_ids=ordered_active,
            execution_epoch=epoch,
            issued_at_ns=issued,
            expires_at_ns=issued + int(args.lease_s * 1e9),
            identity_token=uuid.uuid4().hex[:8],
        )
        publish(next_batch, reason)

    started_monotonic = time.monotonic()
    try:
        publish(current, "initial")
        if not active:
            outcome = "no_goal_allocations_holding"
        else:
            outcome = "running"
        while active:
            if time.monotonic() - started_monotonic > args.max_runtime_s:
                transition(set(), "runtime_timeout_hold")
                outcome = "failed_runtime_timeout_holding"
                break
            states = {
                decision.robot_id: client.state(decision.robot_id)
                for decision in current.decisions
            }
            final_states = states
            current_by_robot = {
                decision.robot_id: decision for decision in current.decisions
            }
            arrived_now: set[str] = set()
            failure: tuple[str, str] | None = None
            feedback_ready = True
            newest_server_time_ns = max(
                int(state.get("server_time_ns", 0)) for state in states.values()
            )
            for robot_id in active:
                state = states[robot_id]
                event = state.get("latest_event")
                if not isinstance(event, dict):
                    feedback_ready = False
                    continue
                if event.get("decision_id") != current_by_robot[robot_id].decision_id:
                    feedback_ready = False
                    continue
                status = str(event.get("status", ""))
                if status == "ARRIVED":
                    arrived_now.add(robot_id)
                elif status in FAILURE_FEEDBACK:
                    failure = (robot_id, status)
                elif status not in ACTIVE_FEEDBACK:
                    feedback_ready = False
                received_at_ns = int(state.get("latest_event_received_at_ns", 0))
                if newest_server_time_ns - received_at_ns > 2_000_000_000:
                    feedback_ready = False
            if failure is not None:
                transition(set(), f"{failure[0]}_{failure[1].lower()}_hold")
                outcome = f"failed_{failure[0]}_{failure[1].lower()}_holding"
                break
            if arrived_now:
                active.difference_update(arrived_now)
                transition(active, "arrival_transition")
                if not active:
                    outcome = "all_arrived_holding_unverified"
                    break
                continue

            expires_at_ns = min(
                current_by_robot[robot_id].expires_at_ns for robot_id in active
            ) if active else time.time_ns()
            if expires_at_ns - newest_server_time_ns <= int(args.renew_before_s * 1e9):
                if not feedback_ready:
                    transition(set(), "feedback_missing_hold")
                    outcome = "failed_feedback_missing_holding"
                    break
                transition(active, "lease_renewal")
                continue
            time.sleep(args.poll_s)
    except KeyboardInterrupt:
        outcome = "operator_interrupted"
        if live_started:
            try:
                transition(set(), "operator_interrupt_hold")
                outcome = "operator_interrupted_holding"
            except Exception as hold_exc:  # noqa: BLE001
                emit("hold_publish_failed", error=str(hold_exc)[:500])
    except Exception as exc:  # noqa: BLE001 - any controller fault removes authority
        outcome = f"controller_error_{type(exc).__name__}"
        emit("controller_error", error=str(exc)[:1000])
        if live_started:
            try:
                transition(set(), "controller_error_hold")
                outcome += "_holding"
            except Exception as hold_exc:  # noqa: BLE001
                emit("hold_publish_failed", error=str(hold_exc)[:500])
    finally:
        client.close()
        log.close()

    final_report = {
        "schema_version": "focus-v2-supervised-episode-run-v1",
        "outcome": outcome,
        "high_level_batches_published": publish_count,
        "live_goal_publication_enabled": True,
        "operator_confirmation": LIVE_CONFIRMATION,
        "robot_velocity_commands_sent_by_hub": False,
        "official_success_verified": False,
        "final_navigation_states": final_states,
        "controller_event_log": str(event_log_path),
    }
    atomic_write_json(output / "episode_report.json", final_report)
    print(json.dumps(final_report, indent=2, sort_keys=True))
    return 0 if outcome == "all_arrived_holding_unverified" else 4


if __name__ == "__main__":
    raise SystemExit(main())
