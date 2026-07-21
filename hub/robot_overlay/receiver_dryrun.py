#!/usr/bin/env python3
"""Robot-side dry-run decision receiver (standalone overlay, requests only).

Polls the hub for the latest decision, applies the robot-local envelope checks
(the full GoalGuard port comes with the ROS receiver), logs every transition
and acknowledges over the wire.  It has no connection to any control topic:
the only output is a JSONL log.

Fail-closed rules mirrored here: expired decision -> local HOLD; transform
version mismatch -> HOLD; GOAL while this robot has no calibration -> HOLD
with an explicit rejection ack.  STOP latches until the operator restarts the
process (deliberately: network recovery must not clear a STOP).
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:18089")
    parser.add_argument("--robot-id", default="robot-0")
    parser.add_argument("--transform-version", default="UNSET")
    parser.add_argument("--poll-s", type=float, default=2.0)
    parser.add_argument("--log", default="receiver_dryrun.jsonl")
    args = parser.parse_args()

    token = os.environ.get("FOCUS_ROBOT_TOKEN", "")
    if not token:
        print("FOCUS_ROBOT_TOKEN is not set", file=sys.stderr)
        return 2

    import requests

    session = requests.Session()
    session.headers["X-Robot-Token"] = token
    log = open(args.log, "a", buffering=1)

    def emit(**fields):
        log.write(json.dumps({"t": time.time(), **fields}) + "\n")

    last_decision_id = None
    stop_latched = False
    polls = 0
    while True:
        polls += 1
        try:
            response = session.get(
                f"{args.base_url}/v1/robots/{args.robot_id}/decisions/latest",
                timeout=5.0,
            )
            response.raise_for_status()
            decision = response.json()
        except Exception as exc:  # noqa: BLE001 - disconnect -> local HOLD
            emit(event="poll_error", action="LOCAL_HOLD", error=str(exc)[:200])
            time.sleep(args.poll_s)
            continue

        decision_id = decision["decision_id"]
        now_ns = time.time_ns()
        if stop_latched:
            action, ack = "STOP_LATCHED", "REJECTED_UNSAFE"
        elif decision["expires_at_ns"] <= now_ns:
            action, ack = "LOCAL_HOLD", "REJECTED_EXPIRED"
        elif decision["transform_version"] != args.transform_version:
            action, ack = "LOCAL_HOLD", "REJECTED_TRANSFORM"
        elif decision["mode"] == "STOP":
            stop_latched = True
            action, ack = "STOP", "ACCEPTED"
        elif decision["mode"] == "GOAL":
            # This overlay has no measured base_T_camera: never execute.
            action, ack = "LOCAL_HOLD", "REJECTED_UNSAFE"
        else:
            action, ack = "HOLD", "ACCEPTED"

        if decision_id != last_decision_id:
            last_decision_id = decision_id
            emit(event="decision", decision_id=decision_id, mode=decision["mode"],
                 reason=decision.get("reason", "")[:200], action=action, ack=ack)
            try:
                session.post(
                    f"{args.base_url}/v1/robots/{args.robot_id}/decisions/{decision_id}/ack",
                    json={
                        "protocol_version": "1.0",
                        "robot_id": args.robot_id,
                        "decision_id": decision_id,
                        "status": ack,
                        "timestamp_ns": now_ns,
                        "detail": action,
                    },
                    timeout=5.0,
                )
            except Exception as exc:  # noqa: BLE001
                emit(event="ack_error", error=str(exc)[:200])
        if polls % 30 == 0:
            emit(event="heartbeat", polls=polls, stop_latched=stop_latched)
        time.sleep(args.poll_s)


if __name__ == "__main__":
    raise SystemExit(main())
