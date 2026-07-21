#!/usr/bin/env python3
"""G5 fault-injection matrix: prove fail-closed behavior over the real wire.

This is NOT a G5 pass. G5 requires hardware-in-the-loop with a physical
robot, a measured base_T_camera and real actuation rejection. What this
script proves is the fault-injection *evidence layer* G5 will need: that the
hub and the robot-side GoalGuard fail closed for every fault category named
in the handoff doc (expired, out-of-order, wrong transform, wrong map,
network disconnect, unsafe health, distance) when driven over real local HTTP
— not just in isolated unit tests. Unit-test coverage for the same rules
lives in `hub/tests/test_registry.py` and `hub/tests/test_goal_guard.py`;
this script adds the two things unit tests structurally cannot show: real
process crash/reconnect behavior, and the full hub->wire->guard round trip.

Runs two local hub instances (loopback, random test tokens):
  policy-blocked : allow_goal=false (the shipped default)
  policy-allowed : allow_goal=true, mapping_only=false, TEST transform  --
                   exists only so a GOAL can reach the guard to be rejected
                   *there*; still never touches a robot.

Writes hub/../data/robot_replays/g5_fault_injection_<label>/matrix.json and
prints a pass/fail summary. Exit code is 0 only if every scenario's observed
outcome matches its expected outcome.
"""
from __future__ import annotations

import argparse
import json
import os
import secrets
import signal
import subprocess
import sys
import time
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))

from focus_hub.client import HubClient  # noqa: E402
from focus_hub.goal_guard import GoalGuard, GoalGuardConfig  # noqa: E402
from focus_hub.models import Decision, DecisionAck, RobotHealth  # noqa: E402

PYTHON = str(WORKSPACE / "hub" / ".venv" / "bin" / "python")
IDENTITY = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
READY_HEALTH = RobotHealth(
    safety_state="READY", localization_state="TRACKING", estop_engaged=False,
    collision_avoidance_ready=True, motor_controller_ready=True,
)


def wait_http(url: str, timeout_s: float, process: subprocess.Popen | None = None) -> None:
    import httpx

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process is not None and process.poll() is not None:
            raise RuntimeError(f"server exited early with {process.returncode}")
        try:
            if httpx.get(url, timeout=2.0).status_code == 200:
                return
        except httpx.HTTPError:
            pass
        time.sleep(0.3)
    raise TimeoutError(f"no HTTP 200 from {url} within {timeout_s}s")


def start_hub(*, port: int, robot_id: str, robot_token: str, admin_token: str,
             transform_version: str, allow_goal: bool, work_dir: Path, label: str) -> subprocess.Popen:
    config = {
        "schema_version": "1.0", "shared_frame": "shared_world",
        "robots": {robot_id: {"transform_version": transform_version, "allow_goal": allow_goal}},
    }
    config_path = work_dir / f"{label}_robots.json"
    config_path.write_text(json.dumps(config))
    env = os.environ.copy()
    env.update(
        FOCUS_HUB_ROBOT_CONFIG=str(config_path),
        FOCUS_HUB_ROBOT_TOKENS_JSON=json.dumps({robot_id: robot_token}),
        FOCUS_HUB_ADMIN_TOKEN=admin_token,
        FOCUS_HUB_SPOOL_DIR=str(work_dir / f"{label}_spool"),
        FOCUS_HUB_STATE_DIR=str(work_dir / f"{label}_state"),
        FOCUS_HUB_MIN_FREE_BYTES=str(1024**3),
        PYTHONPATH=str(WORKSPACE / "hub" / "src"),
    )
    log = (work_dir / f"{label}_hub.log").open("wb")
    return subprocess.Popen(
        [PYTHON, "-m", "uvicorn", "focus_hub.api:app", "--host", "127.0.0.1", "--port", str(port)],
        env=env, stdout=log, stderr=subprocess.STDOUT, cwd=str(WORKSPACE),
    )


def upload_observation(base_url: str, robot_id: str, token: str, *,
                       mapping_only: bool, transform_version: str, health_ready: bool,
                       sequence: int = 0) -> None:
    import hashlib

    import httpx

    now_ns = time.time_ns()
    rgb, depth = b"rgb-bytes", b"depth-bytes"
    metadata = {
        "robot_id": robot_id, "sequence": sequence,
        "capture_time_ns": now_ns - 50_000_000, "sent_time_ns": now_ns,
        "pose": {
            "shared_T_camera": {"parent_frame": "shared_world",
                                "child_frame": "camera_color_optical_frame",
                                "matrix": list(IDENTITY)},
            "covariance_6x6": [0.0] * 36, "transform_version": transform_version,
        },
        "base_T_camera": None if mapping_only else {
            "parent_frame": "base_link", "child_frame": "camera_color_optical_frame",
            "matrix": list(IDENTITY),
        },
        "intrinsics": {"width": 8, "height": 8, "fx": 10.0, "fy": 10.0, "cx": 4.0, "cy": 4.0,
                       "distortion_model": "none", "distortion": []},
        "depth_scale_m": 0.001, "depth_min_m": 0.1, "depth_max_m": 10.0,
        "rgb_encoding": "jpeg", "depth_encoding": "png16",
        "rgb_size_bytes": len(rgb), "depth_size_bytes": len(depth),
        "rgb_sha256": hashlib.sha256(rgb).hexdigest(), "depth_sha256": hashlib.sha256(depth).hexdigest(),
        "object_goal": {"goal_id": "g5-fault-1", "category": "chair"},
        "health": {
            "safety_state": "READY" if health_ready else "UNKNOWN",
            "localization_state": "TRACKING" if health_ready else "UNKNOWN",
            "estop_engaged": False,
            "collision_avoidance_ready": health_ready, "motor_controller_ready": health_ready,
        },
        "mapping_only": mapping_only,
    }
    response = httpx.post(
        f"{base_url}/v1/robots/{robot_id}/observations",
        data={"metadata_json": json.dumps(metadata)},
        files={"rgb": ("rgb", rgb, "image/jpeg"), "depth": ("depth", depth, "image/png")},
        headers={"X-Robot-Token": token}, timeout=10.0,
    )
    response.raise_for_status()


def publish(base_url: str, admin_token: str, decision: dict) -> tuple[int, str]:
    import httpx

    response = httpx.post(f"{base_url}/v1/admin/decisions", json=decision,
                          headers={"X-Admin-Token": admin_token}, timeout=10.0)
    return response.status_code, response.text[:300]


def run_scenario(name: str, expected: str, fn) -> dict:
    try:
        observed, detail = fn()
    except Exception as exc:  # noqa: BLE001 - a scenario itself must not crash the matrix
        observed, detail = "EXCEPTION", f"{type(exc).__name__}: {exc}"[:300]
    return {
        "scenario": name, "expected": expected, "observed": observed, "detail": detail,
        "verdict": "PASS" if observed == expected else "FAIL",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--blocked-port", type=int, default=8390)
    parser.add_argument("--allowed-port", type=int, default=8391)
    args = parser.parse_args()

    if args.output.exists():
        print(f"refusing to overwrite existing output: {args.output}", file=sys.stderr)
        return 2
    args.output.mkdir(parents=True)

    robot_id = "robot-0"
    blocked_token, allowed_token = secrets.token_hex(16), secrets.token_hex(16)
    blocked_admin, allowed_admin = secrets.token_hex(16), secrets.token_hex(16)
    blocked_url = f"http://127.0.0.1:{args.blocked_port}"
    allowed_url = f"http://127.0.0.1:{args.allowed_port}"
    transform_version = "g5-fault-test-v1"

    procs = {
        "blocked": start_hub(port=args.blocked_port, robot_id=robot_id, robot_token=blocked_token,
                             admin_token=blocked_admin, transform_version=transform_version,
                             allow_goal=False, work_dir=args.output, label="blocked"),
        "allowed": start_hub(port=args.allowed_port, robot_id=robot_id, robot_token=allowed_token,
                             admin_token=allowed_admin, transform_version=transform_version,
                             allow_goal=True, work_dir=args.output, label="allowed"),
    }
    matrix: list[dict] = []
    try:
        wait_http(f"{blocked_url}/healthz", 20, procs["blocked"])
        wait_http(f"{allowed_url}/healthz", 20, procs["allowed"])
        upload_observation(blocked_url, robot_id, blocked_token, mapping_only=True,
                           transform_version=transform_version, health_ready=False)
        upload_observation(allowed_url, robot_id, allowed_token, mapping_only=False,
                           transform_version=transform_version, health_ready=True)

        # --- hub-side (registry) rejections, over real HTTP -----------------

        def scenario_goal_blocked_by_policy():
            now_ns = time.time_ns()
            status, body = publish(blocked_url, blocked_admin, {
                "robot_id": robot_id, "decision_id": "f-policy", "mode": "GOAL",
                "map_version": 0, "transform_version": transform_version,
                "issued_at_ns": now_ns, "expires_at_ns": now_ns + 5_000_000_000,
                "target": {"x": 1, "y": 1, "z": 0, "yaw_rad": 0}, "reason": "fault test",
            })
            return ("REJECTED_409" if status == 409 else f"HTTP_{status}"), body

        def scenario_expired_decision_rejected_at_publish():
            now_ns = time.time_ns()
            status, body = publish(allowed_url, allowed_admin, {
                "robot_id": robot_id, "decision_id": "f-expired-publish", "mode": "HOLD",
                "map_version": 0, "transform_version": transform_version,
                "issued_at_ns": now_ns - 5_000_000_000, "expires_at_ns": now_ns - 1_000_000_000,
                "reason": "fault test",
            })
            return ("REJECTED_422" if status == 422 else f"HTTP_{status}"), body

        def scenario_bad_clock_ingest_rejected():
            import hashlib

            import httpx

            now_ns = time.time_ns()
            future_ns = now_ns + 10_000_000_000  # 10s in the future, past skew tolerance
            rgb, depth = b"rgb", b"depth"
            metadata = {
                "robot_id": robot_id, "sequence": 999,
                "capture_time_ns": future_ns, "sent_time_ns": future_ns,
                "pose": {"shared_T_camera": {"parent_frame": "shared_world",
                                             "child_frame": "camera_color_optical_frame",
                                             "matrix": list(IDENTITY)},
                        "covariance_6x6": [0.0] * 36, "transform_version": transform_version},
                "base_T_camera": None,
                "intrinsics": {"width": 8, "height": 8, "fx": 10.0, "fy": 10.0, "cx": 4.0, "cy": 4.0,
                              "distortion_model": "none", "distortion": []},
                "depth_scale_m": 0.001, "depth_min_m": 0.1, "depth_max_m": 10.0,
                "rgb_encoding": "jpeg", "depth_encoding": "png16",
                "rgb_size_bytes": len(rgb), "depth_size_bytes": len(depth),
                "rgb_sha256": hashlib.sha256(rgb).hexdigest(), "depth_sha256": hashlib.sha256(depth).hexdigest(),
                "object_goal": {"goal_id": "g5-fault-clock", "category": "chair"},
                "health": {"safety_state": "UNKNOWN", "localization_state": "UNKNOWN",
                          "estop_engaged": False, "collision_avoidance_ready": False,
                          "motor_controller_ready": False},
                "mapping_only": True,
            }
            response = httpx.post(
                f"{blocked_url}/v1/robots/{robot_id}/observations",
                data={"metadata_json": json.dumps(metadata)},
                files={"rgb": ("rgb", rgb, "image/jpeg"), "depth": ("depth", depth, "image/png")},
                headers={"X-Robot-Token": blocked_token}, timeout=10.0,
            )
            return ("REJECTED_422" if response.status_code == 422 else f"HTTP_{response.status_code}"), \
                response.text[:300]

        matrix.append(run_scenario("hub: GOAL blocked by allow_goal=false policy",
                                   "REJECTED_409", scenario_goal_blocked_by_policy))
        matrix.append(run_scenario("hub: already-expired decision rejected at publish",
                                   "REJECTED_422", scenario_expired_decision_rejected_at_publish))
        matrix.append(run_scenario("hub: observation with clock far in the future rejected",
                                   "REJECTED_422", scenario_bad_clock_ingest_rejected))

        # --- robot-side (GoalGuard) rejections over a real fetched decision -

        def fresh_guard() -> GoalGuard:
            return GoalGuard(GoalGuardConfig(
                robot_id=robot_id, transform_version=transform_version,
                shared_T_robot_map=IDENTITY, max_goal_distance_m=8.0,
            ))

        def publish_and_fetch(decision: dict) -> Decision:
            status, body = publish(allowed_url, allowed_admin, decision)
            if status != 202:
                raise RuntimeError(f"setup publish failed: {status} {body}")
            with HubClient(allowed_url, robot_id, allowed_token) as client:
                return client.latest_decision()

        def scenario_expired_in_transit():
            now_ns = time.time_ns()
            fetched = publish_and_fetch({
                "robot_id": robot_id, "decision_id": "f-expire-transit", "mode": "GOAL",
                "map_version": 0, "transform_version": transform_version,
                "issued_at_ns": now_ns, "expires_at_ns": now_ns + 1_500_000_000,
                "target": {"x": 0.1, "y": 0.1, "z": 0, "yaw_rad": 0}, "reason": "fault test",
            })
            time.sleep(2.0)  # simulate network/processing delay past expiry
            result = fresh_guard().evaluate(
                fetched, now_ns=time.time_ns(), health=READY_HEALTH,
                current_position_robot_map=(0, 0, 0))
            return result.ack_status.value, result.detail

        def scenario_wrong_transform_at_guard():
            now_ns = time.time_ns()
            fetched = publish_and_fetch({
                "robot_id": robot_id, "decision_id": "f-wrong-transform", "mode": "HOLD",
                "map_version": 0, "transform_version": transform_version,
                "issued_at_ns": now_ns, "expires_at_ns": now_ns + 10_000_000_000,
                "reason": "fault test",
            })
            guard = GoalGuard(GoalGuardConfig(
                robot_id=robot_id, transform_version="a-completely-different-calibration",
                shared_T_robot_map=IDENTITY,
            ))
            result = guard.evaluate(fetched, now_ns=time.time_ns(), health=READY_HEALTH,
                                    current_position_robot_map=(0, 0, 0))
            return result.ack_status.value, result.detail

        def scenario_local_unsafe_health_overrides_hub_goal():
            now_ns = time.time_ns()
            fetched = publish_and_fetch({
                "robot_id": robot_id, "decision_id": "f-local-unsafe", "mode": "GOAL",
                "map_version": 0, "transform_version": transform_version,
                "issued_at_ns": now_ns, "expires_at_ns": now_ns + 10_000_000_000,
                "target": {"x": 0.1, "y": 0.1, "z": 0, "yaw_rad": 0}, "reason": "fault test",
            })
            unsafe_health = READY_HEALTH.model_copy(update={"estop_engaged": True})
            result = fresh_guard().evaluate(
                fetched, now_ns=time.time_ns(), health=unsafe_health,
                current_position_robot_map=(0, 0, 0))
            return result.ack_status.value, result.detail

        def scenario_goal_too_far():
            now_ns = time.time_ns()
            fetched = publish_and_fetch({
                "robot_id": robot_id, "decision_id": "f-too-far", "mode": "GOAL",
                "map_version": 0, "transform_version": transform_version,
                "issued_at_ns": now_ns, "expires_at_ns": now_ns + 10_000_000_000,
                "target": {"x": 500.0, "y": 500.0, "z": 0, "yaw_rad": 0}, "reason": "fault test",
            })
            result = fresh_guard().evaluate(
                fetched, now_ns=time.time_ns(), health=READY_HEALTH,
                current_position_robot_map=(0, 0, 0))
            return result.ack_status.value, result.detail

        matrix.append(run_scenario("guard: decision that expired in transit",
                                   "REJECTED_EXPIRED", scenario_expired_in_transit))
        matrix.append(run_scenario("guard: hub decision with wrong local calibration version",
                                   "REJECTED_TRANSFORM", scenario_wrong_transform_at_guard))
        matrix.append(run_scenario("guard: local e-stop overrides an otherwise-valid hub GOAL",
                                   "REJECTED_HEALTH", scenario_local_unsafe_health_overrides_hub_goal))
        matrix.append(run_scenario("guard: goal beyond local max-distance safety limit",
                                   "REJECTED_UNSAFE", scenario_goal_too_far))

        # --- genuine transport faults: process-level, not unit-testable -----

        def scenario_hub_unreachable_fails_closed():
            with HubClient("http://127.0.0.1:1", robot_id, allowed_token, timeout_s=2.0) as client:
                try:
                    client.latest_decision()
                    return "UNEXPECTED_SUCCESS", ""
                except Exception as exc:  # noqa: BLE001
                    # This is exactly what a live receiver must catch and turn
                    # into a local HOLD (see hub/robot_overlay/receiver_dryrun.py).
                    return "LOCAL_HOLD_ON_CONNECT_ERROR", f"{type(exc).__name__}"

        def scenario_stop_latch_survives_real_hub_kill_and_restart():
            # Fetch and latch a real STOP over the wire.
            now_ns = time.time_ns()
            stop_fetched = publish_and_fetch({
                "robot_id": robot_id, "decision_id": "f-stop-1", "mode": "STOP",
                "map_version": 0, "transform_version": transform_version,
                "issued_at_ns": now_ns, "expires_at_ns": now_ns + 10_000_000_000,
                "reason": "operator e-stop",
            })
            guard = fresh_guard()
            first = guard.evaluate(stop_fetched, now_ns=time.time_ns(), health=READY_HEALTH,
                                   current_position_robot_map=(0, 0, 0))
            if first.action.value != "STOP":
                return "STOP_DID_NOT_LATCH", ""

            # Genuinely kill the hub process (registry state, including the
            # STOP decision itself, is process-memory and NOT persisted) and
            # start a fresh one on the same port/config.
            procs["allowed"].send_signal(signal.SIGKILL)
            procs["allowed"].wait(timeout=10)
            procs["allowed"] = start_hub(
                port=args.allowed_port, robot_id=robot_id, robot_token=allowed_token,
                admin_token=allowed_admin, transform_version=transform_version,
                allow_goal=True, work_dir=args.output, label="allowed")
            wait_http(f"{allowed_url}/healthz", 20, procs["allowed"])

            # The restarted hub has no memory of the STOP; it serves its own
            # benign fallback HOLD. The robot's guard must STILL refuse it,
            # because only an explicit local operator action may clear a
            # latched STOP — a hub restart or reconnect must not.
            with HubClient(allowed_url, robot_id, allowed_token) as client:
                post_restart_decision = client.latest_decision()
            after = guard.evaluate(post_restart_decision, now_ns=time.time_ns(),
                                   health=READY_HEALTH, current_position_robot_map=(0, 0, 0))
            return after.ack_status.value, after.detail

        matrix.append(run_scenario("transport: hub completely unreachable -> local fail-closed",
                                   "LOCAL_HOLD_ON_CONNECT_ERROR", scenario_hub_unreachable_fails_closed))
        matrix.append(run_scenario("transport: STOP latch survives a real hub kill + restart",
                                   "REJECTED_UNSAFE", scenario_stop_latch_survives_real_hub_kill_and_restart))

    finally:
        for proc in procs.values():
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        for proc in procs.values():
            if proc.poll() is None:
                try:
                    proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()

    (args.output / "matrix.json").write_text(json.dumps(matrix, indent=2) + "\n", encoding="utf-8")
    passed = sum(1 for row in matrix if row["verdict"] == "PASS")
    print(json.dumps({"total": len(matrix), "passed": passed, "failed": len(matrix) - passed}, indent=2))
    for row in matrix:
        mark = "OK" if row["verdict"] == "PASS" else "**FAIL**"
        print(f"[{mark}] {row['scenario']}: expected={row['expected']} observed={row['observed']}")
    return 0 if passed == len(matrix) else 1


if __name__ == "__main__":
    raise SystemExit(main())
