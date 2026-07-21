#!/usr/bin/env python3
"""Single-robot end-to-end chain rehearsal, fully local and dry-run.

Chain exercised, all over the real wire protocol on loopback:

  replay sender (aligned RGB-D + pose, auth multipart HTTP)
    -> hub API ingest (hash/sequence/clock/transform checks) -> spool
    -> RedNet central semantic BEV map from the spooled bytes
    -> frontier extraction -> GLM-4V frontier choice (local offline server)
    -> decision publish through /v1/admin/decisions
    -> robot-side fetch via /v1/robots/.../decisions/latest
    -> GoalGuard evaluation -> dry-run TinyNav POI JSON artifact + ack

Two lanes, both run by default:

  safety lane   default policy (allow_goal=false, mapping_only uploads): the
                hub MUST reject the GOAL publish with 409; the receiver then
                gets the HOLD fallback.  This proves the fail-closed path.
  rehearsal lane an explicitly-labelled TEST policy (allow_goal=true, a
                placeholder base_T_camera, READY health) on a fresh hub
                instance: the GOAL flows to the guard, which emits the POI
                JSON artifact.  Nothing is ever sent to a robot; both hub
                instances bind 127.0.0.1 and tokens are generated per run.

Requires the GLM server (started automatically unless --glm-url is given a
running instance, or --no-vlm picks the recorded fallback policy).
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

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "hub" / "src"))
sys.path.insert(0, str(WORKSPACE / "dependencies"))
sys.path.insert(0, str(WORKSPACE / "source" / "Focus_realworld"))

from focus_hub.central_mapping import MapperConfig, RedNetSegmenter  # noqa: E402
from focus_hub.client import HubClient  # noqa: E402
from focus_hub.frontiers import extract_frontiers, render_annotated_bev  # noqa: E402
from focus_hub.goal_guard import GoalGuard, GoalGuardConfig  # noqa: E402
from focus_hub.models import Decision, DecisionAck, RobotHealth  # noqa: E402
from focus_hub.pipeline import SpoolMappingPipeline, iter_spooled_observations  # noqa: E402
from focus_hub.tinynav_replay import TinyNavReplayReader  # noqa: E402
from focus_hub.vlm_decision import choose_frontier_fallback, choose_frontier_glm  # noqa: E402

PYTHON = str(WORKSPACE / "hub" / ".venv" / "bin" / "python")
IDENTITY = (1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0)


def wait_for_http(url: str, timeout_s: float, process: subprocess.Popen | None = None) -> None:
    import httpx

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process is not None and process.poll() is not None:
            raise RuntimeError(f"server process exited early with {process.returncode}")
        try:
            if httpx.get(url, timeout=2.0).status_code == 200:
                return
        except httpx.HTTPError:
            pass
        time.sleep(0.5)
    raise TimeoutError(f"no HTTP 200 from {url} within {timeout_s}s")


def start_hub(
    *, port: int, spool_dir: Path, robot_id: str, robot_token: str, admin_token: str,
    transform_version: str, allow_goal: bool, log_path: Path,
) -> subprocess.Popen:
    config = {
        "schema_version": "1.0",
        "shared_frame": "shared_world",
        "robots": {robot_id: {"transform_version": transform_version, "allow_goal": allow_goal}},
    }
    config_path = spool_dir.parent / "robots.json"
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")
    env = os.environ.copy()
    env.update(
        FOCUS_HUB_ROBOT_CONFIG=str(config_path),
        FOCUS_HUB_ROBOT_TOKENS_JSON=json.dumps({robot_id: robot_token}),
        FOCUS_HUB_ADMIN_TOKEN=admin_token,
        FOCUS_HUB_SPOOL_DIR=str(spool_dir),
        FOCUS_HUB_MIN_FREE_BYTES=str(5 * 1024**3),
        PYTHONPATH=str(WORKSPACE / "hub" / "src"),
    )
    log = log_path.open("wb")
    return subprocess.Popen(
        [PYTHON, "-m", "uvicorn", "focus_hub.api:app", "--host", "127.0.0.1", "--port", str(port)],
        env=env, stdout=log, stderr=subprocess.STDOUT, cwd=str(WORKSPACE),
    )


def run_sender(
    *, record: Path, extracted: Path, port: int, robot_id: str, token: str,
    transform_version: str, stride: int, limit: int, command_capable: bool, log_path: Path,
) -> None:
    command = [
        PYTHON, str(WORKSPACE / "hub" / "tools" / "replay_sender.py"),
        "--record", str(record), "--extracted", str(extracted),
        "--base-url", f"http://127.0.0.1:{port}", "--robot-id", robot_id,
        "--token", token, "--transform-version", transform_version,
        "--stride", str(stride), "--limit", str(limit),
    ]
    if command_capable:
        command.append("--command-capable")
    result = subprocess.run(command, capture_output=True, text=True, cwd=str(WORKSPACE))
    log_path.write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode != 0:
        raise RuntimeError(f"replay sender failed: {result.stdout}\n{result.stderr}")


def publish_decision(base_url: str, admin_token: str, decision: Decision) -> tuple[int, str]:
    import httpx

    response = httpx.post(
        f"{base_url}/v1/admin/decisions",
        json=json.loads(decision.model_dump_json()),
        headers={"X-Admin-Token": admin_token},
        timeout=10.0,
    )
    return response.status_code, response.text


def run_lane(
    *, lane: str, args, reader: TinyNavReplayReader, segmenter: RedNetSegmenter,
    out_dir: Path, glm_url: str | None, allow_goal: bool, command_capable: bool,
    transform_version: str,
) -> dict:
    lane_dir = out_dir / lane
    lane_dir.mkdir(parents=True)
    spool_dir = lane_dir / "spool"
    robot_id = "robot-0"
    robot_token = secrets.token_hex(16)
    admin_token = secrets.token_hex(16)
    port = args.hub_port if lane == "safety" else args.hub_port + 1
    base_url = f"http://127.0.0.1:{port}"
    report: dict = {"lane": lane, "allow_goal": allow_goal, "command_capable": command_capable}

    hub = start_hub(
        port=port, spool_dir=spool_dir, robot_id=robot_id, robot_token=robot_token,
        admin_token=admin_token, transform_version=transform_version,
        allow_goal=allow_goal, log_path=lane_dir / "hub.log",
    )
    try:
        wait_for_http(f"{base_url}/healthz", 30, hub)
        run_sender(
            record=args.record, extracted=args.extracted, port=port, robot_id=robot_id,
            token=robot_token, transform_version=transform_version, stride=args.stride,
            limit=args.limit, command_capable=command_capable,
            log_path=lane_dir / "sender.log",
        )
        report["sender"] = (lane_dir / "sender.log").read_text().strip().splitlines()[-1]

        # Map from the spooled (transported) bytes.
        observations = list(iter_spooled_observations(spool_dir, robot_id))
        report["spooled_observations"] = len(observations)
        if not observations:
            raise RuntimeError("no observations reached the spool")
        poses = np.array([o.T_shared_camera[:3, 3] for o in observations])
        margin = MapperConfig().max_range_m + 1.0
        min_xy = poses[:, :2].min(axis=0) - margin
        max_xy = poses[:, :2].max(axis=0) + margin
        config = MapperConfig(map_size_m=float(np.ceil(max(max_xy - min_xy))))
        floor_z = float(np.percentile(
            [o.T_shared_camera[2, 3] for o in observations], 5)) - args.camera_height
        pipeline = SpoolMappingPipeline(
            segmenter, reader.calibration.K_rgb, config,
            (float(min_xy[0]), float(min_xy[1])), floor_z,
        )
        for observation in observations:
            pipeline.process(observation)
        pipeline.save(lane_dir / "map")
        report["map"] = json.loads((lane_dir / "map" / "map_summary.json").read_text())

        grid = pipeline.mapper.map.grid
        frontiers = extract_frontiers(
            grid, pipeline.mapper.map.origin_xy_m, config.resolution_m)
        report["frontiers"] = [
            {"id": f.frontier_id, "x_m": round(f.x_m, 3), "y_m": round(f.y_m, 3),
             "size_cells": f.size_cells}
            for f in frontiers
        ]
        if not frontiers:
            raise RuntimeError("no frontiers found; map too small for a decision")

        robot_rc = None
        if pipeline.last_camera_xy is not None:
            row, col = pipeline.mapper.map.world_to_cell(
                np.array([pipeline.last_camera_xy[0]]), np.array([pipeline.last_camera_xy[1]]))
            robot_rc = (int(row[0]), int(col[0]))
        bev = render_annotated_bev(grid, frontiers, robot_rc)
        import cv2

        cv2.imwrite(str(lane_dir / "bev_annotated.png"), bev)

        if glm_url:
            choice = choose_frontier_glm(
                bev, frontiers, base_url=glm_url, goal_category=args.goal_category)
        else:
            choice = choose_frontier_fallback(frontiers)
        report["choice"] = {
            "frontier_id": choice.frontier.frontier_id,
            "source": choice.source,
            "probabilities": choice.probabilities,
            "raw_content": choice.raw_content,
        }

        if lane == "rehearsal":
            # The registry requires a <3 s fresh observation before a GOAL;
            # mapping + the VLM query take longer, so re-upload the last
            # spooled payload as a fresh heartbeat with the next sequence.
            last = observations[-1]
            entry = spool_dir / robot_id / f"{last.sequence:020d}"
            rgb_bytes = (entry / "rgb.jpg").read_bytes()
            depth_bytes = (entry / "depth.png").read_bytes()
            heartbeat_ns = time.time_ns()
            fresh = last.metadata.model_copy(update={
                "sequence": last.sequence + 1,
                "capture_time_ns": heartbeat_ns - 50_000_000,
                "sent_time_ns": heartbeat_ns,
            })
            with HubClient(base_url, robot_id, robot_token) as client:
                client.upload_bytes(fresh, rgb_bytes, depth_bytes)
            report["heartbeat_sequence"] = fresh.sequence

        now_ns = time.time_ns()
        goal_decision = Decision(
            robot_id=robot_id,
            decision_id=f"e2e-{lane}-goal-1",
            mode="GOAL",
            map_version=0,
            transform_version=transform_version,
            issued_at_ns=now_ns,
            expires_at_ns=now_ns + 30_000_000_000,
            target={"x": choice.frontier.x_m, "y": choice.frontier.y_m, "z": 0.0,
                    "yaw_rad": 0.0},
            frontier_id=choice.frontier.frontier_id,
            reason=f"{choice.source} chose frontier {choice.frontier.frontier_id} "
                   f"for {args.goal_category}",
        )
        status, body = publish_decision(base_url, admin_token, goal_decision)
        report["goal_publish"] = {"status": status, "body": body[:300]}

        if lane == "safety":
            if status != 409:
                raise RuntimeError(f"safety lane expected 409 GOAL rejection, got {status}")
            hold = Decision(
                robot_id=robot_id,
                decision_id=f"e2e-{lane}-hold-1",
                mode="HOLD",
                map_version=0,
                transform_version=transform_version,
                issued_at_ns=time.time_ns(),
                expires_at_ns=time.time_ns() + 30_000_000_000,
                reason=f"GOAL blocked by policy; would explore frontier "
                       f"{choice.frontier.frontier_id} ({choice.source})",
            )
            status_hold, body_hold = publish_decision(base_url, admin_token, hold)
            report["hold_publish"] = {"status": status_hold, "body": body_hold[:300]}
            if status_hold != 202:
                raise RuntimeError(f"HOLD publish failed: {status_hold} {body_hold}")
        elif status != 202:
            raise RuntimeError(f"rehearsal lane GOAL publish failed: {status} {body}")

        # Robot-side receiver, dry-run: fetch, guard, ack.
        with HubClient(base_url, robot_id, robot_token) as client:
            decision = client.latest_decision()
            guard = GoalGuard(GoalGuardConfig(
                robot_id=robot_id,
                transform_version=transform_version,
                shared_T_robot_map=IDENTITY,
                max_goal_distance_m=args.max_goal_distance_m,
            ))
            last = observations[-1]
            result = guard.evaluate(
                decision,
                now_ns=time.time_ns(),
                health=last.metadata.health,
                current_position_robot_map=(
                    float(last.T_shared_camera[0, 3]),
                    float(last.T_shared_camera[1, 3]),
                    float(last.T_shared_camera[2, 3]),
                ),
            )
            ack = DecisionAck(
                robot_id=robot_id,
                decision_id=decision.decision_id,
                status=result.ack_status,
                timestamp_ns=time.time_ns(),
                detail=result.detail[:512],
            )
            client.acknowledge(ack)
        report["receiver"] = {
            "decision_id": decision.decision_id,
            "decision_mode": decision.mode.value,
            "guard_action": result.action.value,
            "ack_status": result.ack_status.value,
            "detail": result.detail,
        }
        (lane_dir / "dryrun_poi.json").write_text(
            (result.poi_json or "{}") + "\n", encoding="utf-8")
        report["dryrun_poi_json"] = result.poi_json
    finally:
        hub.send_signal(signal.SIGINT)
        try:
            hub.wait(timeout=15)
        except subprocess.TimeoutExpired:
            hub.kill()
            hub.wait()
    report["hub_exited"] = hub.returncode
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--extracted", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hub-port", type=int, default=8188)
    parser.add_argument("--glm-port", type=int, default=31511)
    parser.add_argument("--glm-url", default=None,
                        help="use an already-running GLM server at this /v1 base URL")
    parser.add_argument("--no-vlm", action="store_true",
                        help="skip GLM and use the recorded largest-frontier fallback")
    parser.add_argument("--stride", type=int, default=3)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--goal-category", default="chair")
    parser.add_argument("--camera-height", type=float, default=0.4)
    parser.add_argument("--max-goal-distance-m", type=float, default=15.0)
    args = parser.parse_args()

    if args.output.exists():
        print(f"refusing to overwrite existing output: {args.output}", file=sys.stderr)
        return 2
    args.output.mkdir(parents=True)

    reader = TinyNavReplayReader(args.record, args.extracted)
    segmenter = RedNetSegmenter(
        WORKSPACE / "artifacts" / "checkpoints" / "rednet_semmap_mp3d_40.pth")

    glm_process: subprocess.Popen | None = None
    glm_url = args.glm_url
    manifest: dict = {"argv": sys.argv[1:], "lanes": []}
    try:
        if not args.no_vlm and glm_url is None:
            glm_log = (args.output / "glm_server.log").open("wb")
            env = os.environ.copy()
            env["FOCUS_GLM_PORT"] = str(args.glm_port)
            glm_process = subprocess.Popen(
                ["bash", str(WORKSPACE / "hub" / "scripts" / "run_glm_offline.sh")],
                env=env, stdout=glm_log, stderr=subprocess.STDOUT, cwd=str(WORKSPACE),
            )
            glm_url = f"http://127.0.0.1:{args.glm_port}/v1"
            wait_for_http(f"{glm_url}/models", 600, glm_process)
        elif args.no_vlm:
            glm_url = None

        manifest["glm_url"] = glm_url
        manifest["lanes"].append(run_lane(
            lane="safety", args=args, reader=reader, segmenter=segmenter,
            out_dir=args.output, glm_url=glm_url, allow_goal=False,
            command_capable=False, transform_version="UNSET",
        ))
        manifest["lanes"].append(run_lane(
            lane="rehearsal", args=args, reader=reader, segmenter=segmenter,
            out_dir=args.output, glm_url=glm_url, allow_goal=True,
            command_capable=True, transform_version="e2e-test-v1",
        ))
    finally:
        if glm_process is not None:
            glm_process.send_signal(signal.SIGINT)
            try:
                glm_process.wait(timeout=60)
            except subprocess.TimeoutExpired:
                glm_process.kill()
                glm_process.wait()
            manifest["glm_exited"] = glm_process.returncode

    (args.output / "e2e_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    safety, rehearsal = manifest["lanes"]
    ok = (
        safety["goal_publish"]["status"] == 409
        and safety["receiver"]["guard_action"] == "HOLD"
        and rehearsal["goal_publish"]["status"] == 202
        and rehearsal["receiver"]["guard_action"] == "GOAL"
        and rehearsal["dryrun_poi_json"] not in (None, "{}")
    )
    print(json.dumps({
        "safety_goal_publish": safety["goal_publish"]["status"],
        "safety_guard": safety["receiver"]["guard_action"],
        "rehearsal_goal_publish": rehearsal["goal_publish"]["status"],
        "rehearsal_guard": rehearsal["receiver"]["guard_action"],
        "chain_complete": ok,
    }, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
