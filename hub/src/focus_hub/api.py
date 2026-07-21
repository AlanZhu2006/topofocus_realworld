from __future__ import annotations

import hashlib
import hmac
import time
from typing import Annotated

from fastapi import FastAPI, File, Form, Header, HTTPException, UploadFile
from pydantic import ValidationError

from .eventlog import DecisionEventLog
from .models import Decision, DecisionAck, HeartbeatAck, ObservationAck, ObservationMetadata, RobotHeartbeat
from .registry import HubRegistry, RegistryError
from .settings import Settings
from .spool import ObservationSpool, SpoolError


def _require_token(actual: str | None, expected: str, label: str) -> None:
    if not expected or actual is None or not hmac.compare_digest(actual, expected):
        raise HTTPException(status_code=401, detail=f"invalid or missing {label} token")


def create_app(settings: Settings | None = None) -> FastAPI:
    settings = settings or Settings.from_env()
    registry = HubRegistry(settings.policies, state_path=settings.state_dir / "registry_state.json")
    spool = ObservationSpool(settings.spool_dir, min_free_bytes=settings.min_free_bytes)
    decision_log = DecisionEventLog(settings.state_dir / "decision_events.jsonl")
    app = FastAPI(title="Focus two-robot decision hub", version="0.1.0")
    app.state.registry = registry
    app.state.settings = settings

    @app.get("/healthz")
    def health() -> dict[str, object]:
        return {
            "status": "ok",
            "protocol_version": "1.0",
            "robots": registry.robot_ids,
            "goal_output_enabled": {
                robot_id: settings.policies[robot_id].allow_goal for robot_id in registry.robot_ids
            },
        }

    @app.post("/v1/robots/{robot_id}/observations", response_model=ObservationAck)
    async def ingest_observation(
        robot_id: str,
        metadata_json: Annotated[str, Form()],
        rgb: Annotated[UploadFile, File()],
        depth: Annotated[UploadFile, File()],
        x_robot_token: Annotated[str | None, Header()] = None,
    ) -> ObservationAck:
        expected_token = settings.robot_tokens.get(robot_id, "")
        _require_token(x_robot_token, expected_token, "robot")
        try:
            metadata = ObservationMetadata.model_validate_json(metadata_json)
        except ValidationError as exc:
            raise HTTPException(status_code=422, detail=exc.errors()) from exc
        if metadata.robot_id != robot_id:
            raise HTTPException(status_code=422, detail="path and metadata robot_id differ")

        rgb_bytes = await rgb.read(settings.max_rgb_bytes + 1)
        depth_bytes = await depth.read(settings.max_depth_bytes + 1)
        if len(rgb_bytes) > settings.max_rgb_bytes or len(depth_bytes) > settings.max_depth_bytes:
            raise HTTPException(status_code=413, detail="frame payload exceeds configured limit")
        if len(rgb_bytes) != metadata.rgb_size_bytes or len(depth_bytes) != metadata.depth_size_bytes:
            raise HTTPException(status_code=422, detail="payload size does not match metadata")
        rgb_sha = hashlib.sha256(rgb_bytes).hexdigest()
        depth_sha = hashlib.sha256(depth_bytes).hexdigest()
        if not hmac.compare_digest(rgb_sha, metadata.rgb_sha256):
            raise HTTPException(status_code=422, detail="RGB SHA-256 mismatch")
        if not hmac.compare_digest(depth_sha, metadata.depth_sha256):
            raise HTTPException(status_code=422, detail="depth SHA-256 mismatch")

        payload_digest = hashlib.sha256(
            metadata.model_dump_json().encode("utf-8") + rgb_sha.encode("ascii") + depth_sha.encode("ascii")
        ).hexdigest()
        try:
            accepted = registry.accept_observation(metadata, payload_digest)
        except RegistryError as exc:
            raise HTTPException(status_code=exc.status_code, detail=str(exc)) from exc
        if accepted.status == "accepted":
            try:
                spool.write(metadata, rgb_bytes, depth_bytes)
            except (OSError, SpoolError) as exc:
                registry.rollback_observation(metadata, payload_digest, accepted)
                raise HTTPException(status_code=507, detail=str(exc)) from exc
        return ObservationAck(
            robot_id=robot_id,
            sequence=metadata.sequence,
            status=accepted.status,
            received_at_ns=accepted.received_at_ns,
            map_version=accepted.map_version,
        )

    @app.post("/v1/robots/{robot_id}/heartbeat", response_model=HeartbeatAck)
    def ingest_heartbeat(
        robot_id: str,
        heartbeat: RobotHeartbeat,
        x_robot_token: Annotated[str | None, Header()] = None,
    ) -> HeartbeatAck:
        """Lightweight, RGBD-independent liveness+health ping (no images/pose).

        Meant to be posted on its own fast timer, decoupled from the main
        observation upload cycle, so health/liveness stays fresh even when
        the RGBD path is slow or stalled — see registry.HubRegistry's
        freshest-health selection in publish_decision.
        """
        expected_token = settings.robot_tokens.get(robot_id, "")
        _require_token(x_robot_token, expected_token, "robot")
        if heartbeat.robot_id != robot_id:
            raise HTTPException(status_code=422, detail="path and body robot_id differ")
        try:
            received_at_ns = registry.accept_heartbeat(robot_id, heartbeat.health, heartbeat.sent_time_ns)
        except RegistryError as exc:
            raise HTTPException(status_code=exc.status_code, detail=str(exc)) from exc
        return HeartbeatAck(robot_id=robot_id, received_at_ns=received_at_ns, status="accepted")

    @app.get("/v1/robots/{robot_id}/observations/latest")
    def latest_observation(
        robot_id: str,
        x_robot_token: Annotated[str | None, Header()] = None,
    ) -> dict[str, object]:
        """Resume support: a restarting sender continues after last_sequence."""
        _require_token(x_robot_token, settings.robot_tokens.get(robot_id, ""), "robot")
        try:
            state = registry.snapshot(robot_id)
        except RegistryError as exc:
            raise HTTPException(status_code=exc.status_code, detail=str(exc)) from exc
        return {
            "robot_id": robot_id,
            "last_sequence": state.last_sequence,
            "last_received_at_ns": state.last_received_at_ns,
            "map_version": state.map_version,
        }

    @app.get("/v1/robots/{robot_id}/decisions/latest", response_model=Decision)
    def latest_decision(
        robot_id: str,
        x_robot_token: Annotated[str | None, Header()] = None,
    ) -> Decision:
        _require_token(x_robot_token, settings.robot_tokens.get(robot_id, ""), "robot")
        try:
            return registry.effective_decision(robot_id)
        except RegistryError as exc:
            raise HTTPException(status_code=exc.status_code, detail=str(exc)) from exc

    @app.post("/v1/robots/{robot_id}/decisions/{decision_id}/ack", status_code=202)
    def acknowledge_decision(
        robot_id: str,
        decision_id: str,
        ack: DecisionAck,
        x_robot_token: Annotated[str | None, Header()] = None,
    ) -> dict[str, object]:
        _require_token(x_robot_token, settings.robot_tokens.get(robot_id, ""), "robot")
        if ack.robot_id != robot_id or ack.decision_id != decision_id:
            raise HTTPException(status_code=422, detail="ack path and body differ")
        received_at_ns = time.time_ns()
        decision_log.append("ack", {
            "robot_id": robot_id,
            "decision_id": decision_id,
            "status": ack.status.value,
            "robot_timestamp_ns": ack.timestamp_ns,
            "detail": ack.detail,
            "received_at_ns": received_at_ns,
        })
        return {"accepted": True, "received_at_ns": received_at_ns}

    @app.post("/v1/admin/decisions", status_code=202)
    def publish_decision(
        decision: Decision,
        x_admin_token: Annotated[str | None, Header()] = None,
    ) -> dict[str, object]:
        _require_token(x_admin_token, settings.admin_token, "admin")
        try:
            registry.publish_decision(decision)
        except RegistryError as exc:
            decision_log.append("publish_rejected", {
                "robot_id": decision.robot_id,
                "decision_id": decision.decision_id,
                "mode": decision.mode.value,
                "error": str(exc)[:300],
            })
            raise HTTPException(status_code=exc.status_code, detail=str(exc)) from exc
        decision_log.append("publish", {
            "robot_id": decision.robot_id,
            "decision_id": decision.decision_id,
            "mode": decision.mode.value,
            "map_version": decision.map_version,
            "transform_version": decision.transform_version,
            "expires_at_ns": decision.expires_at_ns,
            "reason": decision.reason,
        })
        return {"accepted": True, "decision_id": decision.decision_id}

    @app.post("/v1/admin/map_version/advance", status_code=200)
    def advance_map_version(
        x_admin_token: Annotated[str | None, Header()] = None,
    ) -> dict[str, object]:
        """Advance the shared map version after a fusion cycle completes."""
        _require_token(x_admin_token, settings.admin_token, "admin")
        version = registry.advance_map_version()
        decision_log.append("map_version_advanced", {"map_version": version})
        return {"map_version": version}

    return app


app = create_app()

