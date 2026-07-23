from __future__ import annotations

import hashlib
import hmac
import time
from typing import Annotated

from fastapi import FastAPI, File, Form, Header, HTTPException, Response, UploadFile
from pydantic import ValidationError

from .eventlog import DecisionEventLog
from .models import Decision, DecisionAck, HeartbeatAck, ObservationAck, ObservationMetadata, RobotHeartbeat
from .registry import HubRegistry, RegistryError
from .settings import Settings
from .spool import ObservationSpool, SpoolError
from .transport_v2 import (
    DecisionBatchV2,
    HighLevelDecisionV2,
    NavigationEventAckV2,
    NavigationEventV2,
)
from .v2_registry import V2DecisionRegistry


def _require_token(actual: str | None, expected: str, label: str) -> None:
    if not expected or actual is None or not hmac.compare_digest(actual, expected):
        raise HTTPException(status_code=401, detail=f"invalid or missing {label} token")


def create_app(settings: Settings | None = None) -> FastAPI:
    settings = settings or Settings.from_env()
    registry = HubRegistry(settings.policies, state_path=settings.state_dir / "registry_state.json")
    spool = ObservationSpool(settings.spool_dir, min_free_bytes=settings.min_free_bytes)
    decision_log = DecisionEventLog(settings.state_dir / "decision_events.jsonl")
    v2_registry = V2DecisionRegistry(
        registry,
        settings.policies,
        max_input_age_ns=settings.v2_max_input_age_ns,
    )
    app = FastAPI(title="Focus two-robot decision hub", version="0.1.0")
    app.state.registry = registry
    app.state.settings = settings
    app.state.v2_registry = v2_registry

    @app.get("/healthz")
    def health() -> dict[str, object]:
        return {
            "status": "ok",
            "protocol_version": "1.0",
            "supported_decision_protocols": ["1.0", "2.0"],
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
            # Pydantic's validation context can contain the original
            # ValueError object, which Starlette's JSON response cannot
            # serialize.  Keep the useful location/message/type fields and
            # return the intended 422 instead of turning bad metadata into a
            # secondary 500 response.
            raise HTTPException(
                status_code=422, detail=exc.errors(include_context=False)
            ) from exc
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

    @app.get(
        "/v2/robots/{robot_id}/decisions/latest",
        response_model=HighLevelDecisionV2,
        responses={204: {"description": "No current v2 motion authority"}},
    )
    def latest_v2_decision(
        robot_id: str,
        x_robot_token: Annotated[str | None, Header()] = None,
    ) -> HighLevelDecisionV2 | Response:
        _require_token(x_robot_token, settings.robot_tokens.get(robot_id, ""), "robot")
        try:
            decision = v2_registry.effective_decision(robot_id)
        except RegistryError as exc:
            raise HTTPException(status_code=exc.status_code, detail=str(exc)) from exc
        if decision is None:
            return Response(status_code=204)
        return decision

    @app.post(
        "/v2/robots/{robot_id}/navigation-events",
        response_model=NavigationEventAckV2,
        status_code=202,
    )
    def ingest_v2_navigation_event(
        robot_id: str,
        event: NavigationEventV2,
        x_robot_token: Annotated[str | None, Header()] = None,
    ) -> NavigationEventAckV2:
        _require_token(x_robot_token, settings.robot_tokens.get(robot_id, ""), "robot")
        if event.robot_id != robot_id:
            raise HTTPException(status_code=422, detail="event path and body robot_id differ")
        payload_digest = hashlib.sha256(event.model_dump_json().encode("utf-8")).hexdigest()
        try:
            accepted = v2_registry.accept_event(event, payload_digest)
        except RegistryError as exc:
            decision_log.append("v2_navigation_event_rejected", {
                "robot_id": robot_id,
                "event_id": event.event_id,
                "decision_id": event.decision_id,
                "status": event.status.value,
                "error": str(exc)[:300],
            })
            raise HTTPException(status_code=exc.status_code, detail=str(exc)) from exc
        if accepted.status == "accepted":
            decision_log.append("v2_navigation_event", {
                "robot_id": robot_id,
                "event_id": event.event_id,
                "decision_id": event.decision_id,
                "leg_id": event.leg_id,
                "lease_sequence": event.lease_sequence,
                "status": event.status.value,
                "reason_code": event.reason_code,
                "robot_timestamp_ns": event.observed_at_ns,
                "path_length_m_from_episode_start": event.path_length_m_from_episode_start,
                "velocity_zero_confirmed": event.velocity_zero_confirmed,
                "received_at_ns": accepted.received_at_ns,
            })
        return NavigationEventAckV2(
            robot_id=robot_id,
            event_id=event.event_id,
            status=accepted.status,
            received_at_ns=accepted.received_at_ns,
        )

    @app.post("/v2/admin/decision-batches", status_code=202)
    def publish_v2_decision_batch(
        batch: DecisionBatchV2,
        x_admin_token: Annotated[str | None, Header()] = None,
    ) -> dict[str, object]:
        _require_token(x_admin_token, settings.admin_token, "admin")
        try:
            v2_registry.publish_batch(batch)
        except RegistryError as exc:
            decision_log.append("v2_publish_batch_rejected", {
                "decision_batch_id": batch.decisions[0].decision_batch_id,
                "decision_ids": [decision.decision_id for decision in batch.decisions],
                "modes": [decision.mode.value for decision in batch.decisions],
                "error": str(exc)[:300],
            })
            raise HTTPException(status_code=exc.status_code, detail=str(exc)) from exc
        decision_log.append("v2_publish_batch", {
            "decision_batch_id": batch.decisions[0].decision_batch_id,
            "scene_id": batch.decisions[0].scene_id,
            "episode_id": batch.decisions[0].episode_id,
            "round_index": batch.decisions[0].round_index,
            "source_step": batch.decisions[0].source_step,
            "active_robot_ids": list(
                batch.decisions[0].coordination.active_robot_ids
            ),
            "decisions": [
                {
                    "robot_id": decision.robot_id,
                    "decision_id": decision.decision_id,
                    "leg_id": decision.leg_id,
                    "lease_sequence": decision.lease_sequence,
                    "mode": decision.mode.value,
                    "target_kind": (
                        None if decision.target is None else decision.target.kind
                    ),
                    "map_version": decision.map_provenance.map_version,
                    "map_snapshot_sha256": (
                        decision.map_provenance.map_snapshot_sha256
                    ),
                    "transform_version": (
                        decision.map_provenance.transform_version
                    ),
                    "shared_frame_calibration_id": (
                        decision.map_provenance.shared_frame_calibration_id
                    ),
                    "expires_at_ns": decision.expires_at_ns,
                }
                for decision in batch.decisions
            ],
        })
        return {
            "accepted": True,
            "decision_batch_id": batch.decisions[0].decision_batch_id,
            "decision_ids": [decision.decision_id for decision in batch.decisions],
        }

    @app.get("/v2/admin/robots/{robot_id}/navigation-state")
    def get_v2_navigation_state(
        robot_id: str,
        x_admin_token: Annotated[str | None, Header()] = None,
    ) -> dict[str, object]:
        _require_token(x_admin_token, settings.admin_token, "admin")
        try:
            state = v2_registry.navigation_state(robot_id)
        except RegistryError as exc:
            raise HTTPException(status_code=exc.status_code, detail=str(exc)) from exc
        return {
            "robot_id": robot_id,
            "server_time_ns": time.time_ns(),
            "latest_event_received_at_ns": state.latest_event_received_at_ns,
            "latest_decision": (
                None
                if state.latest_decision is None
                else state.latest_decision.model_dump(mode="json")
            ),
            "latest_event": (
                None
                if state.latest_event is None
                else state.latest_event.model_dump(mode="json")
            ),
        }

    @app.get("/v2/admin/robots/{robot_id}/runtime-readiness")
    def get_v2_runtime_readiness(
        robot_id: str,
        x_admin_token: Annotated[str | None, Header()] = None,
    ) -> dict[str, object]:
        _require_token(x_admin_token, settings.admin_token, "admin")
        try:
            return registry.runtime_readiness(robot_id)
        except RegistryError as exc:
            raise HTTPException(status_code=exc.status_code, detail=str(exc)) from exc

    return app


app = create_app()
