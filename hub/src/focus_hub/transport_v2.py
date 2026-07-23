"""Approved v2 high-level decision and navigation-feedback wire models.

These models carry goals, never velocity commands.  Physical execution stays
disabled by policy until the separate receiver/activation gates pass.
"""
from __future__ import annotations

import base64
from enum import Enum
import hashlib
import hmac
import math
import struct
from typing import Annotated, Literal

from pydantic import Field, field_validator, model_validator

from .models import CommandMode, ROBOT_ID_PATTERN, SHA256_PATTERN, StrictModel


V2_PROTOCOL_VERSION = "2.0"
V2_DECISION_SCHEMA = "focus-high-level-decision-v2"
V2_BATCH_SCHEMA = "focus-decision-batch-v2"
V2_EVENT_SCHEMA = "focus-navigation-event-v2"
V2_PROFILE = "supervised_concurrent_demo_v1"
V2_MAX_LEASE_NS = 10_000_000_000
V2_MAP_RESOLUTION_M = 0.05
V2_SOURCE_GOAL_DILATION_CELLS = 10
V2_MAX_REGION_PNG_BYTES = 1_048_576
V2_MAX_REGION_SIDE = 2048

GoalCategoryV2 = Literal["chair", "bed", "plant", "toilet", "tv", "sofa"]


class InputObservationV2(StrictModel):
    sequence: int = Field(ge=0)
    capture_time_ns: int = Field(gt=0)
    payload_sha256: str = Field(pattern=SHA256_PATTERN)


class MapProvenanceV2(StrictModel):
    map_version: int = Field(ge=0)
    map_snapshot_sha256: str = Field(pattern=SHA256_PATTERN)
    map_format_version: Literal["focus-hub-central-map-v3"]
    frame_id: Literal["shared_world"] = "shared_world"
    resolution_m: float
    transform_version: str = Field(min_length=1, max_length=128)
    shared_frame_calibration_id: str = Field(min_length=1, max_length=128)

    @field_validator("resolution_m")
    @classmethod
    def source_resolution_only(cls, value: float) -> float:
        if not math.isfinite(value) or not math.isclose(
            value, V2_MAP_RESOLUTION_M, rel_tol=0.0, abs_tol=1e-12
        ):
            raise ValueError(f"v2 demo resolution must be {V2_MAP_RESOLUTION_M}")
        return value


class CoordinationV2(StrictModel):
    execution_epoch: int = Field(ge=0)
    active_robot_ids: tuple[str, ...] = Field(max_length=2)

    @field_validator("active_robot_ids")
    @classmethod
    def unique_valid_robot_ids(cls, values: tuple[str, ...]) -> tuple[str, ...]:
        if len(set(values)) != len(values):
            raise ValueError("active_robot_ids contains duplicates")
        import re

        if any(re.fullmatch(ROBOT_ID_PATTERN, value) is None for value in values):
            raise ValueError("active_robot_ids contains an invalid robot ID")
        return values


class SharedGoalPoseV2(StrictModel):
    frame_id: Literal["shared_world"] = "shared_world"
    x: float
    y: float
    z: float = 0.0
    yaw_rad: float

    @field_validator("x", "y", "z", "yaw_rad")
    @classmethod
    def finite_pose(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("goal pose must be finite")
        return value


class FrontierPointTargetV2(StrictModel):
    kind: Literal["FRONTIER_POINT"] = "FRONTIER_POINT"
    frontier_id: str = Field(min_length=1, max_length=128)
    source_goal_dilation_cells: Literal[10] = V2_SOURCE_GOAL_DILATION_CELLS
    pose: SharedGoalPoseV2


class SemanticRegionPayloadV2(StrictModel):
    frame_id: Literal["shared_world"] = "shared_world"
    origin_xy_m: tuple[float, float]
    resolution_m: float
    height: int = Field(gt=0, le=V2_MAX_REGION_SIDE)
    width: int = Field(gt=0, le=V2_MAX_REGION_SIDE)
    row_axis: Literal["+y"] = "+y"
    column_axis: Literal["+x"] = "+x"
    encoding: Literal["png_u8_0_255_base64"] = "png_u8_0_255_base64"
    component_size_cells: int = Field(gt=0)
    payload_size_bytes: int = Field(gt=0, le=V2_MAX_REGION_PNG_BYTES)
    payload_sha256: str = Field(pattern=SHA256_PATTERN)
    payload_base64: str = Field(min_length=1, max_length=1_398_104)

    @field_validator("origin_xy_m")
    @classmethod
    def finite_origin(cls, values: tuple[float, float]) -> tuple[float, float]:
        if not all(math.isfinite(value) for value in values):
            raise ValueError("region origin must be finite")
        return values

    @field_validator("resolution_m")
    @classmethod
    def source_resolution_only(cls, value: float) -> float:
        if not math.isfinite(value) or not math.isclose(
            value, V2_MAP_RESOLUTION_M, rel_tol=0.0, abs_tol=1e-12
        ):
            raise ValueError(f"v2 demo resolution must be {V2_MAP_RESOLUTION_M}")
        return value

    @model_validator(mode="after")
    def validate_png_identity(self) -> "SemanticRegionPayloadV2":
        try:
            payload = base64.b64decode(self.payload_base64, validate=True)
        except (ValueError, TypeError) as exc:
            raise ValueError("region payload_base64 is invalid") from exc
        if len(payload) != self.payload_size_bytes:
            raise ValueError("region PNG size does not match payload_size_bytes")
        if len(payload) > V2_MAX_REGION_PNG_BYTES:
            raise ValueError("region PNG exceeds v2 size limit")
        if not hmac.compare_digest(hashlib.sha256(payload).hexdigest(), self.payload_sha256):
            raise ValueError("region PNG SHA-256 mismatch")
        if len(payload) < 33 or payload[:8] != b"\x89PNG\r\n\x1a\n":
            raise ValueError("region payload is not a PNG")
        chunk_length = struct.unpack(">I", payload[8:12])[0]
        if chunk_length != 13 or payload[12:16] != b"IHDR":
            raise ValueError("region PNG has no canonical IHDR")
        width, height, bit_depth, color_type, compression, filtering, interlace = struct.unpack(
            ">IIBBBBB", payload[16:29]
        )
        if (width, height) != (self.width, self.height):
            raise ValueError("region PNG dimensions do not match metadata")
        if (bit_depth, color_type, compression, filtering, interlace) != (8, 0, 0, 0, 0):
            raise ValueError("region PNG must be non-interlaced 8-bit grayscale")
        return self

    def png_bytes(self) -> bytes:
        return base64.b64decode(self.payload_base64, validate=True)


class DisplayCentroidV2(StrictModel):
    frame_id: Literal["shared_world"] = "shared_world"
    x: float
    y: float
    authority: Literal["display_only"] = "display_only"

    @field_validator("x", "y")
    @classmethod
    def finite_position(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("display centroid must be finite")
        return value


class SemanticRegionTargetV2(StrictModel):
    kind: Literal["SEMANTIC_REGION"] = "SEMANTIC_REGION"
    category: GoalCategoryV2
    source_robot_id: str = Field(pattern=ROBOT_ID_PATTERN)
    evidence_status: Literal["model_inference_map_projected_unverified"] = (
        "model_inference_map_projected_unverified"
    )
    source_goal_dilation_cells: Literal[10] = V2_SOURCE_GOAL_DILATION_CELLS
    region: SemanticRegionPayloadV2
    display_centroid: DisplayCentroidV2


DecisionTargetV2 = Annotated[
    FrontierPointTargetV2 | SemanticRegionTargetV2,
    Field(discriminator="kind"),
]


class HighLevelDecisionV2(StrictModel):
    protocol_version: Literal["2.0"] = V2_PROTOCOL_VERSION
    schema_version: Literal["focus-high-level-decision-v2"] = V2_DECISION_SCHEMA
    profile: Literal["supervised_concurrent_demo_v1"] = V2_PROFILE
    robot_id: str = Field(pattern=ROBOT_ID_PATTERN)
    scene_id: str = Field(min_length=1, max_length=128)
    episode_id: str = Field(min_length=1, max_length=128)
    round_index: int = Field(ge=0)
    source_step: int = Field(ge=0)
    decision_batch_id: str = Field(min_length=1, max_length=128)
    leg_id: str = Field(min_length=1, max_length=128)
    decision_id: str = Field(min_length=1, max_length=128)
    lease_sequence: int = Field(ge=0)
    mode: CommandMode
    coordination: CoordinationV2
    goal_category: GoalCategoryV2
    input_observations: dict[str, InputObservationV2] = Field(min_length=2, max_length=2)
    map_provenance: MapProvenanceV2
    issued_at_ns: int = Field(gt=0)
    expires_at_ns: int = Field(gt=0)
    target: DecisionTargetV2 | None = None
    reason: str = Field(min_length=1, max_length=512)

    @model_validator(mode="after")
    def validate_decision(self) -> "HighLevelDecisionV2":
        if self.source_step != 0 and (self.source_step - 24) % 25 != 0:
            raise ValueError("source_step must follow 0,24,49,...")
        if self.expires_at_ns <= self.issued_at_ns:
            raise ValueError("decision expiry must follow issue time")
        if (
            self.mode in (CommandMode.GOAL, CommandMode.HOLD)
            and self.expires_at_ns - self.issued_at_ns > V2_MAX_LEASE_NS
        ):
            raise ValueError("GOAL/HOLD lease exceeds 10 seconds")
        active = self.robot_id in self.coordination.active_robot_ids
        if self.mode == CommandMode.GOAL and not active:
            raise ValueError("GOAL robot is absent from active_robot_ids")
        if self.mode == CommandMode.HOLD and active:
            raise ValueError("HOLD robot is present in active_robot_ids")
        if self.mode == CommandMode.GOAL and self.target is None:
            raise ValueError("GOAL requires a target")
        if self.mode != CommandMode.GOAL and self.target is not None:
            raise ValueError("HOLD/STOP must not carry a target")
        if isinstance(self.target, SemanticRegionTargetV2):
            if self.target.category != self.goal_category:
                raise ValueError("semantic target category differs from episode goal")
            if self.target.source_robot_id != self.robot_id:
                raise ValueError("semantic target must stay on its source robot")
            if not math.isclose(
                self.target.region.resolution_m,
                self.map_provenance.resolution_m,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                raise ValueError("semantic region/map resolution mismatch")
        return self


class DecisionBatchV2(StrictModel):
    protocol_version: Literal["2.0"] = V2_PROTOCOL_VERSION
    schema_version: Literal["focus-decision-batch-v2"] = V2_BATCH_SCHEMA
    decisions: tuple[HighLevelDecisionV2, ...] = Field(min_length=2, max_length=2)

    @model_validator(mode="after")
    def validate_atomic_pair(self) -> "DecisionBatchV2":
        first, second = self.decisions
        if first.robot_id == second.robot_id:
            raise ValueError("decision batch robot IDs must be unique")
        if first.decision_id == second.decision_id:
            raise ValueError("decision batch decision IDs must be unique")
        if first.leg_id == second.leg_id:
            raise ValueError("decision batch leg IDs must be unique")
        common_fields = (
            "profile",
            "scene_id",
            "episode_id",
            "round_index",
            "source_step",
            "decision_batch_id",
            "coordination",
            "goal_category",
            "input_observations",
        )
        for field_name in common_fields:
            if getattr(first, field_name) != getattr(second, field_name):
                raise ValueError(f"batch decisions differ in {field_name}")
        robot_ids = {decision.robot_id for decision in self.decisions}
        active_ids = set(first.coordination.active_robot_ids)
        if not active_ids.issubset(robot_ids):
            raise ValueError("active_robot_ids contains a robot outside the batch")
        goal_ids = {
            decision.robot_id
            for decision in self.decisions
            if decision.mode == CommandMode.GOAL
        }
        if goal_ids != active_ids:
            raise ValueError("active_robot_ids must exactly match GOAL decisions")
        return self


class NavigationStatusV2(str, Enum):
    RECEIVED = "RECEIVED"
    ACCEPTED = "ACCEPTED"
    NAVIGATING = "NAVIGATING"
    ARRIVED = "ARRIVED"
    HOLDING = "HOLDING"
    STOPPED = "STOPPED"
    REJECTED = "REJECTED"
    OPERATOR_INTERVENTION = "OPERATOR_INTERVENTION"
    LOCAL_ESTOP = "LOCAL_ESTOP"


class LocalPoseV2(StrictModel):
    frame_id: str = Field(min_length=1, max_length=128)
    x: float
    y: float
    yaw_rad: float

    @field_validator("x", "y", "yaw_rad")
    @classmethod
    def finite_pose(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("local pose must be finite")
        return value


class ResolvedLocalGoalV2(LocalPoseV2):
    source_region_sha256: str = Field(pattern=SHA256_PATTERN)
    arrival_radius_m: float = Field(gt=0, le=2.0)
    adapter_name: str = Field(min_length=1, max_length=128)
    adapter_version: str = Field(min_length=1, max_length=64)


class NavigationEventV2(StrictModel):
    protocol_version: Literal["2.0"] = V2_PROTOCOL_VERSION
    schema_version: Literal["focus-navigation-event-v2"] = V2_EVENT_SCHEMA
    robot_id: str = Field(pattern=ROBOT_ID_PATTERN)
    scene_id: str = Field(min_length=1, max_length=128)
    episode_id: str = Field(min_length=1, max_length=128)
    decision_batch_id: str = Field(min_length=1, max_length=128)
    leg_id: str = Field(min_length=1, max_length=128)
    decision_id: str = Field(min_length=1, max_length=128)
    lease_sequence: int = Field(ge=0)
    event_id: str = Field(min_length=1, max_length=128)
    status: NavigationStatusV2
    reason_code: str = Field(pattern=r"^[A-Z][A-Z0-9_]{0,63}$")
    observed_at_ns: int = Field(gt=0)
    local_pose: LocalPoseV2
    path_length_m_from_episode_start: float = Field(ge=0)
    velocity_zero_confirmed: bool
    terminal_observation_sequence: int | None = Field(default=None, ge=0)
    resolved_local_goal: ResolvedLocalGoalV2 | None = None
    detail: str = Field(default="", max_length=512)

    @field_validator("path_length_m_from_episode_start")
    @classmethod
    def finite_path_length(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("path length must be finite")
        return value

    @model_validator(mode="after")
    def validate_status_fields(self) -> "NavigationEventV2":
        if self.status in {
            NavigationStatusV2.ARRIVED,
            NavigationStatusV2.HOLDING,
            NavigationStatusV2.STOPPED,
        } and not self.velocity_zero_confirmed:
            raise ValueError(f"{self.status.value} requires zero-velocity confirmation")
        if self.resolved_local_goal is not None and self.status != NavigationStatusV2.ACCEPTED:
            raise ValueError("resolved_local_goal is recorded only on ACCEPTED")
        return self


class NavigationEventAckV2(StrictModel):
    robot_id: str = Field(pattern=ROBOT_ID_PATTERN)
    event_id: str = Field(min_length=1, max_length=128)
    status: Literal["accepted", "duplicate"]
    received_at_ns: int = Field(gt=0)
