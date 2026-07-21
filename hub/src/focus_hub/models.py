from __future__ import annotations

import math
from enum import Enum
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


PROTOCOL_VERSION = "1.0"
ROBOT_ID_PATTERN = r"^[a-z0-9][a-z0-9_-]{0,31}$"
SHA256_PATTERN = r"^[0-9a-f]{64}$"


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class CommandMode(str, Enum):
    GOAL = "GOAL"
    HOLD = "HOLD"
    STOP = "STOP"


class SafetyState(str, Enum):
    READY = "READY"
    HOLD = "HOLD"
    STOPPED = "STOPPED"
    ESTOP = "ESTOP"
    UNKNOWN = "UNKNOWN"


class LocalizationState(str, Enum):
    TRACKING = "TRACKING"
    DEGRADED = "DEGRADED"
    LOST = "LOST"
    UNKNOWN = "UNKNOWN"


class DecisionAckStatus(str, Enum):
    ACCEPTED = "ACCEPTED"
    COMPLETED = "COMPLETED"
    REJECTED_EXPIRED = "REJECTED_EXPIRED"
    REJECTED_TRANSFORM = "REJECTED_TRANSFORM"
    REJECTED_MAP_VERSION = "REJECTED_MAP_VERSION"
    REJECTED_HEALTH = "REJECTED_HEALTH"
    REJECTED_UNSAFE = "REJECTED_UNSAFE"
    REJECTED_OUT_OF_ORDER = "REJECTED_OUT_OF_ORDER"


class CameraIntrinsics(StrictModel):
    width: int = Field(gt=0, le=8192)
    height: int = Field(gt=0, le=8192)
    fx: float = Field(gt=0)
    fy: float = Field(gt=0)
    cx: float
    cy: float
    distortion_model: str = Field(min_length=1, max_length=64)
    distortion: tuple[float, ...] = Field(default=(), max_length=16)

    @field_validator("cx", "cy", "fx", "fy")
    @classmethod
    def finite_intrinsics(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("camera intrinsics must be finite")
        return value


class RigidTransform(StrictModel):
    """Row-major T_parent_child: p_parent = T_parent_child @ p_child."""

    parent_frame: str = Field(min_length=1, max_length=128)
    child_frame: str = Field(min_length=1, max_length=128)
    matrix: tuple[float, ...] = Field(min_length=16, max_length=16)

    @field_validator("matrix")
    @classmethod
    def validate_se3(cls, values: tuple[float, ...]) -> tuple[float, ...]:
        if not all(math.isfinite(value) for value in values):
            raise ValueError("transform contains a non-finite value")
        if any(abs(values[12 + i] - expected) > 1e-5 for i, expected in enumerate((0, 0, 0, 1))):
            raise ValueError("transform last row must be [0, 0, 0, 1]")

        rotation = (
            (values[0], values[1], values[2]),
            (values[4], values[5], values[6]),
            (values[8], values[9], values[10]),
        )
        for row in rotation:
            norm = sum(component * component for component in row)
            if abs(norm - 1.0) > 2e-2:
                raise ValueError("transform rotation is not unit length")
        for left, right in ((0, 1), (0, 2), (1, 2)):
            dot = sum(rotation[left][i] * rotation[right][i] for i in range(3))
            if abs(dot) > 2e-2:
                raise ValueError("transform rotation is not orthogonal")
        det = (
            rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
            - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
            + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
        )
        if abs(det - 1.0) > 2e-2:
            raise ValueError("transform rotation must have determinant +1")
        return values


class PoseEstimate(StrictModel):
    shared_T_camera: RigidTransform
    covariance_6x6: tuple[float, ...] = Field(min_length=36, max_length=36)
    transform_version: str = Field(min_length=1, max_length=128)

    @field_validator("covariance_6x6")
    @classmethod
    def validate_covariance(cls, values: tuple[float, ...]) -> tuple[float, ...]:
        if not all(math.isfinite(value) for value in values):
            raise ValueError("pose covariance contains a non-finite value")
        if any(values[index] < 0 for index in (0, 7, 14, 21, 28, 35)):
            raise ValueError("pose covariance diagonal must be non-negative")
        return values

    @model_validator(mode="after")
    def validate_frame(self) -> "PoseEstimate":
        if self.shared_T_camera.parent_frame != "shared_world":
            raise ValueError("pose parent frame must be shared_world")
        return self


class RobotHealth(StrictModel):
    safety_state: SafetyState
    localization_state: LocalizationState
    estop_engaged: bool
    collision_avoidance_ready: bool
    motor_controller_ready: bool
    battery_percent: float | None = Field(default=None, ge=0, le=100)
    detail: str = Field(default="", max_length=512)

    def ready_for_goal(self) -> bool:
        return (
            self.safety_state == SafetyState.READY
            and self.localization_state == LocalizationState.TRACKING
            and not self.estop_engaged
            and self.collision_avoidance_ready
            and self.motor_controller_ready
        )


class ObjectGoal(StrictModel):
    goal_id: str = Field(min_length=1, max_length=128)
    category: str = Field(min_length=1, max_length=128)


class ObservationMetadata(StrictModel):
    protocol_version: Literal["1.0"] = PROTOCOL_VERSION
    robot_id: str = Field(pattern=ROBOT_ID_PATTERN)
    sequence: int = Field(ge=0)
    capture_time_ns: int = Field(gt=0)
    sent_time_ns: int = Field(gt=0)
    pose: PoseEstimate
    base_T_camera: RigidTransform | None
    intrinsics: CameraIntrinsics
    depth_scale_m: float = Field(gt=0, le=1)
    depth_min_m: float = Field(ge=0)
    depth_max_m: float = Field(gt=0, le=100)
    rgb_encoding: Literal["jpeg", "png"]
    depth_encoding: Literal["png16"]
    rgb_size_bytes: int = Field(gt=0)
    depth_size_bytes: int = Field(gt=0)
    rgb_sha256: str = Field(pattern=SHA256_PATTERN)
    depth_sha256: str = Field(pattern=SHA256_PATTERN)
    object_goal: ObjectGoal
    health: RobotHealth
    mapping_only: bool = True

    @model_validator(mode="after")
    def validate_observation(self) -> "ObservationMetadata":
        if self.sent_time_ns < self.capture_time_ns:
            raise ValueError("sent_time_ns precedes capture_time_ns")
        if self.depth_min_m >= self.depth_max_m:
            raise ValueError("depth_min_m must be smaller than depth_max_m")
        if self.base_T_camera is not None:
            if self.base_T_camera.parent_frame != "base_link":
                raise ValueError("base_T_camera parent frame must be base_link")
            if self.base_T_camera.child_frame != self.pose.shared_T_camera.child_frame:
                raise ValueError("pose and base_T_camera must refer to the same camera frame")
        if not self.mapping_only and self.base_T_camera is None:
            raise ValueError("a live command-capable observation requires base_T_camera calibration")
        return self


class GoalPose(StrictModel):
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


class Decision(StrictModel):
    protocol_version: Literal["1.0"] = PROTOCOL_VERSION
    robot_id: str = Field(pattern=ROBOT_ID_PATTERN)
    decision_id: str = Field(min_length=1, max_length=128)
    mode: CommandMode
    map_version: int = Field(ge=0)
    transform_version: str = Field(min_length=1, max_length=128)
    issued_at_ns: int = Field(gt=0)
    expires_at_ns: int = Field(gt=0)
    target: GoalPose | None = None
    frontier_id: str | None = Field(default=None, max_length=128)
    reason: str = Field(min_length=1, max_length=512)

    @model_validator(mode="after")
    def validate_decision(self) -> "Decision":
        if self.expires_at_ns <= self.issued_at_ns:
            raise ValueError("decision expiry must be after issue time")
        if self.mode == CommandMode.GOAL and self.target is None:
            raise ValueError("GOAL requires a target")
        if self.mode != CommandMode.GOAL and self.target is not None:
            raise ValueError("HOLD/STOP must not carry a target")
        return self


class ObservationAck(StrictModel):
    robot_id: str
    sequence: int
    status: Literal["accepted", "duplicate"]
    received_at_ns: int
    map_version: int


class RobotHeartbeat(StrictModel):
    """A lightweight, RGBD-independent liveness+health ping.

    Deliberately carries nothing but robot_id/timestamp/health — no images,
    no pose, no map data — so a sender can post this on its own fast timer
    even while its main RGBD fetch/encode/upload cycle is slow or stalled.
    See hub/tools or robot_overlay senders for the independent-thread
    implementation; this model only defines the wire shape.
    """

    protocol_version: Literal["1.0"] = PROTOCOL_VERSION
    robot_id: str = Field(pattern=ROBOT_ID_PATTERN)
    sent_time_ns: int = Field(gt=0)
    health: RobotHealth


class HeartbeatAck(StrictModel):
    robot_id: str
    received_at_ns: int
    status: Literal["accepted"]


class DecisionAck(StrictModel):
    protocol_version: Literal["1.0"] = PROTOCOL_VERSION
    robot_id: str = Field(pattern=ROBOT_ID_PATTERN)
    decision_id: str = Field(min_length=1, max_length=128)
    status: DecisionAckStatus
    timestamp_ns: int = Field(gt=0)
    detail: str = Field(default="", max_length=512)

