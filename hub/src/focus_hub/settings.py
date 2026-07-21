from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path

from .registry import RobotPolicy


@dataclass(frozen=True)
class Settings:
    policies: dict[str, RobotPolicy] = field(
        default_factory=lambda: {
            "robot-0": RobotPolicy(transform_version="UNSET", allow_goal=False),
            "robot-1": RobotPolicy(transform_version="UNSET", allow_goal=False),
        }
    )
    robot_tokens: dict[str, str] = field(default_factory=dict)
    admin_token: str = ""
    spool_dir: Path = Path("runtime/spool")
    state_dir: Path = Path("runtime/state")
    max_rgb_bytes: int = 8 * 1024**2
    max_depth_bytes: int = 16 * 1024**2
    min_free_bytes: int = 20 * 1024**3

    @classmethod
    def from_env(cls) -> "Settings":
        config_path = Path(os.environ.get("FOCUS_HUB_ROBOT_CONFIG", "config/robots.json"))
        policies: dict[str, RobotPolicy] = {}
        if config_path.is_file():
            raw = json.loads(config_path.read_text(encoding="utf-8"))
            for robot_id, value in raw["robots"].items():
                policies[robot_id] = RobotPolicy(
                    transform_version=value["transform_version"],
                    allow_goal=bool(value.get("allow_goal", False)),
                )
        if not policies:
            policies = cls().policies

        token_json = os.environ.get("FOCUS_HUB_ROBOT_TOKENS_JSON", "{}")
        tokens = json.loads(token_json)
        return cls(
            policies=policies,
            robot_tokens={str(key): str(value) for key, value in tokens.items()},
            admin_token=os.environ.get("FOCUS_HUB_ADMIN_TOKEN", ""),
            spool_dir=Path(os.environ.get("FOCUS_HUB_SPOOL_DIR", "runtime/spool")),
            state_dir=Path(os.environ.get("FOCUS_HUB_STATE_DIR", "runtime/state")),
            min_free_bytes=int(os.environ.get("FOCUS_HUB_MIN_FREE_BYTES", str(20 * 1024**3))),
        )

