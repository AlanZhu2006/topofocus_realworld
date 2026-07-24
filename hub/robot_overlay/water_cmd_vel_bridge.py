#!/usr/bin/env python3
"""Guarded ROS Twist to Yunji WATER velocity bridge.

This is the Yunji equivalent of the Go2 ``cmd_vel`` bridge.  TinyNav owns the
online map, global/local planning and trajectory controller.  This process
only converts an already lease-gated ``geometry_msgs/Twist`` into WATER's
short-lived ``/api/joy_control`` command.

The bridge is read-only by default.  Live output requires an explicit flag and
operator confirmation.  Missing/stale/invalid input, unhealthy WATER status,
TCP failure and process shutdown all reduce to zero velocity.  WATER documents
each velocity command as expiring after 0.5 seconds, providing a final chassis
watchdog if this process or its network link disappears.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import socket
import sys
import time
from typing import Any
from urllib.parse import urlencode
import uuid


OVERLAY = Path(__file__).resolve().parent
HUB_SRC = OVERLAY.parent / "src"
if HUB_SRC.is_dir():
    sys.path.insert(0, str(HUB_SRC))

from focus_hub.v2_robot_runtime import (  # noqa: E402
    WaterTcpClient,
    require_water_ok,
)


LIVE_CONFIRMATION = "OPERATOR_PRESENT_AND_YUNJI_CLEAR"
WATER_LINEAR_LIMIT_MPS = 0.5
WATER_ANGULAR_LIMIT_RADPS = 1.0


@dataclass(frozen=True)
class SanitizedVelocity:
    linear_mps: float
    angular_radps: float
    accepted: bool
    reason: str

    @property
    def zero(self) -> bool:
        return abs(self.linear_mps) < 1e-9 and abs(self.angular_radps) < 1e-9


def sanitize_velocity(
    *,
    linear_x: float,
    linear_y: float,
    linear_z: float,
    angular_x: float,
    angular_y: float,
    angular_z: float,
    max_linear_mps: float,
    max_angular_radps: float,
) -> SanitizedVelocity:
    """Validate a differential-drive Twist and clamp its supported axes."""

    values = (
        linear_x,
        linear_y,
        linear_z,
        angular_x,
        angular_y,
        angular_z,
        max_linear_mps,
        max_angular_radps,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return SanitizedVelocity(0.0, 0.0, False, "non_finite_twist")
    if (
        max_linear_mps <= 0.0
        or max_linear_mps > WATER_LINEAR_LIMIT_MPS
        or max_angular_radps <= 0.0
        or max_angular_radps > WATER_ANGULAR_LIMIT_RADPS
    ):
        raise ValueError("bridge limits exceed WATER's documented range")
    unsupported = (linear_y, linear_z, angular_x, angular_y)
    if any(abs(float(value)) > 1e-4 for value in unsupported):
        return SanitizedVelocity(0.0, 0.0, False, "unsupported_twist_axis")
    return SanitizedVelocity(
        linear_mps=float(
            max(-max_linear_mps, min(max_linear_mps, linear_x))
        ),
        angular_radps=float(
            max(-max_angular_radps, min(max_angular_radps, angular_z))
        ),
        accepted=True,
        reason="accepted",
    )


def effective_velocity(
    command: SanitizedVelocity | None,
    *,
    received_monotonic: float,
    now_monotonic: float,
    input_timeout_s: float,
    water_ready: bool,
) -> tuple[SanitizedVelocity, str]:
    """Apply the local freshness and chassis-health watchdogs."""

    if not math.isfinite(input_timeout_s) or input_timeout_s <= 0.0:
        raise ValueError("input_timeout_s must be finite and positive")
    zero = SanitizedVelocity(0.0, 0.0, True, "watchdog_zero")
    if command is None or received_monotonic <= 0.0:
        return zero, "no_guarded_command"
    age_s = now_monotonic - received_monotonic
    if not math.isfinite(age_s) or age_s < 0.0 or age_s > input_timeout_s:
        return zero, "guarded_command_stale"
    if not command.accepted:
        return zero, command.reason
    if not water_ready:
        return zero, "water_health_not_ready"
    return command, "active" if not command.zero else "guarded_zero"


def joy_command_line(
    linear_mps: float,
    angular_radps: float,
    *,
    request_id: str,
) -> bytes:
    values = (linear_mps, angular_radps)
    if not request_id or not all(math.isfinite(value) for value in values):
        raise ValueError("WATER velocity request is invalid")
    if abs(linear_mps) > WATER_LINEAR_LIMIT_MPS + 1e-9:
        raise ValueError("linear velocity exceeds WATER's documented limit")
    if abs(angular_radps) > WATER_ANGULAR_LIMIT_RADPS + 1e-9:
        raise ValueError("angular velocity exceeds WATER's documented limit")
    query = urlencode(
        {
            "linear_velocity": f"{linear_mps:.3f}",
            "angular_velocity": f"{angular_radps:.3f}",
            "uuid": request_id,
        }
    )
    return f"/api/joy_control?{query}\n".encode("utf-8")


class WaterJoyClient:
    """Persistent WATER connection for a 5-10 Hz short-lived velocity stream."""

    def __init__(self, host: str, port: int, *, timeout_s: float) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.socket: socket.socket | None = None
        self.reader: Any | None = None

    def close(self) -> None:
        if self.reader is not None:
            try:
                self.reader.close()
            except OSError:
                pass
        if self.socket is not None:
            try:
                self.socket.close()
            except OSError:
                pass
        self.reader = None
        self.socket = None

    def _connect(self) -> None:
        if self.socket is not None:
            return
        connection = socket.create_connection(
            (self.host, self.port), timeout=self.timeout_s
        )
        connection.settimeout(self.timeout_s)
        self.socket = connection
        self.reader = connection.makefile("rb")

    def send(self, linear_mps: float, angular_radps: float) -> dict[str, object]:
        request_id = f"focus-vel-{uuid.uuid4().hex[:16]}"
        request = joy_command_line(
            linear_mps,
            angular_radps,
            request_id=request_id,
        )
        last_error: Exception | None = None
        for attempt in range(2):
            try:
                self._connect()
                assert self.socket is not None and self.reader is not None
                self.socket.sendall(request)
                for _ in range(8):
                    raw = self.reader.readline()
                    if not raw:
                        raise ConnectionError("WATER closed the velocity stream")
                    response = json.loads(raw)
                    if (
                        isinstance(response, dict)
                        and response.get("type") == "response"
                        and response.get("uuid") == request_id
                    ):
                        return require_water_ok(
                            response, command="/api/joy_control"
                        )
                raise TimeoutError("no matching WATER velocity response")
            except (
                OSError,
                ValueError,
                json.JSONDecodeError,
                ConnectionError,
                TimeoutError,
            ) as exc:
                last_error = exc
                self.close()
                if attempt == 0:
                    continue
        assert last_error is not None
        raise last_error


def parse_water_health(response: dict[str, object]) -> dict[str, object]:
    payload = require_water_ok(response, command="/api/robot_status")
    results = payload.get("results")
    if not isinstance(results, dict):
        raise ValueError("WATER robot_status returned malformed results")
    estop = bool(results.get("estop_state") or results.get("hard_estop_state"))
    error_code = str(results.get("error_code", "00000000"))
    error_free = error_code in {"0", "00000000", "", "None", "none"}
    return {
        "ready": not estop and error_free,
        "estop_engaged": estop,
        "error_code": error_code,
        "move_status": str(results.get("move_status", "")),
        "battery_percent": results.get("power_percent"),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default="/focus_guarded_cmd_vel")
    parser.add_argument(
        "--status-topic", default="/focus/water/cmd_bridge_status"
    )
    parser.add_argument("--robot-host", default="192.168.10.10")
    parser.add_argument("--tcp-port", type=int, default=31001)
    parser.add_argument("--send-rate-hz", type=float, default=5.0)
    parser.add_argument("--status-rate-hz", type=float, default=2.0)
    parser.add_argument("--input-timeout-s", type=float, default=0.30)
    parser.add_argument("--tcp-timeout-s", type=float, default=0.35)
    parser.add_argument("--max-linear-mps", type=float, default=0.15)
    parser.add_argument("--max-angular-radps", type=float, default=0.40)
    parser.add_argument("--enable-live-water-output", action="store_true")
    parser.add_argument("--operator-confirmation", default="")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not args.input_topic.startswith("/") or not args.status_topic.startswith("/"):
        raise SystemExit("ROS topics must be absolute")
    if (
        args.send_rate_hz < 2.0
        or args.send_rate_hz > 10.0
        or args.status_rate_hz <= 0.0
        or args.input_timeout_s <= 0.0
        or args.input_timeout_s >= 0.5
        or args.tcp_timeout_s <= 0.0
    ):
        raise SystemExit("invalid rate/timeout configuration")
    # Validate limits before ROS or a hardware socket is created.
    sanitize_velocity(
        linear_x=0.0,
        linear_y=0.0,
        linear_z=0.0,
        angular_x=0.0,
        angular_y=0.0,
        angular_z=0.0,
        max_linear_mps=args.max_linear_mps,
        max_angular_radps=args.max_angular_radps,
    )
    live = bool(args.enable_live_water_output)
    if live and args.operator_confirmation != LIVE_CONFIRMATION:
        raise SystemExit(
            "live WATER output requires --operator-confirmation "
            + LIVE_CONFIRMATION
        )

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    class WaterCmdVelBridge(Node):
        def __init__(self) -> None:
            super().__init__("focus_water_cmd_vel_bridge")
            self.joy = WaterJoyClient(
                args.robot_host,
                args.tcp_port,
                timeout_s=args.tcp_timeout_s,
            )
            self.status_client = WaterTcpClient(
                args.robot_host,
                args.tcp_port,
                timeout_s=max(0.5, args.tcp_timeout_s),
            )
            self.command: SanitizedVelocity | None = None
            self.command_received_monotonic = 0.0
            self.water_health: dict[str, object] = {
                "ready": False,
                "estop_engaged": False,
                "error_code": "unobserved",
            }
            self.water_health_monotonic = 0.0
            self.last_send_ok_monotonic = 0.0
            self.last_sent = SanitizedVelocity(
                0.0, 0.0, True, "initial_zero"
            )
            self.last_reason = "startup"
            status_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.status_publisher = self.create_publisher(
                String, args.status_topic, status_qos
            )
            self.create_subscription(
                Twist, args.input_topic, self.on_twist, 10
            )
            self.create_timer(1.0 / args.send_rate_hz, self.send_tick)
            self.create_timer(1.0 / args.status_rate_hz, self.status_tick)
            self.get_logger().info(
                "WATER cmd_vel bridge ready: "
                f"live={live}, input={args.input_topic}, "
                f"limits=({args.max_linear_mps:.2f}m/s,"
                f"{args.max_angular_radps:.2f}rad/s), "
                f"watchdog={args.input_timeout_s:.2f}s"
            )

        def on_twist(self, message: Any) -> None:
            self.command = sanitize_velocity(
                linear_x=float(message.linear.x),
                linear_y=float(message.linear.y),
                linear_z=float(message.linear.z),
                angular_x=float(message.angular.x),
                angular_y=float(message.angular.y),
                angular_z=float(message.angular.z),
                max_linear_mps=args.max_linear_mps,
                max_angular_radps=args.max_angular_radps,
            )
            self.command_received_monotonic = time.monotonic()

        def water_ready(self, now: float) -> bool:
            return bool(
                self.water_health.get("ready")
                and now - self.water_health_monotonic
                <= max(1.0, 3.0 / args.status_rate_hz)
            )

        def send_tick(self) -> None:
            now = time.monotonic()
            velocity, reason = effective_velocity(
                self.command,
                received_monotonic=self.command_received_monotonic,
                now_monotonic=now,
                input_timeout_s=args.input_timeout_s,
                water_ready=self.water_ready(now),
            )
            self.last_reason = reason
            if not live:
                self.last_sent = velocity
                return
            try:
                self.joy.send(velocity.linear_mps, velocity.angular_radps)
                self.last_sent = velocity
                self.last_send_ok_monotonic = time.monotonic()
            except Exception as exc:  # noqa: BLE001 - watchdog must keep spinning
                self.joy.close()
                self.water_health["ready"] = False
                self.last_reason = f"water_send_failed:{type(exc).__name__}"
                self.get_logger().error(
                    f"WATER velocity send failed: {exc}",
                    throttle_duration_sec=2.0,
                )

        def status_tick(self) -> None:
            try:
                self.water_health = parse_water_health(
                    self.status_client.request("/api/robot_status")
                )
                self.water_health_monotonic = time.monotonic()
            except Exception as exc:  # noqa: BLE001
                self.water_health = {
                    "ready": False,
                    "estop_engaged": False,
                    "error_code": f"status_error:{type(exc).__name__}",
                }
            now = time.monotonic()
            command_age_s = (
                None
                if self.command_received_monotonic <= 0.0
                else max(0.0, now - self.command_received_monotonic)
            )
            zero_confirmed = bool(
                self.last_sent.zero
                and (
                    not live
                    or now - self.last_send_ok_monotonic
                    <= max(0.5, 2.0 / args.send_rate_hz)
                )
            )
            payload = {
                "schema_version": "focus-water-cmd-bridge-v1",
                "live": live,
                "ready": self.water_ready(now),
                "input_topic": args.input_topic,
                "command_age_s": command_age_s,
                "command_active": not self.last_sent.zero,
                "velocity_zero_confirmed": zero_confirmed,
                "last_reason": self.last_reason,
                "last_output": {
                    "linear_mps": self.last_sent.linear_mps,
                    "angular_radps": self.last_sent.angular_radps,
                },
                "water": self.water_health,
                "provenance": {
                    "api": "/api/joy_control",
                    "vendor_command_ttl_s": 0.5,
                    "classification": (
                        "observed_live_output" if live else "dry_run_preview"
                    ),
                },
            }
            message = String()
            message.data = json.dumps(payload, separators=(",", ":"))
            self.status_publisher.publish(message)

        def stop(self) -> None:
            if live:
                for _ in range(3):
                    try:
                        self.joy.send(0.0, 0.0)
                    except Exception:  # noqa: BLE001
                        pass
                    time.sleep(0.05)
            self.joy.close()

    rclpy.init()
    node = WaterCmdVelBridge()
    exit_code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
        exit_code = 3
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
