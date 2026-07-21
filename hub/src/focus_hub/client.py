from __future__ import annotations

from pathlib import Path

from .models import Decision, DecisionAck, ObservationAck, ObservationMetadata


class HubClient:
    """Small transport client; ROS synchronization deliberately lives outside it."""

    def __init__(self, base_url: str, robot_id: str, token: str, *, timeout_s: float = 5.0) -> None:
        import httpx

        self.robot_id = robot_id
        self._client = httpx.Client(
            base_url=base_url.rstrip("/"),
            headers={"X-Robot-Token": token},
            timeout=timeout_s,
        )

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> "HubClient":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def upload(
        self,
        metadata: ObservationMetadata,
        rgb_path: Path,
        depth_path: Path,
    ) -> ObservationAck:
        with rgb_path.open("rb") as rgb_handle, depth_path.open("rb") as depth_handle:
            return self.upload_bytes(metadata, rgb_handle.read(), depth_handle.read())

    def upload_bytes(
        self,
        metadata: ObservationMetadata,
        rgb: bytes,
        depth: bytes,
    ) -> ObservationAck:
        if metadata.robot_id != self.robot_id:
            raise ValueError("metadata robot_id does not match client")
        response = self._client.post(
            f"/v1/robots/{self.robot_id}/observations",
            data={"metadata_json": metadata.model_dump_json()},
            files={
                "rgb": ("rgb", rgb, "image/jpeg" if metadata.rgb_encoding == "jpeg" else "image/png"),
                "depth": ("depth", depth, "image/png"),
            },
        )
        response.raise_for_status()
        return ObservationAck.model_validate(response.json())

    def latest_decision(self) -> Decision:
        response = self._client.get(f"/v1/robots/{self.robot_id}/decisions/latest")
        response.raise_for_status()
        return Decision.model_validate(response.json())

    def acknowledge(self, ack: DecisionAck) -> None:
        if ack.robot_id != self.robot_id:
            raise ValueError("ack robot_id does not match client")
        response = self._client.post(
            f"/v1/robots/{self.robot_id}/decisions/{ack.decision_id}/ack",
            json=ack.model_dump(mode="json"),
        )
        response.raise_for_status()

