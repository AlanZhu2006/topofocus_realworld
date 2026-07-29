"""Pinned executable-source identity for source-compatible Hub decisions.

The deployment adapter is reviewed against the immutable executable default
path under ``source/Focus_realworld``.  A shadow round must not silently claim
that review after any of those inputs changes.  This module contains only
identity/provenance checks; it performs no network or robot I/O.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Mapping, Sequence


SOURCE_BEHAVIOR_CONTRACT_VERSION = "focus-source-behavior-contract-v1"
SOURCE_ARTIFACT_CLASSIFICATION = (
    "observed immutable authoritative source"
)


@dataclass(frozen=True)
class ReviewedSourceArtifact:
    relative_path: str
    size_bytes: int
    sha256: str

    def manifest_record(self) -> dict[str, object]:
        return {
            "source_path": self.relative_path,
            "size_bytes": self.size_bytes,
            "sha256": self.sha256,
            "classification": SOURCE_ARTIFACT_CLASSIFICATION,
        }


REVIEWED_SOURCE_ARTIFACTS: tuple[ReviewedSourceArtifact, ...] = (
    ReviewedSourceArtifact(
        "source/Focus_realworld/main.py",
        103808,
        "0d241151a9d1cfa77b53198117483287ca9585643fb3bb2df56e12d663f2d674",
    ),
    ReviewedSourceArtifact(
        "source/Focus_realworld/agents/vlm_agents.py",
        46500,
        "992f0174d50b6959d538a418c224907156f784ffd4b35b5ef67c02da3461bee0",
    ),
    ReviewedSourceArtifact(
        "source/Focus_realworld/arguments.py",
        14140,
        "66dc9a94459215d9a51d97bf8f195fd486759d7f34529c60e2a57999665a61d3",
    ),
    ReviewedSourceArtifact(
        "source/Focus_realworld/constants.py",
        28432,
        "6217a75db7e012602b70d6f5c76265cf90ff8d365a6176e5ce293fad5aafd106",
    ),
    ReviewedSourceArtifact(
        "source/Focus_realworld/src/SystemPrompt.py",
        22350,
        "10ac3c18a4bd5438298fdd76972efd362e686608f267700bc56dd8747a1e45f1",
    ),
    ReviewedSourceArtifact(
        "source/Focus_realworld/src/frontier_parser.py",
        10618,
        "7add79b0f8110cf11468c8e8d1d11127d46b84c8427bf5721e7f5a3ed995bcb7",
    ),
    ReviewedSourceArtifact(
        "source/Focus_realworld/src/vlm.py",
        13244,
        "ac0503b2f311c924a794f9c2cf678684e43b7a3797f6dc20a8d9e55a7dc713e8",
    ),
    ReviewedSourceArtifact(
        "source/Focus_realworld/utils/semantic_prediction.py",
        8684,
        "e7c2591235f69ef03917bce813516b53805ab8dbb529fa16b9d9fce7affec95d",
    ),
    ReviewedSourceArtifact(
        "source/Focus_realworld/utils/visualization.py",
        4381,
        "8a989f5ffcab28dbc1a2d000ed5cd144b434a36b7becde32d4d6b556a1a6e582",
    ),
)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def expected_source_artifact_records() -> list[dict[str, object]]:
    """Return the exact ordered records accepted by the v1 contract."""

    return [artifact.manifest_record() for artifact in REVIEWED_SOURCE_ARTIFACTS]


def observe_reviewed_source_artifacts(
    workspace: Path,
) -> list[dict[str, object]]:
    """Verify and record the reviewed immutable source files.

    A changed size or digest is a new source revision that needs an explicit
    review and contract version, not a round that may inherit the old claim.
    """

    root = workspace.resolve()
    records: list[dict[str, object]] = []
    for expected in REVIEWED_SOURCE_ARTIFACTS:
        path = (root / expected.relative_path).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise FileNotFoundError(expected.relative_path)
        observed_size = path.stat().st_size
        observed_sha256 = _sha256_file(path)
        if (
            observed_size != expected.size_bytes
            or observed_sha256 != expected.sha256
        ):
            raise RuntimeError(
                "reviewed source artifact drift: "
                f"{expected.relative_path}; "
                f"expected_size={expected.size_bytes}, "
                f"observed_size={observed_size}, "
                f"expected_sha256={expected.sha256}, "
                f"observed_sha256={observed_sha256}"
            )
        records.append(expected.manifest_record())
    return records


def validate_source_artifact_records(
    records: object,
) -> None:
    """Validate source identities preserved in a new shadow manifest."""

    if (
        not isinstance(records, Sequence)
        or isinstance(records, (str, bytes))
    ):
        raise ValueError("source artifact records must be an ordered list")
    expected = expected_source_artifact_records()
    if len(records) != len(expected):
        raise ValueError("source artifact record count differs from review")
    for index, (actual, wanted) in enumerate(
        zip(records, expected, strict=True)
    ):
        if not isinstance(actual, Mapping) or dict(actual) != wanted:
            raise ValueError(
                f"source artifact record {index} differs from review"
            )
