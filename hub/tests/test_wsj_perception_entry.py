from pathlib import Path

import pytest

from hub.robot_overlay.wsj_perception_entry import bounded_integer, sha256_file


def test_bounded_integer_uses_default(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("FOCUS_TEST_DEPTH", raising=False)
    assert (
        bounded_integer(
            "FOCUS_TEST_DEPTH", default=400, minimum=50, maximum=2_000
        )
        == 400
    )


def test_bounded_integer_rejects_unbounded_history(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("FOCUS_TEST_DEPTH", "10000")
    with pytest.raises(ValueError, match=r"must be in"):
        bounded_integer(
            "FOCUS_TEST_DEPTH", default=400, minimum=50, maximum=2_000
        )


def test_sha256_file(tmp_path: Path) -> None:
    source = tmp_path / "source.py"
    source.write_bytes(b"pinned source\n")
    assert (
        sha256_file(source)
        == "073bd3c4ab4735908691f35310ecc19e8c1ba1bb993fd74f6738e4d0f8dcef72"
    )
