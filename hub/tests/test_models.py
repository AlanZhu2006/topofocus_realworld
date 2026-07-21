from __future__ import annotations

import pytest
from pydantic import ValidationError

from focus_hub.models import RigidTransform


def test_reflection_is_not_a_rigid_rotation():
    with pytest.raises(ValidationError):
        RigidTransform(
            parent_frame="shared_world",
            child_frame="camera",
            matrix=(-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1),
        )


def test_last_row_is_checked():
    with pytest.raises(ValidationError):
        RigidTransform(
            parent_frame="shared_world",
            child_frame="camera",
            matrix=(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1),
        )

