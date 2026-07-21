"""Session-start shared-frame calibration (real-machine analogue of Habitat's
per-episode ``episode.start_position`` reset).

Upstream, every episode resets both agents to the same ``start_position``
(only rotation is randomized — see ``merge_sim_episode_config`` in
habitat-lab), so both agents' ``EpisodicGPSSensor`` readings are already
comparable without any further alignment. Our robots have no such shared
ground truth: each one's pose is only ever known relative to wherever its
own odometry/SLAM stack happened to initialize (TinyNav's
``/slam/keyframe_odom``, Yunji's ``/sensors_fusion/odom``). Faithfully
reproducing the upstream semantics on real hardware means recovering the
same *effect* upstream gets for free: one fixed rigid transform per robot
that maps its own local pose stream into a common ``shared_world`` frame.

The approach: at a session-start instant when the robots are physically
co-located (or separated by a known, measured offset), each robot's own raw
pose (already expressed in that robot's own local frame — exactly what its
sender already publishes as ``pose.shared_T_camera``, despite the
aspirational field name) pins down that one fixed transform. Everything
after that instant is a straight matrix multiply; this module has no
robot-specific knowledge and works identically for any two robots' pose
streams.

Caveat carried over from upstream's own accepted limitation, but made worse
here: Habitat's GPS/compass ground truth never drifts. Live odometry does.
The transform computed here is only as good as the sync-instant poses it
was built from; nothing in this module corrects for drift afterward.
"""
from __future__ import annotations

from collections.abc import Sequence

from .geometry import compose_rigid, invert_rigid

IDENTITY: tuple[float, ...] = (
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
)


def compute_shared_frame_transform(
    reference_pose_at_sync: Sequence[float],
    other_pose_at_sync: Sequence[float],
    reference_to_other_offset: Sequence[float] | None = None,
) -> tuple[float, ...]:
    """Returns ``T_shared_world_from_other_odom``.

    ``shared_world`` is defined, by convention, as the reference robot's own
    local pose frame — so the reference robot's own poses need no further
    transform; only the other robot's poses need this one fixed correction
    applied (via :func:`apply_shared_frame_transform`).

    ``reference_pose_at_sync`` / ``other_pose_at_sync``: each robot's own raw
    pose, in its own local frame, at a moment the two robots were physically
    co-located.

    ``reference_to_other_offset``: optional measured rigid transform from the
    reference robot's pose frame to the other robot's pose frame at that same
    instant, for when they could not be placed exactly coincident (e.g. two
    robots parked side by side facing the same way). Defaults to identity,
    i.e. treats the two poses as referring to the same physical point.
    """
    if reference_to_other_offset is None:
        reference_to_other_offset = IDENTITY
    other_pose_at_sync_in_shared_world = compose_rigid(reference_pose_at_sync, reference_to_other_offset)
    return compose_rigid(other_pose_at_sync_in_shared_world, invert_rigid(other_pose_at_sync))


def apply_shared_frame_transform(
    shared_world_from_other_odom: Sequence[float],
    other_pose_at_t: Sequence[float],
) -> tuple[float, ...]:
    """Maps one of the other robot's own-frame poses into ``shared_world``."""
    return compose_rigid(shared_world_from_other_odom, other_pose_at_t)
