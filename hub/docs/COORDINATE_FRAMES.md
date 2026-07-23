# Shared coordinate contract

Transforms use the same notation as the audited TinyNav semantic mapper:

```text
p_A = T_A_B * p_B
```

Every accepted observation supplies `T_shared_world_camera_color` and its `transform_version`. A robot that localizes in its own saved map computes it from an externally established map alignment:

```text
T_shared_world_camera_color =
    T_shared_world_robot_map * T_robot_map_camera_color
```

The hub never fuses two values merely because both frames are named `map` or `world`. TinyNav `world` resets between sessions and each robot's saved `map` is local to that mapping run.

To convert a hub target back to robot-local map coordinates:

```text
p_robot_map = inverse(T_shared_world_robot_map) * p_shared_world
```

The robot-side goal guard must use exactly the `transform_version` named by the decision. A mismatched or unknown version is a rejection, not an implicit latest-transform lookup.

## Camera and body

The current TinyNav documentation establishes the color optical pose chain but also records that no calibrated `base_link -> camera_link` transform exists. The wire contract therefore distinguishes:

- `T_shared_world_camera_color`: required for central RGB-D mapping;
- `T_base_link_camera_color`: required before an observation may be command-capable.

Body pose follows from:

```text
T_shared_world_base_link =
    T_shared_world_camera_color * inverse(T_base_link_camera_color)
```

`mapping_only=true` permits a frame without body extrinsics for G3 replay. `mapping_only=false` requires the extrinsic and is still insufficient by itself to enable motion: health, map version, command expiry and local safety all have to pass.

## Establishing the shared frame

The current two-robot session has an observed gravity-preserving board fit
plus an independently moved-board holdout. Its calibration ID is
`shared-board-odin1-20260723-v3`; exact deployment transforms and maps are in
[CURRENT_STATUS.md](../../CURRENT_STATUS.md). It is session-bound and must not
be treated as a permanent transform after a robot, mount or starting placement
moves.

For a new session, G4 must choose and document one measurement source, for
example:

- a surveyed common start pose for each robot;
- fiducials visible to both robots;
- an external tracking system;
- map registration followed by a separately checked physical alignment.

The selected method must produce uncertainty, a stable version ID, and a repeatable validation showing that the same wall/landmark from both robots overlays without rotation, reflection or scale error. An ICP result alone is not accepted as a safety calibration without an independent check.
