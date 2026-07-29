# Robot 0 TinyNav reproducibility snapshot

This directory preserves the source state that was actually used by Robot 0 on
2026-07-21 without vendoring the full TinyNav repository or publishing old
credentials.

## Required path

Start from the Apache-2.0 upstream repository and commit:

- URL: `https://github.com/UniflexAI/tinynav.git`
- base commit: `576c082e69580f618a5ff313a3e74f3672abb69f`
- deployment patch: `tinynav-required.patch`
- deployment patch SHA-256:
  `83b0e247d8c7e808894cd14d086281efe5911131574f1da4694fdbbfda417e05`
- formal-runtime patch: `wsj-runtime-required.patch`
- formal-runtime patch SHA-256:
  `99b1e98c16fc2f1c23a2ec853c9e4fef6d50526487c75a62837d6d11d99390cf`

The patch is source-derived from the exact final tree at the observed local
Robot 0 commit `29f26bc058886ff450f02cdc0d6e9977e1c57010`. It includes these five
Robot 0-only commits, none of which was reachable from the inspected
`AlanZhu2006/go2_tinynav` remote on 2026-07-21:

1. `9647f55f0ebc67987bdea8279656a439a9cbe8aa` — Jetson/Go2 deployment.
2. `933fce54ae65e775a1262c346180341f5657c0e4` — deployment completion.
3. `a9710abbec870b3c034891fa906f4862b4721abe` — decouple IMU callbacks.
4. `39783be71d76538ce6b4b0b2c3f97d2bdda32377` — reject incomplete IMU intervals.
5. `29f26bc058886ff450f02cdc0d6e9977e1c57010` — recover by re-anchoring after an invalid interval.

Two safety edits were made to the flattened deployment patch. The legacy VNC helper
now requires `TINYNAV_VNC_PASSWORD` instead of falling back to `nvidia`, and it
does not echo the supplied password. These edits do not touch camera,
perception, mapping, or robot control behavior.

The formal-runtime patch exposes the Go2 bridge watchdog as
`GO2_TIMEOUT_SEC` and passes it to the ROS node as `timeout_sec`. Its default
is the deployed `0.35 s`, so the reconstructed bridge is byte-identical to
the bridge locked by `start_wsj_buildmap_v2.sh`.

Use `../bootstrap_go2.sh`; it validates `manifest.sha256`, applies both patches
to the pinned base, and creates a local reproducibility commit. It does not
launch ROS or the robot.

The clean-clone gate produced deterministic commit
`a6290559b13cedf19c05f7ec64ff91a29b685cbd` with tree
`5281e70451f2f9cc1d5f5464315d803f6f0972bd`. The bootstrap script rejects a
different result.

## Optional experimental archive

`wsj-working-tree.patch` and `working-tree-files/` preserve the separate dirty
working tree that was observed under `/home/nvidia/twork/tinynav`:

- exact tracked diff SHA-256:
  `ca73cf70622606ea5cf6d8120a272ca3fd38282c58834216b05f24dd0c2aa322`;
- 81 untracked source/documentation files, each listed in
  `untracked.sha256`;
- generated `__pycache__` and `.pytest_cache` files are deliberately absent.

This experimental semantic package is not needed for the verified native
TinyNav BuildMap adapter. Some archived navigation scripts can start a Unitree
command bridge when explicitly invoked; merely applying the overlay does not
run them. The default bootstrap omits this layer.

The optional clean-clone gate produced commit
`f9e9c1bce787b5cc3a34fb149931b4f101b6adf8` with tree
`46f4b7cd8c3bdc2ed3729cd56f3d8857aa9d41df`.

## Credential handling

The original five mail patches and Git bundle were inspected locally. An early
commit contained a historical default VNC password, so those raw history files
are intentionally excluded from Git and retained only in the ignored local
`data/private_provenance/` directory. The public flattened patch represents the
same final functional tree plus the two safety edits above.
