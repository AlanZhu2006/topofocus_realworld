# WSJ TinyNav reproducibility snapshot

This directory preserves the source state that was actually used by WSJ on
2026-07-21 without vendoring the full TinyNav repository or publishing old
credentials.

## Required path

Start from the Apache-2.0 upstream repository and commit:

- URL: `https://github.com/UniflexAI/tinynav.git`
- base commit: `576c082e69580f618a5ff313a3e74f3672abb69f`
- patch: `tinynav-required.patch`
- sanitized patch SHA-256:
  `83b0e247d8c7e808894cd14d086281efe5911131574f1da4694fdbbfda417e05`

The patch is source-derived from the exact final tree at the observed local
WSJ commit `29f26bc058886ff450f02cdc0d6e9977e1c57010`. It includes these five
WSJ-only commits, none of which was reachable from the inspected
`AlanZhu2006/go2_tinynav` remote on 2026-07-21:

1. `9647f55f0ebc67987bdea8279656a439a9cbe8aa` — Jetson/Go2 deployment.
2. `933fce54ae65e775a1262c346180341f5657c0e4` — deployment completion.
3. `a9710abbec870b3c034891fa906f4862b4721abe` — decouple IMU callbacks.
4. `39783be71d76538ce6b4b0b2c3f97d2bdda32377` — reject incomplete IMU intervals.
5. `29f26bc058886ff450f02cdc0d6e9977e1c57010` — recover by re-anchoring after an invalid interval.

Two safety edits were then made to the flattened patch. The legacy VNC helper
now requires `TINYNAV_VNC_PASSWORD` instead of falling back to `nvidia`, and it
does not echo the supplied password. These edits do not touch camera,
perception, mapping, or robot control behavior.

Use `../bootstrap_go2.sh`; it validates `manifest.sha256`, applies the patch to
the pinned base, and creates a local reproducibility commit. It does not launch
ROS or the robot.

The clean-clone gate produced deterministic commit
`d9f88ed876bd08e35b8c57b65e6589b10170389f` with tree
`d8538a6c032cce4a7b403dbcfe60a0bce09d5947`. The bootstrap script rejects a
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
`8cc18159c920dc0b5185fe81bd34452676bbad53` with tree
`46f4b7cd8c3bdc2ed3729cd56f3d8857aa9d41df`.

## Credential handling

The original five mail patches and Git bundle were inspected locally. An early
commit contained a historical default VNC password, so those raw history files
are intentionally excluded from Git and retained only in the ignored local
`data/private_provenance/` directory. The public flattened patch represents the
same final functional tree plus the two safety edits above.
