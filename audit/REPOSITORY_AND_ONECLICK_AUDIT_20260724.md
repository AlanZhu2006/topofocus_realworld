# Repository and persistent one-click audit — 2026-07-24

## Scope and evidence boundary

This audit followed the excluded `official-run01` engineering attempts. Its
scope was the local repository, deployment/startup chain and code-only robot
release synchronization—not a physical navigation run.

Baseline publication commit inspected:
`2b1371e7fb4583d488247cf978938f772e737579`.

Classifications used below:

- **observed**: command output or file bytes inspected in this workspace;
- **source-derived**: behavior established from inspected implementation and
  tests but not yet exercised on the physical robots;
- **unverified**: requires a future onsite run.

No file under `source/` or `dependencies/` was changed. No HM3D scene,
simulator dataset, overlay or SIF was downloaded.

## Mechanical repository audit

Observed before the publication commit containing this audit:

| Check | Result |
| --- | --- |
| baseline tracked paths | 601 |
| existing repository paths including staged/untracked publication candidates | 623 |
| Python publication candidates | 331 |
| shell publication candidates | 39 |
| Markdown publication candidates | 93 |
| JSON publication candidates | 16 |
| YAML/YML publication candidates | 75 |
| Python syntax | all 331 existing publication candidates parsed |
| shell syntax | all 39 existing publication candidates passed `bash -n` |
| JSON/YAML syntax | all repository JSON and YAML/YML parsed |
| local Markdown links | 71 project-owned documentation files checked; all targets exist |
| immutable source manifests | `manifests/source-files.sha256` passed |
| TinyNav snapshot manifests | tracked and untracked snapshot manifests passed |
| forbidden runtime/secret paths | none tracked |
| high-confidence credential scan | no match |
| files larger than 50 MiB | none tracked |
| whitespace check | tracked, staged and untracked project changes passed |
| Hub regression suite | 350 tests passed |

`shellcheck`, `ruff`, `mypy`, `yamllint` and `markdownlint` were not installed,
so no result is claimed for those tools. Their absence is not represented as a
pass.

The repository verifier was corrected to inspect staged and untracked project
files as well as the working-tree diff. It now parses JSON/YAML in addition to
Python and shell syntax. A second audit pass corrected its handling of a
deleted side of a rename and binary untracked media: only existing candidates
are sized/parsed, while binary assets still receive size and secret checks but
not text-whitespace rules.

## Startup-chain defects found

### Hard-coded physical session

The predecessor `realworld_oneclick.sh` embedded the July 24 v12 map paths,
calibration ID, transform IDs and tmux names. A new placement could therefore
silently reuse an old physical identity.

Fix: one persistent session schema now binds the Git commit, calibration
artifact identity, map boundaries, generated Hub policies, robot release
roots, remote calibration/base-camera paths, loopback tunnel endpoints and
all managed tmux identities. `debug` and `live` resolve only that manifest.

### Debug could bypass the live gates

The old debug path added both `--allow-blocked-shadow-input` and
`--allow-stale-shadow-input`. That was useful for forensic display but could
not prove live readiness.

Fix: the persistent debug path has no forensic override. It requires fresh,
synchronized, unblocked, command-capable and health-ready observations, then
hash-binds its shadow manifest to the session contract and Git commit. Live
resolution rejects a missing or drifting debug record.

### Shadow-manifest status mismatch

The first session validator expected
`shadow_only_no_motion_authority`; the implemented VLM tool actually finishes
with `complete_shadow_only`.

Fix: the validator now checks the implemented status and explicit
`realworld_session_id` / contract SHA-256 fields. A regression test covers the
real status.

### Old Foxglove process could own the port

The predecessor checked only whether port 8765 was listening. If an old relay
owned it, no new relay was launched and the operator saw an old or blank map.

Fix: the launcher checks exact map paths, both ports, `--fuse`, tmux identity
and preview health. A mismatched project relay is replaced. An unmanaged port
owner causes an explicit failure.

### Map process identity did not prove loaded code

A live tmux window could have the correct map path and still be a process
started before the current code commit.

Fix: map launch commands carry the full session Git object ID. A matching
daemon can be reused; a missing/mismatched marker causes a rebuild from the
immutable session sequence boundary. Map/summary files now share a snapshot
generation ID, and strict input freezing rejects an absent/mismatched
generation.

### Map recovery was blocked before the rebuild code

The first persistent draft resolved `debug`/`live` by requiring already
healthy map files. A missing or blocked map therefore failed before
`ensure_maps` could reconstruct it.

Fix: the launcher resolves and validates the immutable session, code and
same-session debug record with an explicit map-rebuild preparation mode. It
then stops any managed writer for those exact directories, rejects an
unmanaged writer and reconstructs from the saved sequence boundary. Strict
freshness/generation checks still run before the VLM or motion arming.

### A map directory did not independently bind its replay boundary

Transform and calibration metadata alone did not prove that a restarted
daemon used the session's original `start_after_sequence` or semantic
backend.

Fix: each map directory now has a
`focus-realworld-map-session-contract-v1` record containing the Git object,
session/robot IDs, relative map path, sequence boundary, transform,
calibration, goal, semantic backend and YOLO evidence mode. Resume, session
validation and frozen-input validation require exact equality.

### Map/dashboard readers could see a torn live file

`central_map.npz` and `map_summary.json` were atomic, but
`live_status.json` and `latest_rgb.jpg` were written directly.

Fix: live status and latest RGB now use sibling temporary files plus atomic
rename. The map NPZ and summary record one shared generation ID.

### Input selection had a time-of-check/time-of-use window

The old shell preflight checked current files, then the VLM independently
copied files that could have advanced.

Fix: `freeze_realworld_inputs.py` copies only while source inode/size/mtime
and JSON bytes remain stable. It validates map/calibration/transform,
YOLO-source sequence, payload sizes/hashes, health, age and cross-robot skew,
then atomically publishes an immutable accepted directory. The VLM receives
the frozen map hashes and source sequences.

### A stale Hub decision epoch could survive operator sequencing

The old launcher reused a matching Hub process. This complicated the proof
that an old v2 target could not be observed by a newly armed receiver.

Fix: every operator run starts a clean Hub process. Live first uses read-only
robot receivers, starts the clean `GOAL=true` Hub with no old in-memory v2
decision, collects observations in that epoch, freezes inputs and completes a
HOLD-only VLM round. Motion-capable receivers start only afterward. Every exit
returns the Hub and both robots to debug.

### Robot release roots were not checked by one-click

The local Git commit did not prove that the remote launchers/receivers matched.

Fix: calibration and debug/live build a checksum manifest over every tracked
file under `hub/src/focus_hub` and `hub/robot_overlay`, transmit that manifest
through the existing SSH/tmux pane and run `sha256sum -c` in each configured
release root before process changes.

### Board holdout did not prove the board moved

The solver validated residuals on a second pair but did not quantitatively
prove that the second board pose differed from the fit pose.

Fix: a holdout must now move at least 0.10 m or rotate at least 5 degrees. The
measurement and named `board_moved_independently` check are stored in the
artifact. Session promotion requires all named checks. Calibration output is
written atomically.

### Calibration was a multi-command conversational procedure

Selection, solving, deployment, map creation and debug were separate manual
steps.

Fix: `calibrate_realworld_session.sh` performs one interactive board-only
workflow, auto-selects fresh synchronized pairs, deploys one checksummed
artifact, creates the persistent session and invokes strict debug. It has a
failure trap that stops calibration-only streams. It never starts a Go2
bridge or WATER move receiver.

### Credentials appeared in process/tmux metadata

`focus_hub_up.sh` expanded the admin token into the tmux pane command and
printed newly generated robot tokens by default.

Fix: the pane reads the admin token from its chmod-600 runtime file at process
start. Generated robot token values are printed only with the explicit
`--print-generated-tokens` option. Existing/generated token, compact-token and
admin-token files are forced to mode 0600.

### Robot launchers could silently fall back to the predecessor transform

Direct launcher defaults still named the July v3 calibration and transforms.
The persistent wrapper supplied overrides, but an omitted environment value
could silently start the wrong physical identity.

Fix: both WSJ and Yunji v2 launchers now require explicit, filesystem-safe
transform/calibration IDs and absolute calibration paths. Their Hub endpoints
must remain loopback tunnel endpoints.

### Episode reports were insufficiently convenient for SR/SPL collection

The scorer existed, but the physical episode report did not preserve an
explicit start pose and no tool converted one trial plus operator evidence
into the 4 × 5 result file.

Fix: navigation events now carry the robot-local episode start pose. The
controller report emits an evaluation seed with start/stop pose, accumulated
path and planner STOP evidence. `record_realworld_trial.py` requires the
surveyed shortest paths, goal-region judgments and independent terminal
evidence, appends the trial atomically and immediately writes incomplete or
complete SR/SPL progress.

The audit then found one more controller edge: after one robot arrived, the
coordination HOLD could replace its latest `ARRIVED` event before report
generation. The controller now freezes each observed `ARRIVED` event for
evaluation while still allowing final runtime state to advance. The recorder
also rejects non-live/wrong-schema reports, non-boolean STOP evidence and
non-finite path/pose values.

## New operator contract

The authoritative instructions are
[`hub/docs/ONECLICK_SESSION_WORKFLOW.md`](../hub/docs/ONECLICK_SESSION_WORKFLOW.md).
The three entry points are:

```bash
# New physical placement: calibration + strict debug
bash hub/scripts/calibrate_realworld_session.sh \
  --session-id <unique-id> \
  --operator-confirmation OPERATOR_PRESENT_AND_BOARD_ONLY

# Repeat strict no-motion gate
bash hub/scripts/realworld_oneclick.sh \
  --session-file current --mode debug --goal-category chair

# One supervised physical episode
bash hub/scripts/realworld_oneclick.sh \
  --session-file current --mode live \
  --scene-id <scene> --episode-id <episode> --goal-category chair \
  --operator-confirmation OPERATOR_PRESENT_AND_ROBOTS_CLEAR
```

The operator does not need an agent to substitute map/calibration/tmux IDs.
Physical motion still requires one fresh confirmation per invocation; that is
an intentional robot-authority boundary, not missing automation.

## Verification result and remaining physical gate

Observed locally: repository verification and the complete Hub test suite
pass with these changes.

Source-derived: the new session manager, calibration wrapper, strict input
freezer, exact Foxglove replacement, remote-tree checksum gate, live cleanup
and SR/SPL recorder follow the tested contracts.

Unverified: neither the persistent calibration workflow nor the refactored
one-click live path has yet run against both physical robots. No historical
episode is promoted, and SR/SPL remains unavailable until a new session
passes debug and terminal evidence is recorded.

## Final robot synchronization

Observed deployment source:

- Git object:
  `90dd8fe43dad16515017fe4fd9bd017e02277bf6`
  (`feat: add persistent physical experiment sessions`);
- archive construction: `git archive <object> hub`;
- archive contents: 326 entries (300 files and 26 directories), 2,133,790
  bytes;
- archive SHA-256:
  `4298f048591ca8b6a7cfa9d9aa3fe3ba34058965329f32bfba827af72f2a097f`;
- path inspection: no absolute path, `..` component or symbolic link;
- critical runtime manifest: all 175 tracked files under
  `hub/src/focus_hub` and `hub/robot_overlay`, 23,560 bytes, SHA-256
  `bc16cbaa3337b1e27237de64e88d5e0c94cc7e81e64365f95d625f86142bb6bf`.

The archive and manifest were served only from local
`127.0.0.1:8188`. The two already-running SSH reverse tunnels exposed that
listener only as each robot's `127.0.0.1:18089`; no LAN-facing transfer
listener or additional SSH connection was created. The temporary HTTP tmux
session, archive, manifest and directory were removed after verification.
The original local debug Hub was then recreated on `127.0.0.1:8188`; its
observed health response again reported `goal_output_enabled=false` for both
robots.

Observed remote results:

| Host | Release root | Post-extraction verification | Process state |
| --- | --- | --- | --- |
| WSJ `tegra-ubuntu` | `/home/nvidia/topofocus_buildmap_v2_20260723` | archive hash/326 entries matched; all 175 runtime hashes matched; 196 Python files parsed and 39 shell files passed `bash -n` under Python 3.10.12 at `2026-07-23T20:00:18Z` | receiver 0; Go2 bridge 0 |
| Yunji `nyush-nuc` | `/home/nyu/topofocus_buildmap_v2_20260723` | archive hash/326 entries matched; all 175 runtime hashes matched; 196 Python files parsed and 39 shell files passed `bash -n` under Python 3.10.12 at `2026-07-23T20:00:17Z` | receiver 0; live service inactive; debug service inactive |

Before transfer, WSJ had 5,222,384 KiB available in its release filesystem
and Yunji had 57,816,768 KiB. Both release roots existed. Neither robot-side
process was started, stopped or restarted by the synchronization.

Two command-wrapper errors were kept outside the success claim. The first
verification wrapper interpreted a no-match `pgrep` as a `pipefail` error and
stopped before extraction. The corrected wrapper extracted the already
hash-verified archive and passed the 175-file checksum and shell checks, but
its inline Python quotation was stripped by the remote shell. A final
quotation-free parse command then produced the successful results above.
Neither wrapper created a motion process.

This proves byte availability for the persistent-session implementation. It
does **not** prove that a robot process has loaded those bytes, that the new
calibration wrapper succeeds on the cameras, or that a physical episode
works. Those remain onsite gates.
