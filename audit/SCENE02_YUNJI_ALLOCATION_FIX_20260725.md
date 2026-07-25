# Scene 02 Yunji allocation correction — 2026-07-25

## Scope and classification

This record covers the stopped engineering episode
`scene02-plant-run01` under session
`scene02-plant-20260725-201949-wsjfix1`. It is not an SR/SPL sample.

- Robot motion, camera observations, poses and frozen maps are **observed**.
- The guard reconstruction below is **source-derived** from those frozen
  artifacts.
- The correction is regression-tested and replayed without robot contact, but
  remains **physically unverified** until a new supervised episode.

No file under immutable `source/` or `dependencies/` changed.

## Preserved evidence

All paths are below
`hub/runtime/oneclick_scene02-plant-20260725-201949-wsjfix1_live_scene02-plant_20260725_205749_887253521/`.
The runtime directory remains intentionally untracked.

| Artifact | Size | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `episode_report.json` | 10,205 B | `413202307aa70b9b41e1be6756a194ddf1abdc63bc0b0456b2bbf4d44693e64c` | source-derived report over observed feedback |
| `controller_events.jsonl` | 25,428 B | `b0e42f83f998947bf95fac0f1c2af8e421996cc6e2c162017b2c312d2d24bbdf` | source-derived Hub event log |
| `round_00_step_000/vlm_candidate_batch.json` | 4,462 B | `7955a7393bc77006ca1d544e91aea856ade76f5b14c5719fe319cc587482d204` | unmodified source/VLM candidates |
| `round_00_step_000/shadow/shadow_manifest.json` | 20,289 B | `3c72d8788cd3f35f2fd62a02f40760f38cc02a48027e4ab322b07b6da4914568` | frozen source-derived VLM manifest |
| `round_00_step_000/shadow/fused_decision_map.npz` | 21,992 B | `6fbaf182972cb0c6884a17c676fe7023ccc8980a19666f88a5a5c26da3a879cb` | frozen fused map |
| `round_00_step_000/accepted/wsj/central_map.npz` | 24,895 B | `ab084e38262da851b8e281cfa622efd7f4867094c36a697d046cd21946ce5437` | frozen WSJ map |
| `round_00_step_000/accepted/yunji/central_map.npz` | 24,533 B | `6baa1271381948ad60ebd0462c69e60d707aa4f5d1c47f925e960d2008043040` | frozen Yunji map |

## Observed cause

Yunji's first target, frontier `B` at shared-world
`(1.0756559132545487, 4.225607753428049)`, was the open forward corridor
visible in its RGB input. The original fused-map clearance check passed it.
The route guard then measured `0.589132 m < 0.9 m` between the two candidate
corridors and selected WSJ by round-order priority, so Yunji received HOLD.

After WSJ moved, global frontier re-extraction moved Yunji's selected `B`
target through `(0.225656, 6.175608)`, `(-2.824344, 6.375608)` and
`(-1.174344, 7.025608)`. Those later targets were not in Yunji's own observed
free component. The old guard described the resulting endpoint rejection as
a footprint-clearance failure even though the corridor directly in front of
Yunji remained physically open.

The same first-round detector inputs contained `potted plant` confidence
`0.556679` for WSJ and `0.877521` for Yunji.

## Corrective behavior

The deployment adapter now:

1. evaluates each assigned frontier against that robot's own frozen map in
   the shared frame and requires the approach cell to belong to its
   robot-local known-free component;
2. measures arrival-disk intersection against the 5 cm grid-cell footprint,
   rather than only the cell centre;
3. when otherwise safe routes must be serialized, chooses the robot with
   stronger current goal-category detector evidence before falling back to
   rotating source order.

The unchanged first-round artifacts replay as follows:

| Check | Source-derived replay |
| --- | ---: |
| Yunji required clearance | `0.34 m` |
| Yunji source arrival radius | `0.50 m` |
| Nearest reachable safe-cell footprint | `0.491093 m` |
| Reachable safe approach cells | `3` |
| Clearance-guard active robots | `robot-0`, `robot-1` |
| Conflict-serialized leader | `robot-1` (Yunji) |
| Published modes after both guards | WSJ `HOLD`, Yunji `GOAL` |

The replay contacted neither robot and issued no command.

## Verification

- 20 targeted tests passed.
- The complete Hub suite passed: 481 tests, with one unrelated third-party
  deprecation warning.
- Ruff, Python compilation and `git diff --check` passed.

