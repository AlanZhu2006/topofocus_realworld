# Scene 01 chair formal experiment 04 success — 2026-07-25

## Archived outcome

The operator classified session
`20260725-lab21-wallfix-imudebounce-3a2d953`, scene `scene01`, episode
`trial-wallfix-imudebounce-r1` as **formal experiment 04: success** after the
run. The exact operator statement was:

> 可以归档 作为第四次正式实验并且成功

This post-run annotation is joined to, but does not rewrite, the immutable
automatic episode report. That report was intentionally emitted with
`official_success_verified=false` before independent/operator classification
and therefore calls the terminal bundle an unverified candidate.

Yunji (`robot-1`) received a `SEMANTIC_REGION` target for `chair`, reported
`ARRIVED` with reason `LOCAL_PLANNER_ARRIVED`, confirmed zero velocity, and
caused both robots to enter HOLD. The time-aligned terminal Yunji RGB visibly
contains the large white chair. The run completed four source rounds and
published 19 versioned high-level batches.

## SR and SPL

This result enters the separate `physical_0p5m_protocol` track. It does not
enter the pre-surveyed standard track.

| Metric | Value |
| --- | ---: |
| Per-episode SR | `1` |
| Yunji actual path `P` | `3.2102223806424663 m` |
| Start pose | `(-0.2471591459607408, 0.07065533898262284) m` |
| Arrival pose | `(2.695039579179446, -0.8063659845414222) m` |
| Start-to-arrival displacement `D` | `3.070130248072939 m` |
| Source-compatible SPL | `D / max(D, P) = 0.956360614325575` |
| Standard SPL | `null` |

The standard SPL remains unavailable because no independently surveyed
shortest collision-free path `L` was recorded before motion. No `L` is
inferred from the executed trajectory or measured after the run.

After adding this episode, the machine-readable physical track contains three
runtime-bound successful metric samples: `SR=3/3=1.0` and mean
source-compatible `SPL=0.816269191777991`. The operator-provided formal
experiment ordinal is `04`; it is not used as the aggregate metric
denominator because earlier excluded/unbound attempts remain outside this
metric track.

## Runtime behavior

The automatic report preserves these observed/source-derived facts:

- runtime interval: `2026-07-25 16:21:32.351902076 CST` through
  `16:24:10.104611768 CST`;
- session Git commit:
  `3a2d953e7ab5c1b7891829b2101dc5bb52126e77`;
- calibration:
  `shared-board-odin1-20260725-lab21-wallfix-dualreanchor2-v1`;
- strict no-motion debug passed before live;
- `robot-0` accumulated `1.2249606477342507 m`, then held with zero
  velocity;
- `robot-1` accumulated `3.2102223806424663 m`, emitted semantic
  `ARRIVED`, and stopped with zero velocity;
- the terminal-evidence status is `complete_post_arrival_candidate`;
- cleanup disabled Hub goal output and removed the two chassis command paths
  while preserving the warm read-only observation/map core.

The final round also records the minimal fixes exercised by this experiment:
WSJ used a source-ranked remaining frontier after the VLM-selected frontier
failed the unchanged clearance guard, and one bounded WSJ IMU skip was
debounced. These are commits `05447b9` and `3a2d953`, respectively.

## Provenance

Runtime files remain ignored by Git. Their paths, byte sizes, SHA-256 values
and evidence classes are committed here and in
`manifests/realworld_experiment_progress.json`.

| Evidence | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/sessions/20260725-lab21-wallfix-imudebounce-3a2d953/session.json` | 4,338 | `a93931c27db02fbcb61b0edf26e37281b6b3fc13aa17c209cd5a6687af6b369f` | observed/source-derived session contract |
| `hub/runtime/calibration_sessions/20260725-lab21-wallfix-dualreanchor2/shared_frame.json` | 21,485 | `c688becd72e9f320014812c57e3aae15798ad4e731978a063f5bdf2137bae8ed` | observed/source-derived shared-frame calibration |
| `hub/runtime/oneclick_20260725-lab21-wallfix-imudebounce-3a2d953_debug_debug-wallfix-imudebounce_20260725_161937_918937881/shadow/shadow_manifest.json` | 18,831 | `018ab08c62db3cffb58758d6dafe159880c8ff759f7b683e2514b8a1febcb9c3` | observed strict no-motion full-stack debug manifest |
| live `episode_report.json` | 20,592 | `2f9036f80879d142f0beac4f6165a40be83634630d41d84b4aa26c76de1df169` | source-derived from observed robot navigation feedback |
| live `scene_manifest.json` | 35,184 | `c534d037c619b3d47309cc405ba4681d91b26d645e87ea82587d1b9d1d678033` | source-derived continuous scene manifest |
| live `controller_events.jsonl` | 29,368 | `8b2ce0e5357960117196d3b7a5b6aec9df8086d258eae38175de068c58b7878b` | source-derived controller event log |
| live `batch_018_round_3_goal.json` | 6,666 | `7f3ca6460cc5c9c50d9185561d696f0eefe3ef68a7596c2fdfaf534665efe747` | source-derived applied high-level target batch |
| live `terminal/terminal_evidence.json` | 10,440 | `ed3dd6ae80c72900b71175e65e5b98fd41c8415eb6f6755e1557631d631763b3` | source-derived terminal evidence index |
| live `terminal/robot-1/rgb.jpg` | 134,349 | `c9484cbb97df19675fc30af9085fd0620aa4a8b03c682ff28c983249abbc512f` | observed post-arrival RGB visibly containing the chair |

Here, `live` abbreviates:

`hub/runtime/oneclick_20260725-lab21-wallfix-imudebounce-3a2d953_live_scene01_20260725_162131_804477607/`

No existing video was renamed, modified or retrospectively bound to this
episode. No file under immutable `source/` or `dependencies/` was changed.

## Operator metric addendum — 2026-07-25

The original archive boundary above is preserved. The operator subsequently
reported:

> 还有那个具体的standard spl距离 我量了大概是3.25m

and then explicitly corrected the evidence classification:

> 不是 你就把3.25m作为独立测量最短可行路径来算

This addendum therefore records `L≈3.25 m` as the independently measured
shortest feasible path for formal experiment 04. The updated official metric
is:

```text
standard SPL = S * L / max(L, P)
             = 1 * 3.25 / max(3.25, 3.2102223806424663)
             = 1.0
```

The independently measured `L` is approximate and exceeds the
odometry-derived `P` by `0.0397776193575337 m`; the standard formula's
denominator therefore caps the result at `1.0`. The machine-readable standard
track now contains this one successful episode: `SR=1/1=1.0`, mean standard
`SPL=1.0`.
