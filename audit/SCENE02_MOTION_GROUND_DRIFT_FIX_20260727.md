# Scene 02 motion-conditioned ground-drift fix — 2026-07-27

## Observed evidence

The supervised `plant` engineering run
`official-plant-20260727-1941-guardfix2` ended fail-closed after three
completed source rounds. Robot 0 travelled 3.710256 m and robot 1 travelled
3.059219 m before both acknowledged `HOLD`; Hub GOAL output was then disabled.

| Artifact | Bytes | SHA-256 | Classification |
|---|---:|---|---|
| `hub/runtime/oneclick_scene02-plant-20260727-193837-guardfix2_live_scene02-plant_20260727_194249_744388737/episode_report.json` | 9,920 | `ef67e9de6c228e28fe052a2cec3e4b386f077fc3e138a0c86386f00e081e738e` | observed sealed episode report |
| `hub/runtime/oneclick_scene02-plant-20260727-193837-guardfix2_live_scene02-plant_20260727_194249_744388737/round_03_step_074/freeze_rejections.jsonl` | 18,228 | `4b577e7467f34fc0681a8b3568b3d6935b16472bd345e41fd1f292e6845b0519` | observed sealed freeze rejection log |
| `hub/runtime/spool/robot-1/00000000000000274543/metadata.json` | 3,268 | `6ea2ae39c4879fc91964c2fe44a75c9d97e605a7305c55833c6ad117539f1a49` | observed append-only robot observation metadata |
| `hub/runtime/spool/robot-1/00000000000000274544/metadata.json` | 3,273 | `aaacc8b03b5eb94bd7dce206dca1455339110902660a9526ca735753f1a9c1f1` | observed append-only robot observation metadata |
| `hub/runtime/spool/robot-1/00000000000000274545/metadata.json` | 3,267 | `b6be89653941073dc02fbd76cd095845b521942860cfaa70bef841a9bf0170f3` | observed append-only robot observation metadata |

The rejection log identifies robot 1 ground drift at sequence 274545:
3.166 degrees tilt, 0.002 m height delta, and a three-frame streak. The three
outlying fits were 9.059, 6.688, and 3.166 degrees.

## Source-derived diagnosis and policy

Replaying sequences 274540–274550 against the frozen startup plane showed
that sequences 274543 and 274544 occurred during 0.131 m/21.33 degree and
0.119 m/21.54 degree inter-frame base motion. Sequence 274545 was the first
stationary outlier; later moving frames recovered to in-range fits. The old
counter therefore confused quadruped gait pitch/roll with a persistent camera
or calibration change.

The mapper now:

1. rejects every outlying floor frame before map integration;
2. defers and resets the irreversible drift streak while inter-frame robot
   motion exceeds 0.03 m or 2.0 degrees;
3. retains the original fail-closed latch after three consecutive stationary
   outliers.

The same policy is applied to both robots and is emitted in live status and
map-summary provenance.

## Verification classification

- Unit regression and repository tests: source-derived software verification.
- Replay of sequences 274540–274550: source-derived from observed immutable
  RGB-D metadata/payloads; the new policy did not latch.
- Post-fix physical behavior: unverified until the next explicitly authorized
  supervised robot run.
