# Scene 02 stationary ground-drift confirmation and bounded speed update — 2026-07-28

## Observed evidence

The supervised `formal-07-20260728-035013` run completed three coordinated
source rounds before stopping fail-closed. WSJ accumulated 2.192119 m and
Yunji accumulated 2.764005 m of localization-derived path. The previously
fixed motion-bounded cached-occupancy policy did not trigger.

The next synchronized input was rejected because Yunji's mapper latched a
ground-plane drift at sequence 297724. The three outlying stationary fits at
sequences 297722–297724 spanned 2.036098546 s of source capture time. Replaying
the immutable RGB-D observations against the startup plane measured tilt
deltas of 3.213, 3.354 and 5.302 degrees. Sequence 297725 immediately returned
in range at 1.264 degrees. This is observed transient RANSAC fit variation, not
evidence of a persistent calibration or camera-mount change.

| Artifact | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `hub/runtime/oneclick_scene02-plant-20260728-0347-occupancymotion1_live_scene02-plant_20260728_035118_002549710/controller_events.jsonl` | 28,901 | `c13bd6104e6e8eae424a8113632460da808c718bef96e6d1073fc815838bcb34` | observed sealed controller event log |
| `hub/runtime/oneclick_scene02-plant-20260728-0347-occupancymotion1_live_scene02-plant_20260728_035118_002549710/episode_report.json` | 9,907 | `0b6ef4c7b02df27e9627ae425aafc9714fe78f1c0951cbe4b8fbab0d9380bbd4` | observed sealed failed engineering report |
| `hub/runtime/oneclick_scene02-plant-20260728-0347-occupancymotion1_live_scene02-plant_20260728_035118_002549710/round_03_step_074/freeze_rejections.jsonl` | 18,445 | `4a9d7971e68991f405df339f3d9083f992dea870f8e0b7a7e5f27548c177ef91` | observed sealed synchronized-input rejection log |
| `hub/runtime/spool/robot-1/00000000000000297722/metadata.json` | 3,287 | `f5479e3005a8b9cd09b2accd9c3a6473cb7c725ad6172e44d9ccc5b15e527506` | observed append-only Yunji metadata |
| `hub/runtime/spool/robot-1/00000000000000297723/metadata.json` | 3,280 | `eaee0d8f1426c463313d57cb98c230f27efa774740671ccd107986626b610730` | observed append-only Yunji metadata |
| `hub/runtime/spool/robot-1/00000000000000297724/metadata.json` | 3,279 | `c5d3f9aa9a979c31ed3c753b3a88c35c04166a40695fabaa7a439cb972a4a460` | observed append-only Yunji metadata |
| `hub/runtime/spool/robot-1/00000000000000297725/metadata.json` | 3,281 | `3f1af9151bac0615c1a3ed4160692205e5c560fe4f53ed7c4ef13c71e16545d8` | observed append-only Yunji metadata |

## Shared ground-guard correction

The earlier gait correction rejected outlying frames while the robot was
moving and prevented them from advancing the irreversible latch. It did not
cover a short run of noisy fits after odometry had become stationary.

The shared WSJ/Yunji mapper now:

1. rejects every outlying floor frame before map integration, unchanged;
2. requires both three consecutive stationary outliers and at least 5.0 s of
   monotonic source capture time before permanently latching;
3. resets the frame and duration confirmation on motion, an invalid fit, a
   tolerated pure-Z translation, or an in-range fit;
4. treats non-monotonic capture time as no persistence evidence.

An offline replay of sequences 297686–297729 retained the three outliers as
`ground_drift_pending`, recovered at 297725, integrated through 297729 and
reported no mapping block. This is source-derived software validation from
observed immutable payloads; post-fix physical behavior remains unverified.

## Bounded linear-speed update

The preserved WSJ chassis log shows the formal run's nonzero forward commands
were normally 0.150 m/s. Yunji's final WATER bridge was independently observed
starting with the same 0.15 m/s hard cap.

| Artifact | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| `/home/nvidia/.local/state/topofocus/wsj-go2-bridge-20260727T195104Z.log` | 62,634 | `a7caab249c6c098b5c599c3d62be831bdb66e9cb5842a1cf663183eabe9f33f2` | remotely observed WSJ chassis-command log |

Both launchers now use a 0.18 m/s intentional nonzero forward floor. The
deployment wrapper rejects values above 0.20 m/s, WSJ retains its 0.20 m/s
bridge maximum, and Yunji's bridge maximum is aligned to 0.20 m/s. Angular
limits, collision checks, stale-input stops, 20 Hz physical gating, lease
expiry and chassis watchdogs are unchanged. Physical speed behavior remains
unverified until a newly authorized supervised run.
