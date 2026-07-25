# Scene 01 chair physical success — 2026-07-25

## Later formal ordinal annotation

On 2026-07-25 the operator clarified that this manually annotated README
result is **formal experiment 03**. The metric values and original evidence
classification below are unchanged. The two byte-identical master files were
renamed to `experiment_1_success_3.mp4` and
`experiment_1_success_3_dashboard.mov`; their original archived paths remain
part of the historical provenance.

## Outcome

Session `20260725-lab17-nearwall-fix`, scene `scene01-chair`, episode
`trial-05-nearwall-fix` is recorded as the first success in the separate
`physical_0p5m_protocol` progress track:

- SR: `1.0`;
- Yunji actual odometry path `P`: `4.048842201890332 m`;
- Yunji start-to-stop displacement: `3.498394160748945 m`;
- exact Focus-source-compatible SPL:
  `3.498394160748945 / 4.048842201890332 = 0.864048038008398`;
- standard SPL: unavailable because no shortest collision-free path was
  surveyed before this run.

The project uses a `0.5 m` real-world success radius. The observed terminal is
inside that radius and therefore passes normally. For reproducibility, the
immutable episode report still says `failed_robot_or_controller_holding`, and
the Yunji receiver still records `LOCAL_PLANNER_PATH_STALE`, because this run
predated the launcher's change from the old `0.15 m` radius.

## Why Scene 01 passes

The round-2 semantic goal resolved in Yunji's local map to
`(3.5114049269242398, -1.2307825026429273)`. The final zero-velocity pose in
the episode report is
`(3.1903207713161237, -1.2364042666396216)`. Their Euclidean separation is
`0.321133366707683 m`, inside the subsequently declared `0.5 m` real-world
terminal radius.

The uploaded external-view video continuously shows Yunji approach the white
chair and stop beside it. Its terminal frame independently shows both Yunji
and the chair. The paired Dashboard video supplies visualization context.
The user-provided `experiment_1_map.png` shows the corresponding trajectory,
robot markers, frontier letters and projected `chair` region. Its separate
`plant` patch is retained as model output and is not asserted to be a real
plant.
Round 2 also contains a YOLO `chair` detection at confidence
`0.8806157112121582`; that model inference is retained as unverified and is
not treated as the independent target annotation.

The result is included in the declared 0.5 m physical-protocol track. It is
not mixed into the pre-surveyed standard ObjectNav track, because no shortest
collision-free reference path was measured before this run.

## Failure mechanism and forward fix

The old receiver checked a `0.15 m` radius. At about `0.321 m` from the
selected semantic approach point, the planner path became stale before that
strict threshold could produce `ARRIVED`. Commit
`b1762d15e1059281056ef1e6b4e472e9d25258e1` makes both physical launchers pass
an explicit `0.5 m` semantic arrival radius while retaining the source-exact
`0.15 m` default inside the adapter. Future runs can therefore terminate
automatically under the declared physical protocol.

## Provenance

The complete machine-readable evidence list, sizes, SHA-256 values, geometry
and metric arithmetic are in
[`manifests/realworld_experiment_progress.json`](../manifests/realworld_experiment_progress.json).
The most important observed identities are:

| Evidence | Size | SHA-256 | Classification |
| --- | ---: | --- | --- |
| local `episode_report.json` | 9,777 B | `e31d66e4c2a8ed136238b8670c22b9505daf520fc7715f05d4b4ce020dda769d` | source-derived from observed robot feedback; runtime ignored |
| Yunji receiver JSONL | 79,448 B | `513f2f61494a997eafbb6f89ec669051834e31cde8f536306166875884ed0539` | observed robot-local file |
| external-view master | 9,749,574 B | `ab611dde043c78abbbe618a8f2ce95f1f5113b4f6b0df8466038abc1ebafb44c` | observed user-provided master; committed byte-for-byte through Git LFS |
| Dashboard master | 80,244,552 B | `ad0e99c4ebb56e90a5923af636f0269b118349e3353c05511b3e21c5020fae34` | observed user-provided master; committed byte-for-byte through Git LFS |
| public external-view derivative | 15,972,419 B | `62390579a390451b442562029c2d385729d282696605bb347dcbe2d64f11af1c` | source-derived H.264 |
| public terminal poster | 138,572 B | `27340abea65bdb2706aff8254c5257c915bc29aa03d336eaa849f0e5022702bc` | source-derived at `141.0 s` |
| user-provided map screenshot | 67,105 B | `7b9cbfa20b0b09c1dba33f24ae23b8cc64817d52eb48222613e829ceac1e867b` | observed; projected semantics are unverified model inference |

No file under immutable `source/` or `dependencies/` was changed.
