# Live two-robot VLM shadow coordination — 2026-07-22

## Outcome and authority boundary

The real local GLM service completed the full Perception -> Judgment ->
Decision cascade against frozen WSJ and Yunji/Odin RGB-D/map inputs. The maps
were fused in the declared `shared_world` frame and the robots were scheduled
sequentially: WSJ selected frontier D, then D was removed before Yunji selected
frontier B. The final audited run took 16.071 seconds and reported no VLM
errors.

This was deliberately a **shadow-only** result. The coordinator contains no
`GOAL` publication branch, sent no request to either robot, and changed no
`allow_goal` policy. The Hub accepted only two 30-second `HOLD` decisions; its
health endpoint still reported `goal_output_enabled=false` for both robots.
Foxglove received expiring magenta markers explicitly labelled `NO MOTION` on
the existing per-robot `map_pose` topics.

The result does not pass G4 or G5. WSJ's input map was already locked by ground
drift, both source images were stale and 157.261 seconds apart, and the WSJ was
subsequently observed to have rebooted. Those conditions were allowed only by
two explicit forensic overrides and remain recorded in the manifest.

## Observed final run

Run ID: `shadow-20260722-223315-52faeadc`

Runtime evidence directory:
`hub/runtime/vlm_shadow_live_20260722_2234_audited/`

| Robot | Frozen source | YOLO detections in that RGB | Perception yes | Judgment yes | VLM choice | Choice probability | VLM time |
| --- | ---: | --- | ---: | ---: | --- | ---: | ---: |
| WSJ / robot-0 | 15147 | suitcase 0.2800, tv 0.2001 | 0.0600 | 0.9100 | D at `(-2.0718, 4.5745)` m | 0.4802 | 7.989 s |
| Yunji / robot-1 | 165893 | chair 0.9380, refrigerator 0.2184 | 0.7457 | 0.9609 | B at `(-0.5218, 1.5245)` m | 0.5851 | 7.946 s |

The fused decision map exposed four frontiers A/B/C/D. WSJ saw all four; the
second call saw only A/B/C because D had already been allocated. A repeated
real-GLM run produced the same D/B allocation.

WSJ did not detect a chair in its exact frozen RGB. The fused accumulated map
still contained chair evidence, so this run proves sequential frontier/VLM
coordination, not current-frame chair lock-on. Yunji's exact RGB did contain a
high-confidence chair detection. Eleven operator-rejected `plant` cells were
zeroed only in a copied decision tensor; the source maps and their provenance
were not edited.

## Fail-closed timing and map checks

The final manifest records:

- oldest source age: 1,031.168 s;
- cross-robot capture skew: 157.261 s;
- default limits: 30 s age and 5 s skew;
- timing status: `accepted_stale_forensic_override`;
- WSJ block: three consecutive ground-plane outliers at sequence 15150,
  3.700 degrees / 0.130 m from the session plane;
- Yunji block: null.

After this finding, the tool was changed to reject stale/asynchronous inputs by
default. A no-VLM replay of the default gate exited non-zero with both the
1,022.686-second age violation and 157.261-second skew violation. The safe
wrapper additionally requires a caller-supplied current-session calibration
ID and intentionally exposes no stale/blocked override. A separate preflight
mode freezes and validates inputs/frontiers without contacting GLM or Hub.

## Wire and display evidence

The persisted Hub event log contains exactly these publications for the final
run:

- `shadow-20260722-223315-52faeadc-robot-0`, mode `HOLD`, HTTP 202;
- `shadow-20260722-223315-52faeadc-robot-1`, mode `HOLD`, HTTP 202.

Both expired after 30 seconds. The two `shadow_target.json` files were
display-only, carried robot/frame/transform/calibration/expiry fields, and were
validated by the relay before rendering. Malformed, expired or mismatched
targets cannot block the underlying map display. The remote Foxglove client
re-established its WebSocket connection after the relay restart.

## Provenance

The manifest contains the absolute source and preserved path, size, SHA-256
and observed/source-derived classification for every map summary, runtime
status, RGB, depth and metadata input. Representative artifacts are:

| Artifact | Size (B) | SHA-256 | Classification |
| --- | ---: | --- | --- |
| final `shadow_manifest.json` | 13,423 | `ab5e6d8d002560819ea4845cc06bceb3ecad6b2b98dd406e0437c2b2ea757f57` | observed run manifest |
| frozen WSJ `central_map.npz` | 23,586 | `89f19417e3408d3d73ef1290ae5b6c40eeb7e717a47bfcf470a28d2204b1e055` | model/source-derived input |
| frozen WSJ sequence 15147 RGB | 75,511 | `d63a025a389ac0a9eff3c6bab23f7ee399fb098237b245a62b94bd88674bf1e9` | observed input |
| frozen Yunji `central_map.npz` | 22,720 | `311055892869d3cbd04e839b73cd6cd721b6e5710dc6fea91e59ebc3b4efef3d` | model/source-derived input |
| frozen Yunji sequence 165893 RGB | 155,117 | `f2275a9dd8149f3e2e6a123e5fe064310d3f7d758d8255d271b55c3cad3e9ba9` | observed input |
| WSJ decision image | 43,519 | `2d7cab3110502c6e63a764b45bd7043450c25158f3e9fab872615b55815a1b82` | source/model-derived visualization |
| Yunji decision image | 42,774 | `6a6eb0657cf309036d3036176ad9802284251df51e30e0480d9679f279763701` | source/model-derived visualization |

Current implementation artifacts after the timing/preflight hardening:

| Artifact | Size (B) | SHA-256 |
| --- | ---: | --- |
| `hub/src/focus_hub/shadow_coordination.py` | 10,315 | `fa055ee1ad287b22020619ecc59ab78054ef6d314b67c6dde37d14dea2f02a58` |
| `hub/tools/live_vlm_shadow.py` | 26,140 | `9aa47fe56e5d2bae818193c88613cda47b99258ae7574bc769f31cd2c96b2a96` |
| `hub/tools/foxglove_relay.py` | 40,071 | `b782d0fa5c8256c94e09d06d311b8af8dcb82a7f008bb18393b9da41f60efeac` |
| `hub/scripts/run_live_vlm_shadow.sh` | 3,545 | `0ed72ca2e2bdec205a12ba70e7687c8b273f2d793907073421b5656a72c4797d` |

Repository verification passed with 210 collected Hub tests. `source/` and
`dependencies/` were unchanged.

## Next valid gate

After both robots start new odometry sessions, capture a fresh fit pair and an
independently moved-board holdout, create a new calibration ID, and start new
map directories. Only fresh, unblocked, synchronized maps with that exact ID
may enter the safe wrapper. The first on-site run should remain shadow/HOLD;
physical VLM-driven motion still requires the separately missing robot-side
command receiver and hardware-in-the-loop G5 gate.
