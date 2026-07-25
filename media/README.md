# Workspace media provenance

These user-provided files are the project README's physical-experiment
showcase assets. Their bytes were observed in the workspace on 2026-07-24.
Capture device and EXIF metadata were not independently verified.

| Path | Size (bytes) | Dimensions | SHA-256 | Classification |
| --- | ---: | --- | --- | --- |
| `image/calibration.jpg` | 325176 | 1707 × 1280 | `687a87750b3867eecde9cd4edd6f66105e585d32316f6c13536fc6166ff2d909` | observed user-provided calibration-board/setup photo; capture metadata unverified |
| `image/example.png` | 10736 | 686 × 599 | `ecdc053d57ddf23dbd5fc80cc3f5692c96b381cd74245f16da75aa7ebddb5360` | observed user-provided semantic-map reference; renamed byte-for-byte from repository-root `example.png` |
| `image/experiment_1_map.png` | 67105 | 820 × 942 | `7b9cbfa20b0b09c1dba33f24ae23b8cc64817d52eb48222613e829ceac1e867b` | observed user-provided map screenshot bound to the near-chair run; semantic pixels remain model inference |
| `image/showcase_1.jpg` | 355320 | 1280 × 1707 | `a54993201184799d8d77b948cb7d0b139ff6de6fbdff41f03d1b00f519459ee1` | observed user-provided two-robot setup photo; capture metadata unverified |
| `image/showcase_2.jpg` | 863185 | 2486 × 1280 | `0d433920549fd073a15e22c0ae6f420584abd5dc5bfacee07d953b44b251586e` | observed user-provided two-robot setup photo; capture metadata unverified |

The photos and reference image must not be used as ground-truth terminal
evidence for SR/SPL unless a trial record separately identifies and hashes
the exact evidence file under its own collection protocol.

Future failed and successful demo files are indexed in `demo/README.md`.
Original high-resolution video masters remain under the ignored
`media/video/`; only documented, size-bounded derivatives are published.

## 2026-07-25 bound near-chair success recordings

The user-provided success masters are bound to session
`20260725-lab17-nearwall-fix`, episode `trial-05-nearwall-fix`. Yunji stopped
`0.321133 m` from its selected semantic approach point. The onsite operator
adjudicated the run successful under a declared `0.5 m` physical terminal
radius. This contributes `SR=1` and source-compatible `SPL=0.864048` to the
separate operator-adjudicated progress track; standard pre-surveyed SPL remains
unavailable.

| Local ignored master | Size (bytes) | Stream | Duration | SHA-256 |
| --- | ---: | --- | ---: | --- |
| `video/third_view/experiment_1/experiment_1_success.mp4` | 9749574 | HEVC/AAC, 544 × 960, nominal 30 fps | 141.733333 s | `ab611dde043c78abbbe618a8f2ce95f1f5113b4f6b0df8466038abc1ebafb44c` |
| `video/dashboard/experiment_1/experiment_1_success_dashboard.mov` | 80244552 | H.264/AAC, 2392 × 1080, 30 fps | 125.829 s | `ad0e99c4ebb56e90a5923af636f0269b118349e3353c05511b3e21c5020fae34` |

The original automatic outcome remains a failure under the old `0.15 m`
arrival threshold. Public derivatives, exact classification and arithmetic
are indexed in [`demo/README.md`](demo/README.md) and
[`../audit/NEAR_CHAIR_SUCCESS_20260725.md`](../audit/NEAR_CHAIR_SUCCESS_20260725.md).

## 2026-07-25 bound collision recordings

Two additional user-provided masters were observed and explicitly bound to
session `20260725-lab05-yunjireboot4`, episode
`scene01-chair-run01-fastfix`:

| Local ignored master | Size (bytes) | Stream | Duration | SHA-256 |
| --- | ---: | --- | ---: | --- |
| `video/experiment_1_collision.mp4` | 1640232 | HEVC/AAC, 1280 × 720, nominal 30 fps | 12.366667 s | `1b883c310b176ed75587b5672d09a0ab14a604e214663f0c760835f1eb5ec659` |
| `video/experiment_1_collision_dashboard.mov` | 19138555 | H.264, 3420 × 1544, nominal 60 fps | 16.341667 s | `a429b838566efbd4769b1f613ba2530d1c0c972cc1cc685b72f278aac5ecffc9` |

The third-view recording directly shows physical contact. The paired
Dashboard recording provides synchronized visualization context but does not
independently establish semantic target truth or network causality. Original
masters remain unchanged and ignored; H.264 derivatives, posters and complete
classification are in [`demo/README.md`](demo/README.md). The episode is
excluded from SR/SPL.
