# Workspace media provenance

These user-provided files are the project README's physical-experiment
showcase assets. Their bytes were observed in the workspace on 2026-07-24.
Capture device and EXIF metadata were not independently verified.

| Path | Size (bytes) | Dimensions | SHA-256 | Classification |
| --- | ---: | --- | --- | --- |
| `image/calibration.jpg` | 325176 | 1707 × 1280 | `687a87750b3867eecde9cd4edd6f66105e585d32316f6c13536fc6166ff2d909` | observed user-provided calibration-board/setup photo; capture metadata unverified |
| `image/example.png` | 10736 | 686 × 599 | `ecdc053d57ddf23dbd5fc80cc3f5692c96b381cd74245f16da75aa7ebddb5360` | observed user-provided semantic-map reference; renamed byte-for-byte from repository-root `example.png` |
| `image/showcase_1.jpg` | 355320 | 1280 × 1707 | `a54993201184799d8d77b948cb7d0b139ff6de6fbdff41f03d1b00f519459ee1` | observed user-provided two-robot setup photo; capture metadata unverified |
| `image/showcase_2.jpg` | 863185 | 2486 × 1280 | `0d433920549fd073a15e22c0ae6f420584abd5dc5bfacee07d953b44b251586e` | observed user-provided two-robot setup photo; capture metadata unverified |

The photos and reference image must not be used as ground-truth terminal
evidence for SR/SPL unless a trial record separately identifies and hashes
the exact evidence file under its own collection protocol.

Future failed and successful demo files are indexed in `demo/README.md`.
Original high-resolution video masters remain under the ignored
`media/video/`; only documented, size-bounded derivatives are published.

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
