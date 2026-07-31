# Third-party source notices

This repository contains audited snapshots or patches of third-party research software. Their original licenses and notices remain authoritative.

- `dependencies/habitat-lab/`: Habitat Lab snapshot; its included `LICENSE` is MIT.
- `source/Focus_realworld/CogVLM2/`: CogVLM2 source snapshot; see its included `LICENSE` and `MODEL_LICENSE`.
- `hub/robot_overlay/tinynav_snapshot/`: patches against `UniflexAI/tinynav`, whose pinned base contains the Apache License 2.0.
- The reconstructed Robot 0 TinyNav lock resolves `unitree-sdk2py` from
  `junlinp/unitree_sdk2_python@800103eab7e045336b1c40186cda5023dbd05821`;
  the [official Unitree SDK2 Python repository](https://github.com/unitreerobotics/unitree_sdk2_python)
  and the resolved fork retain the BSD 3-Clause license and their own notices.
- The Robot 0 clean-room lock also builds pinned CycloneDDS, GTSAM,
  librealsense, realsense-ros and message_filters revisions. Their upstream
  repositories and bundled license files remain authoritative; the lock fixes
  source identity but does not relicense them.
- `dependencies/RedNet/`: copied from the audited Torch workspace without Git history or a bundled license file. Its exact provenance/license remains **unverified**; do not assume this repository grants additional rights.
- `source/Focus_realworld/`: audited research snapshot without a project-wide license file at its root. Ownership and redistribution terms must be confirmed by the project owner.

Model weights and datasets are not distributed by this Git repository.
`fetch_cleanroom_models.py` downloads only explicitly pinned real-world model
artifacts and requires the operator to acknowledge upstream model licenses;
that acknowledgement does not grant redistribution rights. Obtain every model
from its respective owner under its own terms.
