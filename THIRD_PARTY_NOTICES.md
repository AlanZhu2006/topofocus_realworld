# Third-party source notices

This repository contains audited snapshots or patches of third-party research software. Their original licenses and notices remain authoritative.

- `dependencies/habitat-lab/`: Habitat Lab snapshot; its included `LICENSE` is MIT.
- `source/Focus_realworld/CogVLM2/`: CogVLM2 source snapshot; see its included `LICENSE` and `MODEL_LICENSE`.
- `hub/robot_overlay/tinynav_snapshot/`: patches against `UniflexAI/tinynav`, whose pinned base contains the Apache License 2.0.
- The reconstructed Robot 0 TinyNav lock resolves `unitree-sdk2py` from
  `junlinp/unitree_sdk2_python@800103eab7e045336b1c40186cda5023dbd05821`;
  the [official Unitree SDK2 Python repository](https://github.com/unitreerobotics/unitree_sdk2_python)
  and the resolved fork retain the BSD 3-Clause license and their own notices.
- `dependencies/RedNet/`: copied from the audited Torch workspace without Git history or a bundled license file. Its exact provenance/license remains **unverified**; do not assume this repository grants additional rights.
- `source/Focus_realworld/`: audited research snapshot without a project-wide license file at its root. Ownership and redistribution terms must be confirmed by the project owner.

Model weights and datasets are not distributed by this Git repository. Obtain them from their respective owners under their own terms.
