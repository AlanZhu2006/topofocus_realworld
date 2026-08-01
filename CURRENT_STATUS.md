# Current project status

Snapshot: **2026-08-01, after completing four five-run real-robot campaigns.**

## Formal evaluation

| Scene | Target | Trials | Success | SR | Mean source-compatible SPL | Mean Standard SPL | Independent shortest path |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Scene 01 | Chair | 5 | 5 | 1.0 | 0.726088 | 0.780993 | ≈3.25 m |
| Scene 02 | Plant | 5 | 3 | 0.6 | 0.557367 | 0.531689 | ≈7 m |
| Scene 03 | Plant | 5 | 3 | 0.6 | 0.365962 | 0.546194 | ≈14 m |
| Scene 04 | Plant | 5 | 2 | 0.4 | 0.330311 | 0.361412 | ≈11 m |

Failures contribute zero to both SPL means. The shortest feasible paths are
independent operator measurements; exact per-run paths, classifications and
media bindings are preserved in the scene manifests and dated audit records.

The authoritative campaign index is
[`manifests/realworld_experiment_progress.json`](manifests/realworld_experiment_progress.json).
The main [README](README.md) presents the rollouts, maps and per-run metrics.

## Released system

The public baseline contains the RTX 4090 Hub, shared semantic-map and VLM
coordination layer, versioned expiring target transport, and robot-local
TinyNav planning, guarded control and final stop/reject authority. The
deployment contract is
[`hub/config/deployments/realworld_dual_robot_v1.json`](hub/config/deployments/realworld_dual_robot_v1.json).

`source/` and `dependencies/` remain immutable upstream snapshots. Deployment
and hardware integration code is isolated under `hub/`. No tracked launch
path gives the Hub direct low-level motor authority.

## Canonical documentation

- [Reproduction and deployment](docs/README.md)
- [Current supervised workflow](hub/docs/ONECLICK_SESSION_WORKFLOW.md)
- [Formal and engineering evidence](audit/README.md)
- [Media provenance](media/README.md)

The repository does not assert that a physical session is currently armed.
Every later run requires a new supervised session boundary and explicit onsite
motion authorization.
