# Supervised-demo transport v2 draft audit — 2026-07-23

## Result

Classification: **approved implementation contract, unverified on robot**.
The user approved continuing on 2026-07-23 after replacing the initial
single-active restriction with independent concurrent local navigation. At
the time of this record no v2 schema/API/receiver was implemented, no policy
was enabled and no robot command was sent. Transport v1 and
`allow_goal=false` remain active.

| Artifact | Bytes | SHA-256 | Classification |
|---|---:|---|---|
| `hub/docs/TRANSPORT_V2_DEMO_DRAFT.md` | pending final implementation hash | pending | approved source-derived contract, unverified on robot |
| `hub/docs/TRANSPORT.md` | pending final implementation hash | pending | observed active v1 document with v2 cross-link |

The draft itself records the paths, byte sizes, SHA-256 values and
classifications of the immutable source and current Hub inputs used to derive
the proposal.

## Checks performed

- `git diff --check`: passed.
- `bash hub/scripts/verify_repository.sh --tests`: passed; AST parsing and the
  full Hub test suite completed successfully.
- `git status --short source dependencies`: empty; neither immutable tree was
  changed.
- No SSH session, remote robot, simulator dataset or external download was
  used.

## Approval boundary

The contract defines versioned, expiring high-level targets, full semantic-region
intent, robot-side rejection/STOP authority, independent concurrent dual-robot
execution and append-only navigation feedback. User approval authorizes Hub
implementation only. It still does not enable physical motion; the dry-run
and supervised activation gates in the contract remain mandatory.
