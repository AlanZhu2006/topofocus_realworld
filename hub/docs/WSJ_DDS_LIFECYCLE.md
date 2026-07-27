# WSJ Fast DDS lifecycle contract

## Observed trigger

On 2026-07-27, a newly created WSJ observation sender discovered the existing
camera/perception publishers but did not receive synchronized samples. Samples
resumed only after the sender already existed and the publishers were
recreated. This is classified as an operator-observed live lifecycle failure.

The retained Hub spool independently proves a 183.539 s capture gap around
that recovery; it does not by itself identify DDS as the cause:

| Classification | Source | Bytes | SHA-256 |
| --- | --- | ---: | --- |
| observed, last frame before gap | `hub/runtime/spool/robot-0/00000000000000032727/metadata.json` | 2579 | `3d9b88633691d2775f3785fee45c672ab509cfdaa60a5669b2bb5bedfb759734` |
| observed, first frame after gap | `hub/runtime/spool/robot-0/00000000000000032728/metadata.json` | 2580 | `254eb107e11baddbf48fec1d09f434996bfe135bf03213f8625562f45ab7e100` |

## Enforced order

The deployment now has one authoritative order:

1. close receiver/bridge motion paths;
2. create or preserve the runtime-configurable sender and park its upload
   contract;
3. stop the old perception publisher;
4. restart the camera publisher;
5. restart perception;
6. apply checked calibration metadata atomically without replacing the DDS
   participant.

`start_wsj_calibration_observation.sh` performs that order before board
capture. `recover_wsj_publishers_after_sender.sh` performs the same read-only
recovery only with the exact stationary-operator confirmation.

## Fail-closed state

- A missing, malformed, checksum-mismatched, wrong-deployment, or
  wrong-calibration runtime contract parks the sender.
- A transient Hub sequence lookup is retried without poisoning the checked
  contract and without replacing the DDS participant.
- Publisher recovery writes
  `wsj-tracking-reanchor-required.json` before touching a publisher. The
  marker survives partial recovery and blocks command-capable activation
  until a matching stationary re-anchor or a successful new board calibration
  resolves it.
- A sender frame timeout never restarts the sender. HTTP failures replace only
  the `requests.Session`.
- Remote commands run behind a `set +e` wrapper that always returns control to
  the existing SSH/tmux shell; the inner command's real status is still
  reported by its unique completion marker.

These are source-derived controls. Their local parser, contract, ordering,
shell-syntax, and regression tests pass without issuing a robot command.
