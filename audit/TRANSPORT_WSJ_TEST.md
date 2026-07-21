# Cross-machine transport test: wsj robot -> local hub

Date: 2026-07-18 (Asia/Shanghai).
Sender: `wsj` (Jetson Orin NX, system Python 3.10, numpy/cv2/requests only),
single file deployed to `/home/nvidia/focus_sender/` (new directory, TinyNav
repo untouched; its dirty-worktree state was recorded first in
`audit/wsj_worktree_state_20260718.txt`).
Hub: this machine, loopback :8188. Path: authenticated multipart HTTP over an
SSH reverse tunnel (`wsj:127.0.0.1:18089` -> `local:127.0.0.1:8188`); the
tunnel is a loopback-only test transport, not the production channel.
Per-run random token, delivered as a 600-mode file, removed after the test.

## Measured results (all observed; metrics JSONs archived in
`data/robot_replays/transport_test_20260718/`)

Per keyframe (848x480 RGB JPEG q92 + PNG16 depth aligned infra1->RGB):

| stage (on the Jetson) | mean |
| --- | --- |
| depth align infra1->RGB | 69-72 ms |
| JPEG + PNG16 (level 1) encode | 29-31 ms |
| HTTP upload through tunnel | 30-50 ms |

- Payload ≈358 KB/frame (JPEG ≈100 KB + PNG16 ≈258 KB); 2 Hz ≈ 0.7 MB/s.
- Full record, max rate: **303/303 accepted in 40.2 s (≈7.5 fps, 2.63 MiB/s), 0
  retries**. 2 Hz paced runs: cadence held exactly (0.5 s spool arrival steps).
- PNG level trade (Jetson): level 3 saves 4 % bytes for +10 ms; level 6 saves
  13 % for +63 ms. **Level 1 kept**: CPU headroom matters more than WLAN
  bandwidth at this payload.
- Resume: every restart continued from the hub-reported sequence
  (`GET /v1/robots/{id}/observations/latest`, endpoint added for this); 803
  frames accumulated across all runs with zero sequence conflicts and zero
  content duplicates.
- Hub-side integrity: 563-entry (later 803) spool, hash/size/sequence checks
  all enforced at ingest; map built from the transported bytes matches the
  G3 local-file map in shape (explored 21,910 vs 25,820 cells; the deficit is
  expected — depth->RGB alignment crops FOV and z-buffering drops occluded
  pixels; no geometry error).

## Fault injection (genuine, robot-local)

Early tunnel-kill attempts produced **no** sender-visible outage; forensics
showed why, and it is an operational lesson worth keeping:

- `ssh -O exit` cancels the remote listener but lets established forwarded
  connections drain — a keep-alive HTTP session rides through a "closed"
  tunnel untouched.
- This host's `~/.ssh/config` sets `ControlMaster auto` + `ControlPersist 8h`,
  so plain `ssh -f -N -R ...` multiplexes the forward onto a long-lived shared
  daemon that survives killing the visible client. SSH tunnels are therefore
  hard to fail deterministically — and equally hard to reason about in
  production. **Production must use a dedicated VPN/TLS channel, not SSH.**

The conclusive test ran entirely on the robot against a local TCP proxy
(`tcp_proxy.py`), sender pointed at a dead port:

- start with no listener: first frame retried with exponential backoff and
  succeeded after **6 attempts / 15.6 s** when the proxy came up at t=15;
- proxy killed mid-run: frame recovered after **5 attempts / 7.6 s**;
- **40/40 frames delivered, 9 retries total, 0 duplicates, sender exit 0**;
- separate run with the endpoint dead past the retry budget: sender gave up
  after **9 attempts ≈ 39.5 s** with an explicit error (fail-loud, not hang).

Backoff schedule: 0.5 s doubling, 8 s cap, 8 retries ≈ 39.5 s budget; capture
timestamps are re-stamped (and payload re-hashed) across retry gaps so the
hub's 3 s freshness window is honored rather than bypassed.

## Post-conditions

Robot: sender/proxy processes killed, token file removed; only the sender
package, its logs/metrics and `tcp_proxy.py` remain in `/home/nvidia/focus_sender/`.
Local: hub stopped, ports 8188/18089 free, forward confirmed closed from the
robot, GPU at the 976 MiB unrelated baseline.

## What this proves / does not prove

Proves: the exact wire contract works robot->hub over the real WLAN with
authentication, integrity checks, pacing, resume, retry/recovery and loud
failure; the measured budgets fit a 2 Hz keyframe stream with 3-4x margin.
Does not prove: live ROS 2 topic capture (replayed record instead), the
production security channel (VPN/TLS pending), robot-side receiver, G4, G5.
