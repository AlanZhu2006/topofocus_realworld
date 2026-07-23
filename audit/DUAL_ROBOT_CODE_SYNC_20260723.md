# Dual-robot deployment code sync — 2026-07-23

## Outcome

At `2026-07-23T14:14:28+08:00`, the same local deployment package was copied
to Yunji (`nyush-nuc`) and WSJ (`tegra-ubuntu`). Both remote archives matched
the local SHA-256, all 270 manifest entries verified, the robot-specific
receiver parsed in the robot's existing Python environment, and the deployed
critical-file hashes matched the local working tree.

This was a code-only operation. It did not start ROS, WATER, TinyNav planning,
a command receiver, or a robot bridge; it issued no physical robot command.
The pre-existing sender directories were not overwritten. Yunji's Odin driver
and sender services remained inactive.

## Source provenance

- source workspace:
  `/home/asus/Research/focus_realworld_workspace`
- Git base commit:
  `ee8f84b6646cb08fbcb30fab072b9d0437bf485b`
- branch: `agent/live-map-recovery-20260722`
- classification: locally tested **dirty working-tree deployment snapshot**,
  not a published Git commit
- package content: `hub/{src,robot_overlay,config,tools,scripts,docs,tests}`,
  Hub packaging files, root operator documentation, and the v2 receiver audit
- deliberately excluded: `source/`, `dependencies/`, `hub/runtime/`,
  `.venv`, caches, tokens, `.env`, and the ignored real
  `hub/config/robots.json`

| Artifact | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| temporary deployment archive `topofocus-worktree-20260723T1411CST.tar.gz` | 2,080,461 | `b1c472983b95be1545d1291d86c3c9d0e0360b5f432e3a73480d1e82ba691a03` | locally produced transfer artifact |
| `DEPLOYMENT_MANIFEST.sha256` | 33,398 | `56036757f02db7903f1bba1cb795594065ab30fe46f747f17f9ba786c1422c0b` | locally produced 270-file checksum manifest |

The temporary archive was removed from both robots after extraction and
verification. The local staging directory and archive were also removed after
recording this audit.

## Yunji deployment

- observed host: `nyush-nuc`, user `nyu`
- release:
  `/home/nyu/topofocus_realworld/releases/worktree-20260723T1411CST`
- stable link:
  `/home/nyu/topofocus_realworld/current`
- environment smoke test: ROS Humble plus
  `/home/nyu/odin_ws/install/setup.bash`, system Python 3.10;
  receiver `--help` import passed
- `v2_yunji_receiver.py`:
  `858924d6e6cf3ddf6b336664da1dd7121a33a838913943e16bac48ec21bc5542`
- `odin1_sender.py`:
  `9fb1057ffdfeed8e86bcd60f32fb52c3767b4082a1de4147e8b45c5db8147766`
- observed postcondition: `focus-yunji-odin1-driver.service=inactive`,
  `focus-yunji-odin1-sender.service=inactive`

## WSJ deployment

- observed host: `tegra-ubuntu`, user `nvidia`
- release:
  `/home/nvidia/topofocus_realworld/releases/worktree-20260723T1411CST`
- stable link:
  `/home/nvidia/topofocus_realworld/current`
- environment smoke test: existing
  `/home/nvidia/twork/tinynav/.venv` through `uv run`; receiver `--help`
  import passed. System Python lacks Pydantic and must not be used directly.
- `v2_wsj_receiver.py`:
  `585ab917a338cb892f3735d59887da2cbbf5386c2101cdaa9753e96407f8797c`
- `focus_ros_sender.py`:
  `6f3da0156eda3b80ed4c527f67255955681aa58b29c9a08db4f7d907b691102e`
- observed postcondition: no deployed v2 receiver or guarded-command process
  was running

## Evidence boundary

This proves byte-identical code availability and dependency-level imports on
both robot computers. It does not prove current sensor topics, shared-frame
alignment, WATER reachability, TinyNav graph readiness, or physical motion.
Those remain part of the on-site read-only and short-crawl gates.

## 2026-07-24 final retry3-fix synchronization

At `2026-07-24T02:16:11+08:00`, a final dirty-working-tree deployment
snapshot was synchronized byte-for-byte into the current versioned BuildMap
deployment roots:

- WSJ: `/home/nvidia/topofocus_buildmap_v2_20260723`
- Yunji: `/home/nyu/topofocus_buildmap_v2_20260723`

The package contained 392 entries and deliberately excluded `source/`,
`dependencies/`, `hub/runtime/`, `hub/.venv`, caches and compiled bytecode.
It included the retry3 evidence, the actual v2 heartbeat-authority fix, the
WSJ effective-command floors and the independent odometry/occupancy callback
groups. This is still based on Git commit
`ee8f84b6646cb08fbcb30fab072b9d0437bf485b` on branch
`agent/live-map-recovery-20260722` with a dirty working tree; it is not yet a
published reproducible Git commit.

| Artifact | Bytes | SHA-256 | Classification |
| --- | ---: | --- | --- |
| temporary `topofocus_hub_sync_20260724_retry3fix.tar.gz` | 2,371,165 | `e1b9001fb188a3890037f5e33927d25afa44473fb50a6b8c40b61a6e123b1b72` | locally built; independently observed on both robots before extraction |
| `hub/src/focus_hub/v2_registry.py` | 16,139 | `971fcb459416a38dd81ef44ec272c3909c30219dbe340c2ebcb90ea0aaeff483` | checksum observed locally, WSJ and Yunji |
| `hub/robot_overlay/tinynav_buildmap_goal_router.py` | 37,147 | `4461bf961f307c9efb36158b5a928032e7131b32f754fa1f4be24b925255725c` | checksum observed locally, WSJ and Yunji |
| `hub/robot_overlay/start_wsj_buildmap_v2.sh` | 8,444 | `4bc1fe9a16cc08c569080c98c6b40024fc2bfec1317d0a1e4365fa7c2e66027e` | checksum observed locally and WSJ |
| `hub/robot_overlay/v2_wsj_receiver.py` | 51,888 | `32f3242547895d8a63a3727208b0995d4ec2d0ed05decca7248727f6d627c27d` | checksum observed locally and WSJ |
| `hub/robot_overlay/start_yunji_v2.sh` | 5,827 | `1d1db9a4b1de25e055003ab741bf8ce492f51d2888288561a4577136a179b619` | checksum observed locally and Yunji |
| `hub/robot_overlay/v2_yunji_receiver.py` | 51,648 | `22d88f94bfb95207593df91589d39a5dc88a524c120441bd60e1ced96e510536` | checksum observed locally and Yunji |

Both existing SSH/tmux sessions had timed out before the first transfer
attempt, so no remote GET or extraction occurred in that attempt. Their
existing `ssh` and `sensor-audit` panes were respawned with the original
commands; both reverse tunnels then returned HTTP 200. The two robots each
downloaded the same archive through remote loopback
`127.0.0.1:18089`, verified its full SHA-256, validated the tar stream and
extracted it without changing ownership.

Remote shell syntax checks and read-only Python compilation passed. The
observed post-sync command state was:

- WSJ live receiver count `0`; Go2 bridge count `0`;
- Yunji live receiver count `0`;
- local Hub restored with the debug robot configuration and
  `goal_output_enabled=false` for both robots.

No robot-side process was restarted, so this synchronization issued no
physical command. The new WSJ router and speed floors are staged on disk and
will load on the next controlled stack restart. Both remote temporary
archives, the local temporary archive/directory and the loopback-only HTTP
server were removed after verification.
