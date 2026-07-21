# G0 local verification — 2026-07-18

Target workspace:

`/home/asus/Research/focus_realworld_workspace`

Prepared source:

`GPU:/home/chatsign/data/data-002/focus_realworld_workspace/`

The first rsync connection stopped after a partial model transfer. It was not treated as success. The same command was resumed with `--partial --append-verify`; the second invocation exited 0 and transferred the remaining model blobs, CLIP, YOLO, source and dependencies.

## Prepared-source comparison

These three checksum dry-runs exited 0 and printed no paths:

```bash
rsync -aHnc --out-format='%i %n%L' GPU:<prepared>/source/ <local>/source/
rsync -aHnc --out-format='%i %n%L' GPU:<prepared>/dependencies/ <local>/dependencies/
rsync -aHnc --out-format='%i %n%L' GPU:<prepared>/artifacts/ <local>/artifacts/
```

This compares file content, link structure and selected archive metadata without changing the prepared source.

## Local audit result

Command:

```bash
/usr/bin/python3.10 hub/tools/g0_audit.py --workspace "$PWD" --full-hash
```

Observed result: exit 0.

- 153 Python files across copied source/dependencies and new hub code parsed with zero AST failures.
- GLM cache revision: `3376fea6e54db68587a89bf1ac27a6889bafb867`.
- GLM model shards: 15 of 15.
- GLM snapshot broken links: 0.
- All 23 GLM cache blobs were read and SHA-256 hashed. Every 64-character content-addressed blob matched its filename; failures: 0.
- Forbidden overlay/SIF/ObjectNav zip discovered: 0.

Standalone artifact identities:

| Artifact | Bytes | SHA-256 |
|---|---:|---|
| RedNet `rednet_semmap_mp3d_40.pth` | 656,550,984 | `f94d1c62a73bc05690ae29200d3dbd033ff243e7ce91755d1cd928bde844f995` |
| YOLO `yolov10m.pt` | 33,643,667 | `6dc78f7a88591cec1e8716b8f5c7e3aefa9206684f025d202be34439ccb329a0` |
| CLIP `ViT-B-32.pt` | 353,976,522 | `40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af` |

The repeatable audit code, rather than copied terminal output, is retained at `hub/tools/g0_audit.py`.

## Gate conclusion

G0 passed on this target machine. This means the selected source and four model-asset classes are complete and traceable. It does not imply any model has loaded, any map has updated, or either physical robot has received a command.

