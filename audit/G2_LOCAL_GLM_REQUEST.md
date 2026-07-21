# G2 controlled offline GLM-4V request

Date: 2026-07-18 (Asia/Shanghai)

Target: `asus4090`, Ubuntu 22.04, NVIDIA GeForce RTX 4090 (49140 MiB as
reported by the driver), driver 550.107.02, environment `hub/.venv`
(torch 2.8.0+cu128, transformers 4.51.0).

## Gate commands

Terminal 1 (server, from the read-only upstream entrypoint via the writable
runtime wrapper):

```bash
cd /home/asus/Research/focus_realworld_workspace
FOCUS_GLM_PORT=31511 bash hub/scripts/run_glm_offline.sh
```

Terminal 2 (single controlled request):

```bash
cd /home/asus/Research/focus_realworld_workspace
hub/.venv/bin/python hub/tools/g2_request.py --base-url http://127.0.0.1:31511/v1
```

Result: **PASS**. Request script exit status 0; both HTTP calls returned 200.

## Observed server lifecycle

- `HF_HUB_OFFLINE=1` / `TRANSFORMERS_OFFLINE=1`; model loaded entirely from the
  pinned local snapshot; 15/15 checkpoint shards loaded in ~18 s.
- Attention hooks registered on layers [36, 37, 38, 39] (`top_m=4`); FastV and
  PruMerge both explicitly DISABLED at startup.
- `Application startup complete`, listening only on `http://127.0.0.1:31511`
  (single listener confirmed with `ss -tlnp`, pid 3063142).
- GPU memory: 976 MiB baseline before start (one unrelated pre-existing
  process) → 14,786 MiB after model load → 14,840 MiB after the request →
  976 MiB after shutdown. Model footprint ≈ 13.8 GiB.
- Shutdown via SIGINT: `Application shutdown complete`, `Finished server
  process [3063142]`, process exited 0, port 31511 released, model GPU memory
  fully released.

## Observed request/response

`g2_request.py` sends the local
`source/Focus_realworld/CogVLM2/basic_demo/demo.jpg` base64-inline, prompt
"Reply with only A.", `temperature=0.0`, `max_tokens=1`,
`return_string_probabilities="[A, B]"`.

- `GET /v1/models` → 200; the upstream demo reports model id `cogvlm2-19b`
  even though the loaded weights are the GLM-4V-9B snapshot (naming quirk of
  the upstream entrypoint, recorded as-is; the completion echoes
  `"model": "THUDM/glm-4v-9b"` from the request).
- `POST /v1/chat/completions` → 200. Assistant content: `"A"`.
- Deterministic string probabilities over `[A, B]`:
  `[0.9993736743927002, 0.000626334105618298]`.
- Attention output present: 1600 per-patch weights (sum 1.0, peak patch 1526),
  `attention_index_mapping: null`.
- Server-side `model.generate` time: 2527.6 ms; whole client script wall time
  2.75 s.
- Usage counters are all 0 — the upstream demo does not fill them; recorded
  as a limitation, not patched (upstream stays read-only).

Full JSON response: [G2_response_full.json](G2_response_full.json).
Raw server log retained for this run only in the session scratchpad; the key
lines are quoted above.

## Post-conditions verified

- `ss -tlnp` shows no listener on 31511.
- `nvidia-smi` back at the 976 MiB pre-existing baseline.
- `hub/.venv/bin/python -m pytest hub/tests -q` → 11 passed (unchanged).

## Scope

G2 proves exactly one thing: the decision VLM can serve a controlled,
deterministic, offline request end-to-end on this machine with hooks active
and clean resource release. It is not evidence of navigation quality, replay
mapping (G3), dual-robot fusion (G4), or physical safety behavior (G5).
