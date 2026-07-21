# Audited architecture

## What this repository is

`Focus_realworld` is a two-agent, centralized semantic ObjectNav experiment on a locally modified Habitat/HM3D stack.  A GLM-4V HTTP server is started separately.  `main.py` owns both agents in one process, makes one joint Habitat step, and reports team metrics.

It is not multi-node or decentralized.  “Two robots” means two agent instances in one Python process and one simulator process.

## Runtime flow

1. The GLM server in `CogVLM2/basic_demo/glm4_openai_api_demo_1gpu.py` loads GLM-4V-9B with 4-bit bitsandbytes quantization and exposes an OpenAI-like HTTP endpoint.
2. `main.py` creates one `LLM_Agent` per configured agent.  Each agent loads RedNet, a semantic map, collision state, and an FMM low-level planner.
3. At every joint step, each agent consumes RGB, depth, semantic observations, GPS, compass, and goal data supplied by Habitat.
4. Each agent updates a 480×480 map at 5 cm resolution (24 m extent).  The driver fuses maps and wall masks across agents using `torch.max`.
5. On step 0 and every 25 steps, YOLO and the VLM build a shared frontier/object/history view.  Decision-VLM scores choices A–D from first-token logits.  Candidate frontiers are removed sequentially after allocation, which produces different assignments when alternatives exist.
6. An observed target overrides a frontier.  FMM produces forward/turn/look/stop actions.  The patched Habitat implementation accepts a list of agent actions.
7. Team success is “any agent succeeds”; team SPL uses the maximum per-agent SPL, so it is a custom team metric rather than stock Habitat output.

## TopoFocus / room additions

The source contains geodesic room segmentation, IoU room-ID tracking, room-patch alignment, and active visual-token selection.  It keeps all 1,600 vision patches through ViT, then reduces LLM input to room-pooled tokens plus up to 256 active patch tokens.  The first decision does not prune; only the Decision-VLM path does.

The CLIP room ranking is logged but is not consumed by the observed goal/frontier selection path.  Treat it as diagnostic, not a demonstrated control mechanism.  `envs/docs/step7_hard_pruning_plan.txt` also states that the stored attention visualization was broken and the active-patch selection was geometric.

## Deployment seams

| Existing seam | Physical-robot replacement |
| --- | --- |
| Habitat `reset()` and `step(list_of_actions)` | sensor ingest plus robot command API |
| simulator RGB-D, GPS, compass | calibrated cameras, localization, IMU/odometry, shared transform |
| HM3D semantics/goal success | live semantic estimates and an independently defined success checker |
| discrete FMM action | high-level goal for a robot-local controller, initially |
| in-process agent list | messages from two robot IDs with sequencing and freshness checks |

The mapping, map fusion, frontier allocation, VLM prompting, and high-level target-selection pieces are candidates to reuse.  Simulator reset, episode bookkeeping, ground-truth metrics, and direct Habitat action dispatch are not.
