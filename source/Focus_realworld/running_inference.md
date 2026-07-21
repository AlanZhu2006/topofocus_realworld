# Running Inference — Focus_realworld (Topofocus multi-robot ObjectNav)

Self-contained bundle of the MCoCoNav / Topofocus inference pipeline.
Location on NYU Torch HPC: `/scratch/jl9356/Focus_realworld`

Verified working end-to-end on Torch (1x NVIDIA L40S) on 2026-07-17: the VLM server
loaded `glm-4v-9b` (4-bit) from the bundled cache and answered an image request (HTTP 200).

---

## Singularity image (important)

This pipeline runs **inside a Singularity/Apptainer container** — the conda environment
lives in the bundled overlay `overlay-15GB-500K.ext3` and is mounted into the container.

We used the NYU Torch **shared** image:

    /share/apps/images/cuda12.6.3-cudnn9.5.1-ubuntu22.04.5.sif

- If you are on **Torch**, use that path directly (it's a shared system image).
- If you need your **own copy**, you may **copy it from there** — that is fine:
  `cp /share/apps/images/cuda12.6.3-cudnn9.5.1-ubuntu22.04.5.sif <your_dir>/`
- On a different cluster, any **CUDA 12.6 + cuDNN 9.5 + Ubuntu 22.04** Singularity image works.

---

## What's in the bundle
- Inference code: `main.py` (nav driver), `CogVLM2/` (VLM API server),
  `agents/ envs/ utils/ src/ configs/ detect/` (YOLO), `room_semantic.py`,
  `semantic_mapping.py`, `constants.py`, `arguments.py`, `tasks/`
- Weights: `hf_cache/` (glm-4v-9b VLM + CLIP), `detect/yolov10m.pt`, `rednet_semmap_mp3d_40.pth`
- Conda env: `overlay-15GB-500K.ext3` (env name: `mcoconav`)
- Data: `data/` (HM3D scenes + ObjectNav episodes) — only needed for the Habitat simulator

## Requirements
- Singularity/Apptainer + the image above
- 1 GPU (tested on L40S; glm-4v-9b loads in 4-bit, ~8 GB VRAM)

---

## How to run (two processes sharing one GPU)

Point BASE at this folder first (in each shell):

    export BASE=/scratch/jl9356/Focus_realworld     # or wherever you place it
    export SIF=/share/apps/images/cuda12.6.3-cudnn9.5.1-ubuntu22.04.5.sif

### Terminal 1 — VLM server
    singularity exec --nv --overlay $BASE/overlay-15GB-500K.ext3:ro $SIF /bin/bash
    source /ext3/env.sh && conda activate mcoconav
    export HF_HOME=$BASE/hf_cache HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
    cd $BASE/CogVLM2/basic_demo
    python glm4_openai_api_demo_1gpu.py --port 8000

Wait for: `Uvicorn running on http://127.0.0.1:8000`

### Terminal 2 — nav driver
    singularity exec --nv --overlay $BASE/overlay-15GB-500K.ext3:ro $SIF /bin/bash
    source /ext3/env.sh && conda activate mcoconav
    export HF_HOME=$BASE/hf_cache HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
    cd $BASE
    python main.py -d ./VLM_EXP/run1/ --num_agents 2 \
      --task_config tasks/multi_objectnav_hm3d.yaml --split minival \
      --base_url http://127.0.0.1:8000
    # add --enable_pruning for Topofocus vision-token pruning (R+K tokens instead of 1600)

---

## Notes
- The VLM uses `trust_remote_code`; all remote modeling files are already cached in
  `hf_cache/` (snapshot + `hf_cache/modules/transformers_modules/`), so it runs **offline**
  (no internet needed on compute nodes).
- `data/` is for the Habitat simulator. For a real-world / robot driver you supply your own
  sensor stream and can ignore `data/`.
- Single image per request only (CogVLM2 constraint).
- Grab a GPU interactively on Torch with, e.g.:
  `srun --account=<your_project_account> --gres=gpu:1 --cpus-per-task=4 --mem=48G --time=1:00:00 --pty /bin/bash`
