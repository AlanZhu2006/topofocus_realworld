# G1 local environment verification

Date: 2026-07-18 (Asia/Shanghai)

Target: `asus4090`, Ubuntu 22.04, NVIDIA GeForce RTX 4090 (49140 MiB as
reported by the driver), driver 550.107.02.

## Environment construction

- Dedicated environment: `hub/.venv`, Python 3.10.
- Base: `/home/asus/miniconda3/envs/memnav`, inherited with
  `--system-site-packages` because its PyTorch/CUDA pair was already usable on
  this host.
- Additions were installed with `uv pip install --no-deps`; a first generic
  resolver attempt was interrupted before installation when it selected a new
  CUDA 13 PyTorch stack.
- Rebuild procedure: `hub/scripts/create_g1_env.sh`.

Key recorded versions:

```text
torch==2.8.0+cu128
torchvision==0.23.0+cu128
transformers==4.51.0
accelerate==1.7.0
bitsandbytes==0.49.2
ultralytics==8.4.99
openai-clip==1.0.1
scikit-image==0.23.1
scikit-fmm==2023.4.2
fastapi==0.136.1
uvicorn==0.46.0
pydantic==2.11.10
numpy==1.26.0
opencv-python==4.9.0.80
```

## Gate command and result

```bash
hub/.venv/bin/python hub/tools/g1_preflight.py \
  --workspace /home/asus/Research/focus_realworld_workspace
```

Result: **PASS**, exit status 0, no recorded failures.

- CUDA allocation and kernel execution succeeded on the RTX 4090; the test
  tensor sum was 28.
- RedNet instantiated and loaded
  `rednet_semmap_mp3d_40.pth` (epoch 53; 81,972,552 parameters).
- YOLO loaded `yolov10m.pt` as a detection model.
- CLIP ViT-B/32 loaded from the local checkpoint on CPU in float32.
- GLM-4V loaded its local `chatglm` config and `ChatGLM4Tokenizer` from the
  pinned offline snapshot.
- Hub service and model dependencies imported successfully.

`uv pip check` is not used as the gate because `uv` does not account for the
packages inherited through this virtual environment's system-site layer. The
gate exercises the actual interpreter, CUDA kernel, loaders, and artifacts.

G1 proves environment compatibility and local model loading only. It does not
prove a completed GLM request (G2), replay mapping (G3), dual-robot fusion (G4),
or physical safety behavior (G5).
