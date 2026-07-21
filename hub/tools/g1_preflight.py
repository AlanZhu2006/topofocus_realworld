#!/usr/bin/env python3
from __future__ import annotations

import argparse
import gc
import importlib
import json
import os
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the Focus G1 import/model preflight")
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--skip-model-loads", action="store_true")
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    source = workspace / "source/Focus_realworld"
    rednet_root = workspace / "dependencies/RedNet"
    vision = workspace / "artifacts/vision"
    snapshot = (
        workspace
        / "artifacts/models/hf_cache/hub/models--THUDM--glm-4v-9b/snapshots/3376fea6e54db68587a89bf1ac27a6889bafb867"
    )

    os.environ["HF_HOME"] = str(workspace / "artifacts/models/hf_cache")
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["TRANSFORMERS_OFFLINE"] = "1"
    yolo_config = workspace / "hub/runtime/ultralytics"
    yolo_config.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("YOLO_CONFIG_DIR", str(yolo_config))
    sys.path[:0] = [str(rednet_root), str(workspace / "dependencies"), str(source)]

    results: dict[str, object] = {"python": sys.version, "imports": {}}
    failures: list[dict[str, str]] = []
    modules = (
        "torch",
        "torchvision",
        "transformers",
        "accelerate",
        "bitsandbytes",
        "fastapi",
        "uvicorn",
        "sse_starlette",
        "loguru",
        "ultralytics",
        "clip",
        "cv2",
        "skimage",
        "skfmm",
        "tiktoken",
        "PIL",
    )
    for name in modules:
        try:
            module = importlib.import_module(name)
            results["imports"][name] = getattr(module, "__version__", "present")
        except Exception as exc:
            failures.append({"stage": f"import:{name}", "error": f"{type(exc).__name__}: {exc}"})

    try:
        import torch

        results["torch"] = {
            "version": torch.__version__,
            "cuda_runtime": torch.version.cuda,
            "cuda_available": torch.cuda.is_available(),
            "device": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
        }
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available")
        results["torch"]["kernel_sum"] = torch.arange(8, device="cuda").sum().item()
    except Exception as exc:
        failures.append({"stage": "torch_cuda_kernel", "error": f"{type(exc).__name__}: {exc}"})

    if not args.skip_model_loads:
        try:
            import torch
            from RedNet.RedNet_model import load_rednet

            model = load_rednet(
                torch.device("cpu"),
                str(workspace / "artifacts/checkpoints/rednet_semmap_mp3d_40.pth"),
            )
            results["rednet"] = {"loaded": True, "parameters": sum(p.numel() for p in model.parameters())}
            del model
            gc.collect()
        except Exception as exc:
            failures.append({"stage": "rednet_load", "error": f"{type(exc).__name__}: {exc}"})

        try:
            from ultralytics import YOLO

            model = YOLO(str(vision / "yolov10m.pt"))
            results["yolo"] = {"loaded": True, "task": model.task}
            del model
            gc.collect()
        except Exception as exc:
            failures.append({"stage": "yolo_load", "error": f"{type(exc).__name__}: {exc}"})

        try:
            import clip

            model, _ = clip.load("ViT-B/32", device="cpu", download_root=str(vision))
            results["clip"] = {"loaded": True, "dtype": str(next(model.parameters()).dtype)}
            del model
            gc.collect()
        except Exception as exc:
            failures.append({"stage": "clip_load", "error": f"{type(exc).__name__}: {exc}"})

        try:
            from transformers import AutoConfig, AutoTokenizer

            config = AutoConfig.from_pretrained(snapshot, trust_remote_code=True, local_files_only=True)
            tokenizer = AutoTokenizer.from_pretrained(snapshot, trust_remote_code=True, local_files_only=True)
            results["glm"] = {
                "config_loaded": True,
                "model_type": config.model_type,
                "tokenizer": type(tokenizer).__name__,
                "snapshot": str(snapshot),
            }
        except Exception as exc:
            failures.append({"stage": "glm_offline_config", "error": f"{type(exc).__name__}: {exc}"})

    results["failures"] = failures
    print(json.dumps(results, indent=2, sort_keys=True))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
