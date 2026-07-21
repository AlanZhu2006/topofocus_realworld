#!/usr/bin/env python3
from __future__ import annotations

import argparse
import base64
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Send one controlled offline GLM-4V request")
    parser.add_argument("--base-url", default="http://127.0.0.1:31511/v1")
    parser.add_argument(
        "--image",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "source/Focus_realworld/CogVLM2/basic_demo/demo.jpg",
    )
    args = parser.parse_args()

    import httpx

    encoded = base64.b64encode(args.image.read_bytes()).decode("ascii")
    payload = {
        "model": "THUDM/glm-4v-9b",
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": "Reply with only A."},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{encoded}"}},
                ],
                "return_string_probabilities": "[A, B]",
            }
        ],
        "temperature": 0.0,
        "max_tokens": 1,
        "stream": False,
    }
    with httpx.Client(timeout=300.0) as client:
        models = client.get(f"{args.base_url}/models")
        models.raise_for_status()
        response = client.post(f"{args.base_url}/chat/completions", json=payload)
        response.raise_for_status()
    output = {"models": models.json(), "completion": response.json()}
    print(json.dumps(output, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

