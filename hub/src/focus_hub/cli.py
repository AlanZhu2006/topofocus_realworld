from __future__ import annotations

import argparse


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the Focus two-robot hub transport API")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--workers", type=int, default=1)
    args = parser.parse_args()

    if args.workers != 1:
        parser.error("the in-memory registry currently requires --workers 1")

    import uvicorn

    uvicorn.run("focus_hub.api:app", host=args.host, port=args.port, workers=1)


if __name__ == "__main__":
    main()

