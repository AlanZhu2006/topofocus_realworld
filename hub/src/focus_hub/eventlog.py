"""Durable append-only JSONL log for decision lifecycle events.

Every published decision and every robot acknowledgement is appended with an
fsync, so the decision history survives a hub crash — a prerequisite for the
G5 audit trail (`api.py` previously kept acks in memory only).
"""
from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path


class DecisionEventLog:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._lock = threading.Lock()

    def append(self, event: str, payload: dict) -> None:
        record = {"t_ns": time.time_ns(), "event": event, **payload}
        line = json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n"
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(line)
                handle.flush()
                os.fsync(handle.fileno())
