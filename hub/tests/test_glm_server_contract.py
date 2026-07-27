from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
import sys


HUB = Path(__file__).resolve().parents[1]
MODULE_PATH = HUB / "tools" / "run_glm_server_fail_closed.py"
SPEC = importlib.util.spec_from_file_location(
    "run_glm_server_fail_closed_contract",
    MODULE_PATH,
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_reviewed_glm_source_is_immutable_and_patch_is_exact():
    source_bytes = MODULE.UPSTREAM_SERVER.read_bytes()
    assert hashlib.sha256(source_bytes).hexdigest() == MODULE.UPSTREAM_SHA256

    patched = MODULE.patched_source_text(source_bytes.decode("utf-8"))

    assert "rand_list = [random.random()" not in patched
    assert 'spaced_candidate = f" {candidate}"' in patched
    assert "refusing upstream random or label-mismatched fallback" in patched
    assert f'ModelCard(id="{MODULE.MODEL_CARD_ID}")' in patched
    compile(patched, str(MODULE.UPSTREAM_SERVER), "exec")


def test_glm_launcher_uses_fail_closed_deployment_wrapper():
    launcher = (HUB / "scripts" / "run_glm_offline.sh").read_text(
        encoding="utf-8"
    )

    assert "run_glm_server_fail_closed.py" in launcher
    assert 'exec "$PYTHON_BIN" "$DEMO_DIR/' not in launcher
