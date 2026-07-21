#!/usr/bin/env bash
# Repository-level reproducibility and publication guard.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HUB_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$(cd "$HUB_DIR/.." && pwd)"
run_tests=false
PYTHON_BIN="${FOCUS_TEST_PYTHON:-$HUB_DIR/.venv/bin/python}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tests) run_tests=true; shift ;;
    --python) PYTHON_BIN="$2"; shift 2 ;;
    -h|--help)
      echo "Usage: $0 [--tests] [--python PYTHON]"
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

cd "$WORKSPACE"
git rev-parse --is-inside-work-tree >/dev/null

forbidden_regex='^(artifacts/|data/|logs/|hub/runtime/|hub/\.venv/|hub/config/robots\.json$|\.claude/)|(^|/)(\.token|\.robot_token|\.admin_token|admin_token|tokens\.json)$'
forbidden="$(git ls-files | grep -E "$forbidden_regex" || true)"
if [[ -n "$forbidden" ]]; then
  echo "Forbidden runtime/secret paths are tracked:" >&2
  printf '%s\n' "$forbidden" >&2
  exit 1
fi

oversized="$(git ls-files -z | xargs -0 -r stat -c '%s %n' | awk '$1 > 52428800 {print}')"
if [[ -n "$oversized" ]]; then
  echo "Tracked files larger than 50 MiB are not allowed:" >&2
  printf '%s\n' "$oversized" >&2
  exit 1
fi

git diff --check
(cd "$WORKSPACE" && sha256sum -c manifests/source-files.sha256 >/dev/null)
(cd "$WORKSPACE/hub/robot_overlay/tinynav_snapshot" && sha256sum -c manifest.sha256 >/dev/null)
(cd "$WORKSPACE/hub/robot_overlay/tinynav_snapshot/working-tree-files" && sha256sum -c ../untracked.sha256 >/dev/null)
while IFS=$'\t' read -r link_path link_target; do
  [[ -n "$link_path" && "${link_path:0:1}" != "#" ]] || continue
  [[ -L "$link_path" ]] || {
    echo "Expected source symlink is missing: $link_path" >&2
    exit 1
  }
  [[ "$(readlink "$link_path")" == "$link_target" ]] || {
    echo "Source symlink target changed: $link_path" >&2
    exit 1
  }
done < manifests/source-symlinks.txt

if command -v rg >/dev/null; then
  secret_hits="$(git ls-files -z | xargs -0 rg -n -I \
    '(ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|AKIA[0-9A-Z]{16}|-----BEGIN (RSA |OPENSSH |EC )?PRIVATE KEY-----)' \
    2>/dev/null || true)"
  if [[ -n "$secret_hits" ]]; then
    echo "High-confidence credential pattern found in tracked content:" >&2
    printf '%s\n' "$secret_hits" >&2
    exit 1
  fi
fi

while IFS= read -r -d '' shell_file; do
  bash -n "$shell_file"
done < <(git ls-files -z '*.sh')

python_check="$(command -v python3.10 || command -v python3)"
"$python_check" - <<'PY'
import ast
import subprocess
from pathlib import Path

paths = subprocess.check_output(["git", "ls-files", "*.py"], text=True).splitlines()
for name in paths:
    path = Path(name)
    ast.parse(path.read_text(encoding="utf-8"), filename=name)
print(f"AST parsed {len(paths)} tracked Python files")
PY

if [[ "$run_tests" == true ]]; then
  [[ -x "$PYTHON_BIN" ]] || {
    echo "Test Python does not exist: $PYTHON_BIN" >&2
    exit 1
  }
  "$PYTHON_BIN" -m pytest "$HUB_DIR/tests" -q
fi

echo "Repository verification passed."
