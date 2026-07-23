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

repository_candidates() {
  local candidate
  while IFS= read -r -d '' candidate; do
    [[ -f "$candidate" || -L "$candidate" ]] || continue
    printf '%s\0' "$candidate"
  done < <(
    git ls-files -z --cached --others --exclude-standard -- "$@"
  )
}
forbidden_regex='^(artifacts/|data/|logs/|hub/runtime/|hub/\.venv/|hub/config/robots\.json$|\.claude/)|(^|/)(\.token|\.robot_token|\.admin_token|admin_token|tokens\.json)$'
forbidden="$(repository_candidates | tr '\0' '\n' | grep -E "$forbidden_regex" || true)"
if [[ -n "$forbidden" ]]; then
  echo "Forbidden runtime/secret paths are tracked:" >&2
  printf '%s\n' "$forbidden" >&2
  exit 1
fi

oversized="$(repository_candidates | xargs -0 -r stat -c '%s %n' | awk '$1 > 52428800 {print}')"
if [[ -n "$oversized" ]]; then
  echo "Tracked files larger than 50 MiB are not allowed:" >&2
  printf '%s\n' "$oversized" >&2
  exit 1
fi

git diff --check HEAD --
while IFS= read -r -d '' untracked; do
  [[ -f "$untracked" ]] || continue
  LC_ALL=C grep -Iq . "$untracked" || continue
  whitespace_errors="$(
    git diff --no-index --check /dev/null "$untracked" 2>&1 || true
  )"
  if [[ -n "$whitespace_errors" ]]; then
    echo "Whitespace errors in untracked file:" >&2
    printf '%s\n' "$whitespace_errors" >&2
    exit 1
  fi
done < <(git ls-files -z --others --exclude-standard)
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
  secret_hits="$(repository_candidates | xargs -0 rg -n -I \
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
done < <(repository_candidates '*.sh')

python_check="$(command -v python3.10 || command -v python3)"
"$python_check" - <<'PY'
import ast
import subprocess
from pathlib import Path

paths = subprocess.check_output(
    [
        "git",
        "ls-files",
        "--cached",
        "--others",
        "--exclude-standard",
        "*.py",
    ],
    text=True,
).splitlines()
parsed = 0
for name in paths:
    path = Path(name)
    if not path.is_file():
        continue
    ast.parse(path.read_text(encoding="utf-8"), filename=name)
    parsed += 1
print(f"AST parsed {parsed} repository Python files")
PY

"$python_check" - <<'PY'
import json
from pathlib import Path
import subprocess

import yaml

def candidates(pattern: str) -> list[Path]:
    output = subprocess.check_output(
        [
            "git",
            "ls-files",
            "--cached",
            "--others",
            "--exclude-standard",
            pattern,
        ],
        text=True,
    )
    return [
        path
        for value in output.splitlines()
        if (path := Path(value)).is_file()
    ]

json_paths = candidates("*.json")
yaml_paths = candidates("*.yaml") + candidates("*.yml")
for path in json_paths:
    json.loads(path.read_text(encoding="utf-8"))
for path in yaml_paths:
    yaml.safe_load(path.read_text(encoding="utf-8"))
print(
    f"Parsed {len(json_paths)} JSON and {len(yaml_paths)} YAML repository files"
)
PY

if [[ "$run_tests" == true ]]; then
  [[ -x "$PYTHON_BIN" ]] || {
    echo "Test Python does not exist: $PYTHON_BIN" >&2
    exit 1
  }
  "$PYTHON_BIN" -m pytest "$HUB_DIR/tests" -q
fi

echo "Repository verification passed."
