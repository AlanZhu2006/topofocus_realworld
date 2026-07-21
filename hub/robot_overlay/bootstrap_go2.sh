#!/usr/bin/env bash
# Reconstruct the verified WSJ TinyNav source state in a new checkout. This
# script never starts ROS, navigation, a command bridge, or a physical robot.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SNAPSHOT_DIR="$SCRIPT_DIR/tinynav_snapshot"
BASE_COMMIT="576c082e69580f618a5ff313a3e74f3672abb69f"
SOURCE_URL="${TINYNAV_SOURCE_URL:-https://github.com/UniflexAI/tinynav.git}"
DESTINATION="${TINYNAV_PATCHED_ROOT:-/home/nvidia/twork/tinynav-topofocus}"
BRANCH_NAME="topofocus/wsj-repro-20260721"
EXPECTED_COMMIT="d9f88ed876bd08e35b8c57b65e6589b10170389f"
EXPECTED_TREE="d8538a6c032cce4a7b403dbcfe60a0bce09d5947"
EXPECTED_SEMANTIC_COMMIT="8cc18159c920dc0b5185fe81bd34452676bbad53"
EXPECTED_SEMANTIC_TREE="46f4b7cd8c3bdc2ed3729cd56f3d8857aa9d41df"
with_experimental=false
new_checkout=false

usage() {
  cat <<EOF
Usage: $0 [--destination DIR] [--source-url URL] [--with-experimental-semantic]

Creates a new TinyNav checkout at the pinned Apache-2.0 upstream commit,
applies the credential-sanitized WSJ deployment/IMU patch, and commits the
result locally. Existing dirty checkouts are refused.

The optional semantic overlay is an exact snapshot of separate experimental
work seen on WSJ; it is not required for the verified native BuildMap path.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --destination) DESTINATION="$2"; shift 2 ;;
    --source-url) SOURCE_URL="$2"; shift 2 ;;
    --with-experimental-semantic) with_experimental=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

for command_name in git sha256sum; do
  command -v "$command_name" >/dev/null || {
    echo "Missing required command: $command_name" >&2
    exit 1
  }
done
[[ -f "$SNAPSHOT_DIR/tinynav-required.patch" ]] || {
  echo "Missing required patch: $SNAPSHOT_DIR/tinynav-required.patch" >&2
  exit 1
}
(cd "$SNAPSHOT_DIR" && sha256sum -c manifest.sha256)

if [[ ! -e "$DESTINATION" ]]; then
  parent="$(dirname "$DESTINATION")"
  mkdir -p "$parent"
  GIT_LFS_SKIP_SMUDGE=1 git clone --filter=blob:none --no-checkout \
    "$SOURCE_URL" "$DESTINATION"
  new_checkout=true
elif [[ ! -d "$DESTINATION/.git" ]]; then
  echo "Destination exists but is not a Git checkout: $DESTINATION" >&2
  exit 1
fi

if [[ "$new_checkout" != true ]] && \
   { ! git -C "$DESTINATION" diff --quiet || ! git -C "$DESTINATION" diff --cached --quiet; }; then
  echo "Refusing to modify a dirty TinyNav checkout: $DESTINATION" >&2
  exit 1
fi

if git -C "$DESTINATION" show-ref --verify --quiet "refs/heads/$BRANCH_NAME"; then
  git -C "$DESTINATION" switch "$BRANCH_NAME"
else
  if ! git -C "$DESTINATION" cat-file -e "$BASE_COMMIT^{commit}" 2>/dev/null; then
    git -C "$DESTINATION" fetch origin "$BASE_COMMIT"
  fi
  GIT_LFS_SKIP_SMUDGE=1 git -C "$DESTINATION" switch --detach "$BASE_COMMIT"
  git -C "$DESTINATION" switch -c "$BRANCH_NAME"
  git -C "$DESTINATION" apply --check "$SNAPSHOT_DIR/tinynav-required.patch"
  git -C "$DESTINATION" apply "$SNAPSHOT_DIR/tinynav-required.patch"
  git -C "$DESTINATION" diff --check
  git -C "$DESTINATION" add -A
  env \
    GIT_AUTHOR_NAME="TopoFocus Reproducer" \
    GIT_AUTHOR_EMAIL="repro@topofocus.invalid" \
    GIT_AUTHOR_DATE="2026-07-21T14:30:00+08:00" \
    GIT_COMMITTER_NAME="TopoFocus Reproducer" \
    GIT_COMMITTER_EMAIL="repro@topofocus.invalid" \
    GIT_COMMITTER_DATE="2026-07-21T14:30:00+08:00" \
    git -C "$DESTINATION" commit --no-gpg-sign \
      -m "Apply verified TopoFocus WSJ TinyNav state"
fi

if ! git -C "$DESTINATION" apply --reverse --check \
  "$SNAPSHOT_DIR/tinynav-required.patch"; then
  echo "Required WSJ patch is not exactly represented in $DESTINATION" >&2
  exit 1
fi
actual_required_commit="$(git -C "$DESTINATION" rev-parse "$BRANCH_NAME")"
actual_required_tree="$(git -C "$DESTINATION" rev-parse "$BRANCH_NAME^{tree}")"
[[ "$actual_required_commit" == "$EXPECTED_COMMIT" ]] || {
  echo "Reconstructed commit mismatch: $actual_required_commit" >&2
  exit 1
}
[[ "$actual_required_tree" == "$EXPECTED_TREE" ]] || {
  echo "Reconstructed tree mismatch: $actual_required_tree" >&2
  exit 1
}

if [[ "$with_experimental" == true ]]; then
  overlay_branch="$BRANCH_NAME-semantic"
  if git -C "$DESTINATION" show-ref --verify --quiet "refs/heads/$overlay_branch"; then
    git -C "$DESTINATION" switch "$overlay_branch"
  else
    git -C "$DESTINATION" switch -c "$overlay_branch"
    git -C "$DESTINATION" apply --check "$SNAPSHOT_DIR/wsj-working-tree.patch"
    git -C "$DESTINATION" apply "$SNAPSHOT_DIR/wsj-working-tree.patch"
    cp -a "$SNAPSHOT_DIR/working-tree-files/." "$DESTINATION/"
    (cd "$DESTINATION" && sha256sum -c "$SNAPSHOT_DIR/untracked.sha256")
    git -C "$DESTINATION" add -A
    env \
      GIT_AUTHOR_NAME="TopoFocus Reproducer" \
      GIT_AUTHOR_EMAIL="repro@topofocus.invalid" \
      GIT_AUTHOR_DATE="2026-07-21T14:31:00+08:00" \
      GIT_COMMITTER_NAME="TopoFocus Reproducer" \
      GIT_COMMITTER_EMAIL="repro@topofocus.invalid" \
      GIT_COMMITTER_DATE="2026-07-21T14:31:00+08:00" \
      git -C "$DESTINATION" commit --no-gpg-sign \
        -m "Archive optional WSJ semantic-mapping overlay"
  fi
  [[ "$(git -C "$DESTINATION" rev-parse HEAD)" == "$EXPECTED_SEMANTIC_COMMIT" ]] || {
    echo "Experimental overlay commit mismatch." >&2
    exit 1
  }
  [[ "$(git -C "$DESTINATION" rev-parse HEAD^{tree})" == "$EXPECTED_SEMANTIC_TREE" ]] || {
    echo "Experimental overlay tree mismatch." >&2
    exit 1
  }
fi

git -C "$DESTINATION" status --short
echo "TinyNav source reconstruction complete:"
echo "  checkout: $DESTINATION"
echo "  branch:   $(git -C "$DESTINATION" branch --show-current)"
echo "  HEAD:     $(git -C "$DESTINATION" rev-parse HEAD)"
echo "No ROS process or robot command was started."
