#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$repo_root"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Error: current directory is not a Git repository." >&2
  exit 1
fi

branch="$(git rev-parse --abbrev-ref HEAD)"
timestamp="$(date '+%Y-%m-%d %H:%M:%S %z')"
message="${1:-sync: ${timestamp}}"

git add -A

if git diff --cached --quiet; then
  echo "No changes to commit."
  exit 0
fi

git commit -m "$message"
git push origin "$branch"

echo "Synced to GitHub on branch $branch."
