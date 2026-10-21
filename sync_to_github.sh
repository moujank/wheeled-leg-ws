#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$repo_root"

readonly REMOTE_NAME="origin"
readonly MAIN_BRANCH="main"
readonly TRAIN_BRANCH="wheeled_train"
readonly MUJOCO_BRANCH="wheeled_mujoco"
readonly TRAIN_PREFIX="modules/wheeled_train"
readonly MUJOCO_PREFIX="modules/wheeled_mujoco"
readonly GITHUB_FILE_LIMIT=$((100 * 1024 * 1024))
readonly INDEX_CLEAN_PATTERNS=(
  ".vscode/*.db"
  ".vscode/*.db-*"
  "**/.vscode/*.db"
  "**/.vscode/*.db-*"
)

log() {
  printf '[sync] %s\n' "$*"
}

die() {
  printf 'Error: %s\n' "$*" >&2
  exit 1
}

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  die "current directory is not a Git repository."
fi

branch="$(git rev-parse --abbrev-ref HEAD)"
if [[ "$branch" != "$MAIN_BRANCH" ]]; then
  die "run this script from branch '$MAIN_BRANCH' (current: '$branch')."
fi

if ! git remote get-url "$REMOTE_NAME" >/dev/null 2>&1; then
  die "remote '$REMOTE_NAME' does not exist."
fi

if ! git subtree --help >/dev/null 2>&1; then
  die "'git subtree' is not available in this Git installation."
fi

for prefix in "$TRAIN_PREFIX" "$MUJOCO_PREFIX"; do
  if [[ ! -d "$prefix" ]]; then
    die "required directory '$prefix' does not exist."
  fi
done

timestamp="$(date '+%Y-%m-%d %H:%M:%S %z')"
message="${1:-sync: ${timestamp}}"

check_large_tracked_files() {
  local oversized
  oversized="$(
    git ls-tree -r -l HEAD |
      awk -v limit="$GITHUB_FILE_LIMIT" '$4 != "-" && $4 + 0 > limit {print $5 "\t" $4}'
  )"

  if [[ -n "$oversized" ]]; then
    printf 'Error: tracked files exceed GitHub hard limit (%d bytes):\n' "$GITHUB_FILE_LIMIT" >&2
    printf '%s\n' "$oversized" >&2
    exit 1
  fi
}

split_branch() {
  local prefix="$1"
  git subtree split --prefix="$prefix" HEAD
}

update_local_branch() {
  local branch_name="$1"
  local commit_sha="$2"
  git branch -f "$branch_name" "$commit_sha" >/dev/null
}

drop_indexed_local_databases() {
  local removed=0
  local path

  while IFS= read -r -d '' path; do
    [[ -z "$path" ]] && continue
    git rm --cached --quiet --ignore-unmatch -- "$path"
    removed=1
  done < <(
    git ls-files -z -- "${INDEX_CLEAN_PATTERNS[@]}"
  )

  if (( removed )); then
    log "Removed tracked local database files from the Git index."
  fi
}

git add -A
drop_indexed_local_databases

if git diff --cached --quiet; then
  log "No new changes to commit on $MAIN_BRANCH."
else
  log "Creating commit on $MAIN_BRANCH."
  git commit -m "$message"
fi

check_large_tracked_files

log "Fetching remote refs from $REMOTE_NAME."
git fetch "$REMOTE_NAME"

log "Pushing $MAIN_BRANCH to $REMOTE_NAME."
git push "$REMOTE_NAME" "$MAIN_BRANCH"

log "Splitting $TRAIN_PREFIX into branch $TRAIN_BRANCH."
train_split_sha="$(split_branch "$TRAIN_PREFIX")"
update_local_branch "$TRAIN_BRANCH" "$train_split_sha"

log "Splitting $MUJOCO_PREFIX into branch $MUJOCO_BRANCH."
mujoco_split_sha="$(split_branch "$MUJOCO_PREFIX")"
update_local_branch "$MUJOCO_BRANCH" "$mujoco_split_sha"

log "Pushing split branches to $REMOTE_NAME."
git push --force-with-lease "$REMOTE_NAME" "$TRAIN_BRANCH" "$MUJOCO_BRANCH"

log "Sync completed."
log "$MAIN_BRANCH -> $(git rev-parse --short HEAD)"
log "$TRAIN_BRANCH -> ${train_split_sha:0:7}"
log "$MUJOCO_BRANCH -> ${mujoco_split_sha:0:7}"
