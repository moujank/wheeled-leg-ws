#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$repo_root"

output_file="GIT_HISTORY.md"
generated_at="$(date '+%Y-%m-%d %H:%M:%S %z')"

{
  echo "# Git History"
  echo
  echo "Generated at: $generated_at"
  echo
  echo "This file is generated from Git history. Run \`./update_git_history.sh\` to refresh it."
  echo
  git log \
    --date=iso \
    --pretty=format:'## %h - %s%nAuthor: %an <%ae>%nDate: %ad%n' \
    --stat
} > "$output_file"

echo "Updated $output_file"
