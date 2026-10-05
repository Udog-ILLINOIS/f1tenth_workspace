#!/usr/bin/env bash
# Fails when docs/repo_structure.md doesn't mention every tracked folder (depth 1-2) and every
# submodule's path, fork and branch. Run by the pre-commit and pre-push hooks; checks the index.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
doc=docs/repo_structure.md
[ -f "$doc" ] || { echo "repo structure: $doc is missing" >&2; exit 1; }
missing=()
has() { grep -qF -- "$1" "$doc"; }

# Tracked folders at depth 1 and 2 (submodules show up as entries, their parents as folders)
while read -r d; do
  has "$(basename "$d")" || missing+=("folder $d")
done < <(git ls-files | awk -F/ 'NF>1{print $1} NF>2{print $1"/"$2}' | sort -u)

# Submodules: path basename, fork repo name, tracked branch
if [ -f .gitmodules ]; then
  while read -r key path; do
    name=${key#submodule.}; name=${name%.path}
    url=$(git config -f .gitmodules "submodule.$name.url")
    branch=$(git config -f .gitmodules "submodule.$name.branch" || true)
    fork=$(basename "$url" .git)
    has "$(basename "$path")" || missing+=("submodule path $path")
    has "$fork" || missing+=("fork $fork (for $path)")
    [ -z "$branch" ] || has "$branch" || missing+=("branch $branch (for $path)")
  done < <(git config -f .gitmodules --get-regexp '\.path$')
fi

if [ ${#missing[@]} -gt 0 ]; then
  echo "repo structure: $doc is out of date. Not mentioned:" >&2
  printf '  - %s\n' "${missing[@]}" >&2
  echo "Update the diagram (or bypass once with --no-verify)." >&2
  exit 1
fi
