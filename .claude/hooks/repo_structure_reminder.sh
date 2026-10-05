#!/usr/bin/env bash
# Claude Code hook: on git actions in this project, remind Claude to check docs/repo_structure.md.
#   PreToolUse  (commit, push):                          update the diagram before it goes out
#   PostToolUse (pull, checkout, switch, merge, rebase,  the structure may have changed: update to match
#                submodule, clone)
# Also runs .githooks/check_repo_structure.sh and passes its result along.
input="$(cat)"
cmd="$(jq -r '.tool_input.command // empty' <<<"$input")"
event="$(jq -r '.hook_event_name // empty' <<<"$input")"
case "$event" in
  PreToolUse)  verbs='commit|push' ;;
  PostToolUse) verbs='pull|checkout|switch|merge|rebase|submodule|clone' ;;
  *) exit 0 ;;
esac
verb="$(grep -oE "(^|[;&|(]|&&|\|\|)[[:space:]]*git([[:space:]]+-[^[:space:]]+([[:space:]]+[^-[:space:]][^[:space:]]*)?)*[[:space:]]+($verbs)\b" <<<"$cmd" | grep -oE "($verbs)\$" | head -1)"
[ -n "$verb" ] || exit 0

root="${CLAUDE_PROJECT_DIR:-$(git rev-parse --show-toplevel 2>/dev/null)}"
if check="$(cd "$root" && .githooks/check_repo_structure.sh 2>&1)"; then
  result="Name check: PASS (every tracked folder and submodule path/fork/branch is named)."
else
  result="Name check: FAIL. $check"
fi
if [ "$event" = PreToolUse ]; then
  when="You are about to run git $verb. If this changes folders, submodules, forks, branches, remotes or what is local-only, update docs/repo_structure.md first and include it in the commit."
else
  when="git $verb just ran. Compare docs/repo_structure.md with the current layout (folders, submodules, forks, branches, remotes, local-only parts) and update and commit it if it is out of date."
fi
jq -n --arg e "$event" --arg c "Repo structure check: $when $result The script only checks names, so check arrows and notes by hand." \
  '{hookSpecificOutput: {hookEventName: $e, additionalContext: $c}}'
