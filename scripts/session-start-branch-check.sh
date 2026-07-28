#!/usr/bin/env bash
# SessionStart hook: 检测是否在 main 分支，提醒 Claude 确认会话主题并创建功能分支
set -euo pipefail

branch=$(git branch --show-current 2>/dev/null || true)

if [ "$branch" = "main" ] || [ "$branch" = "master" ]; then
  cat <<'EOF'
{"systemMessage": "📍 On **main** branch. Enter plan mode first — the plan topic will become the branch name (`feature/<topic-slug>`). Create the branch when the plan is approved. Skip if read-only."}
EOF
fi
