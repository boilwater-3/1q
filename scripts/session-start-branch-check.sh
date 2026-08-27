#!/usr/bin/env bash
# SessionStart hook: 检测是否在 main 分支，提醒 Claude 确认会话主题并创建功能分支
set -euo pipefail

branch=$(git branch --show-current 2>/dev/null || true)

if [ "$branch" = "main" ] || [ "$branch" = "master" ]; then
  cat <<'EOF'
{"systemMessage": "📍 On **main**. If this is a read-only discussion, ignore. For any code change: enter plan mode. Evidence-first work uses `evidence/<topic>` then `feat/<topic>`; other work uses `feature/<topic>` when the plan is approved. After the user asks to merge: `--no-ff` into `main` and delete the process branches."}
EOF
fi
