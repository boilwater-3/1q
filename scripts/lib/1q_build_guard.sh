#!/usr/bin/env bash
# 构建门禁：禁止 cmake --build 隐式 reconfigure；CMake/依赖变更须显式 configure。
# shellcheck disable=SC2034

_oneq_run_build_guard() {
  # Windows Git Bash 无 python3 时回退 python（两者均无则报错）
  if command -v python3 >/dev/null 2>&1; then
    python3 "${ONEQ_ROOT}/scripts/lib/1q_build_guard.py" "$1"
  else
    python "${ONEQ_ROOT}/scripts/lib/1q_build_guard.py" "$1"
  fi
}
