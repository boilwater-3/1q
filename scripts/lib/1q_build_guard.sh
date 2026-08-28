#!/usr/bin/env bash
# 构建门禁：禁止 cmake --build 隐式 reconfigure；CMake/依赖变更须显式 configure。
# shellcheck disable=SC2034

_oneq_run_build_guard() {
  python3 "${ONEQ_ROOT}/scripts/lib/1q_build_guard.py" "$1"
}
