#!/usr/bin/env bash
# 在 Git Bash 会话中 source 一次，补 cmake PATH + UCRTContentRoot + 1q 命令。
#
# 一次性安装（写入 ~/.bashrc，新终端自动生效）：
#   echo 'source "/d/1q/1q/scripts/activate_1q_git_bash.sh" 2>/dev/null' >> ~/.bashrc
#
# 或仅在当前仓库会话：
#   source scripts/activate_1q_git_bash.sh

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/1q_env.sh
source "${_SCRIPT_DIR}/lib/1q_env.sh"

_oneq_path_has() {
  case ":${PATH}:" in
    *":$1:"*) return 0 ;;
    *) return 1 ;;
  esac
}

if ! _oneq_path_has "${ONEQ_ROOT}/scripts"; then
  export PATH="${ONEQ_ROOT}/scripts:${PATH}"
fi

if [[ -z "${ONEQ_GIT_BASH_ACTIVATED:-}" ]]; then
  export ONEQ_GIT_BASH_ACTIVATED=1
  if command -v cmake >/dev/null 2>&1; then
    echo "[1q] Git Bash 环境就绪：cmake=$(command -v cmake)"
  else
    echo "[1q] 警告：仍未找到 cmake；可设置 ONEQ_CMAKE_ROOT 后重新 source" >&2
  fi
  if [[ -n "${UCRTContentRoot:-}" ]]; then
    echo "[1q] UCRTContentRoot=${UCRTContentRoot}"
  fi
  if [[ -n "${ONEQ_REAL_CMAKE:-}" ]]; then
    echo "[1q] ONEQ_REAL_CMAKE=${ONEQ_REAL_CMAKE}"
  fi
  echo "[1q] 构建请用：scripts/1q.sh build VisualStudio.15.0-amd64-release [--target ...]"
  echo "[1q] 测试请用：scripts/1q.sh test VisualStudio.15.0-amd64-release -R 'unit::<module>'"
fi

unset _SCRIPT_DIR _oneq_path_has
