#!/usr/bin/env bash
# 在 Git Bash 会话中 source 一次，补 cmake PATH + 1q 命令。
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

# .githooks/pre-commit 自愈激活（幂等；BOM 自动补齐 + completeness 门禁）
if [[ -d "${ONEQ_ROOT}/.githooks" ]] && git -C "${ONEQ_ROOT}" rev-parse --git-dir >/dev/null 2>&1; then
  _oneq_hooks="$(git -C "${ONEQ_ROOT}" config --get core.hooksPath || true)"
  if [[ "${_oneq_hooks}" != ".githooks" ]]; then
    git -C "${ONEQ_ROOT}" config core.hooksPath .githooks
  fi
fi
unset _oneq_hooks

unset _SCRIPT_DIR _oneq_path_has
