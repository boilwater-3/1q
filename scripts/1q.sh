#!/usr/bin/env bash
# 1Q Git Bash 构建/测试统一入口（强制 preset 流，避免 UCRT 与 PATH 踩坑）。
#
# 用法：
#   source scripts/activate_1q_git_bash.sh    # 每个 Git Bash 会话一次（或写入 ~/.bashrc）
#   scripts/1q.sh bootstrap VisualStudio.15.0-amd64
#   scripts/1q.sh configure VisualStudio.15.0-amd64
#   scripts/1q.sh build VisualStudio.15.0-amd64-release [--target T ...]
#   scripts/1q.sh test VisualStudio.15.0-amd64-release [-R regex] [-j N]
#   scripts/1q.sh clean-stale VisualStudio.15.0-amd64 remote_identification_radar
#   scripts/1q.sh doctor
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/1q_env.sh
source "${SCRIPT_DIR}/lib/1q_env.sh"

usage() {
  cat <<'EOF'
1Q Git Bash 工具

  scripts/1q.sh bootstrap <configure_preset>
  scripts/1q.sh configure <configure_preset>
  scripts/1q.sh build <build_preset> [--target T ...] [-j N]
  scripts/1q.sh test <test_preset> [-R regex] [-j N]
  scripts/1q.sh clean-stale <configure_preset> <module_slug>
  scripts/1q.sh doctor

configure_preset 例：VisualStudio.15.0-amd64
build/test preset 例：VisualStudio.15.0-amd64-release | VisualStudio.15.0-amd64-debug

首次使用：source scripts/activate_1q_git_bash.sh
EOF
}

doctor() {
  echo "ONEQ_ROOT=${ONEQ_ROOT}"
  echo "ONEQ_CMAKE_ROOT=${ONEQ_CMAKE_ROOT:-<unset>}"
  echo "uname=$(uname -s 2>/dev/null || echo unknown)"
  case "$(uname -s 2>/dev/null || true)" in
    MINGW*|MSYS*|CYGWIN*) ;;
    Linux*)
      if [[ "${ONEQ_ROOT}" == /mnt/?/* || "${ONEQ_ROOT}" == /d/* || "${ONEQ_ROOT}" == /D/* ]]; then
        echo "SHELL_WARNING=Cursor/WSL bash detected on a Windows repo path."
        echo "  Windows v141 builds MUST use Git Bash (MINGW), not WSL or system32 bash."
        echo "  Symptom: ctest=NOT FOUND (WSL does not resolve ctest.exe as ctest)."
      fi
      ;;
  esac
  if command -v cmake >/dev/null 2>&1; then
    echo "cmake=$(command -v cmake)"
    cmake --version | head -n 1
  else
    echo "cmake=NOT FOUND"
  fi
  _ctest="$(_oneq_resolve_ctest || true)"
  if [[ -n "${_ctest}" ]]; then
    echo "ctest=${_ctest}"
  else
    echo "ctest=NOT FOUND"
  fi
  unset _ctest
  echo "UCRTContentRoot=${UCRTContentRoot:-<unset>}"
  echo "ONEQ_REAL_CMAKE=${ONEQ_REAL_CMAKE:-<unset>}"
  echo "ONEQ_GIT_BASH_ACTIVATED=${ONEQ_GIT_BASH_ACTIVATED:-0}"
}

# Resolve ctest even when the shell only finds Windows *.exe via full path
# (WSL: command -v ctest fails; Git Bash: finds ctest.exe as ctest).
_oneq_resolve_ctest() {
  if command -v ctest >/dev/null 2>&1; then
    command -v ctest
    return 0
  fi
  if [[ -n "${ONEQ_REAL_CMAKE:-}" ]]; then
    local dir candidate
    dir="$(dirname "${ONEQ_REAL_CMAKE}")"
    for candidate in "${dir}/ctest.exe" "${dir}/ctest"; do
      if [[ -x "${candidate}" || -f "${candidate}" ]]; then
        printf '%s' "${candidate}"
        return 0
      fi
    done
  fi
  return 1
}

cmd="${1:-}"
shift || true

case "${cmd}" in
  bootstrap)
    [[ $# -ge 1 ]] || { usage; exit 2; }
    exec bash "${SCRIPT_DIR}/bootstrap_conan.sh" "$1"
    ;;
  configure)
    [[ $# -ge 1 ]] || { usage; exit 2; }
    exec cmake --preset "$1"
    ;;
  build)
    [[ $# -ge 1 ]] || { usage; exit 2; }
    preset="$1"
    shift
    extra=()
    while [[ $# -gt 0 ]]; do
      extra+=("$1")
      shift
    done
    exec cmake --build --preset "${preset}" "${extra[@]}"
    ;;
  test)
    [[ $# -ge 1 ]] || { usage; exit 2; }
    preset="$1"
    shift
    extra=(--output-on-failure)
    while [[ $# -gt 0 ]]; do
      extra+=("$1")
      shift
    done
    if ! printf '%s\n' "${extra[@]}" | grep -q -- '-j'; then
      extra+=(-j 4)
    fi
    _ctest="$(_oneq_resolve_ctest || true)"
    if [[ -z "${_ctest}" ]]; then
      echo "[1q.sh test] ctest not found. Run: source scripts/activate_1q_git_bash.sh && scripts/1q.sh doctor" >&2
      echo "  On Windows use Git Bash (MINGW). WSL/system32 bash cannot see ctest.exe as 'ctest'." >&2
      exit 127
    fi
    exec "${_ctest}" --preset "${preset}" "${extra[@]}"
    ;;
  clean-stale)
    [[ $# -eq 2 ]] || { usage; exit 2; }
    exec bash "${SCRIPT_DIR}/1q_clean_stale.sh" "$1" "$2"
    ;;
  doctor)
    doctor
    ;;
  ""|help|-h|--help)
    usage
    ;;
  *)
    echo "未知子命令: ${cmd}" >&2
    usage
    exit 2
    ;;
esac
