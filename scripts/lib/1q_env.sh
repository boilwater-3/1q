#!/usr/bin/env bash
# 1Q Git Bash 环境：补 PATH（cmake/ctest）、注入 v141 UCRTContentRoot。
# 由 scripts/activate_1q_git_bash.sh、scripts/1q.sh、scripts/bin/cmake 等 source。
# 幂等：重复 source 安全。
#
# 机器级覆盖（可选，不进 git）：scripts/1q_env.local.sh
#   export ONEQ_CMAKE_ROOT='D:/environment/CMake'

if [[ -n "${ONEQ_ENV_LOADED:-}" ]]; then
  return 0 2>/dev/null || exit 0
fi
ONEQ_ENV_LOADED=1

if [[ -z "${ONEQ_ROOT:-}" ]]; then
  ONEQ_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
fi
export ONEQ_ROOT

if [[ -f "${ONEQ_ROOT}/scripts/1q_env.local.sh" ]]; then
  # shellcheck source=/dev/null
  source "${ONEQ_ROOT}/scripts/1q_env.local.sh"
fi

_oneq_prepend_path_if_dir() {
  local dir="$1"
  [[ -d "$dir" ]] || return 0
  case ":${PATH}:" in
    *":${dir}:"*) ;;
    *) PATH="${dir}:${PATH}" ;;
  esac
}

_oneq_cmake_candidates() {
  local root="${ONEQ_ROOT}"
  local drive=""
  if [[ "${root}" == /mnt/?/* ]]; then
    drive="/mnt/${root:5:1}"
  elif [[ "${root}" == /?/* ]]; then
    drive="/${root:1:1}"
  fi

  cat <<EOF
${ONEQ_CMAKE_ROOT:+$ONEQ_CMAKE_ROOT/bin}
${drive}/environment/CMake/bin
/d/environment/CMake/bin
/mnt/d/environment/CMake/bin
/c/Program Files/CMake/bin
/c/Program Files (x86)/CMake/bin
EOF
}

_oneq_resolve_cmake_executable() {
  local dir candidate
  while IFS= read -r dir; do
    [[ -n "$dir" ]] || continue
    for candidate in "${dir}/cmake.exe" "${dir}/cmake"; do
      if [[ -x "$candidate" ]]; then
        export ONEQ_CMAKE_ROOT="${dir%/bin}"
        printf '%s' "$candidate"
        return 0
      fi
    done
  done < <(_oneq_cmake_candidates)
  return 1
}

if [[ -z "${ONEQ_REAL_CMAKE:-}" ]]; then
  if command -v cmake >/dev/null 2>&1; then
    _existing_cmake="$(command -v cmake)"
    case "${_existing_cmake}" in
      "${ONEQ_ROOT}/scripts/bin/cmake"|"${ONEQ_ROOT}/scripts/bin/cmake.exe")
        _resolved="$(_oneq_resolve_cmake_executable || true)"
        [[ -n "${_resolved:-}" ]] && export ONEQ_REAL_CMAKE="${_resolved}"
        ;;
      *)
        export ONEQ_REAL_CMAKE="${_existing_cmake}"
        ;;
    esac
  else
    _resolved="$(_oneq_resolve_cmake_executable || true)"
    [[ -n "${_resolved:-}" ]] && export ONEQ_REAL_CMAKE="${_resolved}"
  fi
fi

if [[ -n "${ONEQ_REAL_CMAKE:-}" ]]; then
  _oneq_prepend_path_if_dir "$(dirname "${ONEQ_REAL_CMAKE}")"
fi

_oneq_prepend_path_if_dir "${ONEQ_ROOT}/scripts/bin"

case "$(uname -s 2>/dev/null || true)" in
  MINGW*|MSYS*|CYGWIN*)
    if [[ -z "${UCRTContentRoot:-}" ]]; then
      export UCRTContentRoot='C:\Program Files (x86)\Windows Kits\10\'
    fi
    ;;
esac

unset _existing_cmake _resolved _oneq_prepend_path_if_dir _oneq_resolve_cmake_executable _oneq_cmake_candidates
