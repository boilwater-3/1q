#!/usr/bin/env bash
# Conan 引导脚本：按 preset 名派生参数并执行 conan install，生成 toolchain 与 CMakeDeps。
#
# 用法：
#   scripts/bootstrap_conan.sh <preset>
#
# 迁移自 ConanBootstrapToolchain.cmake（toolchain 内部触发 conan install）的两步式方案。
# 本脚本只负责"装依赖、生成 toolchain"，配置仍由 cmake --preset 完成：
#
#   scripts/bootstrap_conan.sh llvm-ninja-debug     # 1. 装依赖、生成 conan_toolchain.cmake
#   cmake --preset llvm-ninja-debug                 # 2. 配置（自动发现生成的 toolchain）
#   cmake --build --preset llvm-ninja-debug         # 3. 构建
#
set -euo pipefail

# --- 参数校验 ---
if [[ $# -ne 1 ]]; then
    echo "用法: $0 <preset>" >&2
    echo "支持的 preset: llvm-ninja-debug | llvm-ninja-coverage | llvm-ninja-release" >&2
    exit 2
fi

PRESET="$1"

# 源码根目录（脚本位于 <root>/scripts/ 下）。
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONANFILE="${SOURCE_DIR}/conanfile.py"

if [[ ! -f "${CONANFILE}" ]]; then
    echo "错误: 未找到 conanfile.py: ${CONANFILE}" >&2
    exit 1
fi

# --- 定位 conan 可执行文件 ---
if ! command -v conan >/dev/null 2>&1; then
    echo "错误: 未找到 conan 可执行文件，请先安装 Conan 2.x（pip install conan）" >&2
    exit 1
fi

# C++ 标准，与 cmake/project/ProjectSetup.cmake 的 PROJECT_DEFAULT_CXX_STANDARD 一致。
CPPSTD=17

# --- preset → 参数映射 -------------------------------------------------------
# 每个 preset 派生：binaryDir、build_type（单配置才有）、enable_testing、generator。
# binaryDir 必须与 CMakePresets.json 中各 preset 的 binaryDir 完全一致，否则 cmake 找不到
# 生成的 toolchain。注意 macOS preset 历史上把 binaryDir 覆写为带 -local 后缀。
case "${PRESET}" in
    llvm-ninja-debug)
        BINARY_DIR="${SOURCE_DIR}/build/llvm-ninja-debug-local"
        BUILD_TYPE="Debug"
        ENABLE_TESTING="True"
        GENERATOR=""            # 单配置 Ninja，无需告知 Conan 生成器
        ;;
    llvm-ninja-coverage)
        BINARY_DIR="${SOURCE_DIR}/build/llvm-ninja-coverage-local"
        BUILD_TYPE="Debug"
        ENABLE_TESTING="True"
        GENERATOR=""
        ;;
    llvm-ninja-release)
        BINARY_DIR="${SOURCE_DIR}/build/llvm-ninja-release"
        BUILD_TYPE="Release"
        ENABLE_TESTING="True"
        GENERATOR=""
        ;;
    # 本地用户 preset 变体（CMakeUserPresets.json，不进 git，binaryDir 带 -local）。
    llvm-ninja-debug-local)
        BINARY_DIR="${SOURCE_DIR}/build/llvm-ninja-debug-local"
        BUILD_TYPE="Debug"
        ENABLE_TESTING="True"
        GENERATOR=""
        ;;
    llvm-ninja-release-local)
        BINARY_DIR="${SOURCE_DIR}/build/llvm-ninja-release-local"
        BUILD_TYPE="Release"
        ENABLE_TESTING="True"
        GENERATOR=""
        ;;
    *)
        echo "错误: 不支持的 preset '${PRESET}'" >&2
        echo "支持: llvm-ninja-debug | llvm-ninja-coverage | llvm-ninja-release" >&2
        exit 2
        ;;
esac

# --- 组装 conan install 命令 -------------------------------------------------
CONAN_ARGS=(
    install "${SOURCE_DIR}"
    --output-folder "${BINARY_DIR}"
    --build=missing
    -s "compiler.cppstd=${CPPSTD}"
    -o "&:enable_testing=${ENABLE_TESTING}"
)

# 单配置 preset：显式指定 build_type（Conan 按此分包缓存）。
if [[ -n "${BUILD_TYPE}" ]]; then
    CONAN_ARGS+=(-s "build_type=${BUILD_TYPE}")
fi

# 当前正式 profile 均为单配置 Ninja；Windows provider 重新闭合后再单独恢复。
if [[ -n "${GENERATOR}" ]]; then
    CONAN_ARGS+=(
        -c "tools.cmake.cmaketoolchain:generator=${GENERATOR}"
        -c "tools.microsoft.msbuild:vs_version=14"
        -s "os=Windows"
        -s "arch=x86_64"
        -s "compiler=msvc"
        -s "compiler.version=190"
        -s "compiler.runtime=dynamic"
    )
fi

# --- 执行 -------------------------------------------------------------------
echo "[bootstrap_conan] preset=${PRESET}"
echo "[bootstrap_conan] output-folder=${BINARY_DIR}"
echo "[bootstrap_conan] build_type=${BUILD_TYPE:-<multi-config>}"
echo "[bootstrap_conan] enable_testing=${ENABLE_TESTING}"
echo "[bootstrap_conan] 运行: conan ${CONAN_ARGS[*]}"
echo

conan "${CONAN_ARGS[@]}"

# --- 报告生成的 toolchain 路径 ----------------------------------------------
if [[ -n "${GENERATOR}" ]]; then
    # VS multi-config：toolchain 在 <binaryDir>/build/generators/（不带 build_type）。
    TOOLCHAIN="${BINARY_DIR}/build/generators/conan_toolchain.cmake"
else
    # 单配置：toolchain 在 <binaryDir>/build/<build_type>/generators/。
    TOOLCHAIN="${BINARY_DIR}/build/${BUILD_TYPE}/generators/conan_toolchain.cmake"
fi

echo
echo "[bootstrap_conan] 成功。生成的 toolchain:"
echo "  ${TOOLCHAIN}"
if [[ ! -f "${TOOLCHAIN}" ]]; then
    echo "警告: 预期路径未找到 toolchain 文件，请检查 conan 输出。" >&2
fi
echo
echo "下一步："
echo "  cmake --preset ${PRESET}"
