#!/usr/bin/env bash
# coverage_report.sh — 1q 项目代码覆盖率一键报告生成
#
# 用法:
#   ./tools/coverage_report.sh                          # 默认 llvm-ninja-coverage，全部测试
#   ./tools/coverage_report.sh --label unit             # 仅 unit 测试的覆盖率
#   ./tools/coverage_report.sh --label sar_ci           # 仅 SAR CI 子集
#   ./tools/coverage_report.sh --preset llvm-ninja-coverage --label unit
#   ./tools/coverage_report.sh --no-test                # 跳过 ctest（已跑过测试，只重新生成报告）
#   ./tools/coverage_report.sh --open                   # 生成后自动打开 HTML 报告
#   ./tools/coverage_report.sh --help
#
# 流程:
#   1. 校验指定 preset 的构建目录存在且 ENABLE_COVERAGE=ON
#   2. (可选) 运行 ctest 执行插桩后的测试二进制，产出 .profraw
#      （所有测试层 unit/integration/replay/performance 全跑，profraw 取并集）
#   3. llvm-profdata merge 合并所有 .profraw → 1q.profdata
#   4. llvm-cov show/report 用单二进制解读（避免多 -object 的 mismatched-data）
#   5. llvm-cov show -format=html 生成目录式 HTML 报告
#   6. 打印顶层覆盖率摘要表 (Region/Branch/Function/Line)
#
# 报告输出: build/<preset>/coverage_report/
# 产物路径已被 .gitignore 的 build/ 规则覆盖，无需额外忽略。

set -euo pipefail

# ----------------------------------------------------------------------------
# 参数解析
# ----------------------------------------------------------------------------
PRESET="llvm-ninja-coverage"
LABEL=""
NO_TEST=0
OPEN=0

usage() {
    cat <<'EOF'
coverage_report.sh — 1q 代码覆盖率报告生成

用法:
  ./tools/coverage_report.sh [选项]

选项:
  --preset <name>    构建预设 (默认: llvm-ninja-coverage)
  --label <label>    仅运行指定 CTest label (如 unit / sar_ci / fd_smoke / contract)
                     不指定则运行全部测试
  --no-test          跳过 ctest（假设测试已运行，仅重新生成报告）
  --open             生成后用系统默认浏览器打开 HTML 报告
  -h, --help         显示此帮助

示例:
  ./tools/coverage_report.sh                          # 全量覆盖率基线
  ./tools/coverage_report.sh --label unit             # 单元测试覆盖率
  ./tools/coverage_report.sh --label sar_ci           # SAR CI 子集覆盖率
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --preset) PRESET="$2"; shift 2 ;;
        --label)  LABEL="$2";  shift 2 ;;
        --no-test) NO_TEST=1; shift ;;
        --open)   OPEN=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "错误: 未知参数 '$1'" >&2; usage >&2; exit 1 ;;
    esac
done

# ----------------------------------------------------------------------------
# 路径与工具定位
# ----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# preset 名与 binaryDir 不一定一致（如 llvm-ninja-coverage 的 binaryDir 是
# llvm-ninja-coverage-local）。优先解析 CMakePresets.json 取真实 binaryDir
# （沿 inherits 链查找），解析失败则回退到 build/<preset>。
_parse_binary_dir() {
    local preset="$1" presets="${REPO_ROOT}/CMakePresets.json"
    [[ -f "${presets}" ]] || return 1
    command -v python3 >/dev/null 2>&1 || return 1
    python3 - "${preset}" "${presets}" "${REPO_ROOT}" <<'PYEOF'
import json, sys
preset, presets_path, repo_root = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    data = json.load(open(presets_path))
    by_name = {p["name"]: p for p in data.get("configurePresets", [])}
    seen, cur = set(), preset
    while cur and cur not in seen:
        seen.add(cur)
        p = by_name.get(cur, {})
        bd = p.get("binaryDir")
        if bd:
            print(bd.replace("${sourceDir}", repo_root))
            sys.exit(0)
        cur = p.get("inherits")
except Exception:
    pass
sys.exit(1)
PYEOF
}

BUILD_DIR="$(_parse_binary_dir "${PRESET}")"
if [[ -z "${BUILD_DIR}" ]]; then
    BUILD_DIR="${REPO_ROOT}/build/${PRESET}"
fi

# macOS 上 LLVM 工具随 Xcode CLT 分发，需通过 xcrun 定位；
# Linux 上直接用 PATH 中的 llvm-profdata/llvm-cov。
find_llvm_tool() {
    local tool="$1"
    local path
    if command -v xcrun >/dev/null 2>&1; then
        path="$(xcrun --find "${tool}" 2>/dev/null || true)"
    fi
    if [[ -z "${path}" ]]; then
        path="$(command -v "${tool}" 2>/dev/null || true)"
    fi
    echo "${path}"
}

LLVM_PROFDATA="$(find_llvm_tool llvm-profdata)"
LLVM_COV="$(find_llvm_tool llvm-cov)"

if [[ -z "${LLVM_PROFDATA}" ]]; then
    echo "错误: 未找到 llvm-profdata。" >&2
    echo "  macOS: 安装 Xcode Command Line Tools (xcode-select --install)" >&2
    echo "  Linux: 安装 llvm 包 (含 llvm-profdata)" >&2
    exit 1
fi
if [[ -z "${LLVM_COV}" ]]; then
    echo "错误: 未找到 llvm-cov。" >&2
    echo "  macOS: 安装 Xcode Command Line Tools (xcode-select --install)" >&2
    echo "  Linux: 安装 llvm 包 (含 llvm-cov)" >&2
    exit 1
fi

# ----------------------------------------------------------------------------
# 前置校验：构建目录、ENABLE_COVERAGE、测试二进制
# ----------------------------------------------------------------------------
if [[ ! -d "${BUILD_DIR}" ]]; then
    echo "错误: 构建目录不存在: ${BUILD_DIR}" >&2
    echo "  请先运行:" >&2
    echo "    cmake --preset ${PRESET}" >&2
    echo "    cmake --build --preset ${PRESET}" >&2
    exit 1
fi

if ! grep -q "ENABLE_COVERAGE:BOOL=ON\|ENABLE_COVERAGE:UNINITIALIZED=ON\|ENABLE_COVERAGE-ADVANCED:INTERNAL=ON" "${BUILD_DIR}/CMakeCache.txt" 2>/dev/null \
   || ! grep -qi "ENABLE_COVERAGE.*ON" "${BUILD_DIR}/CMakeCache.txt" 2>/dev/null; then
    # 宽松匹配：只要 ENABLE_COVERAGE 行值为 ON 即可
    if ! grep -E "^ENABLE_COVERAGE:.*=ON$" "${BUILD_DIR}/CMakeCache.txt" >/dev/null 2>&1; then
        echo "错误: 构建 ${PRESET} 未启用 ENABLE_COVERAGE。" >&2
        echo "  覆盖率报告需要用 ENABLE_COVERAGE=ON 的 preset 重新配置:" >&2
        echo "    cmake --preset ${PRESET}  (确认 preset 含 ENABLE_COVERAGE=ON)" >&2
        exit 1
    fi
fi

# 收集所有插桩测试二进制：每个都要跑（收集各自的 .profraw），
# 但只用第一个存在的二进制作为 llvm-cov 解读入口。
#
# 为什么不用多 -object？同一份 .cpp 会被链接进多个测试二进制（如 SarController.cpp
# 同时存在于 unit 和 integration），各二进制 coverage mapping 的 region hash 偶有
# 冲突，llvm-profdata 会丢弃冲突函数并报 "warning: N functions have mismatched
# data"，导致受影响文件显示虚假 0%。改用单二进制解读可消除绝大多数冲突。
#
# 单二进制（1q_unit_tests）通过静态库链接了全部 src 的 coverage mapping，且 profraw
# 仍来自所有测试层（unit/integration/replay/performance 全跑），因此并集覆盖不丢失。
# 实测：mismatched 从 317 降到个位数，5 个假性 0% 文件全部恢复真实数值。
TEST_BINS=()
PRIMARY_COV_BIN=""
for candidate in 1q_unit_tests 1q_contract_tests 1q_integration_tests 1q_replay_fast_tests 1q_performance_tests 1q_fd_tests; do
    if [[ -x "${BUILD_DIR}/bin/${candidate}" ]]; then
        TEST_BINS+=("${BUILD_DIR}/bin/${candidate}")
        if [[ -z "${PRIMARY_COV_BIN}" ]]; then
            PRIMARY_COV_BIN="${BUILD_DIR}/bin/${candidate}"
        fi
    fi
done
if [[ -z "${PRIMARY_COV_BIN}" ]]; then
    echo "错误: 在 ${BUILD_DIR}/bin/ 下未找到任何测试二进制。" >&2
    echo "  请先编译: cmake --build --preset ${PRESET}" >&2
    exit 1
fi

# ----------------------------------------------------------------------------
# 步骤 2: 运行测试（产出 .profraw）
# ----------------------------------------------------------------------------
REPORT_DIR="${BUILD_DIR}/coverage_report"
PROFDATA="${REPORT_DIR}/1q.profdata"
PRORAW_DIR="${REPORT_DIR}/profraw"
mkdir -p "${PRORAW_DIR}"

if [[ "${NO_TEST}" -eq 0 ]]; then
    echo "==> [1/4] 运行测试 (preset=${PRESET}, label=${LABEL:-全部})..."
    # LLVM_PROFILE_FILE 统一指定 .profraw 输出位置与命名，避免散落到各处。
    # %p 进程号、%r 覆盖率运行时避免多进程覆盖。
    export LLVM_PROFILE_FILE="${PRORAW_DIR}/1q-%p-%r.profraw"
    CTEST_ARGS=(ctest --preset "${PRESET}" --output-on-failure -j 4)
    if [[ -n "${LABEL}" ]]; then
        CTEST_ARGS+=(-L "${LABEL}")
    fi
    (cd "${REPO_ROOT}" && "${CTEST_ARGS[@]}") || {
        echo "警告: 部分测试失败，覆盖率数据可能不完整。" >&2
        echo "  （失败的测试不贡献 .profraw，但已通过的测试仍可生成报告）" >&2
    }
    echo ""
else
    echo "==> [1/4] 跳过测试 (--no-test)"
fi

PRORAW_COUNT="$(find "${PRORAW_DIR}" -name '*.profraw' 2>/dev/null | wc -l | tr -d ' ')"
if [[ "${PRORAW_COUNT}" -eq 0 ]]; then
    echo "错误: 未找到任何 .profraw 文件 (${PRORAW_DIR})" >&2
    echo "  确认已用 ENABLE_COVERAGE=ON 编译，且测试可执行文件确实被插桩。" >&2
    exit 1
fi
echo "    收集到 ${PRORAW_COUNT} 个 .profraw 文件"

# ----------------------------------------------------------------------------
# 步骤 3: 合并 .profraw
# ----------------------------------------------------------------------------
echo "==> [2/4] 合并 profraw → profdata..."
"${LLVM_PROFDATA}" merge \
    -o "${PROFDATA}" \
    -sparse \
    $(find "${PRORAW_DIR}" -name '*.profraw' | sort)
echo "    → ${PROFDATA}"

# ----------------------------------------------------------------------------
# 步骤 4: 生成 HTML 报告 + 文本摘要
# ----------------------------------------------------------------------------
HTML_DIR="${REPORT_DIR}/html"
echo "==> [3/4] 生成 HTML 报告..."

# 只统计 src/ 与 include/ 下的项目源码，排除测试自身、第三方、构建产物与生成代码。
# 用单二进制（PRIMARY_COV_BIN）作为 llvm-cov 解读入口——它链接了全部 src 的
# coverage mapping；profraw 已在步骤 2 由所有测试层贡献，merge 进同一份 profdata，
# 因此并集覆盖不丢失，且避免了多 -object 触发的 mismatched-data 数据失真。
SOURCE_DIRS=("${REPO_ROOT}/src" "${REPO_ROOT}/include")
COVERAGE_FILTER_ARGS=("-ignore-filename-regex=.*/generated/.*")

"${LLVM_COV}" show "${PRIMARY_COV_BIN}" \
    -instr-profile="${PROFDATA}" \
    -format=html \
    -project-title "1q" \
    -output-dir="${HTML_DIR}" \
    "${COVERAGE_FILTER_ARGS[@]}" \
    "${SOURCE_DIRS[@]}" \
    > /dev/null 2>&1 || true

# 同时生成文本摘要（终端可读，且便于 CI 日志归档）
SUMMARY_TXT="${REPORT_DIR}/summary.txt"
"${LLVM_COV}" report "${PRIMARY_COV_BIN}" \
    -instr-profile="${PROFDATA}" \
    "${COVERAGE_FILTER_ARGS[@]}" \
    "${SOURCE_DIRS[@]}" \
    > "${SUMMARY_TXT}" 2>&1 || true

echo "    → ${HTML_DIR}/index.html"

# ----------------------------------------------------------------------------
# 步骤 5: 打印顶层覆盖率摘要
# ----------------------------------------------------------------------------
echo "==> [4/4] 覆盖率摘要:"
echo ""
# 从 summary.txt 提取 TOTAL 行（llvm-cov report 末行汇总）并格式化为可读表格。
# llvm-cov report 的 TOTAL 行字段顺序固定：
# TOTAL  <Regions> <Missed> <Cover%>  <Functions> <Missed> <Cover%>  <Lines> <Missed> <Cover%>  <Branches> <Missed> <Cover%>
if grep -E "^TOTAL" "${SUMMARY_TXT}" >/dev/null 2>&1; then
    grep -E "^TOTAL" "${SUMMARY_TXT}" | tail -1 | awk '{
        printf "  ┌──────────────────────┬───────────┬───────────┬───────────┐\n"
        printf "  │ 指标 (Metric)        │  总数      │  未覆盖    │  覆盖率    │\n"
        printf "  ├──────────────────────┼───────────┼───────────┼───────────┤\n"
        printf "  │ Region   (源码区域)  │ %9s │ %9s │ %8s  │\n", $2, $3, $4
        printf "  │ Function (函数)      │ %9s │ %9s │ %8s  │\n", $5, $6, $7
        printf "  │ Line      (行)       │ %9s │ %9s │ %8s  │\n", $8, $9, $10
        printf "  │ Branch   (分支) ★    │ %9s │ %9s │ %8s  │\n", $11, $12, $13
        printf "  └──────────────────────┴───────────┴───────────┴───────────┘\n"
        printf "\n  ★ Branch 为本项目主指标（数值密集代码看重分支覆盖）\n"
    }'
else
    # 兜底：直接打印原始摘要末尾
    tail -5 "${SUMMARY_TXT}"
fi

echo ""
echo "完成。"
echo "  HTML 报告: ${HTML_DIR}/index.html"
echo "  文本摘要: ${SUMMARY_TXT}"
echo "  原始数据: ${PROFDATA}"

if [[ "${OPEN}" -eq 1 ]]; then
    if command -v xdg-open >/dev/null 2>&1; then
        xdg-open "${HTML_DIR}/index.html" >/dev/null 2>&1 || true
    else
        open "${HTML_DIR}/index.html" 2>/dev/null || true
    fi
fi
