# cxx11 guard 回归测试：中文注释误报 + 真实违规检出。
#
# 构建 fixture 头文件树，运行 check_public_header_cxx11.cmake 并断言：
#   - 真实违规（violation.h 中的 std::variant）必须被报告；
#   - 中文注释中提及的禁止 token（cjk_comment.h）不得被误报
#     （file(STRINGS) 对 UTF-8 中文按字节切行曾导致该误报）。
#
# 用法（由 tests/cmake/CompatibilityChecks.cmake 注册为 ctest）：
#   cmake -DSOURCE_DIR=<repo> -DFIXTURE_BASE_DIR=<build>/tests
#         -P check_public_header_cxx11_regression.cmake

if(NOT DEFINED FIXTURE_BASE_DIR)
  message(FATAL_ERROR "FIXTURE_BASE_DIR is required")
endif()

set(_fixture_root "${FIXTURE_BASE_DIR}/public_header_cxx11_fixture")
file(REMOVE_RECURSE "${_fixture_root}")
file(MAKE_DIRECTORY "${_fixture_root}/include/1q")

# 仅注释行含禁止 token 的中文注释头：修复后不得被报告。
file(WRITE "${_fixture_root}/include/1q/cjk_comment.h" [=[
/**
 * @file cjk_comment.h
 * @brief 中文注释提及 std::variant 不应被误报（CJK 行切分回归样例）。
 */
#ifndef FIXTURE_CJK_COMMENT_H_
#define FIXTURE_CJK_COMMENT_H_
namespace fixture {
// 注释行：本行提及 std::variant 与 if constexpr，均须被跳过。
struct OneqCjkComment {
  int value{0};
};
}  // namespace fixture
#endif  // FIXTURE_CJK_COMMENT_H_
]=])

# 真实违规头：代码中的 std::variant 必须被报告。
file(WRITE "${_fixture_root}/include/1q/violation.h" [=[
#ifndef FIXTURE_VIOLATION_H_
#define FIXTURE_VIOLATION_H_
#include <variant>
namespace fixture {
using OneqViolation = std::variant<int, double>;
}
#endif  // FIXTURE_VIOLATION_H_
]=])

execute_process(
  COMMAND "${CMAKE_COMMAND}"
          "-DSOURCE_DIR=${_fixture_root}"
          -P "${CMAKE_CURRENT_LIST_DIR}/check_public_header_cxx11.cmake"
  RESULT_VARIABLE _result
  OUTPUT_VARIABLE _output
  ERROR_VARIABLE _error)

set(_guard_output "${_output}${_error}")

if(_result EQUAL 0)
  message(FATAL_ERROR
      "[cxx11-guard-regression] guard passed on a fixture containing a real violation")
endif()

string(FIND "${_guard_output}" "violation.h" _violation_idx)
if(_violation_idx EQUAL -1)
  message(FATAL_ERROR
      "[cxx11-guard-regression] guard did not report the real violation (violation.h)")
endif()

string(FIND "${_guard_output}" "cjk_comment.h" _cjk_idx)
if(NOT _cjk_idx EQUAL -1)
  message(FATAL_ERROR
      "[cxx11-guard-regression] guard falsely reported the CJK comment header (cjk_comment.h)\n"
      "${_guard_output}")
endif()

message(STATUS
    "[cxx11-guard-regression] PASS: violation detected, CJK comment not flagged")
