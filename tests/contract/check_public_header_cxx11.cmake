# 公共头 C++11 兼容性守护：扫描 include/1q/ 是否出现 C++14/17 特征。
#
# 背景：项目构建标准为 C++17（jsbsim/1.3.1 等依赖要求 cppstd>=17），但
# include/1q/ 公共头必须守 C++11 子集，以保证 VS2015（MSVC 190）消费方可编译。
# VS2015 经 ConanBootstrapToolchain 将 cppstd 映射为 14，故公共头连 C++14
# 特性都不得使用。
#
# 本脚本做静态正则扫描（快速门），与既有编译式检查
# tests/contract/check_sar_cxx11_compat.cmake（强，但仅覆盖 SAR 源）互补：
#   - 本脚本覆盖 include/1q/ 全域公共头，捕获"字面引入"的 14/17 特性。
#   - 编译式检查是最终真相；本脚本提供更早、更广的反馈。
#
# 局限：纯正则无法理解宏展开与模板上下文，可能漏报复杂形式或误报注释/字符串。
# 禁止项刻意选取"高信噪比"特征（即合法 C++11 代码几乎不会出现的 token），
# 以降低误报。被守护的不变量一旦被破坏，应优先考虑改用 C++11 等价写法，
# 而非放宽本检查。

if(NOT DEFINED SOURCE_DIR OR SOURCE_DIR STREQUAL "")
  message(FATAL_ERROR "SOURCE_DIR must be provided")
endif()

set(PUBLIC_INCLUDE_ROOT "${SOURCE_DIR}/include/1q")

file(GLOB_RECURSE PUBLIC_HEADERS
     "${PUBLIC_INCLUDE_ROOT}/*.h"
     "${PUBLIC_INCLUDE_ROOT}/*.hpp")

# 高信噪比 C++14/17 特征。patterns 与 descs 等长并行列表。
# patterns 中 `++` 在 CMake REGEX 中是普通字符（不作为量词），故 std::optional 等
# 不需转义；此处仅当被当作完整 MATCHES 模式时才需注意——本脚本将每个 pattern
# 单独作为 MATCHES 参数，CMake 会把它当普通串比较（非正则）。
set(PATTERNS
    "std::optional"
    "std::variant"
    "std::any"
    "std::string_view"
    "std::filesystem"
    "std::byte"
    "std::make_unique"
    "std::enable_if_t"
    "std::void_t"
    "if constexpr"
    "auto ["
    "[[fallthrough]]"
    "[[nodiscard]]"
    "[[maybe_unused]]")

set(DESCS
    "C++17 std::optional"
    "C++17 std::variant"
    "C++17 std::any"
    "C++17 std::string_view"
    "C++17 std::filesystem"
    "C++17 std::byte"
    "C++14 std::make_unique"
    "C++14 std::enable_if_t"
    "C++17 std::void_t"
    "C++17 if constexpr"
    "C++17 structured bindings"
    "C++17 [[fallthrough]]"
    "C++17 [[nodiscard]]"
    "C++17 [[maybe_unused]]")

list(LENGTH PATTERNS _pattern_count)

set(VIOLATIONS)

foreach(HEADER IN LISTS PUBLIC_HEADERS)
  file(STRINGS "${HEADER}" HEADER_LINES)
  set(_line_no 0)
  foreach(LINE IN LISTS HEADER_LINES)
    math(EXPR _line_no "${_line_no} + 1")
    # 跳过注释行（降低误报）
    string(STRIP "${LINE}" _stripped)
    if(_stripped MATCHES "^(//|/\\*|\\*)")
      continue()
    endif()
    math(EXPR _last "${_pattern_count} - 1")
    foreach(_i RANGE ${_last})
      list(GET PATTERNS ${_i} _pattern)
      # 用 string(FIND) 做字面子串匹配（非正则），避免 CMake REGEX 对 + 的误判。
      string(FIND "${LINE}" "${_pattern}" _pos)
      if(NOT _pos EQUAL -1)
        list(GET DESCS ${_i} _desc)
        list(APPEND VIOLATIONS
             "${HEADER}:${_line_no}: ${_desc}: ${LINE}")
      endif()
    endforeach()
  endforeach()
endforeach()

if(VIOLATIONS)
  list(JOIN VIOLATIONS "\n" VIOLATION_TEXT)
  message(FATAL_ERROR
          "公共头 C++11 兼容性守护失败。\n"
          "规则：include/1q/ 公共头必须守 C++11 子集（VS2015 消费方兼容）。\n"
          "      构建标准为 C++17（jsbsim 等依赖要求），但公共头不得使用\n"
          "      C++14/17 特性。请改用 C++11 等价写法，而非放宽本检查。\n"
          "见：cmake/project/ProjectLanguageDefaults.cmake 注释、\n"
          "      tests/contract/check_sar_cxx11_compat.cmake（编译式最终真相）。\n"
          "违规：\n${VIOLATION_TEXT}")
endif()

message(STATUS "[公共头 C++11 守护] 通过：include/1q/ 未发现 C++14/17 特征。")
