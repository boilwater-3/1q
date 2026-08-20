# 公共头 C++11 兼容性守护：扫描 include/1q/ 是否出现 C++14/17 特征。
#
# 背景：项目构建标准为 C++17（jsbsim/1.3.1 等依赖要求 cppstd>=17），但
# include/1q/ 公共头必须守 C++11 子集，以保证 VS2015（MSVC 190）消费方可编译。
# Conan 引导（scripts/bootstrap_conan.sh）对 VS2015 同样传 compiler.cppstd=17，
# 故公共头连 C++14 特性都不得使用。
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

# 仓库 C/C++ 源文件统一携带 UTF-8 BOM；BOM 粘在首行行首会使首行注释判定
# （^ 锚定的行首匹配）失效，首行 doc 注释中提到的禁止 token 会被误报。
string(ASCII 239 187 191 _UTF8_BOM)

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
    "[[maybe_unused]]"
    "inline constexpr"
    "inline const ")

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
    "C++17 [[maybe_unused]]"
    "C++17 inline constexpr 变量（VS2015 拒绝 inline 数据声明，改用 constexpr）"
    "C++17 inline const 变量（VS2015 拒绝 inline 数据声明，改用 const + IIFE）")

list(LENGTH PATTERNS _pattern_count)

set(VIOLATIONS)

foreach(HEADER IN LISTS PUBLIC_HEADERS)
  # 用 file(READ) 读原始内容并逐 token 扫描：file(STRINGS) 会把含 UTF-8 中文的
  # 文件按 CJK 字节切行，切碎后的注释行片段不再以注释标记开头，导致注释里
  # 提到的禁止 token 被误报（如中文注释中的 "std::variant"）。
  # 行号与注释判定均按真实换行位置计算，与文件编码无关。
  file(READ "${HEADER}" HEADER_CONTENT)
  string(FIND "${HEADER_CONTENT}" "${_UTF8_BOM}" _bom_pos)
  if(_bom_pos EQUAL 0)
    string(SUBSTRING "${HEADER_CONTENT}" 3 -1 HEADER_CONTENT)
  endif()
  math(EXPR _last "${_pattern_count} - 1")
  foreach(_i RANGE ${_last})
    list(GET PATTERNS ${_i} _pattern)
    # 用 string(FIND) 做字面子串匹配（非正则），避免 CMake REGEX 对 + 的误判。
    string(LENGTH "${_pattern}" _pattern_len)
    set(_search_from 0)
    string(SUBSTRING "${HEADER_CONTENT}" "${_search_from}" -1 _rest)
    string(FIND "${_rest}" "${_pattern}" _rel_pos)
    while(NOT _rel_pos EQUAL -1)
      math(EXPR _pos "${_search_from} + ${_rel_pos}")
      # 行号 = token 前的换行数 + 1（用长度差计数，避免行内分号干扰）。
      string(SUBSTRING "${HEADER_CONTENT}" 0 "${_pos}" _prefix)
      string(LENGTH "${_prefix}" _prefix_len)
      string(REPLACE "\n" "" _prefix_no_nl "${_prefix}")
      string(LENGTH "${_prefix_no_nl}" _prefix_no_nl_len)
      math(EXPR _line_no "${_prefix_len} - ${_prefix_no_nl_len} + 1")
      # 跳过注释行（降低误报）：取 token 所在行行首，检查注释标记。
      string(FIND "${_prefix}" "\n" _last_nl REVERSE)
      if(_last_nl EQUAL -1)
        set(_line_start 0)
      else()
        math(EXPR _line_start "${_last_nl} + 1")
      endif()
      math(EXPR _line_len "${_pos} + ${_pattern_len} - ${_line_start}")
      string(SUBSTRING "${HEADER_CONTENT}" "${_line_start}" "${_line_len}" _line_prefix)
      string(STRIP "${_line_prefix}" _stripped)
      if(NOT _stripped MATCHES "^(//|/\\*|\\*)")
        # 违规行全文（报告上下文）：到下一个换行为止。
        string(SUBSTRING "${HEADER_CONTENT}" "${_pos}" -1 _tail_from_token)
        string(FIND "${_tail_from_token}" "\n" _next_nl_rel)
        if(_next_nl_rel EQUAL -1)
          string(SUBSTRING "${HEADER_CONTENT}" "${_line_start}" -1 _line_text)
        else()
          math(EXPR _line_len "${_pos} + ${_next_nl_rel} - ${_line_start}")
          string(SUBSTRING "${HEADER_CONTENT}" "${_line_start}" "${_line_len}" _line_text)
        endif()
        list(GET DESCS ${_i} _desc)
        list(APPEND VIOLATIONS
             "${HEADER}:${_line_no}: ${_desc}: ${_line_text}")
      endif()
      math(EXPR _search_from "${_pos} + 1")
      string(SUBSTRING "${HEADER_CONTENT}" "${_search_from}" -1 _rest)
      string(FIND "${_rest}" "${_pattern}" _rel_pos)
    endwhile()
  endforeach()
endforeach()

if(VIOLATIONS)
  list(JOIN VIOLATIONS "\n" VIOLATION_TEXT)
  message(FATAL_ERROR
          "公共头 C++11 兼容性守护失败。\n"
          "规则：include/1q/ 公共头必须守 C++11 子集（VS2015 消费方兼容）。\n"
          "      构建标准为 C++17（jsbsim 等依赖要求），但公共头不得使用\n"
          "      C++14/17 特性。请改用 C++11 等价写法，而非放宽本检查。\n"
          "见：cmake/project/ProjectSetup.cmake 的语言合同、\n"
          "      tests/contract/check_sar_cxx11_compat.cmake（编译式最终真相）。\n"
          "违规：\n${VIOLATION_TEXT}")
endif()

message(STATUS "[公共头 C++11 守护] 通过：include/1q/ 未发现 C++14/17 特征。")
