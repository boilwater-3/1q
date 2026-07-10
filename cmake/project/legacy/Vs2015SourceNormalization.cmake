# legacy/Vs2015SourceNormalization.cmake
# VS2015 源码规范化：修复旧解析器 Bug
# VS2015(MSVC 190) 解析器 Bug：纯 LF + 多字节 UTF-8 文件导致行计数错位与代码页分词异常
# 默认关闭——配置期不应污染工作区检出

# 非 VS2015 生成器直接跳过。
if(NOT CMAKE_GENERATOR MATCHES "^Visual Studio 14 2015")
  return()
endif()

option(ONEQ_VS2015_NORMALIZE_SOURCE
  "Rewrite source files to CRLF with UTF-8 BOM for legacy VS2015 parsing"
  OFF
)
if(NOT ONEQ_VS2015_NORMALIZE_SOURCE)
  message(STATUS
    "VS2015 source normalisation: disabled; set ONEQ_VS2015_NORMALIZE_SOURCE=ON only for legacy parser issues"
  )
  return()
endif()

message(STATUS "VS2015 source normalisation: scanning source files")

# 递归扫描 src / include / tests 下的 C/C++ 源文件。
file(GLOB_RECURSE _ensure_crlf_files
  "${CMAKE_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_SOURCE_DIR}/src/*.h"
  "${CMAKE_SOURCE_DIR}/src/*.hpp"
  "${CMAKE_SOURCE_DIR}/include/*.h"
  "${CMAKE_SOURCE_DIR}/include/*.hpp"
  "${CMAKE_SOURCE_DIR}/tests/*.cpp"
  "${CMAKE_SOURCE_DIR}/tests/*.h"
  "${CMAKE_SOURCE_DIR}/tests/*.hpp"
)

# UTF-8 BOM 字节序列：EF BB BF（十进制 239 187 191）。
string(ASCII 239 187 191 _utf8_bom)

set(_ensure_crlf_count 0)
foreach(_file IN LISTS _ensure_crlf_files)
  # 以 HEX 读取以做字节级检测：0d0a 为 CRLF，efbbbf 为 UTF-8 BOM。
  file(READ "${_file}" _hex_content HEX)
  string(FIND "${_hex_content}" "0d0a" _crlf_pos)
  string(FIND "${_hex_content}" "efbbbf" _bom_pos)
  # 缺 CRLF 或缺头部 BOM 时重写：先剥离既有 BOM（若存在），再统一前置 BOM。
  if(_crlf_pos LESS 0 OR NOT _bom_pos EQUAL 0)
    file(READ "${_file}" _content)
    string(FIND "${_content}" "${_utf8_bom}" _existing_bom_pos)
    if(_existing_bom_pos EQUAL 0)
      string(LENGTH "${_utf8_bom}" _bom_len)
      string(SUBSTRING "${_content}" ${_bom_len} -1 _content)
    endif()
    file(WRITE "${_file}" "${_utf8_bom}${_content}")
    math(EXPR _ensure_crlf_count "${_ensure_crlf_count} + 1")
  endif()
endforeach()

if(_ensure_crlf_count GREATER 0)
  message(STATUS "VS2015 source normalisation: normalised ${_ensure_crlf_count} files to CRLF+UTF-8-BOM")
else()
  message(STATUS "VS2015 source normalisation: all files already use CRLF with BOM")
endif()

# 清理本脚本引入的所有局部变量，避免泄漏到调用方目录作用域。
unset(_file)
unset(_content)
unset(_hex_content)
unset(_crlf_pos)
unset(_bom_pos)
unset(_utf8_bom)
unset(_ensure_crlf_files)
unset(_ensure_crlf_count)
