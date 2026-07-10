# FeatureClangFormat.cmake
# clang-format 代码格式化配置
# 提供 format / format-check 目标，手动触发，不影响默认构建流程
# UNIX（Linux/macOS）默认开启

# macOS 下 Homebrew 的 LLVM 安装路径（系统自带的 clang 通常不带 clang-format）。
set(_LLVM_HINT_PATHS)
if(APPLE)
    list(APPEND _LLVM_HINT_PATHS
        "/opt/homebrew/opt/llvm/bin"        # Apple Silicon Homebrew 默认前缀
        "/usr/local/opt/llvm/bin")          # Intel Homebrew 默认前缀
    if(DEFINED ENV{HOMEBREW_PREFIX} AND NOT "$ENV{HOMEBREW_PREFIX}" STREQUAL "")
        list(APPEND _LLVM_HINT_PATHS "$ENV{HOMEBREW_PREFIX}/opt/llvm/bin")  # 自定义 Homebrew 前缀
    endif()
endif()

# 查找 clang-format 可执行文件：按版本号降序列出，优先匹配高版本。
find_program(CLANG_FORMAT_EXECUTABLE
    NAMES
        clang-format
        clang-format-20
        clang-format-19
        clang-format-18
        clang-format-17
        clang-format-16
    HINTS ${_LLVM_HINT_PATHS})
unset(_LLVM_HINT_PATHS)

# 未找到 clang-format 时仅告警并跳过目标创建（不阻断整个配置）。
if(NOT CLANG_FORMAT_EXECUTABLE)
    message(WARNING "clang-format: Not found, format targets are unavailable")
    message(WARNING "  └─ Install clang-format (macOS: brew install llvm)")
    return()
endif()

# clang-format 扫描范围：顶层 5 目录 × C/C++ 常见扩展名。
# GLOB_RECURSE 会递归子目录，因此每个目录只需一个 pattern；扩展名集中声明，
# 用双重循环展开目录×扩展名，避免手写笛卡尔积。
set(_clang_format_dirs include src tests examples tools)
set(_clang_format_exts .h .hpp .hh .hxx .cc .cpp .cxx .c)

set(CLANG_FORMAT_GLOB_PATTERNS)
foreach(_dir IN LISTS _clang_format_dirs)
    foreach(_ext IN LISTS _clang_format_exts)
        list(APPEND CLANG_FORMAT_GLOB_PATTERNS "${CMAKE_SOURCE_DIR}/${_dir}/*${_ext}")
    endforeach()
endforeach()
unset(_dir)
unset(_ext)

# 按模式递归扫描源文件。CONFIGURE_DEPENDS 使 CMake 在文件增删后自动重新生成。
set(CLANG_FORMAT_FILES)
foreach(_format_pattern IN LISTS CLANG_FORMAT_GLOB_PATTERNS)
    file(GLOB_RECURSE _format_matches CONFIGURE_DEPENDS "${_format_pattern}")
    if(_format_matches)
        list(APPEND CLANG_FORMAT_FILES ${_format_matches})
    endif()
endforeach()
unset(_format_pattern)
unset(_format_matches)

if(NOT CLANG_FORMAT_FILES)
    message(STATUS "clang-format: No C/C++ files found, targets are skipped")
    return()
endif()

# 多目录模式可能扫到同一文件（符号链接等），去重并排序，保证输出稳定。
list(REMOVE_DUPLICATES CLANG_FORMAT_FILES)
list(SORT CLANG_FORMAT_FILES)

# format：原地格式化；--style=file 读取仓库根 .clang-format。
add_custom_target(format
    COMMAND ${CLANG_FORMAT_EXECUTABLE} -i --style=file ${CLANG_FORMAT_FILES}
    COMMENT "Formatting C/C++ sources with clang-format"
    VERBATIM)

# format-check：只检查不改写；--dry-run --Werror 使任何不符都以非零码退出，
# 适合接入 CI 门禁。
add_custom_target(format-check
    COMMAND ${CLANG_FORMAT_EXECUTABLE} --dry-run --Werror --style=file ${CLANG_FORMAT_FILES}
    COMMENT "Checking C/C++ sources format with clang-format"
    VERBATIM)

list(LENGTH CLANG_FORMAT_FILES _clang_format_file_count)
message(STATUS "clang-format: Enabled targets (format, format-check)")
message(STATUS "  └─ Executable: ${CLANG_FORMAT_EXECUTABLE}")
message(STATUS "  └─ Files: ${_clang_format_file_count}")
unset(_clang_format_file_count)
