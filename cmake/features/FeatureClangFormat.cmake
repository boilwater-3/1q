# clang-format 代码格式化配置
# 提供 format / format-check 目标，手动触发，不影响默认构建流程

set(_LLVM_HINT_PATHS)
if(APPLE)
    list(APPEND _LLVM_HINT_PATHS
        "/opt/homebrew/opt/llvm/bin"
        "/usr/local/opt/llvm/bin")
    if(DEFINED ENV{HOMEBREW_PREFIX} AND NOT "$ENV{HOMEBREW_PREFIX}" STREQUAL "")
        list(APPEND _LLVM_HINT_PATHS "$ENV{HOMEBREW_PREFIX}/opt/llvm/bin")
    endif()
endif()

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

if(NOT CLANG_FORMAT_EXECUTABLE)
    message(WARNING "clang-format: Not found, format targets are unavailable")
    message(WARNING "  └─ Install clang-format (macOS: brew install llvm)")
    return()
endif()

set(CLANG_FORMAT_GLOB_PATTERNS
    "${CMAKE_SOURCE_DIR}/include/*.h"
    "${CMAKE_SOURCE_DIR}/include/*.hpp"
    "${CMAKE_SOURCE_DIR}/include/*.hh"
    "${CMAKE_SOURCE_DIR}/include/*.c"
    "${CMAKE_SOURCE_DIR}/include/*.cc"
    "${CMAKE_SOURCE_DIR}/include/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.h"
    "${CMAKE_SOURCE_DIR}/src/*.hpp"
    "${CMAKE_SOURCE_DIR}/src/*.hh"
    "${CMAKE_SOURCE_DIR}/src/*.c"
    "${CMAKE_SOURCE_DIR}/src/*.cc"
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/tests/*.h"
    "${CMAKE_SOURCE_DIR}/tests/*.hpp"
    "${CMAKE_SOURCE_DIR}/tests/*.hh"
    "${CMAKE_SOURCE_DIR}/tests/*.c"
    "${CMAKE_SOURCE_DIR}/tests/*.cc"
    "${CMAKE_SOURCE_DIR}/tests/*.cpp"
    "${CMAKE_SOURCE_DIR}/examples/*.h"
    "${CMAKE_SOURCE_DIR}/examples/*.hpp"
    "${CMAKE_SOURCE_DIR}/examples/*.hh"
    "${CMAKE_SOURCE_DIR}/examples/*.c"
    "${CMAKE_SOURCE_DIR}/examples/*.cc"
    "${CMAKE_SOURCE_DIR}/examples/*.cpp"
    "${CMAKE_SOURCE_DIR}/tools/*.h"
    "${CMAKE_SOURCE_DIR}/tools/*.hpp"
    "${CMAKE_SOURCE_DIR}/tools/*.hh"
    "${CMAKE_SOURCE_DIR}/tools/*.c"
    "${CMAKE_SOURCE_DIR}/tools/*.cc"
    "${CMAKE_SOURCE_DIR}/tools/*.cpp")

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

list(REMOVE_DUPLICATES CLANG_FORMAT_FILES)
list(SORT CLANG_FORMAT_FILES)

add_custom_target(format
    COMMAND ${CLANG_FORMAT_EXECUTABLE} -i --style=file ${CLANG_FORMAT_FILES}
    COMMENT "Formatting C/C++ sources with clang-format"
    VERBATIM)

add_custom_target(format-check
    COMMAND ${CLANG_FORMAT_EXECUTABLE} --dry-run --Werror --style=file ${CLANG_FORMAT_FILES}
    COMMENT "Checking C/C++ sources format with clang-format"
    VERBATIM)

list(LENGTH CLANG_FORMAT_FILES _clang_format_file_count)
message(STATUS "clang-format: Enabled targets (format, format-check)")
message(STATUS "  └─ Executable: ${CLANG_FORMAT_EXECUTABLE}")
message(STATUS "  └─ Files: ${_clang_format_file_count}")
unset(_clang_format_file_count)
