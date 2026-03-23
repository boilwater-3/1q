# ConanBootstrapToolchain.cmake
#
# 用途：作为 CMAKE_TOOLCHAIN_FILE 的统一 Conan 入口（macOS / Windows）。
# 行为：
#   1) 检查目标 conan_toolchain.cmake 是否存在
#   2) 检查其 C++ 标准是否与当前配置一致
#   3) 缺失或不一致时自动执行 conan install
#   4) include 真实 conan_toolchain.cmake

include("${CMAKE_CURRENT_LIST_DIR}/ProjectLanguageDefaults.cmake")

# CMake 在编译器探测阶段会进入 try_compile 子工程，此时不应再次触发 Conan 安装。
get_property(_oneq_in_try_compile GLOBAL PROPERTY IN_TRY_COMPILE)
if(_oneq_in_try_compile)
    return()
endif()

# 构建类型推断规则：
#   - 单配置生成器（Ninja）：读取 CMAKE_BUILD_TYPE，未设则默认 Release
#   - 多配置生成器（Visual Studio）：CMAKE_BUILD_TYPE 为空，默认 Release
if(CMAKE_BUILD_TYPE)
    set(_conan_build_type "${CMAKE_BUILD_TYPE}")
else()
    set(_conan_build_type "Release")
endif()

# 统一 C++ 标准来源：优先使用外部传入，其次使用项目默认值
if(DEFINED CMAKE_CXX_STANDARD AND NOT CMAKE_CXX_STANDARD STREQUAL "")
    set(_conan_cppstd "${CMAKE_CXX_STANDARD}")
else()
    set(_conan_cppstd "${ONEQ_DEFAULT_CXX_STANDARD}")
endif()
if(_conan_cppstd LESS 11)
    message(FATAL_ERROR "[Conan] CMAKE_CXX_STANDARD must be at least 11")
endif()

set(_conan_real_toolchain
    "${CMAKE_BINARY_DIR}/build/${_conan_build_type}/generators/conan_toolchain.cmake")

set(_conan_needs_install FALSE)
if(NOT EXISTS "${_conan_real_toolchain}")
    set(_conan_needs_install TRUE)
    message(STATUS "[Conan] conan_toolchain.cmake 未找到，自动执行 conan install ...")
else()
    unset(_conan_toolchain_std_line)
    file(STRINGS "${_conan_real_toolchain}" _conan_toolchain_std_line
        REGEX "^[ \t]*set\\(CMAKE_CXX_STANDARD[ \t]+\"?[0-9]+\"?\\)")
    if(_conan_toolchain_std_line)
        string(REGEX MATCH "[0-9]+" _conan_toolchain_cppstd "${_conan_toolchain_std_line}")
        if(NOT _conan_toolchain_cppstd STREQUAL _conan_cppstd)
            set(_conan_needs_install TRUE)
            message(STATUS "[Conan] C++ standard mismatch: current=${_conan_toolchain_cppstd}, expected=${_conan_cppstd}. Reinstalling...")
        endif()
    else()
        set(_conan_needs_install TRUE)
        message(STATUS "[Conan] toolchain missing CMAKE_CXX_STANDARD, reinstalling...")
    endif()
endif()

if(_conan_needs_install)
    find_program(_conan_exe conan)
    if(NOT _conan_exe)
        message(FATAL_ERROR
            "[Conan] 未找到 conan 可执行文件，请先安装 Conan 2.x（pip install conan）")
    endif()

    execute_process(
        COMMAND "${_conan_exe}" install "${CMAKE_SOURCE_DIR}"
            "--output-folder=${CMAKE_BINARY_DIR}"
            --build=missing
            -s "build_type=${_conan_build_type}"
            -s "compiler.cppstd=${_conan_cppstd}"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE _conan_result
        COMMAND_ECHO STDOUT
    )

    if(NOT _conan_result EQUAL 0)
        message(FATAL_ERROR "[Conan] conan install 失败（exit code: ${_conan_result}）")
    endif()
endif()

if(NOT EXISTS "${_conan_real_toolchain}")
    message(FATAL_ERROR
        "[Conan] conan install 完成但仍未找到:\n  ${_conan_real_toolchain}")
endif()

include("${_conan_real_toolchain}")
