# ConanBootstrapToolchain.cmake
#
# 用途：作为 CMAKE_TOOLCHAIN_FILE 的通用入口（macOS / Windows 均适用）。
#   若 conan_toolchain.cmake 尚未生成，自动执行 conan install，
#   再 include 真正的工具链文件。
#
# 构建类型推断规则：
#   - 单配置生成器（Ninja）：读取 CMAKE_BUILD_TYPE，未设则默认 Release
#   - 多配置生成器（Visual Studio）：CMAKE_BUILD_TYPE 为空，默认 Release

if(CMAKE_BUILD_TYPE)
    set(_conan_build_type "${CMAKE_BUILD_TYPE}")
else()
    set(_conan_build_type "Release")
endif()

set(_conan_real_toolchain
    "${CMAKE_BINARY_DIR}/build/${_conan_build_type}/generators/conan_toolchain.cmake")

if(NOT EXISTS "${_conan_real_toolchain}")
    message(STATUS "[Conan] conan_toolchain.cmake 未找到，自动执行 conan install ...")

    find_program(_conan_exe conan)
    if(NOT _conan_exe)
        message(FATAL_ERROR
            "[Conan] 未找到 conan 可执行文件，请先安装 Conan 2.x（pip install conan）")
    endif()

    execute_process(
        COMMAND "${_conan_exe}" install "${CMAKE_SOURCE_DIR}"
            --output-folder "${CMAKE_BINARY_DIR}"
            -s build_type=${_conan_build_type}
            --build missing
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
