# ConanBootstrapToolchain.cmake
#
# 用途：作为 CMAKE_TOOLCHAIN_FILE 的入口。
#   若 conan_toolchain.cmake 尚未生成，自动执行 conan install，
#   再 include 真正的工具链文件。
#
# 适用场景：Windows Conan 预设（Release 构建）。

set(_conan_real_toolchain
    "${CMAKE_BINARY_DIR}/build/Release/generators/conan_toolchain.cmake")

if(NOT EXISTS "${_conan_real_toolchain}")
    message(STATUS "[Conan] conan_toolchain.cmake 未找到，自动执行 conan install ...")

    find_program(_conan_exe conan)
    if(NOT _conan_exe)
        message(FATAL_ERROR "[Conan] 未找到 conan 可执行文件，请先安装 Conan 2.x（pip install conan）")
    endif()

    execute_process(
        COMMAND "${_conan_exe}" install "${CMAKE_SOURCE_DIR}"
            --output-folder "${CMAKE_BINARY_DIR}"
            -s build_type=Release
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
    message(FATAL_ERROR "[Conan] conan install 完成但仍未找到:\n  ${_conan_real_toolchain}")
endif()

include("${_conan_real_toolchain}")
