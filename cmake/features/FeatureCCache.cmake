# FeatureCCache.cmake
# 配置 ccache 缓存编译结果以加速重新编译
# 仅 UNIX（Linux/macOS）默认开启，Windows 下强制 OFF 且不可改

if(USE_CCACHE)
    # 从 PATH 查找 ccache 可执行文件。
    find_program(CCACHE_EXECUTABLE NAMES ccache)
    if(CCACHE_EXECUTABLE)
        # 挂载为编译器启动器：所有 C/C++ 编译调用先经 ccache 命中缓存则跳过实际编译。
        # 这两个变量必须在 add_executable/add_library 之前设置才生效，故放在 features 层早期加载。
        set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_EXECUTABLE}")
        set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_EXECUTABLE}")
        message(STATUS "ccache: Enabled")
        message(STATUS "  └─ Executable: ${CCACHE_EXECUTABLE}")

        # 获取 ccache 版本信息（可选，仅用于配置期日志展示）。
        execute_process(
            COMMAND ${CCACHE_EXECUTABLE} --version
            OUTPUT_VARIABLE CCACHE_VERSION_OUTPUT
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        if(CCACHE_VERSION_OUTPUT)
            # 从 --version 输出中提取形如 "ccache version 4.10.2" 的版本号。
            string(REGEX MATCH "ccache version [0-9]+\\.[0-9]+(\\.[0-9]+)?"
                   CCACHE_VERSION "${CCACHE_VERSION_OUTPUT}")
            if(CCACHE_VERSION)
                message(STATUS "  └─ Version: ${CCACHE_VERSION}")
            endif()
        endif()
    else()
        # 显式请求却未安装：仅告警，不阻断配置（与 clang-format 的失败处理风格一致）。
        message(WARNING "ccache: Requested but not found in PATH")
        message(WARNING "  └─ Install ccache or disable USE_CCACHE option")
    endif()
else()
    message(STATUS "ccache: Disabled")
endif()
