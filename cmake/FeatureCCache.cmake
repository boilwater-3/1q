# ccache 配置
# ccache 缓存编译结果以加速重新编译

if(USE_CCACHE)
    find_program(CCACHE_EXECUTABLE NAMES ccache)
    if(CCACHE_EXECUTABLE)
        set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_EXECUTABLE}")
        set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_EXECUTABLE}")
        message(STATUS "ccache: Enabled")
        message(STATUS "  └─ Executable: ${CCACHE_EXECUTABLE}")
        
        # 获取 ccache 版本信息（可选）
        execute_process(
            COMMAND ${CCACHE_EXECUTABLE} --version
            OUTPUT_VARIABLE CCACHE_VERSION_OUTPUT
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        if(CCACHE_VERSION_OUTPUT)
            string(REGEX MATCH "ccache version [0-9]+\\.[0-9]+(\\.[0-9]+)?" 
                   CCACHE_VERSION "${CCACHE_VERSION_OUTPUT}")
            if(CCACHE_VERSION)
                message(STATUS "  └─ Version: ${CCACHE_VERSION}")
            endif()
        endif()
    else()
        message(WARNING "ccache: Requested but not found in PATH")
        message(WARNING "  └─ Install ccache or disable USE_CCACHE option")
    endif()
else()
    message(STATUS "ccache: Disabled")
endif()
