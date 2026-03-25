# clang-tidy 静态分析配置
# clang-tidy 在编译时执行代码检查，发现潜在bug和代码质量问题

if(ENABLE_CLANG_TIDY)
    set(_LLVM_HINT_PATHS)
    if(APPLE)
        list(APPEND _LLVM_HINT_PATHS
            "/opt/homebrew/opt/llvm/bin"
            "/usr/local/opt/llvm/bin")
        if(DEFINED ENV{HOMEBREW_PREFIX} AND NOT "$ENV{HOMEBREW_PREFIX}" STREQUAL "")
            list(APPEND _LLVM_HINT_PATHS "$ENV{HOMEBREW_PREFIX}/opt/llvm/bin")
        endif()
    endif()

    find_program(CLANG_TIDY_EXECUTABLE
        NAMES
            clang-tidy
            clang-tidy-20
            clang-tidy-19
            clang-tidy-18
            clang-tidy-17
            clang-tidy-16
        HINTS ${_LLVM_HINT_PATHS})
    unset(_LLVM_HINT_PATHS)
    
    if(CLANG_TIDY_EXECUTABLE)
        # 检查是否存在 .clang-tidy 配置文件
        set(CLANG_TIDY_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
        set(_CLANG_TIDY_ARGS "${CLANG_TIDY_EXECUTABLE}")

        if(DEFINED CLANG_TIDY_CHECKS AND NOT CLANG_TIDY_CHECKS STREQUAL "")
            list(APPEND _CLANG_TIDY_ARGS "-checks=${CLANG_TIDY_CHECKS}")
            message(STATUS "clang-tidy: Enabled (whitelisted checks)")
        else()
            message(STATUS "clang-tidy: Enabled (no checks whitelist)")
        endif()

        if(CLANG_TIDY_AUTO_FIX)
            list(APPEND _CLANG_TIDY_ARGS "--fix")
            message(STATUS "  └─ Auto-fix: Enabled (low-risk mechanical fixes)")
        else()
            message(STATUS "  └─ Auto-fix: Disabled")
        endif()

        if(EXISTS "${CLANG_TIDY_CONFIG_FILE}")
            list(APPEND _CLANG_TIDY_ARGS "--config-file=${CLANG_TIDY_CONFIG_FILE}")
            message(STATUS "  └─ Config file: ${CLANG_TIDY_CONFIG_FILE}")
        else()
            message(STATUS "  └─ Config file: not found")
        endif()
        set(CMAKE_CXX_CLANG_TIDY ${_CLANG_TIDY_ARGS})
        unset(_CLANG_TIDY_ARGS)
        
        message(STATUS "  └─ Executable: ${CLANG_TIDY_EXECUTABLE}")
        
        # 获取版本信息
        execute_process(
            COMMAND ${CLANG_TIDY_EXECUTABLE} --version
            OUTPUT_VARIABLE CLANG_TIDY_VERSION_OUTPUT
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        if(CLANG_TIDY_VERSION_OUTPUT)
            string(REGEX MATCH "version [0-9]+\\.[0-9]+(\\.[0-9]+)?" 
                   CLANG_TIDY_VERSION "${CLANG_TIDY_VERSION_OUTPUT}")
            if(CLANG_TIDY_VERSION)
                message(STATUS "  └─ ${CLANG_TIDY_VERSION}")
            endif()
        endif()
        
        message(STATUS "  └─ Note: This will significantly increase build time")
    else()
        message(WARNING "clang-tidy: Requested but not found")
        message(WARNING "  └─ Install clang-tidy or disable ENABLE_CLANG_TIDY option")
        message(WARNING "  └─ Ubuntu/Debian: sudo apt install clang-tidy")
        message(WARNING "  └─ macOS: brew install llvm")
    endif()
else()
    message(STATUS "clang-tidy: Disabled")
endif()
