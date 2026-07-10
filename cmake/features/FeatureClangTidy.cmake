# FeatureClangTidy.cmake
# clang-tidy 静态分析配置
# clang-tidy 在编译时执行代码检查，发现潜在bug和代码质量问题

if(ENABLE_CLANG_TIDY)
    # macOS 下 Homebrew 的 LLVM 安装路径（系统自带的 clang 通常不带 clang-tidy）。
    set(_LLVM_HINT_PATHS)
    if(APPLE)
        list(APPEND _LLVM_HINT_PATHS
            "/opt/homebrew/opt/llvm/bin"        # Apple Silicon Homebrew 默认前缀
            "/usr/local/opt/llvm/bin")          # Intel Homebrew 默认前缀
        if(DEFINED ENV{HOMEBREW_PREFIX} AND NOT "$ENV{HOMEBREW_PREFIX}" STREQUAL "")
            list(APPEND _LLVM_HINT_PATHS "$ENV{HOMEBREW_PREFIX}/opt/llvm/bin")  # 自定义 Homebrew 前缀
        endif()
    endif()

    # 查找 clang-tidy 可执行文件：按版本号降序列出，优先匹配高版本。
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
        # 组装最终传给 clang-tidy 的参数列表：可执行文件 + 可选 -checks 白名单 + 可选 --config-file。
        # CLANG_TIDY_CHECKS 在 BuildOptions.cmake 中定义（默认含一组 readability/modernize/... 检查）。
        set(CLANG_TIDY_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
        set(_CLANG_TIDY_ARGS "${CLANG_TIDY_EXECUTABLE}")

        # 启用的检查项白名单（逗号分隔）。CLANG_TIDY_CHECKS 非空时通过 -checks= 注入。
        if(DEFINED CLANG_TIDY_CHECKS AND NOT CLANG_TIDY_CHECKS STREQUAL "")
            list(APPEND _CLANG_TIDY_ARGS "-checks=${CLANG_TIDY_CHECKS}")
            message(STATUS "clang-tidy: Enabled (whitelisted checks)")
        else()
            message(STATUS "clang-tidy: Enabled (no checks whitelist)")
        endif()

        # 仓库根的 .clang-tidy 配置文件：存在则显式通过 --config-file 指定，
        # 避免在不同工作目录下运行构建时 clang-tidy 找不到配置而回退默认检查项。
        if(EXISTS "${CLANG_TIDY_CONFIG_FILE}")
            list(APPEND _CLANG_TIDY_ARGS "--config-file=${CLANG_TIDY_CONFIG_FILE}")
            message(STATUS "  └─ Config file: ${CLANG_TIDY_CONFIG_FILE}")
        else()
            message(STATUS "  └─ Config file: not found")
        endif()
        # 挂载为编译期静态分析启动器：每个 .cpp 编译时同步运行 clang-tidy。
        # 必须在 add_executable/add_library 之前设置才生效（与 ccache launcher 同理）。
        set(CMAKE_CXX_CLANG_TIDY ${_CLANG_TIDY_ARGS})
        unset(_CLANG_TIDY_ARGS)

        message(STATUS "  └─ Executable: ${CLANG_TIDY_EXECUTABLE}")

        # 获取版本信息（可选，仅用于配置期日志展示）。
        execute_process(
            COMMAND ${CLANG_TIDY_EXECUTABLE} --version
            OUTPUT_VARIABLE CLANG_TIDY_VERSION_OUTPUT
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        if(CLANG_TIDY_VERSION_OUTPUT)
            # 从 --version 输出中提取形如 "version 20.1.0" 的版本号。
            string(REGEX MATCH "version [0-9]+\\.[0-9]+(\\.[0-9]+)?"
                   CLANG_TIDY_VERSION "${CLANG_TIDY_VERSION_OUTPUT}")
            if(CLANG_TIDY_VERSION)
                message(STATUS "  └─ ${CLANG_TIDY_VERSION}")
            endif()
        endif()

        # 提醒：clang-tidy 在每个翻译单元上额外执行分析，构建耗时显著上升。
        message(STATUS "  └─ Note: This will significantly increase build time")
    else()
        # 显式请求却未安装：仅告警，不阻断配置（与 ccache/clang-format 的失败处理风格一致）。
        message(WARNING "clang-tidy: Requested but not found")
        message(WARNING "  └─ Install clang-tidy or disable ENABLE_CLANG_TIDY option")
        message(WARNING "  └─ Ubuntu/Debian: sudo apt install clang-tidy")
        message(WARNING "  └─ macOS: brew install llvm")
    endif()
else()
    message(STATUS "clang-tidy: Disabled")
endif()
