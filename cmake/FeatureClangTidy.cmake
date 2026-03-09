# clang-tidy 静态分析配置
# clang-tidy 在编译时执行代码检查，发现潜在bug和代码质量问题

if(ENABLE_CLANG_TIDY)
    find_program(CLANG_TIDY_EXECUTABLE NAMES clang-tidy clang-tidy-18 clang-tidy-17 clang-tidy-16)
    
    if(CLANG_TIDY_EXECUTABLE)
        # 检查是否存在 .clang-tidy 配置文件
        set(CLANG_TIDY_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
        
        if(EXISTS "${CLANG_TIDY_CONFIG_FILE}")
            set(CMAKE_CXX_CLANG_TIDY 
                "${CLANG_TIDY_EXECUTABLE}"
                "--config-file=${CLANG_TIDY_CONFIG_FILE}")
            message(STATUS "clang-tidy: Enabled (with config file)")
        else()
            # 使用默认检查项
            set(CMAKE_CXX_CLANG_TIDY 
                "${CLANG_TIDY_EXECUTABLE}"
                "-checks=-*,modernize-*,performance-*,readability-*,bugprone-*")
            message(STATUS "clang-tidy: Enabled (default checks)")
            message(STATUS "  └─ Tip: Create .clang-tidy file for custom configuration")
        endif()
        
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
