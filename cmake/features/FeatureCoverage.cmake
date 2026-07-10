# FeatureCoverage.cmake
# LLVM source-based coverage 配置
# 通过 Clang 编译器的 -fprofile-instr-generate/-fcoverage-mapping 选项在编译时插桩，运行测试后生成覆盖率数据

function(apply_coverage_options)
    set(multi_value_args TARGETS LINK_TARGETS)
    cmake_parse_arguments(ARG "" "" "${multi_value_args}" ${ARGN})

    # 覆盖率插桩会增加体积与运行开销，默认关闭；仅 ENABLE_COVERAGE=ON 时才插桩。
    if(NOT ENABLE_COVERAGE)
        return()
    endif()
    if(NOT ARG_TARGETS)
        message(FATAL_ERROR "apply_coverage_options() requires TARGETS")
    endif()
    # 未单独指定链接目标时，复用编译目标列表，保证可执行文件链接期也带上插桩运行时库。
    if(NOT ARG_LINK_TARGETS)
        set(ARG_LINK_TARGETS ${ARG_TARGETS})
    endif()

    # 校验编译器：source-based coverage 工具链（llvm-profdata / llvm-cov）全依赖 LLVM，
    # 非 Clang 编译器直接报错，而非静默使用错误的 flag。
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR
            "ENABLE_COVERAGE requires a Clang-based compiler (Clang/AppleClang) for "
            "LLVM source-based coverage, but CMAKE_CXX_COMPILER_ID is "
            "'${CMAKE_CXX_COMPILER_ID}'. Use an llvm-ninja-* preset or pass "
            "-DCMAKE_CXX_COMPILER=clang++ explicitly.")
    endif()

    # 编译期插桩：往代码中注入计数与映射，运行时产出 *.profraw 原始覆盖率数据。
    foreach(_target IN LISTS ARG_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "Coverage target does not exist: ${_target}")
        endif()
        target_compile_options("${_target}" PRIVATE
            -fprofile-instr-generate   # 生成执行次数统计（profiling data），运行时输出 *.profraw
            -fcoverage-mapping)        # 生成机器指令↔源码行的映射表，支撑逐行覆盖率报告
    endforeach()

    # 链接期：链接插桩运行时库，使程序运行时能写出 *.profraw 数据。
    foreach(_target IN LISTS ARG_LINK_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "Coverage link target does not exist: ${_target}")
        endif()
        target_link_options("${_target}" PRIVATE -fprofile-instr-generate)
    endforeach()

    message(STATUS "Coverage: Enabled (LLVM source-based)")
    message(STATUS "  └─ Compile flags: -fprofile-instr-generate -fcoverage-mapping")
    message(STATUS "  └─ Report: ./tools/coverage_report.sh --preset llvm-ninja-coverage")
endfunction()
