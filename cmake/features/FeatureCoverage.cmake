# LLVM source-based coverage helpers.
# This file defines a target-level function; project targets opt in explicitly.

function(oneq_apply_coverage_options)
    set(multi_value_args TARGETS LINK_TARGETS)
    cmake_parse_arguments(ARG "" "" "${multi_value_args}" ${ARGN})

    if(NOT ENABLE_COVERAGE)
        return()
    endif()
    if(NOT ARG_TARGETS)
        message(FATAL_ERROR "oneq_apply_coverage_options() requires TARGETS")
    endif()
    if(NOT ARG_LINK_TARGETS)
        set(ARG_LINK_TARGETS ${ARG_TARGETS})
    endif()

    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR
            "ENABLE_COVERAGE requires a Clang-based compiler (Clang/AppleClang) for "
            "LLVM source-based coverage, but CMAKE_CXX_COMPILER_ID is "
            "'${CMAKE_CXX_COMPILER_ID}'. Use an llvm-ninja-* preset or pass "
            "-DCMAKE_CXX_COMPILER=clang++ explicitly.")
    endif()

    foreach(_target IN LISTS ARG_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "Coverage target does not exist: ${_target}")
        endif()
        target_compile_options("${_target}" PRIVATE
            -fprofile-instr-generate
            -fcoverage-mapping)
    endforeach()

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
