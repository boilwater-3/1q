# Unity Build helpers.
# This file defines a target-level function; project targets opt in explicitly.

function(oneq_apply_unity_build)
    set(multi_value_args TARGETS)
    cmake_parse_arguments(ARG "" "" "${multi_value_args}" ${ARGN})

    if(NOT ENABLE_UNITY_BUILD)
        message(STATUS "Unity Build: Disabled")
        return()
    endif()
    if(NOT ARG_TARGETS)
        message(FATAL_ERROR "oneq_apply_unity_build() requires TARGETS")
    endif()

    if(NOT DEFINED CMAKE_UNITY_BUILD_BATCH_SIZE)
        set(CMAKE_UNITY_BUILD_BATCH_SIZE 16 CACHE STRING
            "Number of source files to combine in Unity Build")
    endif()

    foreach(_target IN LISTS ARG_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "Unity Build target does not exist: ${_target}")
        endif()
        set_target_properties("${_target}" PROPERTIES
            UNITY_BUILD ON
            UNITY_BUILD_BATCH_SIZE "${CMAKE_UNITY_BUILD_BATCH_SIZE}")
        if(MSVC)
            target_compile_options("${_target}" PRIVATE /bigobj)
        endif()
    endforeach()

    message(STATUS "Unity Build: Enabled")
    message(STATUS "  └─ Batch size: ${CMAKE_UNITY_BUILD_BATCH_SIZE} files per unity")
    if(MSVC)
        message(STATUS "  └─ Added /bigobj flag for MSVC targets")
    endif()
endfunction()
