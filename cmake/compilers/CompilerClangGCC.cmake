# GCC/Clang compiler option helpers.
# This file defines functions only; project targets opt in explicitly.

function(oneq_apply_clang_gcc_options)
    set(one_value_args ENABLE_WARNINGS STACK_SIZE_OPTION)
    set(multi_value_args TARGETS LINK_TARGETS)
    cmake_parse_arguments(ARG "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    if(MSVC)
        message(FATAL_ERROR "oneq_apply_clang_gcc_options() requires a GCC/Clang compiler")
    endif()
    if(NOT ARG_TARGETS)
        message(FATAL_ERROR "oneq_apply_clang_gcc_options() requires TARGETS")
    endif()
    if(NOT DEFINED ARG_STACK_SIZE_OPTION OR ARG_STACK_SIZE_OPTION STREQUAL "")
        set(ARG_STACK_SIZE_OPTION "DEFAULT")
    endif()
    if(NOT ARG_LINK_TARGETS)
        set(ARG_LINK_TARGETS ${ARG_TARGETS})
    endif()

    message(STATUS "Configuring for GCC/Clang compiler")
    message(STATUS "  └─ Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

    foreach(_target IN LISTS ARG_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "Compiler target does not exist: ${_target}")
        endif()
        set_target_properties("${_target}" PROPERTIES POSITION_INDEPENDENT_CODE ON)
        target_compile_options("${_target}" PRIVATE -fvisibility=hidden)

        if(ARG_ENABLE_WARNINGS)
            target_compile_options("${_target}" PRIVATE
                -Wall
                -Wextra
                -Wpedantic
                -Wshadow
                -Wnon-virtual-dtor
                -Wold-style-cast
                -Wcast-align
                -Wunused
                -Woverloaded-virtual
                -Wconversion
                -Wsign-conversion
                -Wdouble-promotion
                -Wformat=2
                -Wimplicit-fallthrough)
            if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
                target_compile_options("${_target}" PRIVATE
                    -Wmisleading-indentation
                    -Wduplicated-cond
                    -Wduplicated-branches
                    -Wlogical-op
                    -Wnull-dereference
                    -Wuseless-cast)
            endif()
        else()
            target_compile_options("${_target}" PRIVATE -Wall)
        endif()

        target_compile_options("${_target}" PRIVATE
            $<$<CONFIG:Debug>:-g3>
            $<$<CONFIG:Debug>:-O0>
            $<$<CONFIG:Debug>:-fno-omit-frame-pointer>
            $<$<CONFIG:Debug>:-fno-inline>
            $<$<CONFIG:Release>:-O3>
            $<$<CONFIG:Release>:-DNDEBUG>
            $<$<CONFIG:Release>:-flto>
            $<$<CONFIG:Release>:-fomit-frame-pointer>
            $<$<CONFIG:RelWithDebInfo>:-O2>
            $<$<CONFIG:RelWithDebInfo>:-g>
            $<$<CONFIG:RelWithDebInfo>:-DNDEBUG>
            $<$<CONFIG:RelWithDebInfo>:-fno-omit-frame-pointer>
            $<$<CONFIG:MinSizeRel>:-Os>
            $<$<CONFIG:MinSizeRel>:-DNDEBUG>
            $<$<CONFIG:MinSizeRel>:-flto>)
    endforeach()

    foreach(_target IN LISTS ARG_LINK_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "Compiler link target does not exist: ${_target}")
        endif()
        if(NOT APPLE)
            if(ARG_STACK_SIZE_OPTION STREQUAL "RECOMMENDED")
                target_link_options("${_target}" PRIVATE -Wl,-z,stack-size=2097152)
            elseif(ARG_STACK_SIZE_OPTION STREQUAL "LARGE_PROJECT")
                target_link_options("${_target}" PRIVATE -Wl,-z,stack-size=4194304)
            elseif(ARG_STACK_SIZE_OPTION STREQUAL "EXTREME_RECURSION")
                target_link_options("${_target}" PRIVATE -Wl,-z,stack-size=8388608)
            endif()
        endif()

        target_link_options("${_target}" PRIVATE
            $<$<CONFIG:Release>:-flto>
            $<$<CONFIG:MinSizeRel>:-flto>)
        if(APPLE)
            target_link_options("${_target}" PRIVATE
                $<$<CONFIG:Release>:-Wl,-dead_strip>
                $<$<CONFIG:MinSizeRel>:-Wl,-dead_strip>)
        else()
            target_link_options("${_target}" PRIVATE
                $<$<CONFIG:Release>:-Wl,--gc-sections>
                $<$<CONFIG:MinSizeRel>:-Wl,--gc-sections>)
        endif()
    endforeach()

    if(APPLE)
        message(STATUS "  └─ Stack size: macOS uses system default (adjustable via ulimit)")
    elseif(ARG_STACK_SIZE_OPTION STREQUAL "DEFAULT")
        message(STATUS "  └─ Stack size: Using system default")
    elseif(ARG_STACK_SIZE_OPTION STREQUAL "RECOMMENDED")
        message(STATUS "  └─ Stack size: 2MB (Recommended)")
    elseif(ARG_STACK_SIZE_OPTION STREQUAL "LARGE_PROJECT")
        message(STATUS "  └─ Stack size: 4MB (Large Project)")
    elseif(ARG_STACK_SIZE_OPTION STREQUAL "EXTREME_RECURSION")
        message(STATUS "  └─ Stack size: 8MB (Extreme Recursion)")
    endif()

    if(ARG_ENABLE_WARNINGS)
        message(STATUS "  └─ Warnings: Enhanced (-Wall -Wextra + additional)")
    else()
        message(STATUS "  └─ Warnings: Standard (-Wall)")
    endif()
endfunction()
