# MSVC compiler option helpers.
# This file defines functions only; project targets opt in explicitly.

function(oneq_apply_msvc_options)
    set(one_value_args ENABLE_WARNINGS STACK_SIZE_OPTION)
    set(multi_value_args TARGETS LINK_TARGETS)
    cmake_parse_arguments(ARG "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    if(NOT MSVC)
        message(FATAL_ERROR "oneq_apply_msvc_options() requires an MSVC compiler")
    endif()
    if(NOT ARG_TARGETS)
        message(FATAL_ERROR "oneq_apply_msvc_options() requires TARGETS")
    endif()
    if(NOT DEFINED ARG_STACK_SIZE_OPTION OR ARG_STACK_SIZE_OPTION STREQUAL "")
        set(ARG_STACK_SIZE_OPTION "DEFAULT")
    endif()
    if(NOT ARG_LINK_TARGETS)
        set(ARG_LINK_TARGETS ${ARG_TARGETS})
    endif()

    message(STATUS "Configuring for MSVC compiler")
    message(STATUS "  └─ Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

    set(_msvc_common_compile_options /MP /Zc:inline)
    if(MSVC_VERSION GREATER_EQUAL 1910)
        list(APPEND _msvc_common_compile_options
            /utf-8
            /permissive-
            /Zc:referenceBinding)
    else()
        message(STATUS "  Legacy MSVC mode: skipping /utf-8 /permissive- /Zc:referenceBinding for VS2015 compatibility")
    endif()

    foreach(_target IN LISTS ARG_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "Compiler target does not exist: ${_target}")
        endif()
        target_compile_options("${_target}" PRIVATE ${_msvc_common_compile_options})
        if(ARG_ENABLE_WARNINGS)
            target_compile_options("${_target}" PRIVATE
                /W4
                /WX-
                /w14242
                /w14254
                /w14263
                /w14265
                /w14287
                /we4289
                /w14296
                /w14311
                /w14545
                /w14546
                /w14547
                /w14549
                /w14555
                /w14619
                /w14640
                /w14826
                /w14905
                /w14906
                /w14928)
        else()
            target_compile_options("${_target}" PRIVATE /W3)
        endif()

        target_compile_options("${_target}" PRIVATE
            $<$<CONFIG:Debug>:/Z7>
            $<$<CONFIG:Debug>:/Od>
            $<$<CONFIG:Debug>:/RTC1>
            $<$<AND:$<CONFIG:Debug>,$<VERSION_GREATER_EQUAL:${MSVC_VERSION},1910>>:/JMC>
            $<$<CONFIG:Release>:/Od>
            $<$<CONFIG:Release>:/Ot>
            $<$<CONFIG:Release>:/Ob2>
            $<$<CONFIG:Release>:/Oi>
            $<$<CONFIG:Release>:/Gy>
            $<$<CONFIG:Release>:/GS->
            $<$<CONFIG:Release>:/Z7>
            $<$<CONFIG:RelWithDebInfo>:/O2>
            $<$<CONFIG:RelWithDebInfo>:/Ob2>
            $<$<CONFIG:RelWithDebInfo>:/Oi>
            $<$<CONFIG:RelWithDebInfo>:/Z7>
            $<$<CONFIG:MinSizeRel>:/O1>
            $<$<CONFIG:MinSizeRel>:/Os>)
    endforeach()

    foreach(_target IN LISTS ARG_LINK_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "Compiler link target does not exist: ${_target}")
        endif()
        if(ARG_STACK_SIZE_OPTION STREQUAL "RECOMMENDED")
            target_link_options("${_target}" PRIVATE /STACK:2097152)
        elseif(ARG_STACK_SIZE_OPTION STREQUAL "LARGE_PROJECT")
            target_link_options("${_target}" PRIVATE /STACK:4194304)
        elseif(ARG_STACK_SIZE_OPTION STREQUAL "EXTREME_RECURSION")
            target_link_options("${_target}" PRIVATE /STACK:8388608)
        endif()

        target_link_options("${_target}" PRIVATE
            $<$<CONFIG:Debug>:/DEBUG:FULL>
            $<$<CONFIG:Debug>:/INCREMENTAL>
            $<$<CONFIG:Release>:/DEBUG:FULL>
            $<$<CONFIG:Release>:/INCREMENTAL:NO>
            $<$<CONFIG:RelWithDebInfo>:/DEBUG:FULL>
            $<$<CONFIG:RelWithDebInfo>:/OPT:REF>
            $<$<CONFIG:RelWithDebInfo>:/OPT:ICF>
            $<$<CONFIG:RelWithDebInfo>:/INCREMENTAL:NO>
            $<$<CONFIG:MinSizeRel>:/OPT:REF>
            $<$<CONFIG:MinSizeRel>:/OPT:ICF>)
    endforeach()

    if(ARG_STACK_SIZE_OPTION STREQUAL "DEFAULT")
        message(STATUS "  └─ Stack size: Using system default (1MB)")
    elseif(ARG_STACK_SIZE_OPTION STREQUAL "RECOMMENDED")
        message(STATUS "  └─ Stack size: 2MB (Recommended)")
    elseif(ARG_STACK_SIZE_OPTION STREQUAL "LARGE_PROJECT")
        message(STATUS "  └─ Stack size: 4MB (Large Project)")
    elseif(ARG_STACK_SIZE_OPTION STREQUAL "EXTREME_RECURSION")
        message(STATUS "  └─ Stack size: 8MB (Extreme Recursion)")
    endif()

    if(ARG_ENABLE_WARNINGS)
        message(STATUS "  └─ Warnings: Enhanced (/W4 + additional checks)")
    else()
        message(STATUS "  └─ Warnings: Standard (/W3)")
    endif()
endfunction()
