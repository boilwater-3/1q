# PrecompiledHeaders.cmake
# 定义项目自有 target 的预编译头（PCH）策略函数。
# 与 UnityBuild、Coverage 同属通用编译特性层，不绑定特定项目。
#
# 首个获得 PCH 的 target 作为 anchor（通常为 oneq_common）；其余 target 通过
# REUSE_FROM 共享同一份 cmake_pch，避免每个组件/测试 exe 各编一遍 Eigen PCH。

function(apply_precompiled_headers)
    set(multi_value_args TARGETS)
    cmake_parse_arguments(ARG "" "" "${multi_value_args}" ${ARGN})

    if(NOT ENABLE_PCH)
        message(STATUS "Precompiled Headers: Disabled")
        return()
    endif()
    if(NOT ARG_TARGETS)
        message(FATAL_ERROR "apply_precompiled_headers() requires TARGETS")
    endif()

    set(_oneq_pch_headers
        <vector>
        <string>
        <array>
        <deque>
        <list>
        <map>
        <set>
        <unordered_map>
        <unordered_set>
        <memory>
        <algorithm>
        <numeric>
        <utility>
        <functional>
        <tuple>
        <sstream>
        <cmath>
        <cstdint>
        <cstring>
        <chrono>)
    if(CMAKE_CXX_STANDARD GREATER_EQUAL 17)
        list(APPEND _oneq_pch_headers
            <optional>
            <variant>)
    endif()
    # Eigen：Kalman/LLT 主路径高频头；PCH 摊销重复解析（Windows preset 默认 ENABLE_PCH=ON）。
    if(TARGET Eigen3::Eigen)
        list(APPEND _oneq_pch_headers
            <Eigen/Core>
            <Eigen/Cholesky>)
    endif()

    get_property(_oneq_pch_anchor GLOBAL PROPERTY ONEQ_PCH_ANCHOR_TARGET)

    foreach(_target IN LISTS ARG_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "PCH target does not exist: ${_target}")
        endif()
        if(_oneq_pch_anchor AND NOT _target STREQUAL _oneq_pch_anchor)
            target_precompile_headers("${_target}" REUSE_FROM "${_oneq_pch_anchor}")
        else()
            target_precompile_headers("${_target}" PRIVATE ${_oneq_pch_headers})
            if(NOT _oneq_pch_anchor)
                set_property(GLOBAL PROPERTY ONEQ_PCH_ANCHOR_TARGET "${_target}")
                set(_oneq_pch_anchor "${_target}")
            endif()
        endif()
    endforeach()

    if(_oneq_pch_anchor)
        message(STATUS "Precompiled Headers: Enabled (anchor=${_oneq_pch_anchor}, REUSE_FROM elsewhere)")
    else()
        message(STATUS "Precompiled Headers: Enabled")
    endif()
endfunction()
