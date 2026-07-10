# PrecompiledHeaders.cmake
# 定义项目自有 target 的预编译头（PCH）策略函数。
# 与 UnityBuild、Coverage 同属通用编译特性层，不绑定特定项目。

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

    foreach(_target IN LISTS ARG_TARGETS)
        if(NOT TARGET "${_target}")
            message(FATAL_ERROR "PCH target does not exist: ${_target}")
        endif()
        target_precompile_headers("${_target}" PRIVATE ${_oneq_pch_headers})
    endforeach()

    message(STATUS "Precompiled Headers: Enabled")
endfunction()
