#[[
@file codegen/FlatBuffers.cmake
@brief FlatBuffers 构建期代码生成配置：定义 flatc 发现与 schema 代码生成函数。

将 schemas/replay/*.fbs 在构建期编译为 *_generated.h，取代手工提交的 checked-in
生成头。flatc 编译器来自 conan flatbuffers 包（FlatcTargets.cmake 暴露的
flatbuffers::flatc imported executable 以及 FLATBUFFERS_FLATC_EXECUTABLE 变量），
版本与运行时头一致（当前 1.12.0）。

@note 本文件只定义函数；实际的 flatc 发现与 schema 编译由
      本文件的模块注册 API 在 find_package(flatbuffers) 与目标创建之后由
      对应 `src/<module>/CMakeLists.txt` 调用。
]]

# 生成头集中输出目录。按 <sensor>/session/generated/ 子目录布局，使
# `#include "<sensor>/session/generated/<name>_generated.h"`（Style-2）能通过
# 把本目录加入 include path 直接解析。
set(ONEQ_FLATBUFFERS_GENERATED_DIR "${CMAKE_BINARY_DIR}/generated" CACHE INTERNAL
    "FlatBuffers 生成头根目录（构建期由 flatc 产生，不进 git）")

#[[
@brief 发现并校验 flatc 编译器，结果写入 ONEQ_FLATC_EXECUTABLE
@note 必须在 find_package(flatbuffers) 之后调用。
      优先级：1) conan 暴露的 flatbuffers::flatc imported target；
              2) FLATBUFFERS_FLATC_EXECUTABLE 变量（同来源另一形态）；
              3) find_program(flatc) fallback（仅用于脱离 conan 的本地诊断）。
@note flatc 未找到时触发 FATAL_ERROR。
]]
macro(setup_flatc)
    set(ONEQ_FLATC_EXECUTABLE "")
    if(TARGET flatbuffers::flatc)
        get_target_property(_oneq_flatc_loc flatbuffers::flatc IMPORTED_LOCATION)
        if(_oneq_flatc_loc)
            set(ONEQ_FLATC_EXECUTABLE "${_oneq_flatc_loc}")
        endif()
    endif()
    if(NOT ONEQ_FLATC_EXECUTABLE AND DEFINED FLATBUFFERS_FLATC_EXECUTABLE)
        set(ONEQ_FLATC_EXECUTABLE "${FLATBUFFERS_FLATC_EXECUTABLE}")
    endif()
    if(NOT ONEQ_FLATC_EXECUTABLE)
        find_program(ONEQ_FLATC_EXECUTABLE NAMES flatc)
    endif()

    if(NOT ONEQ_FLATC_EXECUTABLE)
        message(FATAL_ERROR
            "FlatBuffers: flatc 未找到。conan flatbuffers 包应通过 FlatcTargets.cmake "
            "暴露 flatbuffers::flatc；若缺失请检查 conan install 是否成功。")
    endif()

    # Schema registration can occur inside module helper functions. Persist the
    # resolved executable so flatbuffers_generate(), which has its own scope,
    # always receives the same tool path.
    set(ONEQ_FLATC_EXECUTABLE "${ONEQ_FLATC_EXECUTABLE}" CACHE INTERNAL
        "Resolved FlatBuffers compiler executable")

    execute_process(
        COMMAND ${ONEQ_FLATC_EXECUTABLE} --version
        OUTPUT_VARIABLE _oneq_flatc_version
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    message(STATUS "FlatBuffers: 构建期代码生成已启用")
    message(STATUS "  └─ flatc: ${ONEQ_FLATC_EXECUTABLE}")
    if(_oneq_flatc_version)
        message(STATUS "  └─ ${_oneq_flatc_version}")
    endif()
    unset(_oneq_flatc_version)
    unset(_oneq_flatc_loc)
endmacro()

#[[
@brief 为单个 .fbs schema 创建 flatc 代码生成规则
@param[in] schema_file    待编译的 .fbs schema 路径
@param[in] sensor_subdir  src/ 下的 sensor 目录名（如 sar、electro_optical_sensor），
                          用于在输出目录复刻 <sensor>/session/generated/ 层级，
                          保证 Style-2 include 可解析
@param[out] out_var       返回生成头的绝对路径，调用方将其加入 target 的 SOURCES
                          以建立编译依赖
@note 生成参数与原 checked-in 头一致：--cpp --gen-object-api（Object API 内容
      虽未被手写 codec 消费，但保留以维持与历史生成头字节级一致，避免无谓 diff）。
]]
function(flatbuffers_generate schema_file sensor_subdir out_var)
    get_filename_component(_schema_name "${schema_file}" NAME_WE)
    get_filename_component(_schema_abs "${schema_file}" ABSOLUTE)

    set(_out_subdir "${ONEQ_FLATBUFFERS_GENERATED_DIR}/${sensor_subdir}/session/generated")
    set(_out_header "${_out_subdir}/${_schema_name}_generated.h")

    # add_custom_command 要求 OUTPUT 的目录在命令执行前存在（部分生成器）。
    add_custom_command(
        OUTPUT "${_out_header}"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${_out_subdir}"
        COMMAND ${ONEQ_FLATC_EXECUTABLE}
                --cpp --gen-object-api
                -o "${_out_subdir}"
                "${_schema_abs}"
        DEPENDS "${_schema_abs}"
        COMMENT "FlatBuffers: generating ${_schema_name}_generated.h from ${schema_file}"
        VERBATIM
    )

    set(${out_var} "${_out_header}" PARENT_SCOPE)
endfunction()

# Module-owned schema registration. There is deliberately no central
# schema-to-target manifest: each business module declares the schemas used by
# its codec owner target beside that target's sources and dependencies.
function(oneq_add_flatbuffers_schemas)
    set(one_value_args TARGET MODULE)
    set(multi_value_args SCHEMAS)
    cmake_parse_arguments(ARG "" "${one_value_args}" "${multi_value_args}" ${ARGN})
    if(NOT ARG_TARGET OR NOT ARG_MODULE OR NOT ARG_SCHEMAS)
        message(FATAL_ERROR
            "oneq_add_flatbuffers_schemas() requires TARGET, MODULE and SCHEMAS")
    endif()
    if(NOT TARGET "${ARG_TARGET}")
        message(FATAL_ERROR "FlatBuffers owner target does not exist: ${ARG_TARGET}")
    endif()

    get_property(flatc_ready GLOBAL PROPERTY ONEQ_FLATC_READY)
    if(NOT flatc_ready)
        setup_flatc()
        set_property(GLOBAL PROPERTY ONEQ_FLATC_READY TRUE)
    endif()

    foreach(schema_name IN LISTS ARG_SCHEMAS)
        set(schema_path "${CMAKE_SOURCE_DIR}/schemas/replay/${schema_name}")
        if(NOT EXISTS "${schema_path}")
            message(FATAL_ERROR "FlatBuffers schema does not exist: ${schema_path}")
        endif()
        flatbuffers_generate("${schema_path}" "${ARG_MODULE}" generated_header)
        target_sources("${ARG_TARGET}" PRIVATE "${generated_header}")
        set_property(GLOBAL APPEND PROPERTY ONEQ_FLATBUFFERS_GENERATED_HEADERS
            "${generated_header}")
    endforeach()
    target_include_directories("${ARG_TARGET}" PRIVATE
        "${ONEQ_FLATBUFFERS_GENERATED_DIR}")
endfunction()

function(oneq_finalize_flatbuffers)
    get_property(generated_headers GLOBAL PROPERTY ONEQ_FLATBUFFERS_GENERATED_HEADERS)
    if(NOT generated_headers)
        return()
    endif()
    add_custom_target(oneq_flatbuffers_headers DEPENDS ${generated_headers})
    set(ONEQ_FLATBUFFERS_HEADERS_TARGET oneq_flatbuffers_headers PARENT_SCOPE)
endfunction()
