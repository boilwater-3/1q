# FlatBuffersSetup.cmake
# FlatBuffers 构建期代码生成编排：flatc 发现 → schema 编译 → 目标注入。
#
# 必须在 find_package(flatbuffers) 之后、目标创建之后调用。
# 函数定义来自 cmake/features/FeatureFlatBuffers.cmake，schema 清单来自
# cmake/project/ReplaySchemas.cmake。
#
# 产物：
#   ONEQ_FLATBUFFERS_GENERATED_DIR     — 生成头根目录（CACHE INTERNAL）
#   ONEQ_FLATBUFFERS_GENERATED_HEADERS — 所有生成头的绝对路径列表
#   ONEQ_FLATBUFFERS_HEADERS_TARGET    — 聚合 custom target（CACHE INTERNAL）

# flatc 编译器发现与校验
setup_flatc()

# schema → sensor 子目录:归属 object target 映射。每个 sensor 一对：
# <sensor>_replay.fbs + <sensor>_session_replay.fbs，输出到
# ${ONEQ_FLATBUFFERS_GENERATED_DIR}/<sensor>/session/generated/，以支持
# Style-2 include（#include "<sensor>/session/generated/<name>_generated.h"）。
# 条目格式："schema.fbs:sensor_subdir:object_target"
include("${CMAKE_SOURCE_DIR}/cmake/project/ReplaySchemas.cmake")

set(ONEQ_FLATBUFFERS_GENERATED_HEADERS)
foreach(_fb_entry IN LISTS ONEQ_FLATBUFFERS_SCHEMAS)
    string(REPLACE ":" ";" _fb_parts "${_fb_entry}")
    list(GET _fb_parts 0 _fb_schema_name)
    list(GET _fb_parts 1 _fb_sensor_subdir)
    list(GET _fb_parts 2 _fb_target)
    set(_fb_schema "${CMAKE_SOURCE_DIR}/schemas/replay/${_fb_schema_name}")
    if(EXISTS "${_fb_schema}")
        flatbuffers_generate("${_fb_schema}" "${_fb_sensor_subdir}" _fb_header)
        list(APPEND ONEQ_FLATBUFFERS_GENERATED_HEADERS "${_fb_header}")
        # 把生成头标记为目标源，建立 schema → header → 编译 的依赖链。
        if(TARGET "${_fb_target}")
            target_sources("${_fb_target}" PRIVATE "${_fb_header}")
        endif()
    endif()
endforeach()
unset(_fb_entry)
unset(_fb_parts)
unset(_fb_schema_name)
unset(_fb_sensor_subdir)
unset(_fb_target)
unset(_fb_schema)
unset(_fb_header)
unset(ONEQ_FLATBUFFERS_SCHEMAS)

# 生成头所在的根目录加入所有核心/对象目标的 include 路径（PRIVATE），
# 与 src/ 并列：src/ 解析 checked-in 风格的源内引用，generated 根目录
# 解析构建期产生的 <sensor>/session/generated/<name>_generated.h。
# 同时定义聚合 custom target，供直接编译生成头的测试目标（不经 object
# 库链接的 .cpp）建立对生成头的依赖，避免竞态。
if(ONEQ_FLATBUFFERS_GENERATED_HEADERS)
    target_include_directories(${PROJECT_CORE_TARGET} PRIVATE
        ${ONEQ_FLATBUFFERS_GENERATED_DIR})
    foreach(ONEQ_OBJECT_TARGET IN LISTS ONEQ_OBJECT_TARGETS)
        target_include_directories(${ONEQ_OBJECT_TARGET} PRIVATE
            ${ONEQ_FLATBUFFERS_GENERATED_DIR})
    endforeach()
    unset(ONEQ_OBJECT_TARGET)

    add_custom_target(oneq_flatbuffers_headers DEPENDS
        ${ONEQ_FLATBUFFERS_GENERATED_HEADERS})
    set(ONEQ_FLATBUFFERS_HEADERS_TARGET oneq_flatbuffers_headers CACHE INTERNAL
        "聚合所有 FlatBuffers 生成头的 custom target，供测试目标依赖")
endif()
