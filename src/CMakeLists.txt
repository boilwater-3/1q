# 模块级清单：按子目录维护源文件与安装规则
if(ENABLE_INSTALL)
    include(GNUInstallDirs)
endif()
include(${CMAKE_CURRENT_SOURCE_DIR}/common/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/airborne_radar/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/electronic_surveillance_radar/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/electro_optical_sensor/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/sbirs_sensor/CMakeLists.txt)
# flight_dynamic (机动模块): 默认 OFF，仅在 ONEQ_ENABLE_FLIGHT_DYNAMIC 开启时参与编译
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    include(${CMAKE_CURRENT_SOURCE_DIR}/flight_dynamic/CMakeLists.txt)
endif()
include(${CMAKE_CURRENT_SOURCE_DIR}/sar/CMakeLists.txt)

# --- OBJECT 库 ---
add_library(airborne_engine OBJECT ${AIRBORNE_ENGINE_SOURCES})
add_library(airborne_core OBJECT ${AIRBORNE_CORE_SOURCES})
add_library(esr_engine OBJECT ${ESR_ENGINE_SOURCES})
add_library(esr_core OBJECT ${ESR_CORE_SOURCES})
add_library(eos_engine OBJECT ${EOS_ENGINE_SOURCES})
add_library(eos_core OBJECT ${EOS_CORE_SOURCES})
add_library(sbirs_engine OBJECT ${SBIRS_ENGINE_SOURCES})
add_library(sbirs_core OBJECT ${SBIRS_CORE_SOURCES})
if(ONEQ_ENABLE_FLIGHT_DYNAMIC)
    add_library(fd_engine OBJECT ${FD_ENGINE_SOURCES})
endif()
add_library(sar_core OBJECT ${SAR_CORE_SOURCES})
if(SAR_ENGINE_SOURCES)
  add_library(sar_engine OBJECT ${SAR_ENGINE_SOURCES})
endif()
if(FD_CORE_SOURCES)
  add_library(fd_core OBJECT ${FD_CORE_SOURCES})
endif()

set(ONEQ_OBJECT_TARGETS
    airborne_engine
    airborne_core
    esr_engine
    esr_core
    eos_engine
    eos_core
    sbirs_engine
    sbirs_core
    sar_core
)

if(TARGET sar_engine)
  list(APPEND ONEQ_OBJECT_TARGETS sar_engine)
endif()

if(TARGET fd_engine)
  list(APPEND ONEQ_OBJECT_TARGETS fd_engine)
endif()

if(TARGET fd_core)
  list(APPEND ONEQ_OBJECT_TARGETS fd_core)
endif()

set(PROJECT_SOURCES
    ${ONEQ_COMMON_SOURCES}
    $<TARGET_OBJECTS:airborne_engine>
    $<TARGET_OBJECTS:airborne_core>
    $<TARGET_OBJECTS:esr_engine>
    $<TARGET_OBJECTS:esr_core>
    $<TARGET_OBJECTS:eos_engine>
    $<TARGET_OBJECTS:eos_core>
    $<TARGET_OBJECTS:sbirs_engine>
    $<TARGET_OBJECTS:sbirs_core>
    $<TARGET_OBJECTS:sar_core>
)

if(TARGET sar_engine)
  list(APPEND PROJECT_SOURCES $<TARGET_OBJECTS:sar_engine>)
endif()

if(TARGET fd_engine)
  list(APPEND PROJECT_SOURCES $<TARGET_OBJECTS:fd_engine>)
endif()

if(TARGET fd_core)
  list(APPEND PROJECT_SOURCES $<TARGET_OBJECTS:fd_core>)
endif()
# 追加 vendor OBJECT 目标（flatbuffers_objects, zlib_objects 等）
if(ONEQ_VENDOR_OBJECT_TARGETS)
    foreach(_vendor_obj IN LISTS ONEQ_VENDOR_OBJECT_TARGETS)
        list(APPEND PROJECT_SOURCES $<TARGET_OBJECTS:${_vendor_obj}>)
    endforeach()
    unset(_vendor_obj)
endif()

# --- 主库目标 ---
add_library(${PROJECT_CORE_TARGET} ${PROJECT_SOURCES})
add_library(${PROJECT_ALIAS} ALIAS ${PROJECT_CORE_TARGET})

include(GenerateExportHeader)
set(PROJECT_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include")
set(PROJECT_GENERATED_EXPORT_HEADER
    "${PROJECT_GENERATED_INCLUDE_DIR}/1q/oneq_export.h")
file(MAKE_DIRECTORY "${PROJECT_GENERATED_INCLUDE_DIR}/1q")
generate_export_header(${PROJECT_CORE_TARGET}
    BASE_NAME ONEQ
    EXPORT_FILE_NAME "${PROJECT_GENERATED_EXPORT_HEADER}"
    STATIC_DEFINE ONEQ_STATIC_DEFINE
)

target_include_directories(${PROJECT_CORE_TARGET}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_GENERATED_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
foreach(ONEQ_OBJECT_TARGET IN LISTS ONEQ_OBJECT_TARGETS)
    target_include_directories(${ONEQ_OBJECT_TARGET}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${PROJECT_GENERATED_INCLUDE_DIR}
            ${CMAKE_SOURCE_DIR}/include
    )
endforeach()
unset(ONEQ_OBJECT_TARGET)

# vendor OBJECT 目标需要相同的 include 路径
if(ONEQ_VENDOR_OBJECT_TARGETS)
    foreach(ONEQ_OBJECT_TARGET IN LISTS ONEQ_VENDOR_OBJECT_TARGETS)
        target_include_directories(${ONEQ_OBJECT_TARGET}
            PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}
                ${PROJECT_GENERATED_INCLUDE_DIR}
                ${CMAKE_SOURCE_DIR}/include
        )
    endforeach()
    unset(ONEQ_OBJECT_TARGET)
endif()

# --- 第三方依赖 ---
include(${CMAKE_SOURCE_DIR}/cmake/ProjectDependencies.cmake)

# --- FlatBuffers 构建期代码生成 ---
# 必须在 ProjectDependencies.cmake（find_package(flatbuffers)）之后：flatc
# 通过 flatbuffers::flatc imported target 暴露。
oneq_setup_flatc()

# schema → sensor 子目录:归属 object target 映射。每个 sensor 一对：
# <sensor>_replay.fbs + <sensor>_session_replay.fbs，输出到
# ${ONEQ_FLATBUFFERS_GENERATED_DIR}/<sensor>/session/generated/，以支持
# Style-2 include（#include "<sensor>/session/generated/<name>_generated.h"）。
# 条目格式："schema.fbs:sensor_subdir:object_target"
set(ONEQ_FLATBUFFERS_SCHEMAS
    "sar_replay.fbs:sar:sar_core"
    "sar_session_replay.fbs:sar:sar_core"
    "eos_replay.fbs:electro_optical_sensor:eos_core"
    "eos_session_replay.fbs:electro_optical_sensor:eos_core"
    "esr_replay.fbs:electronic_surveillance_radar:esr_core"
    "esr_session_replay.fbs:electronic_surveillance_radar:esr_core"
    "airborne_radar_replay.fbs:airborne_radar:airborne_core"
    "airborne_radar_session_replay.fbs:airborne_radar:airborne_core"
    "sbirs_replay.fbs:sbirs_sensor:sbirs_core"
    "sbirs_session_replay.fbs:sbirs_sensor:sbirs_core"
)

set(ONEQ_FLATBUFFERS_GENERATED_HEADERS)
foreach(_fb_entry IN LISTS ONEQ_FLATBUFFERS_SCHEMAS)
    string(REPLACE ":" ";" _fb_parts "${_fb_entry}")
    list(GET _fb_parts 0 _fb_schema_name)
    list(GET _fb_parts 1 _fb_sensor_subdir)
    list(GET _fb_parts 2 _fb_target)
    set(_fb_schema "${CMAKE_SOURCE_DIR}/schemas/replay/${_fb_schema_name}")
    if(EXISTS "${_fb_schema}")
        oneq_flatbuffers_generate("${_fb_schema}" "${_fb_sensor_subdir}" _fb_header)
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

# --- 目标属性 ---
include(${CMAKE_SOURCE_DIR}/cmake/TargetProperties.cmake)

# --- PCH ---
if(ENABLE_PCH)
    set(ONEQ_PCH_HEADERS
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
        <chrono>
    )
    if(CMAKE_CXX_STANDARD GREATER_EQUAL 17)
        list(APPEND ONEQ_PCH_HEADERS
            <optional>
            <variant>
        )
    endif()
    target_precompile_headers(${PROJECT_CORE_TARGET} PRIVATE ${ONEQ_PCH_HEADERS})
    foreach(ONEQ_OBJECT_TARGET IN LISTS ONEQ_OBJECT_TARGETS)
        target_precompile_headers(${ONEQ_OBJECT_TARGET} PRIVATE ${ONEQ_PCH_HEADERS})
    endforeach()
    unset(ONEQ_OBJECT_TARGET)
    unset(ONEQ_PCH_HEADERS)
endif()

# --- 安装 ---
include(${CMAKE_SOURCE_DIR}/cmake/ProjectInstall.cmake)
