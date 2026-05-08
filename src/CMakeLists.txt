# 模块级清单：按子目录维护源文件与安装规则
if(ENABLE_INSTALL)
    include(GNUInstallDirs)
endif()
include(${CMAKE_CURRENT_SOURCE_DIR}/common/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/airborne_radar/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/electronic_surveillance_radar/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/electro_optical_sensor/CMakeLists.txt)

# --- OBJECT 库 ---
add_library(airborne_common OBJECT ${AIRBORNE_COMMON_SOURCES})
add_library(airborne_signal OBJECT ${AIRBORNE_SIGNAL_SOURCES})
add_library(airborne_decision OBJECT ${AIRBORNE_DECISION_SOURCES})
add_library(airborne_core OBJECT ${AIRBORNE_CORE_SOURCES})
add_library(esr_model OBJECT ${ESR_MODEL_SOURCES})
add_library(esr_environment OBJECT ${ESR_ENVIRONMENT_SOURCES})
add_library(esr_pipeline OBJECT ${ESR_PIPELINE_SOURCES})
add_library(esr_runtime OBJECT ${ESR_RUNTIME_SOURCES})
add_library(esr_session OBJECT ${ESR_SESSION_SOURCES})

set(ONEQ_OBJECT_TARGETS
    airborne_common
    airborne_signal
    airborne_decision
    airborne_core
    esr_model
    esr_environment
    esr_pipeline
    esr_runtime
    esr_session
)

set(PROJECT_SOURCES
    ${ONEQ_COMMON_SOURCES}
    ${EOS_SOURCES}
    $<TARGET_OBJECTS:airborne_common>
    $<TARGET_OBJECTS:airborne_signal>
    $<TARGET_OBJECTS:airborne_decision>
    $<TARGET_OBJECTS:airborne_core>
    $<TARGET_OBJECTS:esr_model>
    $<TARGET_OBJECTS:esr_environment>
    $<TARGET_OBJECTS:esr_pipeline>
    $<TARGET_OBJECTS:esr_runtime>
    $<TARGET_OBJECTS:esr_session>
)

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

# --- 第三方依赖 ---
include(${CMAKE_SOURCE_DIR}/cmake/ProjectDependencies.cmake)

# --- 目标属性 ---
include(${CMAKE_SOURCE_DIR}/cmake/TargetProperties.cmake)

# --- PCH ---
if(ENABLE_PCH)
    target_precompile_headers(${PROJECT_CORE_TARGET} PRIVATE
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
        target_precompile_headers(${PROJECT_CORE_TARGET} PRIVATE
            <optional>
            <variant>
        )
    endif()
endif()

# --- 安装 ---
include(${CMAKE_SOURCE_DIR}/cmake/ProjectInstall.cmake)
