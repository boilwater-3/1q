# 模块级清单：按子目录维护源文件与安装规则
if(ENABLE_INSTALL)
    include(GNUInstallDirs)
endif()
include(${CMAKE_CURRENT_SOURCE_DIR}/common/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/airborne_radar/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/electronic_surveillance_radar/CMakeLists.txt)
include(${CMAKE_CURRENT_SOURCE_DIR}/electro_optical_sensor/CMakeLists.txt)

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

# 创建库目标及其别名（名称由顶层变量驱动）
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

# 添加源文件到目标
# 设置包含目录
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

# 第三方依赖由包管理器模块处理
# 如需添加依赖，请编辑对应的包管理器配置文件：
#   - cmake/PackageManagerVcpkg.cmake
#   - cmake/PackageManagerConan.cmake
if(WIN32)
    set(PROJECT_ENABLE_SPDLOG OFF)
else()
    set(PROJECT_ENABLE_SPDLOG ON)
endif()

find_package(Eigen3 CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(nanoflann CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
if(PROJECT_ENABLE_SPDLOG)
    find_package(spdlog CONFIG REQUIRED)
endif()

set(ONEQ_LINK_DEPENDENCIES
    Eigen3::Eigen
    Boost::boost
    nanoflann::nanoflann
    nlohmann_json::nlohmann_json
)
foreach(ONEQ_BUILD_TARGET IN ITEMS
    ${PROJECT_CORE_TARGET}
    ${ONEQ_OBJECT_TARGETS}
)
    target_link_libraries(${ONEQ_BUILD_TARGET} PRIVATE ${ONEQ_LINK_DEPENDENCIES})
endforeach()
unset(ONEQ_BUILD_TARGET)

# Conan's VS multi-config generation may only attach header-only include dirs
# to Release. Mirror those include dirs onto this target when available so
# Debug/RelWithDebInfo/MinSizeRel can compile against the same headers.
foreach(_oneq_conan_include_var IN ITEMS
    eigen_INCLUDE_DIRS_RELEASE
    boost_INCLUDE_DIRS_RELEASE
    nanoflann_INCLUDE_DIRS_RELEASE
    nlohmann_json_INCLUDE_DIRS_RELEASE
)
    if(DEFINED ${_oneq_conan_include_var})
        foreach(ONEQ_BUILD_TARGET IN ITEMS
            ${PROJECT_CORE_TARGET}
            ${ONEQ_OBJECT_TARGETS}
        )
            target_include_directories(${ONEQ_BUILD_TARGET} PRIVATE ${${_oneq_conan_include_var}})
        endforeach()
    endif()
endforeach()
unset(_oneq_conan_include_var)
unset(ONEQ_BUILD_TARGET)

if(PROJECT_ENABLE_SPDLOG)
    if(TARGET spdlog::spdlog)
        set(PROJECT_SPDLOG_TARGET spdlog::spdlog)
    elseif(TARGET spdlog::spdlog_header_only)
        message(FATAL_ERROR "spdlog header-only target is not allowed in this configuration")
    else()
        message(FATAL_ERROR "spdlog target is not available")
    endif()
    foreach(ONEQ_BUILD_TARGET IN ITEMS
        ${PROJECT_CORE_TARGET}
        ${ONEQ_OBJECT_TARGETS}
    )
        target_link_libraries(${ONEQ_BUILD_TARGET} PRIVATE ${PROJECT_SPDLOG_TARGET})
    endforeach()
    unset(ONEQ_BUILD_TARGET)
endif()

foreach(ONEQ_BUILD_TARGET IN ITEMS
    ${PROJECT_CORE_TARGET}
    ${ONEQ_OBJECT_TARGETS}
)
    target_compile_definitions(${ONEQ_BUILD_TARGET}
        PRIVATE PROJECT_LOG_BACKEND_SPDLOG=$<BOOL:${PROJECT_ENABLE_SPDLOG}>
    )
endforeach()
unset(ONEQ_BUILD_TARGET)

set(ONEQ_MSVC_RUNTIME_LIBRARY "" CACHE STRING
    "Optional MSVC runtime override (example: MultiThreadedDLL). Empty keeps toolchain/profile default.")
if(MSVC AND NOT ONEQ_MSVC_RUNTIME_LIBRARY STREQUAL "")
    foreach(ONEQ_BUILD_TARGET IN ITEMS
        ${PROJECT_CORE_TARGET}
        ${ONEQ_OBJECT_TARGETS}
    )
        set_property(TARGET ${ONEQ_BUILD_TARGET} PROPERTY
            MSVC_RUNTIME_LIBRARY "${ONEQ_MSVC_RUNTIME_LIBRARY}")
    endforeach()
    unset(ONEQ_BUILD_TARGET)
endif()

if(NOT BUILD_SHARED_LIBS)
    target_compile_definitions(${PROJECT_CORE_TARGET}
        PUBLIC ONEQ_STATIC_DEFINE
    )
    foreach(ONEQ_OBJECT_TARGET IN LISTS ONEQ_OBJECT_TARGETS)
        target_compile_definitions(${ONEQ_OBJECT_TARGET}
            PRIVATE ONEQ_STATIC_DEFINE
        )
    endforeach()
    unset(ONEQ_OBJECT_TARGET)
endif()

# 设置目标属性
set_target_properties(${PROJECT_CORE_TARGET} PROPERTIES
    EXPORT_NAME ${PROJECT_NAME_LOWER}
    OUTPUT_NAME ${PROJECT_NAME_LOWER}
    DEBUG_POSTFIX d
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# 非 Windows 平台默认隐藏符号，仅显式导出公共 API
if(NOT WIN32)
    set_target_properties(${PROJECT_CORE_TARGET} PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN YES
    )
endif()

# PCH设置（预编译头）
if(ENABLE_PCH)
    target_precompile_headers(${PROJECT_CORE_TARGET} PRIVATE
        # 容器
        <vector>
        <string>
        <array>
        <deque>
        <list>
        <map>
        <set>
        <unordered_map>
        <unordered_set>
        
        # 智能指针和内存管理
        <memory>
        
        # 算法和数值
        <algorithm>
        <numeric>
        
        # 实用工具
        <utility>
        <functional>
        <tuple>
        
        # IO
        <sstream>

        # 其他常用
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


# 可选安装规则
if(ENABLE_INSTALL)
    include(CMakePackageConfigHelpers)

    set(PUBLIC_HEADERS_ROOT
        ${CMAKE_SOURCE_DIR}/include/1q/api.hpp
        ${CMAKE_SOURCE_DIR}/include/1q/README.md
    )

    # 安装库目标
    install(TARGETS ${PROJECT_CORE_TARGET}
        EXPORT ${PROJECT_EXPORT_SET}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    # 安装公共头文件（显式白名单，避免内部头被自动导出）
    install(FILES ${PUBLIC_HEADERS_ROOT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q
    )
    install(FILES "${PROJECT_GENERATED_EXPORT_HEADER}"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q
    )

    # 安装导出的 CMake targets 文件
    install(EXPORT ${PROJECT_EXPORT_SET}
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
        FILE ${PROJECT_NAME}Targets.cmake
    )

    # 生成 PackageConfig 文件（供 find_package 使用）
    configure_package_config_file(
        "${CMAKE_SOURCE_DIR}/cmake/ProjectTemplateConfig.cmake.in"
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
        PATH_VARS CMAKE_INSTALL_INCLUDEDIR
    )

    # 生成版本文件
    write_basic_package_version_file(
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )

    # 安装 Config 文件
    install(FILES
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    )
endif()
