# 显式列出源文件（推荐做法，便于版本控制和构建可靠性）
# 按模块分组管理，每次添加/删除源文件时需要手动更新对应模块列表
set(PROJECT_SOURCES
    airborne_radar/decision/pipeline/TacticalCoordinator.cpp
    airborne_radar/decision/pipeline/ControlReducer.cpp
    airborne_radar/decision/classifier/ThreatAssessmentEvaluator.cpp
    airborne_radar/decision/lpi/EmissionControlEvaluator.cpp
    airborne_radar/decision/eccm/SurvivabilityEvaluator.cpp
    airborne_radar/core/controller/RadarController.cpp
    airborne_radar/core/output/DataOutputManager.cpp
    airborne_radar/core/event/EventBus.cpp
    airborne_radar/core/event/CycleEventBus.cpp
    airborne_radar/environment/EnvironmentService.cpp
    airborne_radar/environment/scene/SceneManager.cpp
    airborne_radar/environment/simulation/PropagationModel.cpp
    airborne_radar/environment/database/FeatureRepository.cpp
    airborne_radar/signal/association/DataAssociation.cpp
    airborne_radar/signal/association/DistanceMetric.cpp
    airborne_radar/signal/association/Hypothesiser.cpp
    airborne_radar/signal/association/LapjvSolver.cpp
    airborne_radar/signal/detection/RadarEquations.cpp
    airborne_radar/signal/detection/SignalDetector.cpp
    airborne_radar/signal/pipeline/SignalPipeline.cpp
    airborne_radar/signal/tracking/BoostTrackPool.cpp
    airborne_radar/signal/tracking/SynchronizedTrackPool.cpp
    airborne_radar/signal/tracking/KalmanPredictor.cpp
    airborne_radar/signal/tracking/KalmanUpdater.cpp
    airborne_radar/signal/tracking/EkfFilter.cpp
    airborne_radar/signal/tracking/ImmFilter.cpp
    airborne_radar/signal/tracking/TrackFilter.cpp
    airborne_radar/signal/tracking/TrackLifecycleManager.cpp
)

# 创建库目标及其别名（名称由顶层变量驱动）
add_library(${PROJECT_CORE_TARGET} ${PROJECT_SOURCES})
add_library(${PROJECT_ALIAS} ALIAS ${PROJECT_CORE_TARGET})

# 添加源文件到目标
target_sources(${PROJECT_CORE_TARGET} PRIVATE ${PROJECT_SOURCES})
# 设置包含目录
target_include_directories(${PROJECT_CORE_TARGET}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 第三方依赖由包管理器模块处理
# 如需添加依赖，请编辑对应的包管理器配置文件：
#   - cmake/PackageManagerVcpkg.cmake
#   - cmake/PackageManagerConan.cmake
find_package(Eigen3 CONFIG REQUIRED)
find_package(Sophus CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(eventpp CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)
target_link_libraries(${PROJECT_CORE_TARGET} PRIVATE spdlog::spdlog Eigen3::Eigen Sophus::Sophus eventpp::eventpp Boost::boost)

# 设置与 api.hpp 符号匹配的编译定义
target_compile_definitions(${PROJECT_CORE_TARGET}
    PRIVATE $<$<BOOL:${BUILD_SHARED_LIBS}>:TEMPLATE_LIBRARY_BUILD>
    PUBLIC  $<$<BOOL:${BUILD_SHARED_LIBS}>:TEMPLATE_LIBRARY_SHARED>
    PUBLIC  $<$<NOT:$<BOOL:${BUILD_SHARED_LIBS}>>:TEMPLATE_STATIC>
)

# 设置目标属性
set_target_properties(${PROJECT_CORE_TARGET} PROPERTIES
    EXPORT_NAME ${PROJECT_NAME_LOWER}
    OUTPUT_NAME ${PROJECT_NAME_LOWER}
    DEBUG_POSTFIX d
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

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
        <optional>
        <variant>
        
        # IO
        <iostream>
        <fstream>
        <sstream>
        
        # 其他常用
        <cmath>
        <cstdint>
        <cstring>
        <chrono>
        <exception>
        <stdexcept>
    )
endif()


# 可选安装规则
if(ENABLE_INSTALL)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)

    # 安装库目标
    install(TARGETS ${PROJECT_CORE_TARGET}
        EXPORT ${PROJECT_EXPORT_SET}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    # 安装公共头文件（仅包含 .h 和 .hpp）
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING 
        PATTERN "*.h"
        PATTERN "*.hpp"
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
