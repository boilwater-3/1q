# 显式列出源文件（推荐做法，便于版本控制和构建可靠性）
# 按模块分组管理，每次添加/删除源文件时需要手动更新对应模块列表
set(ONEQ_COMMON_SOURCES
    common/atmosphere/AtmospherePhysics.cpp
    common/geometry/CoordinateTransform.cpp
    common/geometry/GeometryTransform.cpp
    common/numerics/SpectralNumerics.cpp
    common/rcs/RcsPhysics.cpp
    common/trace/TraceSink.cpp
    common/timing/TimingRegimeModel.cpp
)

set(AIRBORNE_COMMON_SOURCES
    airborne_radar/common/model/DecisionTrackSnapshot.cpp
    airborne_radar/common/model/TargetFeature.cpp
    airborne_radar/common/output/TrackOutputQueries.cpp
    airborne_radar/common/utils/TargetFeatureUtils.cpp
)

set(AIRBORNE_SIGNAL_SOURCES
    airborne_radar/signal/association/DataAssociation.cpp
    airborne_radar/signal/association/DistanceMetric.cpp
    airborne_radar/signal/association/Hypothesiser.cpp
    airborne_radar/signal/association/LapjvSolver.cpp
    airborne_radar/signal/assembly/DataOutputManager.cpp
    airborne_radar/signal/detection/RadarEquations.cpp
    airborne_radar/signal/detection/SignalDetector.cpp
    airborne_radar/signal/pipeline/AssociationExecutionSupport.cpp
    airborne_radar/signal/pipeline/ContextBindingSupport.cpp
    airborne_radar/signal/pipeline/ControlProfileEffects.cpp
    airborne_radar/signal/pipeline/CycleContextSupport.cpp
    airborne_radar/signal/pipeline/CycleExecutor.cpp
    airborne_radar/signal/pipeline/DecisionFrameBuilders.cpp
    airborne_radar/signal/pipeline/DetectionExecution.cpp
    airborne_radar/signal/pipeline/JammingEffects.cpp
    airborne_radar/signal/pipeline/OutputAssemblySupport.cpp
    airborne_radar/signal/pipeline/SignalPipeline.cpp
    airborne_radar/signal/pipeline/TrackMeasurementProcessing.cpp
    airborne_radar/signal/runtime/RuntimeAssemblySupport.cpp
    airborne_radar/signal/runtime/ScanScheduleResolver.cpp
    airborne_radar/signal/runtime/SignalComponentFactory.cpp
    airborne_radar/signal/tracking/BoostTrackPool.cpp
    airborne_radar/signal/tracking/EkfFilter.cpp
    airborne_radar/signal/tracking/ImmFilter.cpp
    airborne_radar/signal/tracking/KalmanPredictor.cpp
    airborne_radar/signal/tracking/KalmanUpdater.cpp
    airborne_radar/signal/tracking/SrifPredictor.cpp
    airborne_radar/signal/tracking/SrifUpdater.cpp
    airborne_radar/signal/tracking/SynchronizedTrackPool.cpp
    airborne_radar/signal/tracking/TrackFilter.cpp
    airborne_radar/signal/tracking/TrackLifecycleManager.cpp
    airborne_radar/signal/tracking/TrackLifecycleSnapshots.cpp
    airborne_radar/signal/tracking/TrackSnapshotEmitter.cpp
    airborne_radar/signal/tracking/UdkfPredictor.cpp
    airborne_radar/signal/tracking/UdkfUpdater.cpp
)

set(AIRBORNE_DECISION_SOURCES
    airborne_radar/decision/evaluators/EmissionControlEvaluator.cpp
    airborne_radar/decision/evaluators/SurvivabilityEvaluator.cpp
    airborne_radar/decision/evaluators/SurvivabilityEvaluatorHelpers.cpp
    airborne_radar/decision/evaluators/ThreatAssessmentEvaluator.cpp
    airborne_radar/decision/pipeline/ControlReducer.cpp
    airborne_radar/decision/pipeline/TacticalCoordinator.cpp
)

set(AIRBORNE_CORE_SOURCES
    airborne_radar/core/context/MutableRadarContext.cpp
    airborne_radar/core/context/RadarInputValidation.cpp
    airborne_radar/core/controller/RadarController.cpp
    airborne_radar/core/session/config/ConfigPresets.cpp
    airborne_radar/core/session/config/RadarSessionConfigBuilder.cpp
    airborne_radar/core/session/RadarSession.cpp
    airborne_radar/core/session/RadarTraceSession.cpp
    airborne_radar/environment/database/FeatureRepository.cpp
    airborne_radar/environment/EnvironmentSceneBuilder.cpp
    airborne_radar/environment/EnvironmentService.cpp
    airborne_radar/environment/scene/SceneManager.cpp
    airborne_radar/environment/simulation/PropagationModel.cpp
)

set(ESR_SOURCES
    electronic_surveillance_radar/core/context/EsrCoordinateUtils.cpp
    electronic_surveillance_radar/core/context/EsrInputValidation.cpp
    electronic_surveillance_radar/core/controller/EsrController.cpp
    electronic_surveillance_radar/core/output/EsrOutputManager.cpp
    electronic_surveillance_radar/core/session/EsrSessionConfigResolver.cpp
    electronic_surveillance_radar/core/session/EsrSession.cpp
    electronic_surveillance_radar/core/session/EsrTraceSession.cpp
    electronic_surveillance_radar/environment/EsrEnvironmentService.cpp
    electronic_surveillance_radar/pipeline/HypothesisAssociator.cpp
    electronic_surveillance_radar/pipeline/InterceptPipeline.cpp
    electronic_surveillance_radar/pipeline/KdTreeClusterer.cpp
    electronic_surveillance_radar/pipeline/ObservationPreprocessor.cpp
)

set(EOS_SOURCES
    electro_optical_sensor/environment/EosEnvironmentModel.cpp
    electro_optical_sensor/foundation/EosNoiseModel.cpp
    electro_optical_sensor/foundation/EosOpticalCharacteristics.cpp
    electro_optical_sensor/foundation/EosPropagation.cpp
    electro_optical_sensor/foundation/EosRadiativeTransfer.cpp
    electro_optical_sensor/foundation/EosRadiometry.cpp
    electro_optical_sensor/foundation/EosSpatialSpectrum.cpp
    electro_optical_sensor/foundation/EosStrayLight.cpp
    electro_optical_sensor/core/context/EosCoordinateUtils.cpp
    electro_optical_sensor/core/context/EosInputValidation.cpp
    electro_optical_sensor/core/pipeline/EosPipeline.cpp
    electro_optical_sensor/core/session/EosSession.cpp
    electro_optical_sensor/core/session/EosTraceSession.cpp
)

add_library(airborne_common OBJECT ${AIRBORNE_COMMON_SOURCES})
add_library(airborne_signal OBJECT ${AIRBORNE_SIGNAL_SOURCES})
add_library(airborne_decision OBJECT ${AIRBORNE_DECISION_SOURCES})
add_library(airborne_core OBJECT ${AIRBORNE_CORE_SOURCES})

set(PROJECT_SOURCES
    ${ONEQ_COMMON_SOURCES}
    ${ESR_SOURCES}
    ${EOS_SOURCES}
    $<TARGET_OBJECTS:airborne_common>
    $<TARGET_OBJECTS:airborne_signal>
    $<TARGET_OBJECTS:airborne_decision>
    $<TARGET_OBJECTS:airborne_core>
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
foreach(AIRBORNE_SUBTARGET IN ITEMS airborne_common airborne_signal airborne_decision airborne_core)
    target_include_directories(${AIRBORNE_SUBTARGET}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${PROJECT_GENERATED_INCLUDE_DIR}
            ${CMAKE_SOURCE_DIR}/include
    )
endforeach()
unset(AIRBORNE_SUBTARGET)

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
find_package(eventpp CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(nanoflann CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
if(PROJECT_ENABLE_SPDLOG)
    find_package(spdlog CONFIG REQUIRED)
endif()

set(ONEQ_LINK_DEPENDENCIES
    Eigen3::Eigen
    eventpp::eventpp
    Boost::boost
    nanoflann::nanoflann
    nlohmann_json::nlohmann_json
)
foreach(ONEQ_BUILD_TARGET IN ITEMS
    ${PROJECT_CORE_TARGET}
    airborne_common
    airborne_signal
    airborne_decision
    airborne_core
)
    target_link_libraries(${ONEQ_BUILD_TARGET} PRIVATE ${ONEQ_LINK_DEPENDENCIES})
endforeach()
unset(ONEQ_BUILD_TARGET)

# Conan's VS multi-config generation may only attach header-only include dirs
# to Release. Mirror those include dirs onto this target when available so
# Debug/RelWithDebInfo/MinSizeRel can compile against the same headers.
foreach(_oneq_conan_include_var IN ITEMS
    eigen_INCLUDE_DIRS_RELEASE
    eventpp_INCLUDE_DIRS_RELEASE
    boost_INCLUDE_DIRS_RELEASE
    nanoflann_INCLUDE_DIRS_RELEASE
    nlohmann_json_INCLUDE_DIRS_RELEASE
)
    if(DEFINED ${_oneq_conan_include_var})
        foreach(ONEQ_BUILD_TARGET IN ITEMS
            ${PROJECT_CORE_TARGET}
            airborne_common
            airborne_signal
            airborne_decision
            airborne_core
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
        airborne_common
        airborne_signal
        airborne_decision
        airborne_core
    )
        target_link_libraries(${ONEQ_BUILD_TARGET} PRIVATE ${PROJECT_SPDLOG_TARGET})
    endforeach()
    unset(ONEQ_BUILD_TARGET)
endif()

foreach(ONEQ_BUILD_TARGET IN ITEMS
    ${PROJECT_CORE_TARGET}
    airborne_common
    airborne_signal
    airborne_decision
    airborne_core
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
        airborne_common
        airborne_signal
        airborne_decision
        airborne_core
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
    foreach(AIRBORNE_SUBTARGET IN ITEMS airborne_common airborne_signal airborne_decision airborne_core)
        target_compile_definitions(${AIRBORNE_SUBTARGET}
            PRIVATE ONEQ_STATIC_DEFINE
        )
    endforeach()
    unset(AIRBORNE_SUBTARGET)
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
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)

    set(PUBLIC_HEADERS_COMMON
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/common/control/ControlDirective.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/common/model/DecisionInputFrame.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/common/model/DecisionSourceInfo.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/common/model/DecisionTrackSnapshot.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/common/utils/JammingSemantics.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/common/control/RadarCommand.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/common/control/RadarControlProfile.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/common/utils/RadarOrientationUtils.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/common/model/TargetCategory.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/common/model/TargetFeature.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/common/utils/TargetFeatureUtils.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/common/output/TrackOutputFrame.h
    )
    set(PUBLIC_HEADERS_COMMON_OUTPUT
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/common/output/TrackOutputQueries.h
    )
    set(PUBLIC_HEADERS_CONFIG_ENTRY
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/AntennaPatternConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/ConfigPresets.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarOrientationConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarRuntimeConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarSessionConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/SignalBeamControlConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/SignalDetectionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/SignalPipelineConfig.h
    )
    set(PUBLIC_HEADERS_CORE_CONTEXT
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/core/context/IRadarContext.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/core/context/RadarCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/core/context/RadarInputValidation.h
    )
    set(PUBLIC_HEADERS_CORE_CONTROLLER
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/core/controller/IRadarOutputReader.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/core/controller/RadarController.h
    )
    set(PUBLIC_HEADERS_CORE_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/core/session/RadarCycleResult.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/core/session/RadarSession.h
    )
    set(PUBLIC_HEADERS_DECISION
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/decision/pipeline/ControlReducerTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h
    )
    set(PUBLIC_HEADERS_ENVIRONMENT
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentSceneBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/IEnvironmentService.h
    )
    set(PUBLIC_HEADERS_SIGNAL_PIPELINE
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/signal/pipeline/ISignalPipeline.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/signal/pipeline/SignalPipelineResultTypes.h
    )
    set(PUBLIC_HEADERS_ESR_COMMON
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/common/EsrCoordinateUtils.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/common/EmitterHypothesis.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/common/EmitterObservation.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/common/EmitterTruthState.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/common/EsrOrientationConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/common/EsrOutputFrame.h
    )
    set(PUBLIC_HEADERS_ESR_CORE_CONTEXT
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/core/context/EsrCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/core/context/EsrInputValidation.h
    )
    set(PUBLIC_HEADERS_ESR_CORE_CONTROLLER
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/core/controller/EsrController.h
    )
    set(PUBLIC_HEADERS_ESR_CORE_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/core/session/EsrCycleResult.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/core/session/EsrSession.h
    )
    set(PUBLIC_HEADERS_ESR_ENVIRONMENT
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h
    )
    set(PUBLIC_HEADERS_ESR_PIPELINE
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/pipeline/IInterceptPipeline.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h
    )
    set(PUBLIC_HEADERS_EOS_COMMON
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/common/EosOutputFrame.h
    )
    set(PUBLIC_HEADERS_EOS_FOUNDATION
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/foundation/EosNoiseModel.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/foundation/EosOpticalCharacteristics.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/foundation/EosPropagation.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/foundation/EosRadiometry.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/foundation/EosSpatialSpectrum.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/foundation/EosStrayLight.h
    )
    set(PUBLIC_HEADERS_EOS_CORE_CONTEXT
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/core/context/EosCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/core/context/EosCoordinateUtils.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/core/context/EosInputValidation.h
    )
    set(PUBLIC_HEADERS_EOS_CORE_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/core/session/EosCycleResult.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/core/session/EosSession.h
    )
    set(PUBLIC_HEADERS_ROOT
        ${CMAKE_SOURCE_DIR}/include/1q/api.hpp
    )
    set(PUBLIC_HEADERS_FOUNDATION
        ${CMAKE_SOURCE_DIR}/include/1q/common/coordinate_transform.h
        ${CMAKE_SOURCE_DIR}/include/1q/common/pose_types.h
        ${CMAKE_SOURCE_DIR}/include/1q/common/scan_schedule_types.h
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
    install(FILES ${PUBLIC_HEADERS_FOUNDATION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/common
    )
    install(FILES "${PROJECT_GENERATED_EXPORT_HEADER}"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q
    )
    install(FILES ${PUBLIC_HEADERS_COMMON}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/common
    )
    install(FILES ${PUBLIC_HEADERS_COMMON_OUTPUT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/common/output
    )
    install(FILES ${PUBLIC_HEADERS_CONFIG_ENTRY}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/config
    )
    install(FILES ${PUBLIC_HEADERS_CORE_CONTEXT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/core/context
    )
    install(FILES ${PUBLIC_HEADERS_CORE_CONTROLLER}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/core/controller
    )
    install(FILES ${PUBLIC_HEADERS_CORE_SESSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/core/session
    )
    install(FILES ${PUBLIC_HEADERS_DECISION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/decision/pipeline
    )
    install(FILES ${PUBLIC_HEADERS_ENVIRONMENT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/environment
    )
    install(FILES ${PUBLIC_HEADERS_SIGNAL_PIPELINE}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/signal/pipeline
    )
    install(FILES ${PUBLIC_HEADERS_ESR_COMMON}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/common
    )
    install(FILES ${PUBLIC_HEADERS_ESR_CORE_CONTEXT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/core/context
    )
    install(FILES ${PUBLIC_HEADERS_ESR_CORE_CONTROLLER}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/core/controller
    )
    install(FILES ${PUBLIC_HEADERS_ESR_CORE_SESSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/core/session
    )
    install(FILES ${PUBLIC_HEADERS_ESR_ENVIRONMENT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/environment
    )
    install(FILES ${PUBLIC_HEADERS_ESR_PIPELINE}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/pipeline
    )
    install(FILES ${PUBLIC_HEADERS_EOS_COMMON}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/common
    )
    install(FILES ${PUBLIC_HEADERS_EOS_FOUNDATION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/foundation
    )
    install(FILES ${PUBLIC_HEADERS_EOS_CORE_CONTEXT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/core/context
    )
    install(FILES ${PUBLIC_HEADERS_EOS_CORE_SESSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/core/session
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
