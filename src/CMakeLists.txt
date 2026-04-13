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
    airborne_radar/model/DecisionTrackSnapshot.cpp
    airborne_radar/output/TrackOutputQueries.cpp
    airborne_radar/model/TargetFeatureUtils.cpp
)

set(AIRBORNE_SIGNAL_SOURCES
    airborne_radar/signal/association/DataAssociation.cpp
    airborne_radar/signal/association/DistanceMetric.cpp
    airborne_radar/signal/association/Hypothesiser.cpp
    airborne_radar/signal/association/LapjvSolver.cpp
    airborne_radar/signal/assembly/DataOutputManager.cpp
    airborne_radar/signal/detection/RadarEquations.cpp
    airborne_radar/signal/detection/SignalDetector.cpp
    airborne_radar/signal/pipeline/core/AssociationExecutionSupport.cpp
    airborne_radar/signal/pipeline/core/ContextBindingSupport.cpp
    airborne_radar/signal/pipeline/effects/ControlProfileEffects.cpp
    airborne_radar/signal/pipeline/core/CycleContextSupport.cpp
    airborne_radar/signal/pipeline/core/CycleExecutor.cpp
    airborne_radar/signal/pipeline/assembly/DecisionFrameBuilders.cpp
    airborne_radar/signal/pipeline/core/DetectionExecution.cpp
    airborne_radar/signal/pipeline/effects/JammingEffects.cpp
    airborne_radar/signal/pipeline/assembly/OutputAssemblySupport.cpp
    airborne_radar/signal/pipeline/core/SignalPipeline.cpp
    airborne_radar/signal/pipeline/assembly/TrackMeasurementProcessing.cpp
    airborne_radar/signal/pipeline/assembly/RuntimeAssemblySupport.cpp
    airborne_radar/signal/pipeline/core/ScanScheduleResolver.cpp
    airborne_radar/signal/pipeline/assembly/SignalComponentFactory.cpp
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
    airborne_radar/session/MutableRadarContext.cpp
    airborne_radar/session/RadarInputValidation.cpp
    airborne_radar/runtime/ControlCommandMapper.cpp
    airborne_radar/runtime/CycleTelemetryLogger.cpp
    airborne_radar/runtime/RadarCycleOrchestrator.cpp
    airborne_radar/runtime/RadarController.cpp
    airborne_radar/session/RadarSessionCompositionRoot.cpp
    airborne_radar/session/RadarSessionConfigPresets.cpp
    airborne_radar/session/RadarSessionConfigBuilder.cpp
    airborne_radar/session/RuntimeConfigResolver.cpp
    airborne_radar/session/RadarSession.cpp
    airborne_radar/session/RadarTraceSession.cpp
    airborne_radar/environment/FeatureRepository.cpp
    airborne_radar/environment/EnvironmentSceneBuilder.cpp
    airborne_radar/environment/EnvironmentService.cpp
    airborne_radar/environment/SceneManager.cpp
    airborne_radar/environment/PropagationModel.cpp
)

set(ESR_MODEL_SOURCES
    electronic_surveillance_radar/model/EsrCoordinateUtils.cpp
)

set(ESR_ENVIRONMENT_SOURCES
    electronic_surveillance_radar/environment/EsrEnvironmentService.cpp
)

set(ESR_PIPELINE_SOURCES
    electronic_surveillance_radar/pipeline/MutableEsrContext.cpp
    electronic_surveillance_radar/pipeline/HypothesisAssociator.cpp
    electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.cpp
    electronic_surveillance_radar/pipeline/InterceptPostProcessingExecutor.cpp
    electronic_surveillance_radar/pipeline/InterceptPipeline.cpp
    electronic_surveillance_radar/pipeline/KdTreeClusterer.cpp
    electronic_surveillance_radar/pipeline/ObservationPreprocessor.cpp
)

set(ESR_RUNTIME_SOURCES
    electronic_surveillance_radar/runtime/EsrController.cpp
    electronic_surveillance_radar/runtime/EsrCycleTelemetryLogger.cpp
    electronic_surveillance_radar/output/EsrOutputManager.cpp
)

set(ESR_SESSION_SOURCES
    electronic_surveillance_radar/session/EsrInputValidation.cpp
    electronic_surveillance_radar/session/EsrSessionCompositionRoot.cpp
    electronic_surveillance_radar/session/EsrSessionConfigResolver.cpp
    electronic_surveillance_radar/session/EsrRuntimeConfigResolver.cpp
    electronic_surveillance_radar/session/EsrSession.cpp
    electronic_surveillance_radar/session/EsrTraceSession.cpp
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
    electro_optical_sensor/utils/EosCoordinateUtils.cpp
    electro_optical_sensor/model/EosInputValidation.cpp
    electro_optical_sensor/runtime/EosController.cpp
    electro_optical_sensor/runtime/EosCycleOrchestrator.cpp
    electro_optical_sensor/signal/pipeline/EosPipeline.cpp
    electro_optical_sensor/runtime/EosPipelineConfigMapper.cpp
    electro_optical_sensor/runtime/EosRuntimeConfigResolver.cpp
    electro_optical_sensor/session/EosSessionCompositionRoot.cpp
    electro_optical_sensor/session/EosSession.cpp
    electro_optical_sensor/session/EosTraceSession.cpp
)

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
    eventpp_INCLUDE_DIRS_RELEASE
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
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)

    set(PUBLIC_HEADERS_COMMON_CONTROL
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/control/ControlDirective.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/control/RadarCommand.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/control/RadarControlProfile.h
    )
    set(PUBLIC_HEADERS_COMMON_MODEL
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/DecisionInputFrame.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/DecisionSourceInfo.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/DecisionTrackSnapshot.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/RadarOrientationConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/TargetCategory.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/TargetFeature.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/TargetFeatureBuilder.h
    )
    set(PUBLIC_HEADERS_COMMON_UTILS
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/JammingSemantics.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/model/TargetFeatureUtils.h
    )
    set(PUBLIC_HEADERS_COMMON_OUTPUT
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/output/TrackOutputFrame.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/output/TrackOutputQueries.h
    )
    set(PUBLIC_HEADERS_CONFIG_ENTRY
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarSessionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarSessionConfigPresets.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarRuntimeConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/RadarSessionConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/airborne_radar_config.hpp
    )
    set(PUBLIC_HEADERS_SIGNAL_CONFIG
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/AntennaPatternConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/SignalBeamControlConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/SignalDetectionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/SignalLifecycleConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/SignalPipelineConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/config/SignalTrackingConfig.h
    )
    set(PUBLIC_HEADERS_CORE_CONTEXT
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarInputValidation.h
    )
    set(PUBLIC_HEADERS_CORE_CONTROLLER
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/IRadarContext.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/IRadarOutputReader.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/RadarController.h
    )
    set(PUBLIC_HEADERS_CORE_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarCycleResult.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarSessionFactory.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarSession.h
    )
    set(PUBLIC_HEADERS_DECISION
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/ControlReducerTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/ITacticalDecisionEngine.h
    )
    set(PUBLIC_HEADERS_ENVIRONMENT
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentDefaultConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentModelConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentRuntimeConfigPatchBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentSceneBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/EnvironmentTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/IEnvironmentService.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/environment/airborne_radar_environment.hpp
    )
    set(PUBLIC_HEADERS_SIGNAL_PIPELINE
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/ISignalPipeline.h
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/SignalPipelineResultTypes.h
    )
    set(PUBLIC_HEADERS_EXTENSION_ENTRY
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/extension/airborne_radar_extension.hpp
    )
    set(PUBLIC_HEADERS_ESR_MODEL
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/model/EsrCoordinateUtils.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/model/EmitterHypothesis.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/model/EmitterObservation.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/model/EmitterTruthState.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/model/EsrOrientationConfig.h
    )
    set(PUBLIC_HEADERS_ESR_OUTPUT
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/output/EsrOutputFrame.h
    )
    set(PUBLIC_HEADERS_ESR_EXTENSION
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/extension/EsrController.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/extension/IEsrContext.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/extension/IInterceptPipeline.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h
    )
    set(PUBLIC_HEADERS_ESR_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/session/EsrCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/session/EsrCycleResult.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/session/EsrInputValidation.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/session/EsrSession.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/session/EsrSessionFactory.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/session/EsrTraceSession.h
    )
    set(PUBLIC_HEADERS_ESR_ENVIRONMENT
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentRuntimeConfigPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentRuntimeConfigPatchBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentSceneBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/environment/electronic_surveillance_radar_environment.hpp
    )
    set(PUBLIC_HEADERS_ESR_CONFIG
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrHardwareConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrMissionControlConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrLayeredConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrSessionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/config/electronic_surveillance_radar_config.hpp
    )
    set(PUBLIC_HEADERS_EOS_COMMON
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/utils/EosCoordinateUtils.h
    )
    set(PUBLIC_HEADERS_EOS_OUTPUT
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/output/EosOutputFrame.h
    )
    set(PUBLIC_HEADERS_EOS_FOUNDATION
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h
    )
    set(PUBLIC_HEADERS_EOS_MODEL
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/model/EosCycleInput.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/model/EosInputValidation.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/model/EosCycleResult.h
    )
    set(PUBLIC_HEADERS_EOS_SESSION
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosSession.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/session/EosTraceSession.h
    )
    set(PUBLIC_HEADERS_EOS_ENVIRONMENT
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/environment/EosEnvironmentConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/environment/EosEnvironmentConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/environment/EosEnvironmentRuntimeConfigPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/environment/EosEnvironmentRuntimeConfigPatchBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/environment/EosEnvironmentTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/environment/IEosEnvironmentService.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/environment/electro_optical_sensor_environment.hpp
    )
    set(PUBLIC_HEADERS_EOS_EXTENSION
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/extension/EosController.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/extension/EosPipelineTypes.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/extension/IEosPipeline.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/extension/electro_optical_sensor_extension.hpp
    )
    set(PUBLIC_HEADERS_EOS_CONFIG
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/EosWorkMode.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/EosSessionConfig.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/EosSessionConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/config/electro_optical_sensor_config.hpp
    )
    set(PUBLIC_HEADERS_ROOT
        ${CMAKE_SOURCE_DIR}/include/1q/api.hpp
        ${CMAKE_SOURCE_DIR}/include/1q/README.md
    )
    set(PUBLIC_HEADERS_MODULE_ENTRY_AIRBORNE
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/airborne_radar.hpp
    )
    set(PUBLIC_HEADERS_MODULE_ENTRY_ESR
        ${CMAKE_SOURCE_DIR}/include/1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp
    )
    set(PUBLIC_HEADERS_MODULE_ENTRY_EOS
        ${CMAKE_SOURCE_DIR}/include/1q/electro_optical_sensor/electro_optical_sensor.hpp
    )
    set(PUBLIC_HEADERS_FOUNDATION
        ${CMAKE_SOURCE_DIR}/include/1q/common/coordinate_transform.h
        ${CMAKE_SOURCE_DIR}/include/1q/common/pose_types.h
        ${CMAKE_SOURCE_DIR}/include/1q/common/scan_schedule_types.h
    )
    set(PUBLIC_HEADERS_TRACE
        ${CMAKE_SOURCE_DIR}/include/1q/common/trace/TraceSink.h
    )
    set(PUBLIC_HEADERS_AIRBORNE_TOOLS
        ${CMAKE_SOURCE_DIR}/include/1q/airborne_radar/session/RadarTraceSession.h
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
    install(FILES ${PUBLIC_HEADERS_MODULE_ENTRY_AIRBORNE}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar
    )
    install(FILES ${PUBLIC_HEADERS_MODULE_ENTRY_ESR}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar
    )
    install(FILES ${PUBLIC_HEADERS_MODULE_ENTRY_EOS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor
    )
    install(FILES ${PUBLIC_HEADERS_FOUNDATION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/common
    )
    install(FILES ${PUBLIC_HEADERS_TRACE}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/common/trace
    )
    install(FILES "${PROJECT_GENERATED_EXPORT_HEADER}"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q
    )
    install(FILES ${PUBLIC_HEADERS_COMMON_CONTROL}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/extension/control
    )
    install(FILES ${PUBLIC_HEADERS_COMMON_MODEL}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/model
    )
    install(FILES ${PUBLIC_HEADERS_COMMON_UTILS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/model
    )
    install(FILES ${PUBLIC_HEADERS_COMMON_OUTPUT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/output
    )
    install(FILES ${PUBLIC_HEADERS_CONFIG_ENTRY}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/config
    )
    install(FILES ${PUBLIC_HEADERS_SIGNAL_CONFIG}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/config
    )
    install(FILES ${PUBLIC_HEADERS_CORE_CONTEXT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/session
    )
    install(FILES ${PUBLIC_HEADERS_CORE_CONTROLLER}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/extension
    )
    install(FILES ${PUBLIC_HEADERS_CORE_SESSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/session
    )
    install(FILES ${PUBLIC_HEADERS_DECISION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/extension
    )
    install(FILES ${PUBLIC_HEADERS_ENVIRONMENT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/environment
    )
    install(FILES ${PUBLIC_HEADERS_SIGNAL_PIPELINE}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/extension
    )
    install(FILES ${PUBLIC_HEADERS_AIRBORNE_TOOLS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/session
    )
    install(FILES ${PUBLIC_HEADERS_EXTENSION_ENTRY}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/airborne_radar/extension
    )
    install(FILES ${PUBLIC_HEADERS_ESR_MODEL}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/model
    )
    install(FILES ${PUBLIC_HEADERS_ESR_OUTPUT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/output
    )
    install(FILES ${PUBLIC_HEADERS_ESR_EXTENSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/extension
    )
    install(FILES ${PUBLIC_HEADERS_ESR_SESSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/session
    )
    install(FILES ${PUBLIC_HEADERS_ESR_ENVIRONMENT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/environment
    )
    install(FILES ${PUBLIC_HEADERS_ESR_CONFIG}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electronic_surveillance_radar/config
    )
    install(FILES ${PUBLIC_HEADERS_EOS_COMMON}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/utils
    )
    install(FILES ${PUBLIC_HEADERS_EOS_OUTPUT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/output
    )
    install(FILES ${PUBLIC_HEADERS_EOS_MODEL}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/model
    )
    install(FILES ${PUBLIC_HEADERS_EOS_FOUNDATION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/foundation
    )
    install(FILES ${PUBLIC_HEADERS_EOS_SESSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/session
    )
    install(FILES ${PUBLIC_HEADERS_EOS_ENVIRONMENT}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/environment
    )
    install(FILES ${PUBLIC_HEADERS_EOS_EXTENSION}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/extension
    )
    install(FILES ${PUBLIC_HEADERS_EOS_CONFIG}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/1q/electro_optical_sensor/config
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
