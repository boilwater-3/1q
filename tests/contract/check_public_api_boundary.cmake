set(PUBLIC_INCLUDE_DIR "${SOURCE_DIR}/include/1q")

# ── AR 推荐公开主路径（四域 + 会话 + Builder + 统一入口） ──────────
set(AR_PUBLIC_PRIMARY_HEADERS
    "airborne_radar/airborne_radar.hpp"
    "airborne_radar/config/RadarHardwareConfig.h"
    "airborne_radar/config/RadarMissionConfig.h"
    "airborne_radar/config/RadarPolicyConfig.h"
    "airborne_radar/config/RadarEnvironmentConfig.h"
    "airborne_radar/config/RadarSessionConfig.h"
    "airborne_radar/config/RadarRuntimeConfigPatch.h"
    "airborne_radar/config/RadarRuntimeConfigBuilder.h"
    "airborne_radar/config/RadarSessionConfigBuilder.h"
    "airborne_radar/config/airborne_radar_config.hpp"
)

# ── AR 环境域 ────────────────────────────────────────────────────────
set(AR_ENVIRONMENT_HEADERS
    "airborne_radar/environment/EnvironmentConfig.h"
    "airborne_radar/environment/EnvironmentRuntimeConfigPatch.h"
    "airborne_radar/environment/EnvironmentSceneBuilder.h"
    "airborne_radar/environment/EnvironmentTypes.h"
    "airborne_radar/environment/IEnvironmentService.h"
    "airborne_radar/environment/airborne_radar_environment.hpp"
)

# ── AR 扩展域 ────────────────────────────────────────────────────────
set(AR_EXTENSION_HEADERS
    "airborne_radar/extension/ControlReducerTypes.h"
    "airborne_radar/extension/IRadarCommandBus.h"
    "airborne_radar/extension/IRadarContext.h"
    "airborne_radar/extension/IRadarContextReader.h"
    "airborne_radar/extension/IRadarControlProfileStore.h"
    "airborne_radar/extension/ISignalPipeline.h"
    "airborne_radar/extension/ITacticalDecisionEngine.h"
    "airborne_radar/extension/IOverrideControlStrategy.h"
    "airborne_radar/extension/RadarController.h"
    "airborne_radar/extension/SignalPipelineResultTypes.h"
    "airborne_radar/extension/airborne_radar_extension.hpp"
    "airborne_radar/extension/control/ControlDirective.h"
    "airborne_radar/extension/control/RadarCommand.h"
    "airborne_radar/extension/control/RadarControlProfile.h"
)

# ── AR 模型域 ────────────────────────────────────────────────────────
set(AR_MODEL_HEADERS
    "airborne_radar/model/DecisionInputFrame.h"
    "airborne_radar/model/DecisionSourceInfo.h"
    "airborne_radar/model/TrackStateSnapshot.h"
    "airborne_radar/model/JammingSemantics.h"
    "airborne_radar/model/RadarOrientationConfig.h"
    "airborne_radar/model/TargetCategory.h"
)

# ── AR 输出域 ────────────────────────────────────────────────────────
set(AR_OUTPUT_HEADERS
)

# ── AR 会话域 ────────────────────────────────────────────────────────
set(AR_SESSION_HEADERS
    "airborne_radar/session/RadarCycleInput.h"
    "airborne_radar/session/RadarCycleInputBuilder.h"
    "airborne_radar/session/RadarCycleOutputBuilder.h"
    "airborne_radar/session/RadarCycleResult.h"
    "airborne_radar/session/RadarEnvironmentInput.h"
    "airborne_radar/session/RadarEnvironmentInputPatch.h"
    "airborne_radar/session/RadarEnvironmentInputState.h"
    "airborne_radar/session/RadarExternalInputAdapter.h"
    "airborne_radar/session/RadarExternalOutputAdapter.h"
    "airborne_radar/session/RadarInputValidation.h"
    "airborne_radar/session/RadarSceneTypes.h"
    "airborne_radar/session/RadarSceneTargetUtils.h"
    "airborne_radar/session/RadarReplaySession.h"
    "airborne_radar/session/RadarSession.h"
    "airborne_radar/session/RadarSessionFactory.h"
    "airborne_radar/session/RadarTraceSession.h"
)

# ── 顶层入口 ─────────────────────────────────────────────────────────
set(ROOT_HEADER
    "api.hpp"
)

# ── EOS 模块入口 ─────────────────────────────────────────────────────
set(EOS_MODULE_ENTRY_HEADERS
    "electro_optical_sensor/electro_optical_sensor.hpp"
)

# ── EOS 配置域 ────────────────────────────────────────────────────────
set(EOS_CONFIG_HEADERS
    "electro_optical_sensor/config/EosEnvironmentConfig.h"
    "electro_optical_sensor/config/EosHardwareConfig.h"
    "electro_optical_sensor/config/EosMissionConfig.h"
    "electro_optical_sensor/config/EosPolicyConfig.h"
    "electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
    "electro_optical_sensor/config/EosRuntimeConfigPatch.h"
    "electro_optical_sensor/config/EosSessionConfig.h"
    "electro_optical_sensor/config/EosSessionConfigBuilder.h"
    "electro_optical_sensor/config/electro_optical_sensor_config.hpp"
)

# ── EOS 环境域 ────────────────────────────────────────────────────────
set(EOS_ENVIRONMENT_HEADERS
    "electro_optical_sensor/environment/EosEnvironmentConfig.h"
    "electro_optical_sensor/environment/EosEnvironmentRuntimeConfigPatch.h"
    "electro_optical_sensor/environment/EosEnvironmentTypes.h"
    "electro_optical_sensor/environment/IEosEnvironmentService.h"
    "electro_optical_sensor/environment/electro_optical_sensor_environment.hpp"
)

# ── EOS 扩展域 ────────────────────────────────────────────────────────
set(EOS_EXTENSION_HEADERS
    "electro_optical_sensor/extension/EosController.h"
    "electro_optical_sensor/extension/EosPipelineTypes.h"
    "electro_optical_sensor/extension/IEosPipeline.h"
    "electro_optical_sensor/extension/electro_optical_sensor_extension.hpp"
)

# ── EOS 基础域 ────────────────────────────────────────────────────────
set(EOS_FOUNDATION_HEADERS
    "electro_optical_sensor/foundation/EosRadiativeTransfer.h"
)

# ── EOS 会话域 ────────────────────────────────────────────────────────
set(EOS_SESSION_HEADERS
    "electro_optical_sensor/session/EosCycleInput.h"
    "electro_optical_sensor/session/EosCycleInputBuilder.h"
    "electro_optical_sensor/session/EosCycleOutputBuilder.h"
    "electro_optical_sensor/session/EosCycleResult.h"
    "electro_optical_sensor/session/EosEnvironmentInput.h"
    "electro_optical_sensor/session/EosEnvironmentInputPatch.h"
    "electro_optical_sensor/session/EosEnvironmentInputState.h"
    "electro_optical_sensor/session/EosExternalInputAdapter.h"
    "electro_optical_sensor/session/EosExternalOutputAdapter.h"
    "electro_optical_sensor/session/EosInputValidation.h"
    "electro_optical_sensor/session/EosSceneTypes.h"
    "electro_optical_sensor/session/EosSession.h"
    "electro_optical_sensor/session/EosSessionFactory.h"
    "electro_optical_sensor/session/EosTraceSession.h"
    "electro_optical_sensor/session/EosReplaySession.h"
)

# ── ESR 公开头 ──────────────────────────────────────────────────────
set(ESR_MODULE_ENTRY_HEADERS
    "electronic_surveillance_radar/electronic_surveillance_radar.hpp"
)

# ── ESR 配置域 ────────────────────────────────────────────────────────
set(ESR_CONFIG_HEADERS
    "electronic_surveillance_radar/config/EsrEnvironmentConfig.h"
    "electronic_surveillance_radar/config/EsrHardwareConfig.h"
    "electronic_surveillance_radar/config/EsrMissionConfig.h"
    "electronic_surveillance_radar/config/EsrPolicyConfig.h"
    "electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
    "electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
    "electronic_surveillance_radar/config/EsrSessionConfig.h"
    "electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
    "electronic_surveillance_radar/config/electronic_surveillance_radar_config.hpp"
)

# ── ESR 环境域 ────────────────────────────────────────────────────────
set(ESR_ENVIRONMENT_HEADERS
    "electronic_surveillance_radar/environment/EsrEnvironmentConfig.h"
    "electronic_surveillance_radar/environment/EsrEnvironmentRuntimeConfigPatch.h"
    "electronic_surveillance_radar/environment/EsrEnvironmentSceneBuilder.h"
    "electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
    "electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
    "electronic_surveillance_radar/environment/electronic_surveillance_radar_environment.hpp"
)

# ── ESR 扩展域 ────────────────────────────────────────────────────────
set(ESR_EXTENSION_HEADERS
    "electronic_surveillance_radar/extension/EsrController.h"
    "electronic_surveillance_radar/extension/IEsrContext.h"
    "electronic_surveillance_radar/extension/IInterceptPipeline.h"
    "electronic_surveillance_radar/extension/InterceptPipelineTypes.h"
)

# ── ESR 模型域 ────────────────────────────────────────────────────────
set(ESR_MODEL_HEADERS
    "electronic_surveillance_radar/model/EmitterHypothesis.h"
    "electronic_surveillance_radar/model/EmitterObservation.h"
)

# ── ESR 会话域 ────────────────────────────────────────────────────────
set(ESR_SESSION_HEADERS
    "electronic_surveillance_radar/session/EsrCycleInput.h"
    "electronic_surveillance_radar/session/EsrCycleInputBuilder.h"
    "electronic_surveillance_radar/session/EsrCycleOutputBuilder.h"
    "electronic_surveillance_radar/session/EsrCycleResult.h"
    "electronic_surveillance_radar/session/EsrEnvironmentInput.h"
    "electronic_surveillance_radar/session/EsrEnvironmentInputPatch.h"
    "electronic_surveillance_radar/session/EsrEnvironmentInputState.h"
    "electronic_surveillance_radar/session/EsrExternalInputAdapter.h"
    "electronic_surveillance_radar/session/EsrExternalOutputAdapter.h"
    "electronic_surveillance_radar/session/EsrInputValidation.h"
    "electronic_surveillance_radar/session/EsrSceneTypes.h"
    "electronic_surveillance_radar/session/EsrSession.h"
    "electronic_surveillance_radar/session/EsrSessionFactory.h"
    "electronic_surveillance_radar/session/EsrTraceSession.h"
    "electronic_surveillance_radar/session/EsrReplaySession.h"
)

# ── 飞行动力学 ───────────────────────────────────────────────────────
set(FD_HEADERS
    "flight_dynamic/config/AircraftDefinition.h"
    "flight_dynamic/config/FlightDynamicConfig.h"
    "flight_dynamic/flight_dynamic.hpp"
    "flight_dynamic/model/FlightDynamicInput.h"
    "flight_dynamic/model/FlightDynamicOutput.h"
    "flight_dynamic/model/VehicleState.h"
    "flight_dynamic/session/FlightDynamicSession.h"
    "flight_dynamic/session/FlightDynamicSessionFactory.h"
)

# ── 坐标工具 ─────────────────────────────────────────────────────────
set(COORDINATE_HEADERS
    "coordinate/attitude_transform.h"
    "coordinate/position_transform.h"
    "coordinate/types.h"
    "coordinate/velocity_transform.h"
)

# ── 跨域基础 ─────────────────────────────────────────────────────────
set(FOUNDATION_HEADERS
    "foundation/atmospheric_types.h"
    "foundation/json_reader.h"
    "foundation/pose_types.h"
    "foundation/scan_schedule_types.h"
    "replay/ReplayTrace.h"
    "trace/TraceSink.h"
)

set(EXPECTED_PUBLIC_HEADERS
    ${AR_PUBLIC_PRIMARY_HEADERS}
    ${AR_ENVIRONMENT_HEADERS}
    ${AR_EXTENSION_HEADERS}
    ${AR_MODEL_HEADERS}
    ${AR_OUTPUT_HEADERS}
    ${AR_SESSION_HEADERS}
    ${ROOT_HEADER}
    ${EOS_MODULE_ENTRY_HEADERS}
    ${EOS_CONFIG_HEADERS}
    ${EOS_ENVIRONMENT_HEADERS}
    ${EOS_EXTENSION_HEADERS}
    ${EOS_FOUNDATION_HEADERS}
    ${EOS_SESSION_HEADERS}
    ${ESR_MODULE_ENTRY_HEADERS}
    ${ESR_CONFIG_HEADERS}
    ${ESR_ENVIRONMENT_HEADERS}
    ${ESR_EXTENSION_HEADERS}
    ${ESR_MODEL_HEADERS}
    ${ESR_SESSION_HEADERS}
    ${FD_HEADERS}
    ${COORDINATE_HEADERS}
    ${FOUNDATION_HEADERS}
)

file(GLOB_RECURSE ACTUAL_PUBLIC_HEADERS
     RELATIVE "${PUBLIC_INCLUDE_DIR}"
     "${PUBLIC_INCLUDE_DIR}/*.h"
     "${PUBLIC_INCLUDE_DIR}/*.hpp")

list(SORT EXPECTED_PUBLIC_HEADERS)
list(SORT ACTUAL_PUBLIC_HEADERS)

if(NOT ACTUAL_PUBLIC_HEADERS STREQUAL EXPECTED_PUBLIC_HEADERS)
  message(FATAL_ERROR
          "Public header whitelist drifted.\n"
          "Expected: ${EXPECTED_PUBLIC_HEADERS}\n"
          "Actual: ${ACTUAL_PUBLIC_HEADERS}")
endif()

execute_process(
    COMMAND find "${PUBLIC_INCLUDE_DIR}" -type d -empty
    OUTPUT_VARIABLE EMPTY_DIRS
    OUTPUT_STRIP_TRAILING_WHITESPACE)

if(NOT "${EMPTY_DIRS}" STREQUAL "")
  message(FATAL_ERROR
          "Empty directories are not allowed under include/1q:\n${EMPTY_DIRS}")
endif()

foreach(HEADER IN LISTS ACTUAL_PUBLIC_HEADERS)
  file(READ "${PUBLIC_INCLUDE_DIR}/${HEADER}" HEADER_CONTENT)

  if(HEADER_CONTENT MATCHES "#[ \t]*include[ \t]*[<\"]boost/"
     OR HEADER_CONTENT MATCHES "boost::any")
    message(FATAL_ERROR
            "Public header must not expose Boost: ${HEADER}")
  endif()

  if(HEADER_CONTENT MATCHES "#[ \t]*include[ \t]*[<\"]Eigen/")
    message(FATAL_ERROR
            "Public header must not expose Eigen: ${HEADER}")
  endif()
endforeach()

# Guardrail: AR 公开配置头不能重新引入 expert 命名空间，避免公开语义回退。
set(AR_CONFIG_HEADERS_NO_EXPERT_NAMESPACE
    "airborne_radar/config/RadarHardwareConfig.h"
    "airborne_radar/config/RadarPolicyConfig.h")

foreach(HEADER IN LISTS AR_CONFIG_HEADERS_NO_EXPERT_NAMESPACE)
  file(READ "${PUBLIC_INCLUDE_DIR}/${HEADER}" AR_CONFIG_HEADER_CONTENT)
  if(AR_CONFIG_HEADER_CONTENT MATCHES "namespace[ \t]+expert[ \t]*\\{")
    message(FATAL_ERROR
            "Public AR config header must not expose namespace expert: ${HEADER}")
  endif()
endforeach()

# Guardrail: prevent legacy top-level WithXxx/EnableXxx APIs from returning to
# RadarSessionConfigBuilder. Only grouped editors are allowed.
set(RADAR_SESSION_BUILDER_HEADER
    "${PUBLIC_INCLUDE_DIR}/airborne_radar/config/RadarSessionConfigBuilder.h")
file(READ "${RADAR_SESSION_BUILDER_HEADER}" RADAR_SESSION_BUILDER_CONTENT)

set(FORBIDDEN_BUILDER_METHOD_PATTERNS
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithDetection[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithBeamControl[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithTracking[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithLifecycle[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithEnvironmentDefault[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*EnablePhysicsDetection[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithMinDetectionMarginDb[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithPulseCount[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithTransmitterConfig[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithAntennaConfig[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithReceiverConfig[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithDetectionPolicy[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithPeakPowerW[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithFrequencyHz[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithBandwidthHz[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithPulseWidthS[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithPrfHz[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithMainBeamGainDb[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithNoiseFigureDb[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithRadarWorkSubMode[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithScanCenterDeg[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithDwellCenterDeg[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*EnableCommandedBeamwidth[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithCommandedBeamwidthDeg[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithLifecycleConfirmHits[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithLifecycleMaxMissBeforeLost[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithLifecycleMaxLostCycles[ \t]*\\("
    "RadarSessionConfigBuilder[ \t]*&[ \t]*WithJammingDetectionThresholdDb[ \t]*\\(")

foreach(FORBIDDEN_PATTERN IN LISTS FORBIDDEN_BUILDER_METHOD_PATTERNS)
  if(RADAR_SESSION_BUILDER_CONTENT MATCHES "${FORBIDDEN_PATTERN}")
    message(FATAL_ERROR
            "Legacy top-level RadarSessionConfigBuilder API reintroduced: ${FORBIDDEN_PATTERN}\n"
            "Use grouped editors only: Detection()/Beam()/Tracking()/Lifecycle()/Environment().")
  endif()
endforeach()

# Guardrail: ESR Session builder keeps semantic-only surface; domain-level
# whole-config setters must use direct field assignment.
set(ESR_SESSION_BUILDER_HEADER
    "${PUBLIC_INCLUDE_DIR}/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h")
file(READ "${ESR_SESSION_BUILDER_HEADER}" ESR_SESSION_BUILDER_CONTENT)

set(ESR_FORBIDDEN_SESSION_METHOD_PATTERNS
    "EsrSessionConfigBuilder[ \t]*&[ \t]*WithHardwareConfig[ \t]*\\("
    "EsrSessionConfigBuilder[ \t]*&[ \t]*WithMissionConfig[ \t]*\\("
    "EsrSessionConfigBuilder[ \t]*&[ \t]*WithPolicyConfig[ \t]*\\("
    "EsrSessionConfigBuilder[ \t]*&[ \t]*WithEnvironmentConfig[ \t]*\\(")

foreach(FORBIDDEN_PATTERN IN LISTS ESR_FORBIDDEN_SESSION_METHOD_PATTERNS)
  if(ESR_SESSION_BUILDER_CONTENT MATCHES "${FORBIDDEN_PATTERN}")
    message(FATAL_ERROR
            "Legacy ESR session-builder domain setter reintroduced: ${FORBIDDEN_PATTERN}\n"
            "Use direct field assignment for domain-level overrides.")
  endif()
endforeach()

# Guardrail (M7-F): deleted internal legacy config shells must not reappear.
set(AR_FORBIDDEN_INTERNAL_PATHS
        "${CMAKE_SOURCE_DIR}/src/airborne_radar/config/legacy"
    "${CMAKE_SOURCE_DIR}/src/airborne_radar/config/internal/SessionConfigPipelineMapper.h"
    "${CMAKE_SOURCE_DIR}/src/airborne_radar/config/internal/ExpertToEngineeringMapping.h")

foreach(FORBIDDEN_PATH IN LISTS AR_FORBIDDEN_INTERNAL_PATHS)
    if(EXISTS "${FORBIDDEN_PATH}")
        message(FATAL_ERROR
                        "Forbidden internal legacy path reintroduced: ${FORBIDDEN_PATH}\n"
                        "M7-F requires legacy config shells, SessionConfigPipelineMapper, and ExpertToEngineeringMapping to stay removed.")
    endif()
endforeach()
