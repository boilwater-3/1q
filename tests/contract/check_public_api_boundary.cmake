set(PUBLIC_INCLUDE_DIR "${SOURCE_DIR}/include/1q")

set(EXPECTED_PUBLIC_HEADERS
    "common/coordinate_transform.h"
    "common/pose_types.h"
    "common/scan_schedule_types.h"
    "common/trace/TraceSink.h"
    "airborne_radar/extension/control/ControlDirective.h"
    "airborne_radar/model/DecisionInputFrame.h"
    "airborne_radar/model/DecisionSourceInfo.h"
    "airborne_radar/model/DecisionTrackSnapshot.h"
    "airborne_radar/model/JammingSemantics.h"
    "airborne_radar/extension/control/RadarCommand.h"
    "airborne_radar/extension/control/RadarControlProfile.h"
    "airborne_radar/model/TargetCategory.h"
    "airborne_radar/model/TargetFeature.h"
    "airborne_radar/model/TargetFeatureBuilder.h"
    "airborne_radar/model/TargetFeatureUtils.h"
    "airborne_radar/output/TrackOutputFrame.h"
    "airborne_radar/output/TrackOutputQueries.h"
    "airborne_radar/config/AntennaPatternConfig.h"
    "airborne_radar/config/RadarSessionConfig.h"
    "airborne_radar/config/RadarSessionConfigPresets.h"
    "airborne_radar/model/RadarOrientationConfig.h"
    "airborne_radar/config/RadarRuntimeConfigBuilder.h"
    "airborne_radar/config/RadarSessionConfigBuilder.h"
    "airborne_radar/config/SignalBeamControlConfig.h"
    "airborne_radar/config/SignalDetectionConfig.h"
    "airborne_radar/config/SignalLifecycleConfig.h"
    "airborne_radar/config/SignalPipelineConfig.h"
    "airborne_radar/config/SignalTrackingConfig.h"
    "airborne_radar/config/airborne_radar_config.hpp"
    "airborne_radar/extension/IRadarContext.h"
    "airborne_radar/session/RadarCycleInput.h"
    "airborne_radar/session/RadarInputValidation.h"
    "airborne_radar/extension/IRadarOutputReader.h"
    "airborne_radar/extension/RadarController.h"
    "airborne_radar/session/RadarCycleResult.h"
    "airborne_radar/session/RadarSession.h"
    "airborne_radar/session/RadarSessionFactory.h"
    "airborne_radar/session/RadarTraceSession.h"
    "airborne_radar/extension/ControlReducerTypes.h"
    "airborne_radar/extension/ITacticalDecisionEngine.h"
    "airborne_radar/extension/airborne_radar_extension.hpp"
    "airborne_radar/environment/EnvironmentConfig.h"
    "airborne_radar/environment/EnvironmentDefaultConfigBuilder.h"
    "airborne_radar/environment/EnvironmentSceneBuilder.h"
    "airborne_radar/environment/EnvironmentTypes.h"
    "airborne_radar/extension/IEnvironmentService.h"
    "airborne_radar/extension/ISignalPipeline.h"
    "airborne_radar/extension/SignalPipelineResultTypes.h"
    "airborne_radar/airborne_radar.hpp"
    "electronic_surveillance_radar/common/EmitterHypothesis.h"
    "electronic_surveillance_radar/common/EmitterObservation.h"
    "electronic_surveillance_radar/common/EsrCoordinateUtils.h"
    "electronic_surveillance_radar/common/EmitterTruthState.h"
    "electronic_surveillance_radar/common/EsrOrientationConfig.h"
    "electronic_surveillance_radar/common/EsrOutputFrame.h"
    "electronic_surveillance_radar/core/context/EsrCycleInput.h"
    "electronic_surveillance_radar/core/context/EsrInputValidation.h"
    "electronic_surveillance_radar/core/controller/EsrController.h"
    "electronic_surveillance_radar/core/session/EsrCycleResult.h"
    "electronic_surveillance_radar/core/session/EsrSession.h"
    "electronic_surveillance_radar/tools/EsrTraceSession.h"
    "electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
    "electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
    "electronic_surveillance_radar/pipeline/IInterceptPipeline.h"
    "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
    "electronic_surveillance_radar/electronic_surveillance_radar.hpp"
    "electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
    "electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
    "electronic_surveillance_radar/config/electronic_surveillance_radar_config.hpp"
    "electro_optical_sensor/utils/EosCoordinateUtils.h"
    "electro_optical_sensor/output/EosOutputFrame.h"
    "electro_optical_sensor/foundation/EosRadiativeTransfer.h"
    "electro_optical_sensor/model/EosCycleInput.h"
    "electro_optical_sensor/model/EosInputValidation.h"
    "electro_optical_sensor/model/EosCycleResult.h"
    "electro_optical_sensor/session/EosSession.h"
    "electro_optical_sensor/session/EosTraceSession.h"
    "electro_optical_sensor/environment/EosEnvironmentTypes.h"
    "electro_optical_sensor/extension/EosController.h"
    "electro_optical_sensor/extension/EosPipelineTypes.h"
    "electro_optical_sensor/extension/IEosEnvironmentService.h"
    "electro_optical_sensor/extension/IEosPipeline.h"
    "electro_optical_sensor/extension/electro_optical_sensor_extension.hpp"
    "electro_optical_sensor/electro_optical_sensor.hpp"
    "electro_optical_sensor/config/EosSessionConfigBuilder.h"
    "electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
    "electro_optical_sensor/config/electro_optical_sensor_config.hpp"
    "api.hpp")

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
