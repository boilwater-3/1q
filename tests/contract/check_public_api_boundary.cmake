set(PUBLIC_INCLUDE_DIR "${SOURCE_DIR}/include/1q")

set(EXPECTED_PUBLIC_HEADERS
    "airborne_radar/airborne_radar.hpp"
    "airborne_radar/config/RadarDetailedSessionConfigBuilder.h"
    "airborne_radar/config/RadarRuntimeConfigBuilder.h"
    "airborne_radar/config/RadarSessionConfig.h"
    "airborne_radar/config/RadarSessionConfigBuilder.h"
    "airborne_radar/config/PipelineConfig.h"
    "airborne_radar/config/airborne_radar_config.hpp"
    "airborne_radar/config/expert/ExpertPipelineConfig.h"
    "airborne_radar/config/expert/beam/BeamControlConfig.h"
    "airborne_radar/config/expert/beam/BeamPointingConfig.h"
    "airborne_radar/config/expert/beam/BeamSchedulerConfig.h"
    "airborne_radar/config/expert/detection/AntennaConfig.h"
    "airborne_radar/config/expert/detection/AntennaPatternConfig.h"
    "airborne_radar/config/expert/detection/DetectionConfig.h"
    "airborne_radar/config/expert/detection/DetectionPolicyConfig.h"
    "airborne_radar/config/expert/detection/RcsPhysicsConfig.h"
    "airborne_radar/config/expert/detection/ReceiverConfig.h"
    "airborne_radar/config/expert/detection/TransmitterConfig.h"
    "airborne_radar/config/expert/lifecycle/ImmConfig.h"
    "airborne_radar/config/expert/lifecycle/LifecycleConfig.h"
    "airborne_radar/config/expert/tracking/AssociationConfig.h"
    "airborne_radar/config/expert/tracking/KalmanConfig.h"
    "airborne_radar/config/expert/tracking/TrackingConfig.h"
    "airborne_radar/config/presets/RadarSessionConfigPresets.h"
    "airborne_radar/config/presets/PipelineConfigPresets.h"
    "airborne_radar/config/semantic/AntennaProfiles.h"
    "airborne_radar/config/semantic/DetectionProfiles.h"
    "airborne_radar/config/semantic/LifecycleProfiles.h"
    "airborne_radar/config/semantic/TrackingProfiles.h"
    "airborne_radar/environment/EnvironmentConfig.h"
    "airborne_radar/environment/EnvironmentDefaultConfigBuilder.h"
    "airborne_radar/environment/EnvironmentRuntimeConfigPatch.h"
    "airborne_radar/environment/EnvironmentRuntimeConfigPatchBuilder.h"
    "airborne_radar/environment/EnvironmentSceneBuilder.h"
    "airborne_radar/environment/EnvironmentTypes.h"
    "airborne_radar/environment/IEnvironmentService.h"
    "airborne_radar/environment/airborne_radar_environment.hpp"
    "airborne_radar/extension/ControlReducerTypes.h"
    "airborne_radar/extension/IRadarContext.h"
    "airborne_radar/extension/IRadarOutputReader.h"
    "airborne_radar/extension/ISignalPipeline.h"
    "airborne_radar/extension/ITacticalDecisionEngine.h"
    "airborne_radar/extension/RadarController.h"
    "airborne_radar/extension/SignalPipelineResultTypes.h"
    "airborne_radar/extension/airborne_radar_extension.hpp"
    "airborne_radar/extension/control/ControlDirective.h"
    "airborne_radar/extension/control/RadarCommand.h"
    "airborne_radar/extension/control/RadarControlProfile.h"
    "airborne_radar/model/DecisionInputFrame.h"
    "airborne_radar/model/DecisionSourceInfo.h"
    "airborne_radar/model/DecisionTrackSnapshot.h"
    "airborne_radar/model/JammingSemantics.h"
    "airborne_radar/model/RadarOrientationConfig.h"
    "airborne_radar/model/TargetCategory.h"
    "airborne_radar/model/TargetFeature.h"
    "airborne_radar/model/TargetFeatureBuilder.h"
    "airborne_radar/model/TargetFeatureUtils.h"
    "airborne_radar/output/TrackOutputFrame.h"
    "airborne_radar/output/TrackOutputQueries.h"
    "airborne_radar/session/RadarCycleInput.h"
    "airborne_radar/session/RadarCycleResult.h"
    "airborne_radar/session/RadarExternalInputAdapter.h"
    "airborne_radar/session/RadarInputValidation.h"
    "airborne_radar/session/RadarSession.h"
    "airborne_radar/session/RadarSessionFactory.h"
    "airborne_radar/session/RadarTraceSession.h"
    "api.hpp"
    "electro_optical_sensor/config/EosDetailedSessionConfigBuilder.h"
    "electro_optical_sensor/config/EosEnvironmentConfig.h"
    "electro_optical_sensor/config/EosHardwareConfig.h"
    "electro_optical_sensor/config/EosMissionConfig.h"
    "electro_optical_sensor/config/EosPolicyConfig.h"
    "electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
    "electro_optical_sensor/config/EosRuntimeConfigPatch.h"
    "electro_optical_sensor/config/EosSessionConfig.h"
    "electro_optical_sensor/config/EosSessionConfigBuilder.h"
    "electro_optical_sensor/config/EosWorkMode.h"
    "electro_optical_sensor/config/electro_optical_sensor_config.hpp"
    "electro_optical_sensor/electro_optical_sensor.hpp"
    "electro_optical_sensor/environment/EosEnvironmentConfig.h"
    "electro_optical_sensor/environment/EosEnvironmentConfigBuilder.h"
    "electro_optical_sensor/environment/EosEnvironmentRuntimeConfigPatch.h"
    "electro_optical_sensor/environment/EosEnvironmentRuntimeConfigPatchBuilder.h"
    "electro_optical_sensor/environment/EosEnvironmentTypes.h"
    "electro_optical_sensor/environment/IEosEnvironmentService.h"
    "electro_optical_sensor/environment/electro_optical_sensor_environment.hpp"
    "electro_optical_sensor/extension/EosController.h"
    "electro_optical_sensor/extension/EosPipelineTypes.h"
    "electro_optical_sensor/extension/IEosPipeline.h"
    "electro_optical_sensor/extension/electro_optical_sensor_extension.hpp"
    "electro_optical_sensor/foundation/EosRadiativeTransfer.h"
    "electro_optical_sensor/model/EosCycleInput.h"
    "electro_optical_sensor/model/EosCycleResult.h"
    "electro_optical_sensor/model/EosInputValidation.h"
    "electro_optical_sensor/output/EosOutputFrame.h"
    "electro_optical_sensor/session/EosExternalInputAdapter.h"
    "electro_optical_sensor/session/EosSession.h"
    "electro_optical_sensor/session/EosTraceSession.h"
    "electronic_surveillance_radar/config/EsrDetectionPolicyConfig.h"
    "electronic_surveillance_radar/config/EsrEnvironmentPolicyConfig.h"
    "electronic_surveillance_radar/config/EsrHardwareConfig.h"
    "electronic_surveillance_radar/config/EsrMissionControlConfig.h"
    "electronic_surveillance_radar/config/EsrPolicyConfig.h"
    "electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
    "electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
    "electronic_surveillance_radar/config/EsrScanPolicyConfig.h"
    "electronic_surveillance_radar/config/EsrSessionConfig.h"
    "electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
    "electronic_surveillance_radar/config/EsrWorkMode.h"
    "electronic_surveillance_radar/config/electronic_surveillance_radar_config.hpp"
    "electronic_surveillance_radar/electronic_surveillance_radar.hpp"
    "electronic_surveillance_radar/environment/EsrEnvironmentConfig.h"
    "electronic_surveillance_radar/environment/EsrEnvironmentConfigBuilder.h"
    "electronic_surveillance_radar/environment/EsrEnvironmentRuntimeConfigPatch.h"
    "electronic_surveillance_radar/environment/EsrEnvironmentRuntimeConfigPatchBuilder.h"
    "electronic_surveillance_radar/environment/EsrEnvironmentSceneBuilder.h"
    "electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
    "electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
    "electronic_surveillance_radar/environment/electronic_surveillance_radar_environment.hpp"
    "electronic_surveillance_radar/extension/EsrController.h"
    "electronic_surveillance_radar/extension/IEsrContext.h"
    "electronic_surveillance_radar/extension/IInterceptPipeline.h"
    "electronic_surveillance_radar/extension/InterceptPipelineTypes.h"
    "electronic_surveillance_radar/model/EmitterHypothesis.h"
    "electronic_surveillance_radar/model/EmitterObservation.h"
    "electronic_surveillance_radar/model/EmitterTruthState.h"
    "electronic_surveillance_radar/model/EsrOrientationConfig.h"
    "electronic_surveillance_radar/output/EsrOutputFrame.h"
    "electronic_surveillance_radar/session/EsrCycleInput.h"
    "electronic_surveillance_radar/session/EsrCycleResult.h"
    "electronic_surveillance_radar/session/EsrExternalInputAdapter.h"
    "electronic_surveillance_radar/session/EsrInputValidation.h"
    "electronic_surveillance_radar/session/EsrSession.h"
    "electronic_surveillance_radar/session/EsrSessionFactory.h"
    "electronic_surveillance_radar/session/EsrTraceSession.h"
    "foundation/atmospheric_types.h"
    "foundation/coordinate_transform.h"
    "foundation/pose_types.h"
    "foundation/scan_schedule_types.h"
    "trace/TraceSink.h"
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
