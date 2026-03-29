set(PUBLIC_INCLUDE_DIR "${SOURCE_DIR}/include/1q")

set(EXPECTED_PUBLIC_HEADERS
    "common/pose_types.h"
    "common/scan_schedule_types.h"
    "airborne_radar/common/utils/AntennaPatternUtils.h"
    "airborne_radar/common/control/ControlDirective.h"
    "airborne_radar/common/model/DecisionInputFrame.h"
    "airborne_radar/common/model/DecisionSourceInfo.h"
    "airborne_radar/common/model/DecisionTrackSnapshot.h"
    "airborne_radar/common/utils/JammingSemantics.h"
    "airborne_radar/common/utils/MathUtils.h"
    "airborne_radar/common/control/RadarCommand.h"
    "airborne_radar/common/control/RadarControlProfile.h"
    "airborne_radar/common/utils/RadarOrientationUtils.h"
    "airborne_radar/common/model/TargetCategory.h"
    "airborne_radar/common/model/TargetFeature.h"
    "airborne_radar/common/model/TargetFeatureBuilder.h"
    "airborne_radar/common/utils/TargetFeatureUtils.h"
    "airborne_radar/common/output/TrackOutputFrame.h"
    "airborne_radar/common/output/TrackOutputQueries.h"
    "airborne_radar/config/AntennaPatternConfig.h"
    "airborne_radar/config/ConfigPresets.h"
    "airborne_radar/config/RadarOrientationConfig.h"
    "airborne_radar/config/RadarSessionConfigBuilder.h"
    "airborne_radar/config/RadarWorkMode.h"
    "airborne_radar/core/context/IRadarContext.h"
    "airborne_radar/core/context/RadarCycleInput.h"
    "airborne_radar/core/context/RadarInputValidation.h"
    "airborne_radar/core/controller/IRadarOutputReader.h"
    "airborne_radar/core/controller/RadarController.h"
    "airborne_radar/core/session/RadarCycleResult.h"
    "airborne_radar/core/session/RadarSession.h"
    "airborne_radar/decision/pipeline/ControlReducerTypes.h"
    "airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"
    "airborne_radar/environment/EnvironmentSceneBuilder.h"
    "airborne_radar/environment/EnvironmentTypes.h"
    "airborne_radar/environment/IEnvironmentService.h"
    "airborne_radar/signal/detection/DetectionTypes.h"
    "airborne_radar/signal/pipeline/ISignalPipeline.h"
    "airborne_radar/signal/pipeline/SignalPipelineTypes.h"
    "electronic_surveillance_radar/common/EmitterHypothesis.h"
    "electronic_surveillance_radar/common/EmitterObservation.h"
    "electronic_surveillance_radar/common/EmitterTruthState.h"
    "electronic_surveillance_radar/common/EsrOrientationConfig.h"
    "electronic_surveillance_radar/common/EsrOutputFrame.h"
    "electronic_surveillance_radar/core/context/EsrCycleInput.h"
    "electronic_surveillance_radar/core/context/EsrInputValidation.h"
    "electronic_surveillance_radar/core/controller/EsrController.h"
    "electronic_surveillance_radar/core/session/EsrCycleResult.h"
    "electronic_surveillance_radar/core/session/EsrSession.h"
    "electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
    "electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
    "electronic_surveillance_radar/pipeline/IInterceptPipeline.h"
    "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
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
