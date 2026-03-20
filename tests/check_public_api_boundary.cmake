set(PUBLIC_INCLUDE_DIR "${SOURCE_DIR}/include/1q")

set(EXPECTED_PUBLIC_HEADERS
    "airborne_radar/common/AntennaPatternConfig.h"
    "airborne_radar/common/AntennaPatternUtils.h"
    "airborne_radar/common/ConfigPresets.h"
    "airborne_radar/common/ControlDirective.h"
    "airborne_radar/common/DecisionInputFrame.h"
    "airborne_radar/common/DecisionSourceInfo.h"
    "airborne_radar/common/DecisionTrackSnapshot.h"
    "airborne_radar/common/JammingSemantics.h"
    "airborne_radar/common/MathUtils.h"
    "airborne_radar/common/RadarCommand.h"
    "airborne_radar/common/RadarControlProfile.h"
    "airborne_radar/common/RadarOrientationConfig.h"
    "airborne_radar/common/RadarOrientationUtils.h"
    "airborne_radar/common/TargetCategory.h"
    "airborne_radar/common/TargetFeature.h"
    "airborne_radar/common/TargetFeatureBuilder.h"
    "airborne_radar/common/TargetFeatureUtils.h"
    "airborne_radar/common/TrackOutputFrame.h"
    "airborne_radar/core/context/IRadarContext.h"
    "airborne_radar/core/context/RadarCycleInput.h"
    "airborne_radar/core/context/RadarInputValidation.h"
    "airborne_radar/core/controller/RadarController.h"
    "airborne_radar/core/output/IRadarOutputReader.h"
    "airborne_radar/core/output/TrackOutputQueries.h"
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
