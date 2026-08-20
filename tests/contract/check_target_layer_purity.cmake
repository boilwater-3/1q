# Target-processing layering purity guard (contract.md §目标处理分层契约 规则 1).
#
# 依赖方向单向：
#   - 传感器层（AR/ESR/EOS/SAR/SBIRS/RIR）不得 include 估计/推演/决策层头；
#   - 算法面（threat_assessment / target_inference）不得 include 传感器具体类型；
#   - fusion 核心不得 include 传感器具体类型或决策/推演层；
#     唯一豁免：SensorAdapters.h/.cpp（契约声明的官方适配触点）。

if(NOT DEFINED SOURCE_DIR OR SOURCE_DIR STREQUAL "")
  message(FATAL_ERROR "SOURCE_DIR must be provided")
endif()

get_filename_component(SOURCE_DIR "${SOURCE_DIR}" ABSOLUTE)

include("${CMAKE_CURRENT_LIST_DIR}/ReadSourceLines.cmake")

set(SENSOR_LAYER_RE "1q/(airborne_radar|electronic_surveillance_radar|electronic_countermeasure|electro_optical_sensor|sbirs_sensor|sar|remote_identification_radar)/")
set(ESTIMATION_LAYER_RE "1q/fusion/")
set(DECISION_LAYER_RE "1q/threat_assessment/")
set(INFERENCE_LAYER_RE "1q/target_inference/")

set(VIOLATIONS)

function(check_dir_forbidden_includes dir title forbidden_re)
    if(NOT IS_DIRECTORY "${dir}")
        return()
    endif()
    file(GLOB_RECURSE FILES "${dir}/*.h" "${dir}/*.hpp" "${dir}/*.cpp")
    foreach(FILE IN LISTS FILES)
        oneq_read_source_lines(INCLUDE_LINES "${FILE}")
        list(FILTER INCLUDE_LINES INCLUDE REGEX "^[ \t]*#include[ \t]+[\"<]1q/")
        foreach(LINE IN LISTS INCLUDE_LINES)
            if(LINE MATCHES "${forbidden_re}")
                list(APPEND VIOLATIONS "${FILE}: ${title} forbidden include -> ${LINE}")
            endif()
        endforeach()
    endforeach()
    set(VIOLATIONS ${VIOLATIONS} PARENT_SCOPE)
endfunction()

# 传感器层：不得依赖估计/推演/决策层。
foreach(SENSOR airborne_radar electronic_surveillance_radar electronic_countermeasure
             electro_optical_sensor sbirs_sensor sar remote_identification_radar)
    check_dir_forbidden_includes("${SOURCE_DIR}/src/${SENSOR}"
        "sensor layer" "${ESTIMATION_LAYER_RE}|${DECISION_LAYER_RE}|${INFERENCE_LAYER_RE}")
    check_dir_forbidden_includes("${SOURCE_DIR}/include/1q/${SENSOR}"
        "sensor layer" "${ESTIMATION_LAYER_RE}|${DECISION_LAYER_RE}|${INFERENCE_LAYER_RE}")
endforeach()

# 决策层：不感知传感器。
check_dir_forbidden_includes("${SOURCE_DIR}/src/threat_assessment"
    "decision layer" "${SENSOR_LAYER_RE}|${ESTIMATION_LAYER_RE}|${INFERENCE_LAYER_RE}")
check_dir_forbidden_includes("${SOURCE_DIR}/include/1q/threat_assessment"
    "decision layer" "${SENSOR_LAYER_RE}|${ESTIMATION_LAYER_RE}|${INFERENCE_LAYER_RE}")

# 推演层：不感知传感器。
check_dir_forbidden_includes("${SOURCE_DIR}/src/target_inference"
    "inference layer" "${SENSOR_LAYER_RE}|${ESTIMATION_LAYER_RE}|${DECISION_LAYER_RE}")
check_dir_forbidden_includes("${SOURCE_DIR}/include/1q/target_inference"
    "inference layer" "${SENSOR_LAYER_RE}|${ESTIMATION_LAYER_RE}|${DECISION_LAYER_RE}")

# 估计层（fusion 核心）：不感知传感器与决策/推演层；SensorAdapters 豁免。
file(GLOB_RECURSE FUSION_IMPL_FILES
     "${SOURCE_DIR}/src/fusion/*.h"
     "${SOURCE_DIR}/src/fusion/*.cpp"
     "${SOURCE_DIR}/include/1q/fusion/*.h")
foreach(FILE IN LISTS FUSION_IMPL_FILES)
    get_filename_component(FILE_NAME "${FILE}" NAME)
    if(FILE_NAME STREQUAL "SensorAdapters.h" OR FILE_NAME STREQUAL "SensorAdapters.cpp")
        continue()
    endif()
    oneq_read_source_lines(INCLUDE_LINES "${FILE}")
    list(FILTER INCLUDE_LINES INCLUDE REGEX "^[ \t]*#include[ \t]+[\"<]1q/")
    foreach(LINE IN LISTS INCLUDE_LINES)
        if(LINE MATCHES "${SENSOR_LAYER_RE}|${DECISION_LAYER_RE}|${INFERENCE_LAYER_RE}")
            list(APPEND VIOLATIONS "${FILE}: estimation core forbidden include -> ${LINE}")
        endif()
    endforeach()
endforeach()

if(VIOLATIONS)
  list(REMOVE_DUPLICATES VIOLATIONS)
  list(SORT VIOLATIONS)
  list(JOIN VIOLATIONS "\n" VIOLATION_TEXT)
  message(FATAL_ERROR
      "target-layer purity guard failed (contract.md §目标处理分层契约 规则 1):\n${VIOLATION_TEXT}")
endif()

message(STATUS "target_layer_purity_guard: layering include directions clean.")
