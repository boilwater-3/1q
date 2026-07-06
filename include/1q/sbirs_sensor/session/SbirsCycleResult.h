/**
 * @file SbirsCycleResult.h
 * @brief 定义 SBIRS-inspired 单周期执行结果。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_RESULT_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_RESULT_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/sbirs_sensor/session/SbirsInputValidation.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"

namespace sbirs_sensor {
namespace session {

struct ONEQ_API SbirsOutputFrame {
  std::uint32_t cycle_index{0U};
  float scan_azimuth_deg{0.0f};
  output::SbirsDetectionRecordList detections{};
};

struct ONEQ_API SbirsCycleResult {
  std::uint32_t input_cycle_index{0U};
  SbirsOutputFrame output_frame{};
  attribution::SbirsDetectionAttributionRecordList detection_attributions{};
  ValidationIssueList validation_issues{};
  bool has_validation_error{false};
  bool executed_this_cycle{false};
  bool reused_previous_output{false};
  SbirsPipelineAbortReason abort_reason{SbirsPipelineAbortReason::kNone};
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_CYCLE_RESULT_H_
