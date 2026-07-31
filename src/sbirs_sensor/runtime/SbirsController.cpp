#include "sbirs_sensor/runtime/SbirsController.h"

#include "1q/sbirs_sensor/session/SbirsInputValidation.h"

namespace sbirs_sensor {
namespace runtime {

SbirsController::SbirsController(const config::SbirsInternalExecutionConfig& config)
    : pipeline_(config), frame_rate_hz_(config.session.mission.frame_rate_hz) {}

void SbirsController::ApplyConfig(const config::SbirsInternalExecutionConfig& config,
                                  const SbirsRuntimeConfigImpact& impact) {
  pipeline_.ApplyConfig(config, impact);
}

session::SbirsCycleResult SbirsController::RunOnce(const session::SbirsCycleInput& input) {
  session::SbirsCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.validation_issues = session::ValidateSbirsCycleInput(input, frame_rate_hz_);
  result.has_validation_error = session::HasValidationError(result.validation_issues);
  if (result.has_validation_error) {
    result.abort_reason = session::SbirsPipelineAbortReason::kValidationRejected;
    if (has_latest_output_) {
      result.output_frame = latest_output_;
      result.reused_previous_output = true;
    }
    return result;
  }

  const pipeline::SbirsPipelineResult pipeline_result = pipeline_.RunCycle(input);
  result.output_frame.cycle_index = input.cycle_index;
  result.output_frame.scan_azimuth_deg = pipeline_result.scan_azimuth_deg;
  // raw output 仅进 detected==true 的 record；失败诊断 attribution 仍保留进 result 层。
  for (const pipeline::SbirsPipelineDetection& detection : pipeline_result.detections) {
    result.detection_attributions.push_back(detection.attribution);
    if (!detection.record.detected) {
      continue;
    }
    result.output_frame.detections.push_back(detection.record);
  }
  result.executed_this_cycle = true;
  result.abort_reason = session::SbirsPipelineAbortReason::kNone;
  latest_output_ = result.output_frame;
  has_latest_output_ = true;
  return result;
}

}  // namespace runtime
}  // namespace sbirs_sensor
