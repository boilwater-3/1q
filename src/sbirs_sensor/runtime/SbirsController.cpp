#include "sbirs_sensor/runtime/SbirsController.h"

#include "1q/sbirs_sensor/session/SbirsInputValidation.h"

namespace sbirs_sensor {
namespace runtime {

SbirsController::SbirsController(const config::SbirsInternalExecutionConfig& config)
    : pipeline_(config) {}

void SbirsController::ApplyConfig(const config::SbirsInternalExecutionConfig& config) {
  pipeline_.ApplyConfig(config);
}

session::SbirsCycleResult SbirsController::RunOnce(const session::SbirsCycleInput& input) {
  session::SbirsCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.validation_issues = session::ValidateSbirsCycleInput(input);
  result.has_validation_error = session::HasValidationError(result.validation_issues);
  if (result.has_validation_error) {
    result.abort_reason = session::SbirsPipelineAbortReason::kValidationRejected;
    if (has_latest_output_) {
      result.output_frame = latest_output_;
      result.reused_previous_output = true;
    }
    return result;
  }

  const pipeline::SbirsPipelineSnapshot snapshot = pipeline_.CaptureRuntimeState();
  const pipeline::SbirsPipelineResult pipeline_result = pipeline_.RunCycle(input);
  result.output_frame.cycle_index = input.cycle_index;
  result.output_frame.scan_azimuth_deg = pipeline_result.scan_azimuth_deg;
  // raw output 仅进 detected==true 的 record；失败诊断 attribution 仍保留进 result 层。
  std::size_t detected_attribution_count = 0U;
  for (const pipeline::SbirsPipelineDetection& detection : pipeline_result.detections) {
    result.detection_attributions.push_back(detection.attribution);
    if (!detection.record.detected) {
      continue;
    }
    result.output_frame.detections.push_back(detection.record);
    ++detected_attribution_count;
  }
  // 契约：每条 raw detection 必有对应 attribution（按 detected 对齐，失败诊断不计入）。
  if (result.output_frame.detections.size() != detected_attribution_count) {
    pipeline_.RestoreRuntimeState(snapshot);
    result.abort_reason = session::SbirsPipelineAbortReason::kOutputContractViolation;
    return result;
  }
  result.executed_this_cycle = true;
  result.abort_reason = session::SbirsPipelineAbortReason::kNone;
  latest_output_ = result.output_frame;
  has_latest_output_ = true;
  return result;
}

}  // namespace runtime
}  // namespace sbirs_sensor
