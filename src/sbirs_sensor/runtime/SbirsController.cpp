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
  for (const pipeline::SbirsPipelineDetection& detection : pipeline_result.detections) {
    if (!detection.record.detected) {
      continue;
    }
    result.output_frame.detections.push_back(detection.record);
    result.detection_attributions.push_back(detection.attribution);
  }
  if (result.output_frame.detections.size() != result.detection_attributions.size()) {
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
