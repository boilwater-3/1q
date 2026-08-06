#include "sbirs_sensor/runtime/SbirsController.h"

#include "1q/sbirs_sensor/session/SbirsInputValidation.h"
#include "sbirs_sensor/session/SbirsDiagnosticUtils.h"

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
    session::RecordAbort(&result, session::SbirsPipelineAbortReason::kValidationRejected,
                         "input_validation", "SBIRS input validation failed.",
                         /*is_validation=*/true);
    return result;
  }

  const pipeline::SbirsPipelineResult pipeline_result = pipeline_.RunCycle(input);

  if (!pipeline_result.executed) {
    session::RecordAbort(&result, session::SbirsPipelineAbortReason::kSensorPoweredOff,
                         "sensor_powered_off", "SBIRS sensor is powered off or in standby mode.",
                         /*is_validation=*/false);
    result.executed_this_cycle = false;
    result.status = session::SbirsCycleStatus::kPoweredOff;
    result.output_frame.cycle_index = input.cycle_index;
    result.output_frame.scan_azimuth_deg = pipeline_result.scan_azimuth_deg;
    return result;
  }

  result.output_frame.cycle_index = input.cycle_index;
  result.output_frame.scan_azimuth_deg = pipeline_result.scan_azimuth_deg;
  // 规则 13b：正常执行周期按目标排除的 kInfo 诊断转写进结构化结果（abort 路径不变）。
  result.diagnostics = pipeline_result.diagnostics;
  // raw output 仅进 detected==true 的 record；失败诊断 attribution 仍保留进 result 层。
  for (const pipeline::SbirsPipelineDetection& detection : pipeline_result.detections) {
    result.detection_attributions.push_back(detection.attribution);
    if (!detection.record.detected) {
      continue;
    }
    result.output_frame.detections.push_back(detection.record);
  }
  result.executed_this_cycle = true;
  result.status = session::SbirsCycleStatus::kCompleted;
  result.abort_reason = session::SbirsPipelineAbortReason::kNone;
  return result;
}

}  // namespace runtime
}  // namespace sbirs_sensor
