#include "sbirs_sensor/session/SbirsDiagnosticUtils.h"

#include <utility>

#include "common/logging/ProjectLog.h"

namespace sbirs_sensor {
namespace session {

const char* AbortReasonToDiagnosticCode(SbirsPipelineAbortReason reason) {
  switch (reason) {
    case SbirsPipelineAbortReason::kNone:
      return "";
    case SbirsPipelineAbortReason::kValidationRejected:
      return "validation_rejected";
    case SbirsPipelineAbortReason::kSensorPoweredOff:
      return "sensor_powered_off";
  }
  return "unknown";
}

void RecordAbort(SbirsCycleResult* result, SbirsPipelineAbortReason reason,
                 const char* detail_code, const std::string& message, bool is_validation) {
  if (is_validation) {
    result->has_validation_error = true;
  }
  result->abort_reason = reason;
  result->status = is_validation ? SbirsCycleStatus::kRejectedInvalidInput
                                 : SbirsCycleStatus::kRejectedExecution;

  // 结构化诊断（细粒度）
  SbirsDiagnosticIssue issue;
  issue.severity = SbirsDiagnosticSeverity::kError;
  issue.code = std::string("sbirs.") + detail_code;
  issue.message = message;
  result->diagnostics.push_back(std::move(issue));

  // 人读日志
  PROJECT_LOG_ERROR("SBIRS {}: {} — {}", AbortReasonToDiagnosticCode(reason), detail_code, message);
}

}  // namespace session
}  // namespace sbirs_sensor
