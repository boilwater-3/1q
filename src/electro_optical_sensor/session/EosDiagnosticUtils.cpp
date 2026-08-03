#include "electro_optical_sensor/session/EosDiagnosticUtils.h"

#include <utility>

#include "common/logging/ProjectLog.h"

namespace electro_optical_sensor {
namespace session {

const char* AbortReasonToDiagnosticCode(EosPipelineAbortReason reason) {
  switch (reason) {
    case EosPipelineAbortReason::kNone:
      return "";
    case EosPipelineAbortReason::kValidationRejected:
      return "validation_rejected";
    case EosPipelineAbortReason::kOutputContractViolation:
      return "output_contract_violation";
    case EosPipelineAbortReason::kRuntimeStateRestoreRejected:
      return "runtime_state_restore_rejected";
    case EosPipelineAbortReason::kSensorPoweredOff:
      return "sensor_powered_off";
  }
  return "unknown";
}

void RecordAbort(EosCycleResult* result, EosPipelineAbortReason reason,
                 const char* detail_code, const std::string& message, bool is_validation) {
  if (is_validation) {
    result->has_validation_error = true;
  }
  result->abort_reason = reason;
  result->status = is_validation ? EosCycleStatus::kRejectedInvalidInput
                                 : EosCycleStatus::kRejectedExecution;

  // 结构化诊断（细粒度）
  EosDiagnosticIssue issue;
  issue.severity = EosDiagnosticSeverity::kError;
  issue.code = std::string("eos.") + detail_code;
  issue.message = message;
  result->diagnostics.push_back(std::move(issue));

  // 人读日志
  PROJECT_LOG_ERROR("EOS {}: {} — {}", AbortReasonToDiagnosticCode(reason), detail_code, message);
}

}  // namespace session
}  // namespace electro_optical_sensor
