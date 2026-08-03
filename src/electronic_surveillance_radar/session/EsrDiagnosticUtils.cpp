#include "electronic_surveillance_radar/session/EsrDiagnosticUtils.h"

#include <utility>

#include "common/logging/ProjectLog.h"

namespace electronic_surveillance_radar {
namespace session {

const char* AbortReasonToDiagnosticCode(EsrPipelineAbortReason reason) {
  switch (reason) {
    case EsrPipelineAbortReason::kNone:
      return "";
    case EsrPipelineAbortReason::kValidationRejected:
      return "validation_rejected";
    case EsrPipelineAbortReason::kRuntimeStateRestoreRejected:
      return "runtime_state_restore_rejected";
    case EsrPipelineAbortReason::kOutputContractViolation:
      return "output_contract_violation";
    case EsrPipelineAbortReason::kSensorPoweredOff:
      return "sensor_powered_off";
    case EsrPipelineAbortReason::kRfReceiverRejected:
      return "rf_receiver_rejected";
  }
  return "unknown";
}

void RecordAbort(EsrCycleResult* result, EsrPipelineAbortReason reason,
                 const char* detail_code, const std::string& message, bool is_validation) {
  if (is_validation) {
    result->has_validation_error = true;
  }
  result->abort_reason = reason;
  result->status = is_validation ? EsrCycleExecutionStatus::kRejected
                                 : EsrCycleExecutionStatus::kRejected;

  // 结构化诊断（细粒度）
  EsrDiagnosticIssue issue;
  issue.severity = EsrDiagnosticSeverity::kError;
  issue.code = std::string("esr.") + detail_code;
  issue.message = message;
  result->diagnostics.push_back(std::move(issue));

  // 人读日志
  PROJECT_LOG_ERROR("ESR {}: {} — {}", AbortReasonToDiagnosticCode(reason), detail_code, message);
}

}  // namespace session
}  // namespace electronic_surveillance_radar
