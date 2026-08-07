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
                 const char* detail_code, const std::string& message) {
  result->abort_reason = reason;
  result->status = EsrCycleExecutionStatus::kRejected;

  // 结构化诊断（细粒度；统一问题列表模型，规则 14：phase 由中止原因推导）
  EsrIssue issue;
  issue.severity = EsrIssueSeverity::kError;
  issue.phase = (reason == EsrPipelineAbortReason::kValidationRejected)
                    ? EsrIssuePhase::kInputValidation
                    : (reason == EsrPipelineAbortReason::kOutputContractViolation)
                          ? EsrIssuePhase::kOutputContract
                          : EsrIssuePhase::kExecution;
  issue.code = std::string("esr.") + detail_code;
  issue.message = message;
  result->issues.push_back(std::move(issue));

  // 中译：ESR 中止记录（原因码 — 细粒度码 — 消息）。
  // 标识：三写之三（人读日志）——标识本周期中止的粗粒度原因与细粒度码，
  //       供排查中止路径；仅用于人读，不用于状态判断（规则 3）。
  PROJECT_LOG_ERROR("ESR {}: {} — {}", AbortReasonToDiagnosticCode(reason), detail_code, message);
}

}  // namespace session
}  // namespace electronic_surveillance_radar
