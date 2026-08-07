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
                 const char* detail_code, const std::string& message) {
  result->abort_reason = reason;
  result->status = reason == EosPipelineAbortReason::kValidationRejected
                       ? EosCycleStatus::kRejectedInvalidInput
                       : EosCycleStatus::kRejectedExecution;

  // 结构化诊断（细粒度；统一问题列表模型，规则 14：phase 由中止原因推导）
  EosIssue issue;
  issue.severity = EosIssueSeverity::kError;
  issue.phase = (reason == EosPipelineAbortReason::kValidationRejected)
                    ? EosIssuePhase::kInputValidation
                    : (reason == EosPipelineAbortReason::kOutputContractViolation)
                          ? EosIssuePhase::kOutputContract
                          : EosIssuePhase::kExecution;
  issue.code = std::string("eos.") + detail_code;
  issue.message = message;
  result->issues.push_back(std::move(issue));

  // 中译：EOS 中止记录（原因码 — 细粒度码 — 消息）。
  // 标识：三写之三（人读日志）——标识本周期中止的粗粒度原因与细粒度码，
  //       供排查中止路径；仅用于人读，不用于状态判断（规则 3）。
  PROJECT_LOG_ERROR("EOS {}: {} — {}", AbortReasonToDiagnosticCode(reason), detail_code, message);
}

}  // namespace session
}  // namespace electro_optical_sensor
