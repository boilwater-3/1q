#include "airborne_radar/session/ArDiagnosticUtils.h"

#include <utility>

#include "1q/airborne_radar/session/ArIssueCodes.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace session {

const char* AbortReasonToDiagnosticCode(SignalCycleAbortReason reason) {
  switch (reason) {
    case SignalCycleAbortReason::kNone:
      return "";
    case SignalCycleAbortReason::kLifecycleUnavailable:
      return "lifecycle_unavailable";
    case SignalCycleAbortReason::kInvalidEnvironmentCycle:
      return "invalid_environment_cycle";
    case SignalCycleAbortReason::kRuntimePreparationFailed:
      return "runtime_preparation_failed";
    case SignalCycleAbortReason::kValidationRejected:
      return "validation_rejected";
    case SignalCycleAbortReason::kSensorPoweredOff:
      return "sensor_powered_off";
  }
  return "unknown";
}

void RecordAbort(ArCycleResult* result, SignalCycleAbortReason reason,
                 const char* detail_code, const std::string& message) {
  result->abort_reason = reason;
  result->status = ArCycleStatus::kRejectedExecution;

  // 结构化诊断（细粒度；统一问题列表模型，规则 14：phase 由中止原因推导）。
  // SignalCycleAbortReason 无 kOutputContractViolation；kValidationRejected → kInputValidation，
  // 其余 → kExecution。detail_code 为 ArIssueCodes.h 完整 code 常量，调用方负责传注册表常量。
  ArIssue issue;
  issue.severity = ArIssueSeverity::kError;
  issue.phase = (reason == SignalCycleAbortReason::kValidationRejected)
                    ? ArIssuePhase::kInputValidation
                    : ArIssuePhase::kExecution;
  issue.code = detail_code;
  issue.message = message;
  result->issues.push_back(std::move(issue));

  // 中译：AR 中止记录（原因码 — 细粒度码 — 消息）。
  // 标识：三写之三（人读日志）——标识本周期中止的粗粒度原因与细粒度码，
  //       供排查中止路径；仅用于人读，不用于状态判断（规则 3）。
  PROJECT_LOG_ERROR("AR {}: {} — {}", AbortReasonToDiagnosticCode(reason), detail_code, message);
}

}  // namespace session
}  // namespace airborne_radar
