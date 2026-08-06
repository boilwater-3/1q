#include "airborne_radar/session/ArDiagnosticUtils.h"

#include <utility>

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
                 const char* detail_code, const std::string& message, bool is_validation) {
  if (is_validation) {
    result->has_validation_error = true;
  }
  result->abort_reason = reason;
  result->status = is_validation ? ArCycleStatus::kRejectedInvalidInput
                                 : ArCycleStatus::kRejectedExecution;

  // 结构化诊断（细粒度）
  ArDiagnosticIssue issue;
  issue.severity = ArDiagnosticSeverity::kError;
  issue.code = std::string("ar.") + detail_code;
  issue.message = message;
  result->diagnostics.push_back(std::move(issue));

  // 中译：AR 中止记录（原因码 — 细粒度码 — 消息）。
  // 标识：三写之三（人读日志）——标识本周期中止的粗粒度原因与细粒度码，
  //       供排查中止路径；仅用于人读，不用于状态判断（规则 3）。
  PROJECT_LOG_ERROR("AR {}: {} — {}", AbortReasonToDiagnosticCode(reason), detail_code, message);
}

}  // namespace session
}  // namespace airborne_radar
