#include "sar/session/SarDiagnosticUtils.h"

#include <utility>

#include "1q/sar/session/SarIssueCodes.h"
#include "common/logging/ProjectLog.h"

namespace sar {
namespace session {

const char* AbortReasonToDiagnosticCode(SarPipelineAbortReason reason) {
  switch (reason) {
    case SarPipelineAbortReason::kNone:
      return "";
    case SarPipelineAbortReason::kValidationRejected:
      return "validation_rejected";
    case SarPipelineAbortReason::kPipelineExecutionFailed:
      return "pipeline_execution_failed";
    case SarPipelineAbortReason::kExternalInputRejected:
      return "external_input_rejected";
    case SarPipelineAbortReason::kRuntimeStateRestoreRejected:
      return "runtime_state_restore_rejected";
    case SarPipelineAbortReason::kSensorPoweredOff:
      return "sensor_powered_off";
  }
  return "unknown";
}

SarIssue MakeInfoDiagnostic(const char* code, const std::string& message) {
  SarIssue issue;
  issue.severity = SarIssueSeverity::kInfo;
  issue.phase = SarIssuePhase::kExecution;
  issue.code = code;
  issue.message = message;
  return issue;
}

SarIssue MakeWarningDiagnostic(const char* code, const std::string& message) {
  SarIssue issue;
  issue.severity = SarIssueSeverity::kWarning;
  issue.phase = SarIssuePhase::kExecution;
  issue.code = code;
  issue.message = message;
  return issue;
}

namespace {

// 规则 14b：issue 的 phase 由中止原因推导（kValidationRejected /
// kExternalInputRejected 属输入校验阶段，其余为执行阶段）。
SarIssuePhase PhaseForAbortReason(SarPipelineAbortReason reason) {
  switch (reason) {
    case SarPipelineAbortReason::kValidationRejected:
    case SarPipelineAbortReason::kExternalInputRejected:
      return SarIssuePhase::kInputValidation;
    default:
      return SarIssuePhase::kExecution;
  }
}

void WriteAbort(SarCycleResult* result, SarPipelineAbortReason reason,
                const char* detail_code, const std::string& message) {
  result->abort_reason = reason;
  result->status = (reason == SarPipelineAbortReason::kValidationRejected ||
                    reason == SarPipelineAbortReason::kExternalInputRejected)
                       ? SarCycleStatus::kRejectedInvalidInput
                       : SarCycleStatus::kRejectedExecution;

  // 结构化诊断（细粒度；统一问题列表模型，规则 14：phase 由中止原因推导；
  // detail_code 为 SarIssueCodes.h 完整 code 常量，调用方负责传注册表常量）
  SarIssue issue;
  issue.severity = SarIssueSeverity::kError;
  issue.phase = PhaseForAbortReason(reason);
  issue.code = detail_code;
  issue.message = message;
  result->issues.push_back(std::move(issue));

  // 中译：SAR 中止记录（原因码 — 细粒度码 — 消息）。
  // 标识：三写之三（人读日志）——标识本周期中止的粗粒度原因与细粒度码，
  //       供排查中止路径；仅用于人读，不用于状态判断（规则 3）。
  PROJECT_LOG_ERROR("SAR {}: {} — {}", AbortReasonToDiagnosticCode(reason), detail_code, message);
}

}  // namespace

void RecordAbort(SarCycleResult* result, SarPipelineAbortReason reason,
                 const char* detail_code, const std::string& message) {
  WriteAbort(result, reason, detail_code, message);
}

}  // namespace session
}  // namespace sar
