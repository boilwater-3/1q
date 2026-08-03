#include "sar/session/SarDiagnosticUtils.h"

#include <utility>

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

SarDiagnosticIssue MakeInfoDiagnostic(const char* code, const std::string& message) {
  SarDiagnosticIssue issue;
  issue.severity = SarDiagnosticSeverity::kInfo;
  issue.code = code;
  issue.message = message;
  return issue;
}

SarDiagnosticIssue MakeWarningDiagnostic(const char* code, const std::string& message) {
  SarDiagnosticIssue issue;
  issue.severity = SarDiagnosticSeverity::kWarning;
  issue.code = code;
  issue.message = message;
  return issue;
}

namespace {

void WriteAbort(SarCycleResult* result, SarPipelineAbortReason reason,
                const char* detail_code, const std::string& message, bool is_validation) {
  result->has_error = true;
  result->abort_reason = reason;
  result->status = is_validation ? SarCycleStatus::kRejectedInvalidInput
                                 : SarCycleStatus::kRejectedExecution;

  // 结构化诊断（细粒度，供 DebugView 消费）
  SarDiagnosticIssue issue;
  issue.severity = SarDiagnosticSeverity::kError;
  issue.code = std::string("sar.") + detail_code;
  issue.message = message;
  result->diagnostics.push_back(std::move(issue));

  // 人读日志（不用于状态判断）
  PROJECT_LOG_ERROR("SAR {}: {} — {}", AbortReasonToDiagnosticCode(reason), detail_code, message);
}

}  // namespace

void RecordAbort(SarCycleResult* result, SarPipelineAbortReason reason,
                 const char* detail_code, const std::string& message) {
  WriteAbort(result, reason, detail_code, message, /*is_validation=*/false);
}

void RecordValidationAbort(SarCycleResult* result, SarPipelineAbortReason reason,
                           const char* detail_code, const std::string& message) {
  WriteAbort(result, reason, detail_code, message, /*is_validation=*/true);
}

}  // namespace session
}  // namespace sar
