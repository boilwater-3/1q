#include "sar/session/SarDiagnosticUtils.h"

#include <utility>

namespace sar {
namespace session {

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

void RecordAbort(SarCycleResult* result, const std::string& tag, const std::string& message) {
  result->has_error = true;
  result->abort_reason = tag;
  SarDiagnosticIssue issue;
  issue.severity = SarDiagnosticSeverity::kError;
  issue.code = "sar." + tag;
  issue.message = message;
  result->diagnostics.push_back(std::move(issue));
}

}  // namespace session
}  // namespace sar

