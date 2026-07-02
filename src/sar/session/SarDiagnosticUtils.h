#ifndef ONEQ_SRC_SAR_SESSION_SAR_DIAGNOSTIC_UTILS_H_
#define ONEQ_SRC_SAR_SESSION_SAR_DIAGNOSTIC_UTILS_H_

#include <string>

#include "1q/sar/session/SarCycleResult.h"

namespace sar {
namespace session {

SarDiagnosticIssue MakeInfoDiagnostic(const char* code, const std::string& message);

// kWarning 级诊断：用于物理可行但需关注的退化（如回波 clipping、斜距与标称值严重
// 错配）。不阻断当周期执行，但会被 ApplyDiagnosticsPolicy 在 enable_diagnostics=true
// 时保留（kInfo 也会保留），让退化从正常诊断里浮出来。
SarDiagnosticIssue MakeWarningDiagnostic(const char* code, const std::string& message);

void RecordAbort(SarCycleResult* result, const std::string& tag, const std::string& message);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_DIAGNOSTIC_UTILS_H_

