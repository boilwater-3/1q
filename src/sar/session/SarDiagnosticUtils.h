#ifndef ONEQ_SRC_SAR_SESSION_SAR_DIAGNOSTIC_UTILS_H_
#define ONEQ_SRC_SAR_SESSION_SAR_DIAGNOSTIC_UTILS_H_

#include <string>

#include "1q/sar/session/SarCycleResult.h"

namespace sar {
namespace session {

SarDiagnosticIssue MakeInfoDiagnostic(const char* code, const std::string& message);

void RecordAbort(SarCycleResult* result, const std::string& tag, const std::string& message);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_DIAGNOSTIC_UTILS_H_

