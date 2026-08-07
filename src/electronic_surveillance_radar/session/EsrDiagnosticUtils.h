/**
 * @file EsrDiagnosticUtils.h
 * @brief ESR 单周期诊断条目构造与中止记录工具函数。
 */

#ifndef ONEQ_SRC_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_DIAGNOSTIC_UTILS_H_
#define ONEQ_SRC_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_DIAGNOSTIC_UTILS_H_

#include <string>

#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief 将 EsrPipelineAbortReason 粗粒度枚举转换为诊断码字符串。
 */
const char* AbortReasonToDiagnosticCode(EsrPipelineAbortReason reason);

/**
 * @brief 将本周期标记为中止（三写：abort_reason + issues + 日志）。
 * @param[in] reason 粗粒度中止原因；phase 由原因推导（kValidationRejected → kInputValidation，
 *            其余 → kExecution）。
 */
void RecordAbort(EsrCycleResult* result, EsrPipelineAbortReason reason,
                 const char* detail_code, const std::string& message);

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_SRC_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_DIAGNOSTIC_UTILS_H_
