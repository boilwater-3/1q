/**
 * @file SarDiagnosticUtils.h
 * @brief SAR 单周期诊断条目构造与中止记录工具函数。
 */

#ifndef ONEQ_SRC_SAR_SESSION_SAR_DIAGNOSTIC_UTILS_H_
#define ONEQ_SRC_SAR_SESSION_SAR_DIAGNOSTIC_UTILS_H_

#include <string>

#include "1q/sar/session/SarCycleResult.h"

namespace sar {
namespace session {

/**
 * @brief 将 SarPipelineAbortReason 粗粒度枚举转换为诊断码字符串。
 * @param[in] reason 中止原因枚举。
 * @return 对应的粗粒度诊断码字符串（如 "pipeline_execution_failed"）。
 */
const char* AbortReasonToDiagnosticCode(SarPipelineAbortReason reason);

/**
 * @brief 构造 kInfo 级执行诊断条目（phase = kExecution）。
 * @param[in] code 诊断码（如 "sar.example"）。
 * @param[in] message 诊断描述。
 * @return kInfo 严重级的 SarIssue。
 */
SarIssue MakeInfoDiagnostic(const char* code, const std::string& message);

/**
 * @brief 构造 kWarning 级执行诊断条目（phase = kExecution）。
 * @param[in] code 诊断码（如 "sar.example"）。
 * @param[in] message 诊断描述。
 * @return kWarning 严重级的 SarIssue。
 */
SarIssue MakeWarningDiagnostic(const char* code, const std::string& message);

/**
 * @brief 将本周期标记为中止（三写：abort_reason + issues + 日志）。
 * @param[out] result 单周期结果。
 * @param[in] reason 粗粒度中止原因枚举。
 * @param[in] detail_code 细粒度诊断码（如 "snr_below_minimum"），写入 issues。
 * @param[in] message 中止描述。
 * @note 规则 14b：issue 的 phase 由中止原因推导 —— kValidationRejected /
 *       kExternalInputRejected → kInputValidation，其余 → kExecution。
 */
void RecordAbort(SarCycleResult* result, SarPipelineAbortReason reason,
                 const char* detail_code, const std::string& message);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_DIAGNOSTIC_UTILS_H_
