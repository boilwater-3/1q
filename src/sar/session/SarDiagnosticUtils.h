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
 * @brief 构造 kInfo 级诊断条目。
 * @param[in] code 诊断码（如 "sar.example"）。
 * @param[in] message 诊断描述。
 * @return kInfo 严重级的 SarDiagnosticIssue。
 */
SarDiagnosticIssue MakeInfoDiagnostic(const char* code, const std::string& message);

/**
 * @brief 构造 kWarning 级诊断条目。
 * @param[in] code 诊断码（如 "sar.example"）。
 * @param[in] message 诊断描述。
 * @return kWarning 严重级的 SarDiagnosticIssue。
 */
SarDiagnosticIssue MakeWarningDiagnostic(const char* code, const std::string& message);

/**
 * @brief 将本周期标记为执行中止（三写：abort_reason + diagnostics + 日志）。
 * @param[out] result 单周期结果。
 * @param[in] reason 粗粒度中止原因枚举。
 * @param[in] detail_code 细粒度诊断码（如 "snr_below_minimum"），写入 diagnostics。
 * @param[in] message 中止描述。
 */
void RecordAbort(SarCycleResult* result, SarPipelineAbortReason reason,
                 const char* detail_code, const std::string& message);

/**
 * @brief 将本周期标记为校验中止（三写：abort_reason + diagnostics + 日志）。
 * @param[out] result 单周期结果。
 * @param[in] reason 粗粒度中止原因枚举（应为 kValidationRejected 或 kExternalInputRejected）。
 * @param[in] detail_code 细粒度诊断码（如 "invalid_config"），写入 diagnostics。
 * @param[in] message 中止描述。
 */
void RecordValidationAbort(SarCycleResult* result, SarPipelineAbortReason reason,
                           const char* detail_code, const std::string& message);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_DIAGNOSTIC_UTILS_H_
