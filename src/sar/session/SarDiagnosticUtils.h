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
 * @brief 构造 kInfo 级诊断条目。
 * @param[in] code 诊断码（如 "sar.example"）。
 * @param[in] message 诊断描述。
 * @return kInfo 严重级的 SarDiagnosticIssue。
 */
SarDiagnosticIssue MakeInfoDiagnostic(const char* code, const std::string& message);

/**
 * @brief 构造 kWarning 级诊断条目。
 *
 * 用于物理可行但需关注的退化（如回波 clipping、斜距与标称值严重错配）。不阻断当周期
 * 执行，但会被 ApplyDiagnosticsPolicy 在 enable_diagnostics=true 时保留（kInfo 也会
 * 保留），让退化从正常诊断里浮出来。
 * @param[in] code 诊断码（如 "sar.example"）。
 * @param[in] message 诊断描述。
 * @return kWarning 严重级的 SarDiagnosticIssue。
 */
SarDiagnosticIssue MakeWarningDiagnostic(const char* code, const std::string& message);

/**
 * @brief 将本周期标记为中止并向结果写入结构化错误诊断。
 * @param[out] result 单周期结果，has_error 置真、abort_reason 写入 tag，并追加一条
 *                   code 为 "sar.{tag}"、严重级 kError 的诊断。
 * @param[in] tag 中止原因标签（同时作为 abort_reason 与诊断码后缀）。
 * @param[in] message 中止描述。
 */
void RecordAbort(SarCycleResult* result, const std::string& tag, const std::string& message);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_DIAGNOSTIC_UTILS_H_

