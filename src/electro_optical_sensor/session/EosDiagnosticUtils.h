/**
 * @file EosDiagnosticUtils.h
 * @brief EOS 单周期诊断条目构造与中止记录工具函数。
 */

#ifndef ONEQ_SRC_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_DIAGNOSTIC_UTILS_H_
#define ONEQ_SRC_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_DIAGNOSTIC_UTILS_H_

#include <string>

#include "1q/electro_optical_sensor/session/EosCycleResult.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief 将 EosPipelineAbortReason 粗粒度枚举转换为诊断码字符串。
 */
const char* AbortReasonToDiagnosticCode(EosPipelineAbortReason reason);

/**
 * @brief 将本周期标记为中止（三写：abort_reason + issues + 日志）。
 * @param[in] reason 粗粒度中止原因；phase 由原因推导（kValidationRejected → kInputValidation，
 *            kOutputContractViolation → kOutputContract，其余 → kExecution）。
 * @param[in] detail_code 细粒度 issue code（EosIssueCodes.h 注册表常量，完整字符串）。
 */
void RecordAbort(EosCycleResult* result, EosPipelineAbortReason reason,
                 const char* detail_code, const std::string& message);

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_SRC_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_DIAGNOSTIC_UTILS_H_
