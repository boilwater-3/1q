/**
 * @file ArDiagnosticUtils.h
 * @brief AR 单周期诊断条目构造与中止记录工具函数。
 */

#ifndef ONEQ_SRC_AIRBORNE_RADAR_SESSION_AR_DIAGNOSTIC_UTILS_H_
#define ONEQ_SRC_AIRBORNE_RADAR_SESSION_AR_DIAGNOSTIC_UTILS_H_

#include <string>

#include "1q/airborne_radar/session/ArCycleResult.h"

namespace airborne_radar {
namespace session {

/**
 * @brief 将 SignalCycleAbortReason 粗粒度枚举转换为诊断码字符串。
 */
const char* AbortReasonToDiagnosticCode(SignalCycleAbortReason reason);

/**
 * @brief 将本周期标记为中止（三写：abort_reason + diagnostics + 日志）。
 */
void RecordAbort(ArCycleResult* result, SignalCycleAbortReason reason,
                 const char* detail_code, const std::string& message, bool is_validation);

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_SRC_AIRBORNE_RADAR_SESSION_AR_DIAGNOSTIC_UTILS_H_
