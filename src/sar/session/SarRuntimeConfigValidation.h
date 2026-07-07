/**
 * @file SarRuntimeConfigValidation.h
 * @brief SAR 单周期 step-time 运行期配置校验。
 */

#ifndef ONEQ_SRC_SAR_SESSION_SAR_RUNTIME_CONFIG_VALIDATION_H_
#define ONEQ_SRC_SAR_SESSION_SAR_RUNTIME_CONFIG_VALIDATION_H_

#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleResult.h"

namespace sar {
namespace session {

/**
 * @brief 校验本周期会话配置是否满足执行前置条件。
 *
 * 检查 hardware/mission 字段的正数性、跨字段物理约束（如采样窗口能否容纳完整 LFM 脉冲）
 * 以及与外部 raw IQ、各成像阶段开关的一致性。
 * @param[in] config 本周期会话配置。
 * @param[in] has_external_raw_iq 输入是否携带外部完整孔径 raw IQ。
 * @param[out] result 单周期结果，校验失败时写入结构化错误诊断（abort_reason / sar.<tag>）。
 * @return 校验通过返回 true；失败返回 false 并已向 result 写入错误诊断。
 */
bool ValidateRuntimeConfigForStep(const config::SarSessionConfig& config,
                                  bool has_external_raw_iq,
                                  SarCycleResult* result);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_RUNTIME_CONFIG_VALIDATION_H_

