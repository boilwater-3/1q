/**
 * @file SarSessionConfigValidation.h
 * @brief SAR 会话配置校验工具。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_VALIDATION_H_

#include "1q/api.hpp"
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleResult.h"

namespace sar {
namespace config {

/**
 * @brief 校验最终 SAR 会话配置的合法性。
 *
 * 发现的问题统一以 `session::SarIssue` 承载（规则 14）：severity 为
 * `SarIssueSeverity::kError`、phase 为 `SarIssuePhase::kInputValidation`、
 * `code` 为字符串 "sar.validation.\<snake_case\>"，`field` 关联触发字段路径，
 * `message` 提供面向调用方的人读说明，`location` 保持默认（kGlobal）。
 *
 * 检查项包括：
 * - 载频、带宽、PRF、采样率为正；
 * - 方位孔径长度、标称斜距、平台速度为正；
 * - 发射功率为正，天线增益、接收机噪声系数与系统损耗为有限值；
 * - 方位脉冲数、距离采样点数非零；
 * - 期望分辨率（方位/地距）为正；
 * - 地形参考高程、大气损耗与表面后向散射系数为有限值，且大气损耗非负；
 * - 距离采样窗口能容纳完整脉冲宽度（ceil(pulse_width*sample_rate) <= range_sample_count）。
 *
 * @param config 待校验的最终会话配置。
 * @return 按发现顺序返回的校验问题列表（统一问题列表模型）。
 */
ONEQ_API session::SarIssueList ValidateSarSessionConfig(
    const config::SarSessionConfig& config) noexcept;

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_VALIDATION_H_
