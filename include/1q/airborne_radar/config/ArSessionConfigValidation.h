/**
 * @file ArSessionConfigValidation.h
 * @brief 机载雷达会话配置校验入口。
 *
 * 会话配置静态校验的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArOutputTypes.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

/**
 * @brief 对会话初始化配置执行静态结构校验。
 *
 * 校验范围：指令态波束宽度启用时须为有限正值；未启用指令态覆盖时，每个天线轴必须
 * 提供正的名义波束宽度，或提供可结合有效发射频率推导波束宽度的正物理孔径；
 * 发射频率须为有限正值；物理 RCS 评估频率须为 0 或有限正值；机械/电子扫描限位
 * 下限不大于上限。
 *
 * 命中问题时返回统一问题列表（`session::ArIssueList`）：每条 `session::ArIssue` 的
 * `severity` 固定为 `session::ArIssueSeverity::kError`，`phase` 固定为
 * `session::ArIssuePhase::kInputValidation`，`code` 形如
 * `"ar.validation.<snake_case>"`（如 `"ar.validation.transmitter_frequency_invalid"`），
 * `field` 保留触发问题的配置字段路径，`message` 保留问题说明文本，
 * `location` 保持默认（kGlobal）。
 *
 * @param[in] config 待校验的会话初始化配置。
 * @return 校验问题列表；为空表示配置通过校验。
 * @note 该函数为 noexcept，仅做只读检查，不修改输入配置。
 */
ONEQ_API session::ArIssueList
ValidateArSessionConfig(const config::ArSessionConfig& config) noexcept;

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_
