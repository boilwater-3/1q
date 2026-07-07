/**
 * @file SbirsSessionConfigValidation.h
 * @brief 定义 SBIRS-inspired 配置校验入口。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_VALIDATION_H_

#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/foundation/validation_types.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"

namespace sbirs_sensor {
namespace config {

/** @brief 单条配置校验结果，包含严重级别与人读消息。 */
struct ONEQ_API ValidationIssue {
  oneq::foundation::ValidationSeverity severity{oneq::foundation::ValidationSeverity::kInfo}; /**< 问题严重级别 */
  std::string message{}; /**< 人读问题描述 */
};

/** @brief 校验问题列表。 */
using ValidationIssueList = std::vector<ValidationIssue>;

/**
 * @brief 校验 `SbirsSessionConfig` 各域取值合理性（波段正序、孔径/FOV/帧率为正、
 *        距离门控有序非负、扫描速率非负、检测门限非负）。
 * @param[in] config 待校验的会话配置
 * @return 校验问题列表；为空表示配置通过校验
 * @note 该函数为纯校验，不修改配置，不抛异常。
 */
ONEQ_API ValidationIssueList ValidateSbirsSessionConfig(const SbirsSessionConfig& config);

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_VALIDATION_H_
