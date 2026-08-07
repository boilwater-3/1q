/**
 * @file EosSessionConfigValidation.h
 * @brief EOS 会话配置校验工具。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_VALIDATION_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief 校验最终 EOS 会话配置的合法性。
 *
 * 校验问题采用统一问题列表模型（session_contract.md 规则 14）：每条问题以
 * `session::EosIssue` 表达，severity 固定为 `kError`、phase 为
 * `kInputValidation`，code 为带模块前缀的字符串 `"eos.validation.<snake_case>"`
 * （如 `eos.validation.horizontal_fov_not_positive`），机器消费只认 code。
 *
 * 检查项包括：
 * - 视场角为正；
 * - 扫描角速度与帧率为正；
 * - 扫描方位起止角一致。
 * - 环境 preset 枚举有效；
 * - 启用标准大气物理观测时，气压、温度和相对湿度有效。
 *
 * @param config 待校验的最终会话配置。
 * @return 按发现顺序返回的校验问题列表。
 */
ONEQ_API session::EosIssueList
ValidateEosSessionConfig(const config::EosSessionConfig& config) noexcept;

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_VALIDATION_H_
