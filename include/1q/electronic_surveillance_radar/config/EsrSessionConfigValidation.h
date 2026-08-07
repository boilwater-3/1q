/**
 * @file EsrSessionConfigValidation.h
 * @brief ESR 会话配置校验工具（统一问题列表模型）。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_VALIDATION_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electronic_surveillance_radar/session/EsrOutputTypes.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief 校验最终 ESR 会话配置的合法性。
 *
 * 检查项包括：
 * - 扫描数据率为正；
 * - 接收频段上下限一致；
 * - 波束宽度为正；
 * - 显式扫描边界均为有限值，且起止顺序一致。
 *
 * 统一问题列表模型（session_contract.md 规则 14）：返回 `session::EsrIssueList`，
 * 每条问题 severity=`EsrIssueSeverity::kError`、phase=`EsrIssuePhase::kInputValidation`，
 * code 以字符串编码为 "esr.validation.<snake_case>"（机器消费只认 code），
 * field 保留原字段路径，message 保留原说明。
 *
 * @param[in] config 待校验的最终会话配置。
 * @return 按发现顺序返回的校验问题列表。
 */
ONEQ_API session::EsrIssueList
ValidateEsrSessionConfig(const config::EsrSessionConfig& config) noexcept;

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_VALIDATION_H_
