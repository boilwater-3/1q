/**
 * @file EsrSessionConfigValidation.h
 * @brief ESR 会话配置校验工具。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_VALIDATION_H_

#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief ConfigValidationCode 表示 ESR 会话配置校验问题编码。
 */
enum class ConfigValidationCode {
  kNone = 0,
  kScanRateNotPositive,         /**< 扫描数据率 <= 0 */
  kReceiverBandLowerAboveUpper, /**< 接收频段下限 >= 上限 */
  kBeamAzWidthNotPositive,      /**< 方位波束宽度 <= 0 */
  kBeamElWidthNotPositive,      /**< 俯仰波束宽度 <= 0 */
  kExplicitScanBoundsAzSwapped, /**< 显式扫描方位起止颠倒 */
  kExplicitScanBoundsElSwapped, /**< 显式扫描俯仰起止颠倒 */
  kExplicitScanBoundsNotFinite, /**< 显式扫描边界包含 NaN 或 Inf */
  kReceiverRfHardwareInvalid,   /**< 接收天线、极化或线性输入边界非法 */
  kTuningPlanInvalid,           /**< 调谐窗口非法或超出硬件频段 */
  kMissionEnumInvalid,          /**< 工作模式、扫描起点或扫描顺序枚举非法 */
  kScanCenterNotFinite,         /**< 中心扫描模式的中心角非法 */
  kDetectionPolicyInvalid,      /**< 探测策略数值非法 */
  kEnvironmentInvalid           /**< 环境预设或启用的大气参数非法 */
};

/**
 * @brief ConfigValidationIssue 描述一条 ESR 会话配置校验结果。
 */
struct ConfigValidationIssue {
  ConfigValidationCode code{ConfigValidationCode::kNone}; /**< 问题编码 */
  std::string field{};                                    /**< 关联字段名 */
  std::string message{};                                  /**< 简短说明 */
};

/** @brief ValidationIssueList 表示 ESR 会话配置校验问题列表。 */
using ValidationIssueList = std::vector<ConfigValidationIssue>;

/**
 * @brief 校验最终 ESR 会话配置的合法性。
 *
 * 检查项包括：
 * - 扫描数据率为正；
 * - 接收频段上下限一致；
 * - 波束宽度为正；
 * - 显式扫描边界均为有限值，且起止顺序一致。
 *
 * @param[in] config 待校验的最终会话配置。
 * @return 按发现顺序返回的校验问题列表。
 */
ONEQ_API ValidationIssueList
ValidateEsrSessionConfig(const config::EsrSessionConfig& config) noexcept;

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_VALIDATION_H_
