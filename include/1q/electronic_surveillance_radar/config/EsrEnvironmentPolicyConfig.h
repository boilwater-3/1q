/**
 * @file EsrEnvironmentPolicyConfig.h
 * @brief 定义 ESR 环境策略配置。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_POLICY_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_POLICY_CONFIG_H_

#include "1q/api.hpp"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrEnvironmentPreset 描述对外环境预设。
 */
enum class ONEQ_API EsrEnvironmentPreset {
  kStandard = 0, /**< 标准环境 */
  kLowClutter,   /**< 低杂波环境 */
  kDenseClutter, /**< 高杂波环境 */
  kJammed        /**< 强干扰环境 */
};

/**
 * @brief EsrEnvironmentPolicyConfig 描述环境策略语义输入。
 */
struct ONEQ_API EsrEnvironmentPolicyConfig {
  EsrEnvironmentPreset preset{EsrEnvironmentPreset::kStandard};
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_POLICY_CONFIG_H_
