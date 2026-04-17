/**
 * @file EosStrayLightPolicyConfig.h
 * @brief 定义 EOS 杂散光抑制策略配置。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_STRAY_LIGHT_POLICY_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_STRAY_LIGHT_POLICY_CONFIG_H_

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosStrayLightProfile 描述杂散光防护档位。
 */
enum class EosStrayLightProfile {
  kDisabled = 0,
  kStandardHood,
  kEnhancedHood
};

/**
 * @brief EosStrayLightPolicyConfig 描述对外可配的杂散光策略。
 */
struct EosStrayLightPolicyConfig {
  EosStrayLightProfile profile{EosStrayLightProfile::kDisabled}; /**< 杂散光防护档位 */
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_STRAY_LIGHT_POLICY_CONFIG_H_
