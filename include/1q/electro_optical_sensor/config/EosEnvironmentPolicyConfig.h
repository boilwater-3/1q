/**
 * @file EosEnvironmentPolicyConfig.h
 * @brief 定义 EOS 环境策略配置。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_POLICY_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_POLICY_CONFIG_H_

#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosEnvironmentPreset 描述高层环境预设。
 */
enum class EosEnvironmentPreset {
  kStandard = 0,
  kHumid,
  kDusty,
  kTurbulent,
  kMaritime
};

/**
 * @brief EosEnvironmentPolicyConfig 描述环境策略输入。
 */
struct EosEnvironmentPolicyConfig {
  environment::EosEnvironmentModelType model_type{
      environment::EosEnvironmentModelType::kSimplified}; /**< 环境模型类型 */
  EosEnvironmentPreset preset{EosEnvironmentPreset::kStandard}; /**< 环境预设 */
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_POLICY_CONFIG_H_
