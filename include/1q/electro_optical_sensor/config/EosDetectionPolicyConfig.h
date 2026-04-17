/**
 * @file EosDetectionPolicyConfig.h
 * @brief 定义 EOS 探测策略配置。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_DETECTION_POLICY_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_DETECTION_POLICY_CONFIG_H_

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosDetectionProfile 描述探测策略档位。
 */
enum class EosDetectionProfile {
  kConservative = 0,
  kBalanced,
  kAggressive
};

/**
 * @brief EosDetectionPolicyConfig 描述对外可配的探测语义。
 */
struct EosDetectionPolicyConfig {
  EosDetectionProfile profile{EosDetectionProfile::kBalanced}; /**< 探测策略档位 */
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_DETECTION_POLICY_CONFIG_H_
