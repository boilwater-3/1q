/**
 * @file EsrDetectionPolicyConfig.h
 * @brief 定义 ESR 探测策略配置。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_DETECTION_POLICY_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_DETECTION_POLICY_CONFIG_H_

#include "1q/api.hpp"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrDetectionProfile 描述对外探测策略档位。
 */
enum class ONEQ_API EsrDetectionProfile {
  kConservative = 0, /**< 保守策略，降低虚警 */
  kBalanced,         /**< 均衡策略 */
  kSensitive         /**< 高灵敏策略，提升检出 */
};

/**
 * @brief EsrDetectionPolicyConfig 描述对外探测策略语义。
 */
struct ONEQ_API EsrDetectionPolicyConfig {
  EsrDetectionProfile profile{EsrDetectionProfile::kBalanced};
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_DETECTION_POLICY_CONFIG_H_
