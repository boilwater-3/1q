/**
 * @file EsrPolicyConfig.h
 * @brief 定义 ESR 策略域配置。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_POLICY_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_POLICY_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrDetectionPolicyConfig.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrPolicyConfig 描述 ESR 的统一策略域输入。
 */
struct ONEQ_API EsrPolicyConfig {
  EsrDetectionPolicyConfig detection{}; /**< 探测策略子域 */
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_POLICY_CONFIG_H_
