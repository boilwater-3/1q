/**
 * @file SbirsPolicyConfig.h
 * @brief 定义 SBIRS-inspired 检测、误差和调度策略。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace config {

struct ONEQ_API SbirsDetectionPolicyConfig {
  float wide_min_snr_linear{4.0f};
  float narrow_min_snr_linear{6.0f};
};

struct ONEQ_API SbirsErrorModelConfig {
  float angular_sigma_deg{0.05f};
  float range_fraction_sigma{0.001f};
  unsigned int random_seed{1U};
};

struct ONEQ_API SbirsSchedulerConfig {
  bool single_narrow_resource{true};
};

struct ONEQ_API SbirsPolicyConfig {
  SbirsDetectionPolicyConfig detection{};
  SbirsErrorModelConfig error_model{};
  SbirsSchedulerConfig scheduler{};
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_POLICY_CONFIG_H_
