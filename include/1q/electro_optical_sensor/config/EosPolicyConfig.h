/**
 * @file EosPolicyConfig.h
 * @brief 定义 EOS 策略域配置。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_POLICY_CONFIG_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_POLICY_CONFIG_H_

#include "1q/api.hpp"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosDetectionPolicyConfig 描述探测策略参数。
 */
struct ONEQ_API EosDetectionPolicyConfig {
  float minimum_snr_db{6.0f}; /**< 最小检测信噪比门限（单位：dB） */
  float detection_sensitivity_w{1.0e-12f}; /**< 探测灵敏度（单位：W） */
  float visible_reference_irradiance_w_m2{800.0f}; /**< 可见光参考辐照度（单位：W/m^2） */
};

/**
 * @brief EosStrayLightPolicyConfig 描述杂散光抑制策略参数。
 */
struct ONEQ_API EosStrayLightPolicyConfig {
  bool enable_straylight_filter{false}; /**< 是否启用杂散光抑制 */
  float hood_inner_half_angle_deg{12.0f}; /**< 遮光罩内半角（单位：deg） */
  float hood_outer_half_angle_deg{75.0f}; /**< 遮光罩外半角（单位：deg） */
  float hood_min_suppression_ratio{0.20f}; /**< 最小抑制比 */
  float hood_max_suppression_ratio{0.85f}; /**< 最大抑制比 */
};

/**
 * @brief EosPolicyConfig 描述策略域配置。
 */
struct ONEQ_API EosPolicyConfig {
  EosDetectionPolicyConfig detection{};
  EosStrayLightPolicyConfig stray_light{};
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_POLICY_CONFIG_H_
