/**
 * @file DetectionPolicyConfig.h
 * @brief 定义 expert 探测判决配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_DETECTION_POLICY_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_DETECTION_POLICY_CONFIG_H_

namespace airborne_radar {
namespace config {
namespace expert {
namespace detection {

/**
 * @brief expert 探测门限与判决配置。
 */
struct DetectionPolicyConfig {
  float cfar_pfa{1e-6f}; /**< CFAR 虚警概率目标值。 */
  float min_snr_db{-10.0f}; /**< 最小信噪比门限。 */
};

}  // namespace detection
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_DETECTION_POLICY_CONFIG_H_
