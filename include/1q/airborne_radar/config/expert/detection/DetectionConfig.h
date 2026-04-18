/**
 * @file DetectionConfig.h
 * @brief 定义 expert 探测配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_DETECTION_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_DETECTION_CONFIG_H_

#include "1q/airborne_radar/config/expert/detection/AntennaConfig.h"
#include "1q/airborne_radar/config/expert/detection/DetectionPolicyConfig.h"
#include "1q/airborne_radar/config/expert/detection/RcsPhysicsConfig.h"
#include "1q/airborne_radar/config/expert/detection/ReceiverConfig.h"
#include "1q/airborne_radar/config/expert/detection/TransmitterConfig.h"
#include "1q/airborne_radar/config/semantic/DetectionProfiles.h"

namespace airborne_radar {
namespace config {
namespace expert {
namespace detection {

/**
 * @brief expert 探测聚合配置。
 */
struct DetectionConfig {
  bool enable_physics_detection{false}; /**< 是否启用物理雷达方程检测链。 */
  TransmitterConfig transmitter{}; /**< 发射机参数。 */
  AntennaConfig antenna{}; /**< 天线参数。 */
  ReceiverConfig receiver{}; /**< 接收机参数。 */
  DetectionPolicyConfig detection_policy{}; /**< 探测判决参数。 */
  RcsPhysicsConfig rcs_physics{}; /**< RCS 物理建模参数。 */
  float min_detection_margin_db{-2.0f}; /**< 最小探测裕量。 */
  int pulse_count{10}; /**< 脉冲积累数。 */
  semantic::SwerlingModel swerling_model{semantic::SwerlingModel::kSwerling0}; /**< 目标起伏模型。 */
};

}  // namespace detection
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_DETECTION_CONFIG_H_
