/**
 * @file DetectionConfig.h
 * @brief 定义 semantic 探测配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SEMANTIC_DETECTION_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_SEMANTIC_DETECTION_CONFIG_H_

#include "1q/airborne_radar/config/semantic/AntennaPatternConfig.h"
#include "1q/airborne_radar/config/semantic/profiles/DetectionProfiles.h"

namespace airborne_radar {
namespace config {
namespace semantic {

/**
 * @brief semantic 探测配置。
 */
struct DetectionConfig {
  bool enable_physics_detection{false}; /**< 是否启用物理雷达方程检测链路。 */
  RadarHardwareProfile hardware_profile{RadarHardwareProfile::kGenericAirborneXBand}; /**< 平台硬件语义档位。 */
  DetectionIntentProfile intent_profile{DetectionIntentProfile::kBalanced}; /**< 探测意图语义档位。 */
  SwerlingModel swerling_model{kSwerling0}; /**< 目标起伏模型。 */
  AntennaPatternConfig antenna_pattern{}; /**< 方向图语义配置。 */
  RcsFusionProfile rcs_fusion_profile{RcsFusionProfile::kDisabled}; /**< RCS 融合语义档位。 */
};

}  // namespace semantic
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SEMANTIC_DETECTION_CONFIG_H_
