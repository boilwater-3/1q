/**
 * @file ExpertPipelineConfig.h
 * @brief 内部 legacy 聚合配置定义，仅用于过渡期内部实现。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_LEGACY_EXPERT_EXPERT_PIPELINE_CONFIG_H_
#define AIRBORNE_RADAR_SRC_CONFIG_LEGACY_EXPERT_EXPERT_PIPELINE_CONFIG_H_

#include "1q/airborne_radar/config/RadarHardwareConfig.h"
#include "1q/airborne_radar/config/RadarPolicyConfig.h"

namespace airborne_radar {
namespace config {
namespace expert {

/**
 * @brief expert 流水线聚合配置。
 */
struct ExpertPipelineConfig {
  DetectionConfig detection{}; /**< expert 探测配置。 */
  BeamControlConfig beam_control{}; /**< expert 波束控制配置。 */
  TrackingConfig tracking{}; /**< expert 跟踪配置。 */
  AssociationConfig association{}; /**< expert 关联配置。 */
  LifecycleConfig lifecycle{}; /**< expert 生命周期配置。 */
  ImmConfig imm{}; /**< expert IMM 配置。 */
};

}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_LEGACY_EXPERT_EXPERT_PIPELINE_CONFIG_H_
