/**
 * @file ExpertPipelineConfig.h
 * @brief 定义 expert 流水线聚合配置壳。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_EXPERT_PIPELINE_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_EXPERT_PIPELINE_CONFIG_H_

#include "1q/airborne_radar/config/expert/beam/BeamControlConfig.h"
#include "1q/airborne_radar/config/expert/detection/DetectionConfig.h"
#include "1q/airborne_radar/config/expert/lifecycle/ImmConfig.h"
#include "1q/airborne_radar/config/expert/lifecycle/LifecycleConfig.h"
#include "1q/airborne_radar/config/expert/tracking/AssociationConfig.h"
#include "1q/airborne_radar/config/expert/tracking/TrackingConfig.h"

namespace airborne_radar {
namespace config {
namespace expert {

using beam::BeamControlConfig;
using detection::AntennaConfig;
using detection::AntennaPatternConfig;
using detection::AntennaPatternModelType;
using detection::DetectionConfig;
using detection::DetectionPolicyConfig;
using detection::RcsPhysicsConfig;
using detection::ReceiverConfig;
using detection::TransmitterConfig;
using lifecycle::ImmConfig;
using lifecycle::LifecycleConfig;
using tracking::AssociationConfig;
using tracking::KalmanConfig;
using tracking::KalmanUpdateBackend;
using tracking::TrackingConfig;

/**
 * @brief expert 流水线聚合配置。
 */
struct ExpertPipelineConfig {
  detection::DetectionConfig detection{}; /**< expert 探测配置。 */
  beam::BeamControlConfig beam_control{}; /**< expert 波束控制配置。 */
  tracking::TrackingConfig tracking{}; /**< expert 跟踪配置。 */
  tracking::AssociationConfig association{}; /**< expert 关联配置。 */
  lifecycle::LifecycleConfig lifecycle{}; /**< expert 生命周期配置。 */
  lifecycle::ImmConfig imm{}; /**< expert IMM 配置。 */
};

}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_EXPERT_PIPELINE_CONFIG_H_
