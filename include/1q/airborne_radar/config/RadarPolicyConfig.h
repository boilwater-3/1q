/**
 * @file RadarPolicyConfig.h
 * @brief 定义雷达策略域公开配置。
 *
 * 策略域承载调度、关联、跟踪与生命周期策略。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_POLICY_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_POLICY_CONFIG_H_

#include "1q/airborne_radar/config/expert/beam/BeamControlConfig.h"
#include "1q/airborne_radar/config/expert/lifecycle/ImmConfig.h"
#include "1q/airborne_radar/config/expert/lifecycle/LifecycleConfig.h"
#include "1q/airborne_radar/config/expert/tracking/AssociationConfig.h"
#include "1q/airborne_radar/config/expert/tracking/TrackingConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {
namespace expert {

using beam::BeamControlConfig;
using lifecycle::ImmConfig;
using lifecycle::LifecycleConfig;
using tracking::AssociationConfig;
using tracking::KalmanUpdateBackend;
using tracking::TrackingConfig;

}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

namespace airborne_radar {
namespace config {

using expert::AssociationConfig;
using expert::BeamControlConfig;
using expert::ImmConfig;
using expert::KalmanUpdateBackend;
using expert::LifecycleConfig;
using expert::TrackingConfig;

/**
 * @brief 雷达策略域配置。
 *
 * 当前阶段策略域承载调度、关联、跟踪与生命周期策略。
 */
struct ONEQ_API RadarPolicyConfig {
  BeamControlConfig beam_control{};
  AssociationConfig association{};
  TrackingConfig tracking{};
  LifecycleConfig lifecycle{};
  ImmConfig imm{};
};

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_POLICY_CONFIG_H_
