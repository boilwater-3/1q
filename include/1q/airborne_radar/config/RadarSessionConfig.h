/**
 * @file RadarSessionConfig.h
 * @brief 定义 RadarSession 的四域初始化配置结构。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_

#include "1q/airborne_radar/config/expert/ExpertPipelineConfig.h"
#include "1q/airborne_radar/environment/EnvironmentConfig.h"
#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

using RadarEnvironmentConfig = environment::EnvironmentDefaultConfig;

/**
 * @brief 雷达硬件域配置。
 *
 * 当前阶段硬件域承载探测链路固有能力参数。
 */
struct ONEQ_API RadarHardwareConfig {
  expert::DetectionConfig detection{}; /**< 固有探测链路能力配置。 */
};

/**
 * @brief 雷达任务域配置。
 *
 * 当前阶段任务域承载工作子模式与波束指向运行态。
 */
struct ONEQ_API RadarMissionConfig {
  model::RadarOrientationConfig orientation{}; /**< 任务态波束指向与扫描状态。 */
};

/**
 * @brief 雷达策略域配置。
 *
 * 当前阶段策略域承载调度、关联、跟踪与生命周期策略。
 */
struct ONEQ_API RadarPolicyConfig {
  expert::BeamControlConfig beam_control{}; /**< 波束调度与指向策略。 */
  expert::AssociationConfig association{};  /**< 数据关联策略。 */
  expert::TrackingConfig tracking{};        /**< 跟踪滤波策略。 */
  expert::LifecycleConfig lifecycle{};      /**< 航迹生命周期策略。 */
  expert::ImmConfig imm{};                  /**< IMM 策略。 */
};

}  // namespace config

namespace session {

/**
 * @brief RadarSession 初始化配置（四域公开模型）。
 */
struct ONEQ_API RadarSessionConfig {
  config::RadarHardwareConfig hardware{};       /**< 硬件域。 */
  config::RadarMissionConfig mission{};         /**< 任务域。 */
  config::RadarPolicyConfig policy{};           /**< 策略域。 */
  config::RadarEnvironmentConfig environment{}; /**< 环境域。 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_
