/**
 * @file RadarSessionConfig.h
 * @brief 定义 RadarSession 的四域初始化配置结构。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_

#include "1q/airborne_radar/config/PipelineConfig.h"
#include "1q/airborne_radar/environment/EnvironmentConfig.h"
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

inline RadarHardwareConfig BuildRadarHardwareConfig(const PipelineConfig& pipeline_config) {
  RadarHardwareConfig config;
  config.detection = pipeline_config.expert.detection;
  return config;
}

inline RadarMissionConfig BuildRadarMissionConfig(const PipelineConfig& pipeline_config) {
  RadarMissionConfig config;
  config.orientation = pipeline_config.orientation;
  return config;
}

inline RadarPolicyConfig BuildRadarPolicyConfig(const PipelineConfig& pipeline_config) {
  RadarPolicyConfig config;
  config.beam_control = pipeline_config.expert.beam_control;
  config.association = pipeline_config.expert.association;
  config.tracking = pipeline_config.expert.tracking;
  config.lifecycle = pipeline_config.expert.lifecycle;
  config.imm = pipeline_config.expert.imm;
  return config;
}

inline PipelineConfig BuildPipelineConfig(const RadarHardwareConfig& hardware,
                                          const RadarMissionConfig& mission,
                                          const RadarPolicyConfig& policy) {
  PipelineConfig pipeline_config;
  pipeline_config.expert.detection = hardware.detection;
  pipeline_config.expert.beam_control = policy.beam_control;
  pipeline_config.expert.association = policy.association;
  pipeline_config.expert.tracking = policy.tracking;
  pipeline_config.expert.lifecycle = policy.lifecycle;
  pipeline_config.expert.imm = policy.imm;
  pipeline_config.orientation = mission.orientation;
  return pipeline_config;
}

}  // namespace config

namespace session {

/**
 * @brief RadarSession 初始化配置（四域公开模型）。
 */
struct ONEQ_API RadarSessionConfig {
  config::RadarHardwareConfig hardware{};    /**< 硬件域。 */
  config::RadarMissionConfig mission{};      /**< 任务域。 */
  config::RadarPolicyConfig policy{};        /**< 策略域。 */
  config::RadarEnvironmentConfig environment{}; /**< 环境域。 */
};

inline RadarSessionConfig BuildRadarSessionConfig(
    const config::PipelineConfig& pipeline_config,
    const config::RadarEnvironmentConfig& environment = {}) {
  RadarSessionConfig config;
  config.hardware = config::BuildRadarHardwareConfig(pipeline_config);
  config.mission = config::BuildRadarMissionConfig(pipeline_config);
  config.policy = config::BuildRadarPolicyConfig(pipeline_config);
  config.environment = environment;
  return config;
}

inline config::PipelineConfig BuildLegacyPipelineConfig(const RadarSessionConfig& config) {
  return config::BuildPipelineConfig(config.hardware, config.mission, config.policy);
}

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_H_
