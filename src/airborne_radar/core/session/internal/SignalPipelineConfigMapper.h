/**
 * @file SignalPipelineConfigMapper.h
 * @brief 定义 RadarSession 内部使用的配置映射函数。
 */

#ifndef AIRBORNE_RADAR_SRC_CORE_SESSION_INTERNAL_SIGNAL_PIPELINE_CONFIG_MAPPER_H_
#define AIRBORNE_RADAR_SRC_CORE_SESSION_INTERNAL_SIGNAL_PIPELINE_CONFIG_MAPPER_H_

#include "1q/airborne_radar/config/SignalPipelineConfig.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"

namespace airborne_radar {
namespace core {
namespace session {
namespace internal {

inline signal::pipeline::SignalPipelineConfig ToPipelineSignalPipelineConfig(
    const common::config::SignalPipelineConfig& config) {
  signal::pipeline::SignalPipelineConfig pipeline_config;
  pipeline_config.detection = config.detection;
  pipeline_config.beam_control = config.beam_control;
  pipeline_config.tracking.enable_kalman_filter = config.tracking.enable_kalman_filter;
  pipeline_config.tracking.kalman_measurement_noise_std =
      config.tracking.kalman_measurement_noise_std;
  pipeline_config.lifecycle.enable_auto_lifecycle_manager =
      config.lifecycle.enable_auto_lifecycle_manager;
  pipeline_config.lifecycle.lifecycle_config.confirm_hits =
      config.lifecycle.lifecycle_config.confirm_hits;
  pipeline_config.lifecycle.lifecycle_config.max_miss_before_lost =
      config.lifecycle.lifecycle_config.max_miss_before_lost;
  pipeline_config.lifecycle.lifecycle_config.max_lost_cycles =
      config.lifecycle.lifecycle_config.max_lost_cycles;
  pipeline_config.lifecycle.enable_imm_lifecycle = config.lifecycle.enable_imm_lifecycle;
  return pipeline_config;
}

}  // namespace internal
}  // namespace session
}  // namespace core
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CORE_SESSION_INTERNAL_SIGNAL_PIPELINE_CONFIG_MAPPER_H_
