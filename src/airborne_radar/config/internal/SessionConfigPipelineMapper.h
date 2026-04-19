/**
 * @file SessionConfigPipelineMapper.h
 * @brief 定义四域会话配置到内部 pipeline 装配配置的共享映射。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_INTERNAL_SESSION_CONFIG_PIPELINE_MAPPER_H_
#define AIRBORNE_RADAR_SRC_CONFIG_INTERNAL_SESSION_CONFIG_PIPELINE_MAPPER_H_

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "airborne_radar/config/legacy/PipelineConfig.h"

namespace airborne_radar {
namespace config {
namespace internal {

inline PipelineConfig BuildPipelineConfigFromSessionConfig(
    const session::RadarSessionConfig& session_config) {
  PipelineConfig pipeline_config;
  pipeline_config.expert.detection = session_config.hardware.detection;
  pipeline_config.expert.beam_control = session_config.policy.beam_control;
  pipeline_config.expert.association = session_config.policy.association;
  pipeline_config.expert.tracking = session_config.policy.tracking;
  pipeline_config.expert.lifecycle = session_config.policy.lifecycle;
  pipeline_config.expert.imm = session_config.policy.imm;
  pipeline_config.orientation = session_config.mission.orientation;
  return pipeline_config;
}

}  // namespace internal
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_INTERNAL_SESSION_CONFIG_PIPELINE_MAPPER_H_
