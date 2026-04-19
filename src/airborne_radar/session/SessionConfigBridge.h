#ifndef AIRBORNE_RADAR_SESSION_SESSION_CONFIG_BRIDGE_H_
#define AIRBORNE_RADAR_SESSION_SESSION_CONFIG_BRIDGE_H_

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "airborne_radar/config/internal/SessionConfigPipelineMapper.h"

namespace airborne_radar {
namespace session {
namespace internal {

/**
 * @brief 四域公开会话配置到内部 pipeline 装配配置的唯一翻译边界。
 */
inline config::PipelineConfig BuildPipelineConfigFromSessionConfig(
    const RadarSessionConfig& config) {
  return config::internal::BuildPipelineConfigFromSessionConfig(config);
}

inline RadarSessionConfig BuildSessionConfigFromRuntimeState(
    const config::RadarHardwareConfig& hardware, const config::RadarMissionConfig& mission,
    const config::RadarPolicyConfig& policy) {
  RadarSessionConfig config;
  config.hardware = hardware;
  config.mission = mission;
  config.policy = policy;
  return config;
}

}  // namespace internal
}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_SESSION_CONFIG_BRIDGE_H_
