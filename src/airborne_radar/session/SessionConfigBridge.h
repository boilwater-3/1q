#ifndef AIRBORNE_RADAR_SESSION_SESSION_CONFIG_BRIDGE_H_
#define AIRBORNE_RADAR_SESSION_SESSION_CONFIG_BRIDGE_H_

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "airborne_radar/config/execution/InternalExecutionConfig.h"
#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"

namespace airborne_radar {
namespace session {
namespace internal {

inline config::execution::InternalExecutionConfig BuildExecutionConfigFromSessionConfig(
    const RadarSessionConfig& config) {
  return config::mapping::MapSessionToExecution(config);
}

inline RadarSessionConfig BuildSessionConfigFromExecutionConfig(
    const config::execution::InternalExecutionConfig& execution_config) {
  RadarSessionConfig config;
  config.hardware.detection = execution_config.hardware_detection;
  config.mission.orientation = execution_config.mission_orientation;
  config.policy.beam_control = execution_config.policy_beam_control;
  config.policy.association = execution_config.policy_association;
  config.policy.tracking = execution_config.policy_tracking;
  config.policy.lifecycle = execution_config.policy_lifecycle;
  config.policy.imm = execution_config.policy_imm;
  return config;
}

}  // namespace internal
}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_SESSION_CONFIG_BRIDGE_H_
