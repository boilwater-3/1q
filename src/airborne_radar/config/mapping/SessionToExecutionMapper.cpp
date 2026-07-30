#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"

#include "airborne_radar/config/mapping/EngineeringResolvers.h"
#include "airborne_radar/config/mapping/MappingTransforms.h"

namespace airborne_radar {
namespace config {
namespace mapping {

execution::InternalExecutionConfig MapSessionToExecution(
    const config::ArSessionConfig& session_config) {
  execution::InternalExecutionConfig exec;

  exec.sensor_enabled = session_config.mission.power_on;
  exec.decision_control = session_config.policy.decision_control;
  exec.anti_vgpo_max_acceleration_mps2 =
      session_config.policy.decision_control.anti_vgpo_max_acceleration_mps2;
  exec.detection.engineering =
      ResolveDetectionEngineering(session_config.hardware, session_config.policy.detection);
  exec.detection.beam_control = session_config.policy.beam_control;
  exec.association.policy.unassigned_cost =
      SigmaToSquaredCost(session_config.policy.association.distance_gate_sigma);
  exec.tracking.policy = session_config.policy.tracking;
  exec.lifecycle.policy = session_config.policy.lifecycle;
  exec.detection.orientation = session_config.mission.orientation;

  ReconcilePolicyToEngineering(exec);

  return exec;
}

}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar
