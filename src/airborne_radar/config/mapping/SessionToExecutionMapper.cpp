#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"

#include "airborne_radar/config/mapping/EngineeringResolvers.h"

namespace airborne_radar {
namespace config {
namespace mapping {

execution::InternalExecutionConfig MapSessionToExecution(
    const config::RadarSessionConfig& session_config) {
  execution::InternalExecutionConfig exec;

  exec.detection.hardware = session_config.hardware.detection;
  exec.detection.engineering = session_config.hardware.detection;
  exec.detection.beam_control = session_config.policy.beam_control;
  exec.association.policy = session_config.policy.association;
  exec.tracking.policy = session_config.policy.tracking;
  exec.lifecycle.policy = session_config.policy.lifecycle;
  exec.detection.orientation = session_config.mission.orientation;

  exec.tracking.engineering = ResolveTrackingEngineering(exec.tracking.policy);
  exec.lifecycle.engineering = ResolveLifecycleEngineering(exec.lifecycle.policy);

  exec.association.unassigned_cost = exec.association.policy.unassigned_cost;

  if (exec.lifecycle.engineering.enable_imm_lifecycle) {
    exec.lifecycle.imm_model_noise_diff_coeffs =
        BuildDefaultImmNoiseDiffCoeffs(exec.lifecycle.policy.model_count_hint);
  }

  return exec;
}

}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar
