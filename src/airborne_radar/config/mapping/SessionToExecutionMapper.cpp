#include <vector>

#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"

#include "airborne_radar/config/mapping/EngineeringResolvers.h"

namespace airborne_radar {
namespace config {
namespace mapping {

execution::InternalExecutionConfig MapSessionToExecution(
    const session::RadarSessionConfig& session_config) {
  execution::InternalExecutionConfig exec;

  exec.detection.hardware = session_config.hardware.detection;
  exec.detection.beam_control = session_config.policy.beam_control;
  exec.association.policy = session_config.policy.association;
  exec.tracking.policy = session_config.policy.tracking;
  exec.lifecycle.policy = session_config.policy.lifecycle;
  exec.detection.orientation = session_config.mission.orientation;

  exec.detection.engineering = ResolveDetectionEngineering(exec.detection.hardware);
  exec.tracking.engineering = ResolveTrackingEngineering(exec.tracking.policy);
  exec.lifecycle.engineering = ResolveLifecycleEngineering(exec.lifecycle.policy);

  exec.tracking.speed_decay_ratio_on_loss = exec.tracking.policy.speed_decay_ratio_on_loss;
  exec.tracking.rcs_decay_ratio_on_loss = exec.tracking.policy.rcs_decay_ratio_on_loss;
  exec.association.unassigned_cost = exec.association.policy.unassigned_cost;

  if (exec.lifecycle.engineering.enable_imm_lifecycle) {
    exec.lifecycle.imm_model_noise_diff_coeffs = std::vector<float>{0.5f, 4.0f};
  }

  return exec;
}

}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar
