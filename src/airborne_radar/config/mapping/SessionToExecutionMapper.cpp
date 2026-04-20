#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"

#include "airborne_radar/config/defaults/ExecutionDefaults.h"
#include "airborne_radar/config/mapping/EngineeringResolvers.h"

namespace airborne_radar {
namespace config {
namespace mapping {

execution::InternalExecutionConfig MapSessionToExecution(
    const session::RadarSessionConfig& session_config) {
  execution::InternalExecutionConfig exec;

  exec.hardware_detection = session_config.hardware.detection;
  exec.policy_beam_control = session_config.policy.beam_control;
  exec.policy_association = session_config.policy.association;
  exec.policy_tracking = session_config.policy.tracking;
  exec.policy_lifecycle = session_config.policy.lifecycle;
  exec.policy_imm = session_config.policy.imm;
  exec.mission_orientation = session_config.mission.orientation;

  exec.detection_engineering = ResolveDetectionEngineering(exec.hardware_detection);
  exec.tracking_engineering = ResolveTrackingEngineering(exec.policy_tracking);
  exec.lifecycle_engineering = ResolveLifecycleEngineering(exec.policy_lifecycle);

  exec.tracking_speed_decay_ratio_on_loss = exec.policy_tracking.speed_decay_ratio_on_loss;
  exec.tracking_rcs_decay_ratio_on_loss = exec.policy_tracking.rcs_decay_ratio_on_loss;
  exec.association_unassigned_cost = exec.policy_association.unassigned_cost;

  if (exec.lifecycle_engineering.enable_imm_lifecycle) {
    exec.imm_model_noise_diff_coeffs = defaults::DefaultImmModelNoiseDiffCoeffs();
  }

  return exec;
}

}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar
