#include "1q/airborne_radar/session/RadarCycleInputBuilder.h"

namespace airborne_radar {
namespace session {

bool RadarCycleInputBuilder::Build(const RadarExternalPoseInput& platform,
                                   const std::vector<TargetExternalKinematics>& targets,
                                   float dt_sec, RadarCycleInput* output) {
  return Build(platform, targets, dt_sec, RadarEnvironmentInput{}, output);
}

bool RadarCycleInputBuilder::Build(const RadarExternalPoseInput& platform,
                                   const std::vector<TargetExternalKinematics>& targets,
                                   float dt_sec, const RadarEnvironmentInput& environment,
                                   RadarCycleInput* output) {
  if (output == nullptr) {
    return false;
  }

  RadarLocalFrameReference reference;
  if (!TryMakeRadarPoseFromExternalKinematics(platform, &reference, &output->platform_pose)) {
    return false;
  }

  output->cycle_index = 0U;
  output->dt_sec = dt_sec;
  output->environment = environment;
  output->scene.clear();

  for (std::size_t i = 0; i < targets.size(); ++i) {
    RadarSceneTarget target;
    if (!TryMakeTargetFromExternalKinematics(targets[i].external_target_id, targets[i], reference,
                                             output->platform_pose.velocity_mps, &target)) {
      return false;
    }
    output->scene.push_back(target);
  }
  return true;
}

}  // namespace session
}  // namespace airborne_radar
