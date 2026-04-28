#include "1q/airborne_radar/session/RadarCycleInputBuilder.h"

namespace airborne_radar {
namespace session {

bool RadarCycleInputBuilder::Build(const RadarExternalPoseInput& platform,
                                   const std::vector<TargetExternalKinematics>& targets,
                                   float dt_sec,
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
  output->scene.clear();

  for (std::size_t i = 0; i < targets.size(); ++i) {
    RadarSceneTarget target;
    if (!TryMakeTargetFromExternalKinematics(static_cast<std::uint64_t>(i), targets[i], reference,
                                             output->platform_pose.velocity_mps, &target)) {
      return false;
    }
    output->scene.push_back(target);
  }
  return true;
}

}  // namespace session
}  // namespace airborne_radar
