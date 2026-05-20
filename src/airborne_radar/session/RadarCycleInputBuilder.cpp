#include "1q/airborne_radar/session/RadarCycleInputBuilder.h"

namespace airborne_radar {
namespace session {

bool RadarCycleInputBuilder::Build(const RadarExternalPoseInput& platform,
                                   const std::vector<RadarExternalTargetInput>& targets,
                                   float dt_sec, RadarCycleInput* output,
                                   RadarCoordinateStatus* status) {
  return Build(platform, targets, dt_sec, RadarEnvironmentInput{}, output, status);
}

bool RadarCycleInputBuilder::Build(const RadarExternalPoseInput& platform,
                                   const std::vector<RadarExternalTargetInput>& targets,
                                   float dt_sec, const RadarEnvironmentInput& environment,
                                   RadarCycleInput* output, RadarCoordinateStatus* status) {
  if (status != nullptr) {
    *status = RadarCoordinateStatus::kOk;
  }

  if (output == nullptr) {
    if (status != nullptr) {
      *status = RadarCoordinateStatus::kNullOutput;
    }
    return false;
  }

  RadarLocalFrameReference reference;
  if (!TryMakeRadarPoseFromExternalKinematics(platform, &reference, &output->platform_pose, status)) {
    return false;
  }

  output->cycle_index = 0U;
  output->dt_sec = dt_sec;
  output->platform_altitude_m = static_cast<float>(reference.origin_lla.altitude_m);
  output->environment = environment;
  output->scene.clear();

  for (std::size_t i = 0; i < targets.size(); ++i) {
    RadarSceneTarget target;
    if (!TryMakeTargetFromExternalKinematics(targets[i], reference,
                                             output->platform_pose.velocity_mps, &target, status)) {
      return false;
    }
    output->scene.push_back(target);
  }
  return true;
}

}  // namespace session
}  // namespace airborne_radar
