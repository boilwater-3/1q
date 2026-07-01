#include "1q/airborne_radar/session/ArCycleInputAdapter.h"

namespace airborne_radar {
namespace session {

bool ArCycleInputAdapter::Build(const ArExternalPoseInput& platform,
                                const std::vector<ArExternalTargetInput>& targets,
                                float dt_sec, ArCycleInput* output,
                                ArCoordinateStatus* status) {
  if (!Build(platform, targets, dt_sec, ArEnvironmentInput{}, output, status)) {
    return false;
  }
  output->has_environment = false;
  return true;
}

bool ArCycleInputAdapter::Build(const ArExternalPoseInput& platform,
                                const std::vector<ArExternalTargetInput>& targets,
                                float dt_sec, const ArEnvironmentInput& environment,
                                ArCycleInput* output, ArCoordinateStatus* status) {
  if (status != nullptr) {
    *status = ArCoordinateStatus::kOk;
  }

  if (output == nullptr) {
    if (status != nullptr) {
      *status = ArCoordinateStatus::kNullOutput;
    }
    return false;
  }

  oneq::coordinate::LocalFrameReference reference;
  if (!TryMakeArPoseFromExternalKinematics(platform, &reference, &output->platform_pose, status)) {
    return false;
  }

  output->cycle_index = 0U;
  output->dt_sec = dt_sec;
  output->platform_altitude_m = static_cast<float>(reference.origin_lla.altitude_m);
  output->has_environment = true;
  output->environment = environment;
  output->scene.clear();

  for (std::size_t i = 0; i < targets.size(); ++i) {
    ArSceneTarget target;
    if (!TryMakeArTargetFromExternalKinematics(targets[i], reference,
                                               output->platform_pose.velocity_mps, &target, status)) {
      return false;
    }
    output->scene.push_back(target);
  }
  return true;
}

}  // namespace session
}  // namespace airborne_radar
