#include "1q/airborne_radar/session/ArCycleInputAdapter.h"

namespace airborne_radar {
namespace session {

bool ArCycleInputAdapter::Build(const ArExternalPoseInput& platform,
                                const std::vector<ArExternalTargetInput>& targets, float dt_sec,
                                ArCycleInput* output, ArCoordinateStatus* status) {
  if (!Build(platform, targets, dt_sec, ArEnvironmentInput{}, output, status)) {
    return false;
  }
  return true;
}

bool ArCycleInputAdapter::Build(const ArExternalPoseInput& platform,
                                const std::vector<ArExternalTargetInput>& targets, float dt_sec,
                                const ArEnvironmentInput& environment, ArCycleInput* output,
                                ArCoordinateStatus* status) {
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
  oneq::foundation::PoseState platform_pose;
  if (!TryMakeArPoseFromExternalKinematics(platform, &reference, &platform_pose, status)) {
    return false;
  }
  for (std::size_t index = 0U; index < targets.size(); ++index) {
    ArSceneTarget local_target;
    if (!TryMakeArTargetFromExternalKinematics(targets[index], reference,
                                               platform_pose.velocity_mps,
                                               &local_target, status)) {
      return false;
    }
  }

  output->cycle_index = 1U;
  output->cycle_start_time_s = 0.0;
  output->dt_sec = dt_sec;
  output->platform = platform;
  output->targets = targets;
  output->environment = environment;
  output->interference = oneq::electromagnetics::RfEmissionFrame{};
  return true;
}

}  // namespace session
}  // namespace airborne_radar
