#include "1q/electro_optical_sensor/session/EosCycleInputAdapter.h"

#include "1q/coordinate/position_transform.h"

namespace electro_optical_sensor {
namespace session {

bool EosCycleInputAdapter::Build(const EosExternalPoseInput& platform,
                                 const std::vector<EosExternalTargetInput>& targets, float dt_sec,
                                 EosCycleInput* output, EosCoordinateStatus* status) {
  return Build(platform, targets, dt_sec, EosEnvironmentInput{}, output, status);
}

bool EosCycleInputAdapter::Build(const EosExternalPoseInput& platform,
                                 const std::vector<EosExternalTargetInput>& targets, float dt_sec,
                                 const EosEnvironmentInput& environment, EosCycleInput* output,
                                 EosCoordinateStatus* status) {
  if (output == nullptr) {
    if (status != nullptr) {
      *status = EosCoordinateStatus::kNullOutput;
    }
    return false;
  }

  oneq::coordinate::LocalFrameReference reference;
  if (!oneq::coordinate::TryEcefToLla(platform.platform_position_ecef_m, &reference.origin_lla)) {
    if (status != nullptr) {
      *status = EosCoordinateStatus::kCoordinateTransformFail;
    }
    return false;
  }
  reference.frame_attitude_deg = platform.platform_attitude_deg;

  if (!TryMakeEosPoseFromExternalKinematics(platform, reference, &output->platform_pose, status)) {
    return false;
  }

  output->cycle_index = 0U;
  output->dt_sec = dt_sec;
  output->platform_altitude_m = static_cast<float>(reference.origin_lla.altitude_m);
  output->environment = environment;
  output->scene.clear();

  for (std::size_t i = 0; i < targets.size(); ++i) {
    EosSceneTarget target;
    std::uint64_t target_id = targets[i].target_id;
    if (target_id == 0U) {
      target_id = static_cast<std::uint64_t>(i);
    }
    if (!TryMakeEosSceneTargetFromExternalInput(target_id, targets[i],
                                                reference, output->platform_pose, &target,
                                                status)) {
      return false;
    }
    output->scene.push_back(target);
  }
  return true;
}

}  // namespace session
}  // namespace electro_optical_sensor
