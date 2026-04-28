#include "1q/electro_optical_sensor/session/EosCycleInputBuilder.h"

namespace electro_optical_sensor {
namespace session {

bool EosCycleInputBuilder::Build(const EosExternalPoseInput& platform,
                                 const std::vector<EosExternalTargetInput>& targets,
                                 float dt_sec,
                                 EosCycleInput* output,
                                 EosCoordinateStatus* status) {
  if (output == nullptr) {
    if (status != nullptr) {
      *status = EosCoordinateStatus::kNullOutput;
    }
    return false;
  }

  EosCoordinateReference reference;
  if (!oneq::foundation::TryEcefToLla(platform.platform_position_ecef_m, &reference.origin_lla)) {
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
  output->scene.clear();

  for (std::size_t i = 0; i < targets.size(); ++i) {
    EosSceneTarget target;
    if (!TryMakeEosSceneTargetFromExternalInput(static_cast<std::uint64_t>(i), targets[i], reference,
                                                output->platform_pose, &target, status)) {
      return false;
    }
    output->scene.push_back(target);
  }
  return true;
}

}  // namespace session
}  // namespace electro_optical_sensor
