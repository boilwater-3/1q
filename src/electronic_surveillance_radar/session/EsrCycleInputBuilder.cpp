#include "1q/electronic_surveillance_radar/session/EsrCycleInputBuilder.h"

namespace electronic_surveillance_radar {
namespace session {

bool EsrCycleInputBuilder::Build(const EsrExternalPoseInput& platform,
                                 const std::vector<EsrExternalEmitterInput>& emitters,
                                 float dt_sec,
                                 EsrCycleInput* output,
                                 EsrCoordinateStatus* status) {
  if (output == nullptr) {
    if (status != nullptr) {
      *status = EsrCoordinateStatus::kNullOutput;
    }
    return false;
  }

  EsrCoordinateReference reference;
  if (!oneq::foundation::TryEcefToLla(platform.platform_position_ecef_m, &reference.origin_lla)) {
    if (status != nullptr) {
      *status = EsrCoordinateStatus::kCoordinateTransformFail;
    }
    return false;
  }
  reference.frame_attitude_deg = platform.platform_attitude_deg;

  if (!TryMakeEsrPoseFromExternalKinematics(platform, reference, &output->platform_pose, status)) {
    return false;
  }

  output->cycle_index = 0U;
  output->dt_sec = dt_sec;
  output->scene.clear();

  for (const auto& emitter_input : emitters) {
    EsrSceneEmitter emitter;
    if (!TryMakeEsrSceneEmitterFromExternalInput(emitter_input, reference, &emitter, status)) {
      return false;
    }
    output->scene.push_back(emitter);
  }
  return true;
}

}  // namespace session
}  // namespace electronic_surveillance_radar
