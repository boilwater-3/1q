#include "1q/electronic_surveillance_radar/session/EsrCycleInputBuilder.h"

#include "1q/coordinate/position_transform.h"

namespace electronic_surveillance_radar {
namespace session {

bool EsrCycleInputBuilder::Build(const EsrExternalPoseInput& platform,
                                 const std::vector<EsrExternalEmitterInput>& emitters, float dt_sec,
                                 EsrCycleInput* output, EsrCoordinateStatus* status) {
  return Build(platform, emitters, dt_sec, EsrEnvironmentInput{}, output, status);
}

bool EsrCycleInputBuilder::Build(const EsrExternalPoseInput& platform,
                                 const std::vector<EsrExternalEmitterInput>& emitters, float dt_sec,
                                 const EsrEnvironmentInput& environment, EsrCycleInput* output,
                                 EsrCoordinateStatus* status) {
  if (output == nullptr) {
    if (status != nullptr) {
      *status = EsrCoordinateStatus::kNullOutput;
    }
    return false;
  }

  EsrCoordinateReference reference;
  if (!oneq::coordinate::TryEcefToLla(platform.platform_position_ecef_m, &reference.origin_lla)) {
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
  output->platform_altitude_m = static_cast<float>(reference.origin_lla.altitude_m);
  output->environment = environment;
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
