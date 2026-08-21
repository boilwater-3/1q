#include "1q/coordinate/scene_transform.h"

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"

namespace oneq {
namespace coordinate {

bool TryMakeEnuSceneState(const ExternalKinematics& kinematics,
                          const LlaPositionDegM& anchor_lla, EnuSceneState* out) {
  if (out == nullptr) {
    return false;
  }
  *out = EnuSceneState{};

  EnuPositionM position_enu;
  switch (kinematics.position_frame) {
    case PositionFrame::kEcef:
      if (!TryEcefToEnu(kinematics.position_ecef_m, anchor_lla, &position_enu)) {
        return false;
      }
      break;
    case PositionFrame::kLla:
      if (!TryLlaToEnu(kinematics.position_lla_deg_m, anchor_lla, &position_enu)) {
        return false;
      }
      break;
    default:
      return false;
  }

  EnuVelocityMps velocity_enu;
  if (!TryEcefToEnuVelocity(kinematics.velocity_mps, anchor_lla, &velocity_enu)) {
    return false;
  }

  out->position_enu_m = position_enu;
  out->velocity_enu_mps = velocity_enu;
  return true;
}

}  // namespace coordinate
}  // namespace oneq
