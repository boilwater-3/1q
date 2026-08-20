#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"

#include "1q/coordinate/scene_transform.h"

namespace electro_optical_sensor {
namespace session {

namespace {

void SetStatus(EosCoordinateStatus value, EosCoordinateStatus* status) {
  if (status != nullptr) {
    *status = value;
  }
}

}  // namespace

bool TryMakeEosSceneTargetFromExternalInput(
    std::uint64_t target_id, const EosExternalTargetInput& input,
    const oneq::coordinate::LocalFrameReference& reference, EosSceneTarget* target,
    EosCoordinateStatus* status) {
  if (target == nullptr) {
    SetStatus(EosCoordinateStatus::kNullOutput, status);
    return false;
  }

  oneq::coordinate::EnuSceneState enu;
  if (!oneq::coordinate::TryMakeEnuSceneState(input.kinematics, reference.origin_lla, &enu)) {
    SetStatus(EosCoordinateStatus::kCoordinateTransformFail, status);
    return false;
  }

  target->target_id = target_id;
  target->target_name = input.target_name;
  target->position_x = static_cast<float>(enu.position_enu_m.east_m);
  target->position_y = static_cast<float>(enu.position_enu_m.north_m);
  target->position_z = static_cast<float>(enu.position_enu_m.up_m);
  target->velocity_x = static_cast<float>(enu.velocity_enu_mps.east_mps);
  target->velocity_y = static_cast<float>(enu.velocity_enu_mps.north_mps);
  target->velocity_z = static_cast<float>(enu.velocity_enu_mps.up_mps);
  target->appearance = input.appearance;
  SetStatus(EosCoordinateStatus::kOk, status);
  return true;
}

}  // namespace session
}  // namespace electro_optical_sensor
