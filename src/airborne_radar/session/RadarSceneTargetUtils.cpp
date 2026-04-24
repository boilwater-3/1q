#include "1q/airborne_radar/session/RadarSceneTargetUtils.h"

#include <cmath>
#include <cstddef>

namespace airborne_radar {
namespace session {

namespace {

float ComputeNorm3(float x, float y, float z) { return std::sqrt(x * x + y * y + z * z); }

}  // namespace

RadarSceneTarget MakeSceneTarget(std::uint64_t external_target_id, float position_x,
                                 float position_y, float position_z, float velocity_x,
                                 float velocity_y, float velocity_z, float rcs,
                                 int swerling_type) {
  RadarSceneTarget target;
  target.external_target_id = external_target_id;
  target.velocity_x = velocity_x;
  target.velocity_y = velocity_y;
  target.velocity_z = velocity_z;
  target.rcs = rcs;
  target.position_x = position_x;
  target.position_y = position_y;
  target.position_z = position_z;
  target.target_swerling_type = swerling_type;
  NormalizeSceneTargetGeometry(&target);
  return target;
}

RadarSceneTarget MakeGroundSceneTarget(std::uint64_t external_target_id, float position_x,
                                       float position_y, float rcs, float velocity_x,
                                       float velocity_y, int swerling_type) {
  return MakeSceneTarget(external_target_id, position_x, position_y, 0.0f, velocity_x, velocity_y,
                         0.0f, rcs, swerling_type);
}

RadarSceneTarget MakeAirSceneTarget(std::uint64_t external_target_id, float position_x,
                                    float position_y, float position_z, float velocity_x,
                                    float velocity_y, float velocity_z, float rcs,
                                    int swerling_type) {
  return MakeSceneTarget(external_target_id, position_x, position_y, position_z, velocity_x,
                         velocity_y, velocity_z, rcs, swerling_type);
}

void NormalizeSceneTargetGeometry(RadarSceneTarget* target) {
  if (target == nullptr) {
    return;
  }
  if (target->range_m > 0.0f) {
    return;
  }
  if (target->position_x == 0.0f && target->position_y == 0.0f && target->position_z == 0.0f) {
    return;
  }
  target->range_m = ComputeNorm3(target->position_x, target->position_y, target->position_z);
}

void NormalizeSceneTargetGeometry(RadarSceneTargetList* targets) {
  if (targets == nullptr) {
    return;
  }
  for (std::size_t i = 0; i < targets->size(); ++i) {
    NormalizeSceneTargetGeometry(&(*targets)[i]);
  }
}

}  // namespace session
}  // namespace airborne_radar
