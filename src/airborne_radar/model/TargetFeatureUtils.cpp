#include "1q/airborne_radar/model/TargetFeatureUtils.h"

#include <cmath>

namespace airborne_radar {
namespace model {

namespace {

float ComputeNorm3(float x, float y, float z) { return std::sqrt(x * x + y * y + z * z); }

bool HasCartesianPosition(const model::TargetFeature& target) {
  return target.has_cartesian_position;
}

void RefreshDerivedKinematics(model::TargetFeature* target) {
  if (target == nullptr) {
    return;
  }
  target->current_track_speed =
      ComputeNorm3(target->current_track_velocity_x, target->current_track_velocity_y,
                   target->current_track_velocity_z);
}

}  // namespace

model::TargetFeature MakeTargetFromCartesian(std::uint64_t external_target_id, float position_x,
                                             float position_y, float position_z, float velocity_x,
                                             float velocity_y, float velocity_z, float rcs,
                                             int swerling_type) {
  model::TargetFeature target(velocity_x, velocity_y, velocity_z, rcs, 0.0f, swerling_type,
                              external_target_id);
  target.position_x = position_x;
  target.position_y = position_y;
  target.position_z = position_z;
  target.has_cartesian_position = true;
  NormalizeTargetGeometry(&target);
  return target;
}

model::TargetFeature MakeGroundTarget(std::uint64_t external_target_id, float position_x,
                                      float position_y, float rcs, float velocity_x,
                                      float velocity_y, int swerling_type) {
  return MakeTargetFromCartesian(external_target_id, position_x, position_y, 0.0f, velocity_x,
                                 velocity_y, 0.0f, rcs, swerling_type);
}

model::TargetFeature MakeAirTarget(std::uint64_t external_target_id, float position_x,
                                   float position_y, float position_z, float velocity_x,
                                   float velocity_y, float velocity_z, float rcs,
                                   int swerling_type) {
  return MakeTargetFromCartesian(external_target_id, position_x, position_y, position_z, velocity_x,
                                 velocity_y, velocity_z, rcs, swerling_type);
}

void NormalizeTargetGeometry(model::TargetFeature* target) {
  if (target == nullptr) {
    return;
  }

  RefreshDerivedKinematics(target);
  if (target->range_m > 0.0f || !HasCartesianPosition(*target)) {
    return;
  }
  target->range_m = ComputeNorm3(target->position_x, target->position_y, target->position_z);
}

void NormalizeTargetGeometry(model::TargetFeatureList* targets) {
  if (targets == nullptr) {
    return;
  }
  for (std::size_t i = 0; i < targets->size(); ++i) {
    NormalizeTargetGeometry(&(*targets)[i]);
  }
}

}  // namespace model
}  // namespace airborne_radar
