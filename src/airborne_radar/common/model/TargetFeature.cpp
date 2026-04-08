#include "1q/airborne_radar/model/TargetFeature.h"

#include <cmath>

namespace airborne_radar {
namespace model {

TargetFeature::TargetFeature(float velocity_x, float velocity_y, float velocity_z, float rcs,
                             float range, int swerling_type, std::uint64_t ext_target_id)
    : external_target_id(ext_target_id),
      current_track_velocity_x(velocity_x),
      current_track_velocity_y(velocity_y),
      current_track_velocity_z(velocity_z),
      current_track_speed(
          std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y + velocity_z * velocity_z)),
      current_track_rcs(rcs),
      range_m(range),
      target_swerling_type(swerling_type) {}

}  // namespace model
}  // namespace airborne_radar
