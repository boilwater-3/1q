#include "airborne_radar/session/RadarSceneTargetConversion.h"

#include <cmath>

namespace airborne_radar {
namespace session {

model::TargetFeature ToModelTargetFeature(const RadarSceneTarget& input) {
  model::TargetFeature out;
  out.external_target_id = input.external_target_id;
  out.current_track_velocity_x = input.velocity_x;
  out.current_track_velocity_y = input.velocity_y;
  out.current_track_velocity_z = input.velocity_z;
  out.current_track_speed = std::sqrt(input.velocity_x * input.velocity_x +
                                      input.velocity_y * input.velocity_y +
                                      input.velocity_z * input.velocity_z);
  out.current_track_rcs = input.rcs;
  out.range_m = input.range_m;
  out.has_cartesian_position = input.has_cartesian_position;
  out.position_x = input.position_x;
  out.position_y = input.position_y;
  out.position_z = input.position_z;
  out.target_swerling_type = input.target_swerling_type;
  return out;
}

model::TargetFeatureList ToModelTargetFeatureList(const RadarSceneTargetList& input) {
  model::TargetFeatureList out;
  out.reserve(input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    out.push_back(ToModelTargetFeature(input[i]));
  }
  return out;
}

RadarSceneTarget ToSceneTarget(const model::TargetFeature& input) {
  RadarSceneTarget out;
  out.external_target_id = input.external_target_id;
  out.velocity_x = input.current_track_velocity_x;
  out.velocity_y = input.current_track_velocity_y;
  out.velocity_z = input.current_track_velocity_z;
  out.rcs = input.current_track_rcs;
  out.range_m = input.range_m;
  out.has_cartesian_position = input.has_cartesian_position;
  out.position_x = input.position_x;
  out.position_y = input.position_y;
  out.position_z = input.position_z;
  out.target_swerling_type = input.target_swerling_type;
  return out;
}

RadarSceneTargetList ToSceneTargetList(const model::TargetFeatureList& input) {
  RadarSceneTargetList out;
  out.reserve(input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    out.push_back(ToSceneTarget(input[i]));
  }
  return out;
}

}  // namespace session
}  // namespace airborne_radar
