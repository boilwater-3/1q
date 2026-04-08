#include "1q/airborne_radar/model/DecisionTrackSnapshot.h"

#include <cmath>

namespace airborne_radar {
namespace model {

DecisionTrackSnapshot::DecisionTrackSnapshot(float velocity_x_in, float velocity_y_in,
                                             float velocity_z_in, float rcs_in,
                                             float acceleration_x_in, float acceleration_y_in,
                                             float acceleration_z_in, bool jamming_detected_in,
                                             std::uint64_t external_target_id_in,
                                             std::uint64_t association_key_in) {
  state.association_key = association_key_in;
  state.external_target_id = external_target_id_in;
  state.velocity_x = velocity_x_in;
  state.velocity_y = velocity_y_in;
  state.velocity_z = velocity_z_in;
  state.speed = std::sqrt(velocity_x_in * velocity_x_in + velocity_y_in * velocity_y_in +
                          velocity_z_in * velocity_z_in);
  state.acceleration_x = acceleration_x_in;
  state.acceleration_y = acceleration_y_in;
  state.acceleration_z = acceleration_z_in;
  state.acceleration =
      std::sqrt(acceleration_x_in * acceleration_x_in + acceleration_y_in * acceleration_y_in +
                acceleration_z_in * acceleration_z_in);
  state.rcs = rcs_in;
  state.jamming_detected = jamming_detected_in;
}

}  // namespace model
}  // namespace airborne_radar
