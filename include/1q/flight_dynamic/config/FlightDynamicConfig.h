#ifndef ONEQ_FLIGHT_DYNAMIC_CONFIG_FLIGHTDYNAMICCONFIG_H_
#define ONEQ_FLIGHT_DYNAMIC_CONFIG_FLIGHTDYNAMICCONFIG_H_

#include <string>
#include "1q/coordinate/types.h"

namespace oneq {
namespace flight_dynamic {
namespace config {

struct FlightDynamicConfig {
  std::string aircraft_model;
  std::string aircraft_root_dir;
  double dt_sec = 0.005;
  bool do_trim = true;
  bool silent_mode = true;
  int integrator_rate_rotational = 3;
  int integrator_rate_translational = 3;
  int integrator_pos_rotational = 1;
  int integrator_pos_translational = 4;
  int gravity_model = 1;
  coordinate::ExternalKinematics initial_kinematics;
};

}  // namespace config
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_CONFIG_FLIGHTDYNAMICCONFIG_H_
