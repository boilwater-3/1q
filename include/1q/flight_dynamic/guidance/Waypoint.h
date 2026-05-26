#ifndef ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINT_H_
#define ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINT_H_

#include <string>

namespace oneq {
namespace flight_dynamic {
namespace guidance {

struct Waypoint {
  double latitude_rad = 0.0;
  double longitude_rad = 0.0;
  double altitude_m = 0.0;
  double radius_m = 100.0;
  std::string name;
};

}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINT_H_
