#ifndef ONEQ_FLIGHT_DYNAMIC_GUIDANCE_MANEUVER_H_
#define ONEQ_FLIGHT_DYNAMIC_GUIDANCE_MANEUVER_H_

#include <vector>

#include "1q/flight_dynamic/guidance/Waypoint.h"

namespace oneq {
namespace flight_dynamic {

namespace adapter {
class JsbsimAdapter;
}
namespace autopilot {
class Autopilot;
}
namespace guidance {
class WaypointManager;
}

namespace guidance {

enum class ManeuverType {
  kFlyToWaypoint,
  kOrbit,
  kSetHeading,
  kSetAltitude,
  kSetPitch,
  kSetRoll,
};

struct Maneuver {
  ManeuverType type;
  Waypoint target;
  double value = 0.0;
  double duration_sec = 0.0;
};

class ManeuverExecutor {
 public:
  ManeuverExecutor(adapter::JsbsimAdapter& adapter,
                   autopilot::Autopilot& ap,
                   WaypointManager& wp_manager);

  void ExecuteFlyTo(const Waypoint& target);
  void ExecuteOrbit(const Waypoint& center, double radius_m, double duration_sec = 0.0);
  void ExecuteSetHeading(double heading_rad);
  void ExecuteSetAltitude(double altitude_m);
  void ExecuteSetPitch(double pitch_deg, double duration_sec);
  void ExecuteSetRoll(int roll_mode);

  bool IsManeuverComplete() const;
  void Update(double dt_sec);
  void Abort();

 private:
  adapter::JsbsimAdapter& adapter_;
  autopilot::Autopilot& ap_;
  WaypointManager& wp_manager_;
  Maneuver current_maneuver_;
  bool active_ = false;
  double elapsed_sec_ = 0.0;
};

}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_GUIDANCE_MANEUVER_H_
