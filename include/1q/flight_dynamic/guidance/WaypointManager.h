#ifndef ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINTMANAGER_H_
#define ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINTMANAGER_H_

#include <vector>

#include "1q/flight_dynamic/guidance/Waypoint.h"

namespace oneq {
namespace flight_dynamic {

namespace adapter {
class JsbsimAdapter;
}

namespace guidance {

class WaypointManager {
 public:
  explicit WaypointManager(adapter::JsbsimAdapter& adapter);

  void AddWaypoint(const Waypoint& wp);
  void ClearWaypoints();
  void SetActiveWaypoint(size_t index);
  void Start();

  bool AdvanceToNext();
  bool IsAtTarget(double threshold_m = -1.0) const;
  bool IsAtOrPastTarget(double threshold_m = -1.0) const;
  bool IsFinished() const;

  double GetDistanceToActiveM() const;
  double GetHeadingToActiveRad() const;
  size_t GetActiveIndex() const { return active_index_; }
  size_t GetWaypointCount() const { return waypoints_.size(); }

 private:
  void SetLegStartFromCurrentLocation();
  bool HasPassedActiveWaypoint() const;
  void ApplyActiveWaypoint();

  adapter::JsbsimAdapter& adapter_;
  std::vector<Waypoint> waypoints_;
  size_t active_index_ = 0;
  bool started_ = false;
  double leg_start_latitude_rad_ = 0.0;
  double leg_start_longitude_rad_ = 0.0;
};

}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINTMANAGER_H_
