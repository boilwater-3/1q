#include "1q/flight_dynamic/guidance/WaypointManager.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"

#include <cmath>

namespace oneq {
namespace flight_dynamic {
namespace guidance {

WaypointManager::WaypointManager(adapter::JsbsimAdapter& adapter)
    : adapter_(adapter) {}

void WaypointManager::AddWaypoint(const Waypoint& wp) {
  waypoints_.push_back(wp);
}

void WaypointManager::ClearWaypoints() {
  waypoints_.clear();
  active_index_ = 0;
  started_ = false;
}

void WaypointManager::SetActiveWaypoint(size_t index) {
  if (index < waypoints_.size()) {
    active_index_ = index;
    ApplyActiveWaypoint();
  }
}

void WaypointManager::Start() {
  if (waypoints_.empty()) return;
  started_ = true;
  active_index_ = 0;
  ApplyActiveWaypoint();
}

bool WaypointManager::AdvanceToNext() {
  if (active_index_ + 1 < waypoints_.size()) {
    ++active_index_;
    ApplyActiveWaypoint();
    return true;
  }
  return false;
}

bool WaypointManager::IsAtTarget(double threshold_m) const {
  if (!started_ || waypoints_.empty()) return false;
  double distance_m = GetDistanceToActiveM();
  const auto& wp = waypoints_[active_index_];
  double thresh = (threshold_m > 0.0) ? threshold_m : wp.radius_m;
  return distance_m < thresh;
}

bool WaypointManager::IsFinished() const {
  return !started_ || active_index_ >= waypoints_.size();
}

double WaypointManager::GetDistanceToActiveM() const {
  double dist_ft = adapter_.GetProperty("guidance/wp-distance");
  return dist_ft * 0.3048;
}

double WaypointManager::GetHeadingToActiveRad() const {
  return adapter_.GetProperty("guidance/wp-heading-rad");
}

void WaypointManager::ApplyActiveWaypoint() {
  if (active_index_ >= waypoints_.size()) return;
  const auto& wp = waypoints_[active_index_];
  adapter_.SetProperty("guidance/target_wp_latitude_rad", wp.latitude_rad);
  adapter_.SetProperty("guidance/target_wp_longitude_rad", wp.longitude_rad);
  adapter_.SetProperty("ap/active-waypoint",
                       static_cast<double>(active_index_));
}

}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq
