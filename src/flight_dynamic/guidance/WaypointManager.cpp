#include "1q/flight_dynamic/guidance/WaypointManager.h"

#include <cmath>

#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "math/FGLocation.h"
#include "models/FGPropagate.h"

namespace oneq {
namespace flight_dynamic {
namespace guidance {

namespace {

constexpr double kEarthRadiusM = 6378137.0;

struct LocalPointM {
  double north_m = 0.0;
  double east_m = 0.0;
};

LocalPointM ToLocalMeters(double latitude_rad, double longitude_rad, double origin_latitude_rad,
                          double origin_longitude_rad) {
  const double mean_latitude_rad = 0.5 * (latitude_rad + origin_latitude_rad);
  LocalPointM point;
  point.north_m = (latitude_rad - origin_latitude_rad) * kEarthRadiusM;
  point.east_m =
      (longitude_rad - origin_longitude_rad) * kEarthRadiusM * std::cos(mean_latitude_rad);
  return point;
}

}  // namespace

WaypointManager::WaypointManager(adapter::JsbsimAdapter& adapter) : adapter_(adapter) {}

void WaypointManager::AddWaypoint(const Waypoint& wp) { waypoints_.push_back(wp); }

void WaypointManager::ClearWaypoints() {
  waypoints_.clear();
  active_index_ = 0;
  started_ = false;
}

void WaypointManager::SetActiveWaypoint(size_t index) {
  if (index < waypoints_.size()) {
    active_index_ = index;
    if (active_index_ == 0) {
      SetLegStartFromCurrentLocation();
    } else {
      const auto& previous_wp = waypoints_[active_index_ - 1];
      leg_start_latitude_rad_ = previous_wp.latitude_rad;
      leg_start_longitude_rad_ = previous_wp.longitude_rad;
    }
    ApplyActiveWaypoint();
  }
}

void WaypointManager::Start() {
  if (waypoints_.empty()) return;
  started_ = true;
  active_index_ = 0;
  SetLegStartFromCurrentLocation();
  ApplyActiveWaypoint();
}

bool WaypointManager::AdvanceToNext() {
  if (active_index_ + 1 < waypoints_.size()) {
    ++active_index_;
    const auto& previous_wp = waypoints_[active_index_ - 1];
    leg_start_latitude_rad_ = previous_wp.latitude_rad;
    leg_start_longitude_rad_ = previous_wp.longitude_rad;
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

bool WaypointManager::IsAtOrPastTarget(double threshold_m) const {
  return IsAtTarget(threshold_m) || HasPassedActiveWaypoint();
}

bool WaypointManager::IsFinished() const { return !started_ || active_index_ >= waypoints_.size(); }

double WaypointManager::GetDistanceToActiveM() const {
  if (active_index_ < waypoints_.size() && started_) {
    const auto& wp = waypoints_[active_index_];
    const auto& loc = adapter_.GetPropagate().GetLocation();
    return loc.GetDistanceTo(wp.longitude_rad, wp.latitude_rad) * 0.3048;
  }
  return 0.0;
}

double WaypointManager::GetHeadingToActiveRad() const {
  if (active_index_ < waypoints_.size() && started_) {
    const auto& wp = waypoints_[active_index_];
    const auto& loc = adapter_.GetPropagate().GetLocation();
    return loc.GetHeadingTo(wp.longitude_rad, wp.latitude_rad);
  }
  return 0.0;
}

void WaypointManager::SetLegStartFromCurrentLocation() {
  const auto& loc = adapter_.GetPropagate().GetLocation();
  leg_start_latitude_rad_ = loc.GetGeodLatitudeRad();
  leg_start_longitude_rad_ = loc.GetLongitude();
}

bool WaypointManager::HasPassedActiveWaypoint() const {
  if (!started_ || active_index_ >= waypoints_.size()) return false;

  const auto& wp = waypoints_[active_index_];
  const auto& loc = adapter_.GetPropagate().GetLocation();
  const LocalPointM target = ToLocalMeters(wp.latitude_rad, wp.longitude_rad,
                                           leg_start_latitude_rad_, leg_start_longitude_rad_);
  const LocalPointM aircraft = ToLocalMeters(loc.GetGeodLatitudeRad(), loc.GetLongitude(),
                                             leg_start_latitude_rad_, leg_start_longitude_rad_);

  const double leg_norm_sq = target.north_m * target.north_m + target.east_m * target.east_m;
  if (leg_norm_sq < 1.0) return false;

  const double target_to_aircraft_north_m = aircraft.north_m - target.north_m;
  const double target_to_aircraft_east_m = aircraft.east_m - target.east_m;
  const double leg_norm = std::sqrt(leg_norm_sq);
  
  const double along_track_past_m =
      (target.north_m * target_to_aircraft_north_m + target.east_m * target_to_aircraft_east_m) / leg_norm;

  const double cross_track_m = std::abs(
      (target.north_m * target_to_aircraft_east_m - target.east_m * target_to_aircraft_north_m) / leg_norm);

  // If the aircraft is too far off-track, crossing the perpendicular plane shouldn't count as "passing".
  // This prevents premature sequencing during large sweeping turns onto the leg.
  double corridor_width_m = std::max(3000.0, wp.radius_m * 3.0);
  if (cross_track_m > corridor_width_m) return false;

  return along_track_past_m > 0.0;
}

void WaypointManager::ApplyActiveWaypoint() {
  if (active_index_ >= waypoints_.size()) return;
  const auto& wp = waypoints_[active_index_];
  adapter_.SetProperty("guidance/target_wp_latitude_rad", wp.latitude_rad);
  adapter_.SetProperty("guidance/target_wp_longitude_rad", wp.longitude_rad);
  adapter_.SetProperty("ap/active-waypoint", static_cast<double>(active_index_));
}

}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq
