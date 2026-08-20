#include "1q/flight_dynamic/guidance/WaypointManager.h"

#include <algorithm>
#include <cmath>

#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/guidance/GreatCircleTrackGeometry.h"
#include "math/FGLocation.h"
#include "models/FGPropagate.h"

namespace oneq {
namespace flight_dynamic {
namespace guidance {

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

WaypointProximity WaypointManager::ResolveProximity() const {
  if (!started_ || active_index_ >= waypoints_.size()) return WaypointProximity{};

  const auto& wp = waypoints_[active_index_];
  const auto& loc = adapter_.GetPropagate().GetLocation();

  // 距离/航向沿用既有查询的同一计算（JSBSim 英尺→米），保证判定行为逐位不变。
  WaypointProximity prox;
  prox.distance_m = loc.GetDistanceTo(wp.longitude_rad, wp.latitude_rad) * 0.3048;
  prox.heading_to_rad = loc.GetHeadingTo(wp.longitude_rad, wp.latitude_rad);

  const great_circle_track::TrackMetricsM track = great_circle_track::ResolveTrackMetricsM(
      leg_start_latitude_rad_, leg_start_longitude_rad_, wp.latitude_rad, wp.longitude_rad,
      loc.GetGeodLatitudeRad(), loc.GetLongitude());
  prox.valid = track.valid;
  prox.cross_track_m = track.cross_track_m;
  prox.along_track_m = track.along_track_m;
  prox.leg_length_m = track.leg_length_m;
  if (!track.valid) return prox;

  // 法平面穿越 + corridor 防护（横向偏差超过 max(3000, 3×radius) 时不判越过，
  // 防止大转弯中提前切航点）。
  const double corridor_width_m = std::max(3000.0, wp.radius_m * 3.0);
  prox.plane_crossed =
      std::fabs(track.cross_track_m) <= corridor_width_m &&
      track.along_track_m > track.leg_length_m;
  return prox;
}

bool WaypointManager::IsAtTarget(double threshold_m) const {
  if (!started_ || waypoints_.empty()) return false;
  const auto& wp = waypoints_[active_index_];
  const double thresh = (threshold_m > 0.0) ? threshold_m : wp.radius_m;
  return ResolveProximity().distance_m < thresh;
}

bool WaypointManager::IsAtOrPastTarget(double threshold_m) const {
  if (!started_ || waypoints_.empty()) return false;
  const auto& wp = waypoints_[active_index_];
  const double thresh = (threshold_m > 0.0) ? threshold_m : wp.radius_m;
  const WaypointProximity prox = ResolveProximity();
  return prox.distance_m < thresh || prox.plane_crossed;
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
