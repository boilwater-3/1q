#include "1q/flight_dynamic/guidance/Maneuver.h"

#include <cmath>

#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/guidance/WaypointManager.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "math/FGLocation.h"
#include "models/FGPropagate.h"

namespace oneq {
namespace flight_dynamic {
namespace guidance {

namespace {

constexpr double kEarthRadiusM = 6378137.0;

double NormalizeRad(double angle_rad) {
  while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
  while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
  return angle_rad;
}

double ComputeClockwiseOrbitHeadingRad(const JSBSim::FGLocation& location, const Waypoint& center,
                                       double radius_m) {
  double lat_rad = location.GetGeodLatitudeRad();
  double lon_rad = location.GetLongitude();
  double cos_lat = std::cos(center.latitude_rad);
  double north_m = (lat_rad - center.latitude_rad) * kEarthRadiusM;
  double east_m = (lon_rad - center.longitude_rad) * kEarthRadiusM * cos_lat;
  double distance_m = std::hypot(north_m, east_m);
  if (distance_m < 1.0) {
    return location.GetHeadingTo(center.longitude_rad, center.latitude_rad);
  }

  double radial_angle = std::atan2(east_m, north_m);
  double tangent_heading = radial_angle + M_PI / 2.0;
  double radial_error = distance_m - radius_m;
  double intercept = std::atan2(radial_error, radius_m);
  return NormalizeRad(tangent_heading + intercept);
}

}  // namespace

ManeuverExecutor::ManeuverExecutor(adapter::JsbsimAdapter& adapter, autopilot::Autopilot& ap,
                                   WaypointManager& wp_manager)
    : adapter_(adapter), ap_(ap), wp_manager_(wp_manager) {}

void ManeuverExecutor::ExecuteFlyTo(const Waypoint& target) {
  current_maneuver_.type = ManeuverType::kFlyToWaypoint;
  current_maneuver_.target = target;
  active_ = true;
  elapsed_sec_ = 0.0;

  wp_manager_.ClearWaypoints();
  wp_manager_.AddWaypoint(target);
  wp_manager_.Start();

  ap_.SetHeadingSourceIsWaypoint(true);
  ap_.SetRollAttitudeMode(1);
  ap_.SetRollAutopilotOn(true);
  ap_.SetHeadingHold(true);
  ap_.SetAltitudeTargetM(target.altitude_m);
  ap_.SetAltitudeHold(true);
}

void ManeuverExecutor::ExecuteOrbit(const Waypoint& center, double radius_m) {
  current_maneuver_.type = ManeuverType::kOrbit;
  current_maneuver_.target = center;
  current_maneuver_.value = radius_m;
  active_ = true;
  elapsed_sec_ = 0.0;

  // Use a waypoint at center + offset for circular track
  Waypoint orbit_wp = center;
  orbit_wp.radius_m = radius_m;
  wp_manager_.ClearWaypoints();
  wp_manager_.AddWaypoint(orbit_wp);
  wp_manager_.Start();

  ap_.SetHeadingSourceIsWaypoint(false);
  ap_.SetRollAttitudeMode(1);
  ap_.SetRollAutopilotOn(true);
  ap_.SetHeadingHold(true);
  ap_.SetAltitudeTargetM(center.altitude_m);
  ap_.SetAltitudeHold(true);
}

void ManeuverExecutor::ExecuteSetHeading(double heading_rad) {
  current_maneuver_.type = ManeuverType::kSetHeading;
  current_maneuver_.value = heading_rad;
  active_ = true;
  elapsed_sec_ = 0.0;

  ap_.SetHeadingSourceIsWaypoint(false);
  ap_.SetHeadingTargetRad(heading_rad);
  ap_.SetRollAttitudeMode(1);
  ap_.SetRollAutopilotOn(true);
  ap_.SetHeadingHold(true);
}

void ManeuverExecutor::ExecuteSetAltitude(double altitude_m) {
  current_maneuver_.type = ManeuverType::kSetAltitude;
  current_maneuver_.value = altitude_m;
  active_ = true;
  elapsed_sec_ = 0.0;

  ap_.SetAltitudeTargetM(altitude_m);
  ap_.SetAltitudeHold(true);
}

void ManeuverExecutor::ExecuteSetPitch(double pitch_deg, double duration_sec) {
  current_maneuver_.type = ManeuverType::kSetPitch;
  current_maneuver_.value = pitch_deg;
  current_maneuver_.duration_sec = duration_sec;
  active_ = true;
  elapsed_sec_ = 0.0;

  ap_.SetPitchTargetDeg(pitch_deg);
  ap_.SetPitchHold(true);
}

void ManeuverExecutor::ExecuteSetRoll(int roll_mode) {
  current_maneuver_.type = ManeuverType::kSetRoll;
  current_maneuver_.value = static_cast<double>(roll_mode);
  active_ = true;
  elapsed_sec_ = 0.0;

  ap_.SetRollAttitudeMode(roll_mode);
  ap_.SetRollAutopilotOn(true);
}

bool ManeuverExecutor::IsManeuverComplete() const {
  if (!active_) return true;

  switch (current_maneuver_.type) {
    case ManeuverType::kFlyToWaypoint:
      return wp_manager_.IsAtTarget();
    case ManeuverType::kOrbit:
      return false;
    case ManeuverType::kSetHeading: {
      double angle_error = std::abs(ap_.GetAngleToHeadingRad());
      return angle_error < 0.035;  // ~2 degrees
    }
    case ManeuverType::kSetAltitude: {
      double alt_error = std::abs(ap_.GetAltitudeASLM() - current_maneuver_.value);
      return alt_error < 10.0;  // within 10m
    }
    case ManeuverType::kSetPitch:
      return current_maneuver_.duration_sec > 0.0 && elapsed_sec_ >= current_maneuver_.duration_sec;
    case ManeuverType::kSetRoll:
      return true;
  }
  return true;
}

void ManeuverExecutor::Update(double dt_sec) {
  if (!active_) return;
  elapsed_sec_ += dt_sec;
  if (current_maneuver_.type == ManeuverType::kOrbit) {
    double radius_m = std::abs(current_maneuver_.value);
    if (radius_m < 1.0) radius_m = 1.0;
    double heading_rad = ComputeClockwiseOrbitHeadingRad(adapter_.GetPropagate().GetLocation(),
                                                         current_maneuver_.target, radius_m);
    ap_.SetHeadingTargetRad(heading_rad);
  }
}

void ManeuverExecutor::Abort() { active_ = false; }

}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq
