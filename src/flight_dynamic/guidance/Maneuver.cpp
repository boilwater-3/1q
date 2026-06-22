#include "1q/flight_dynamic/guidance/Maneuver.h"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/guidance/WaypointManager.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/adapter/PropertyNames.h"
#include "flight_dynamic/propulsion/EngineManager.h"
#include "math/FGLocation.h"
#include "models/FGPropagate.h"

namespace oneq {
namespace flight_dynamic {
namespace guidance {

namespace {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kLandingHighSpeedOrbitFactor = 1.35;
constexpr double kLandingHighAltitudeOrbitMinSpeedMps = 120.0;
constexpr double kLandingHighAltitudeOrbitMinRadiusM = 3000.0;
constexpr double kLandingHighAltitudeOrbitRadiusPerSpeed = 20.0;
constexpr double kLandingPatternCaptureMarginM = 500.0;
constexpr double kLandingPatternOrbitMinRadiusM = 2000.0;
constexpr double kLandingPatternOrbitRadiusPerSpeed = 15.0;
constexpr double kLandingOrbitHeadingMinSpeedMps = 10.0;
constexpr double kLandingApproachEntrySpeedFactor = 1.15;
constexpr double kLandingOrbitDisabledHighSpeedMps = 200.0;
constexpr double kLandingIdleThrottle = 0.10;
constexpr double kLandingFbwDecelElevator = -0.05;
constexpr double kLandingLevelSinkTargetMps = 0.0;
constexpr double kLandingHighSpeedFallbackElevatorGain = 0.05;
constexpr double kLandingHighSpeedFallbackElevatorLimit = 0.15;
constexpr double kLandingApproachHighAltitudeBandM = 500.0;
constexpr double kLandingApproachThrottleBase = 0.30;
constexpr double kLandingApproachThrottleAltGain = 0.0005;
constexpr double kLandingApproachThrottleMin = 0.10;
constexpr double kLandingApproachThrottleMax = 0.50;
constexpr double kLandingApproachSpeedElevatorGain = 0.005;
constexpr double kLandingApproachElevatorMin = -0.15;
constexpr double kLandingApproachElevatorMax = 0.10;
constexpr double kLandingTargetFpaDeg = -3.0;
constexpr double kLandingRadToDeg = 57.3;
constexpr double kLandingFpaElevatorGain = 0.10;
constexpr double kLandingFpaElevatorLimit = 0.20;
constexpr double kLandingNearPatternThrottleBase = 0.30;
constexpr double kLandingNearPatternThrottleSpeedGain = 0.005;
constexpr double kLandingNearPatternThrottleMin = 0.10;
constexpr double kLandingNearPatternThrottleMax = 0.60;
constexpr double kLandingFinalTooFastFactor = 1.30;
constexpr double kLandingFinalTooFastElevator = -0.10;
constexpr double kLandingFlareMinAltitudeM = 15.0;
constexpr double kLandingFlareAltitudePerSpeed = 0.60;
constexpr double kLandingHeavyFlareInitialElevator = -0.08;
constexpr double kLandingStandardFlareInitialElevator = -0.25;
constexpr double kLandingFinalSinkTargetMps = -3.0;
constexpr double kLandingFinalThrottleBase = 0.25;
constexpr double kLandingFinalThrottleSinkGain = 0.05;
constexpr double kLandingFinalThrottleMin = 0.0;
constexpr double kLandingFinalThrottleMax = 0.60;
constexpr double kLandingFinalThrottleCapSpeedFactor = 1.05;
constexpr double kLandingHeavyFlareScaleM = 30.0;
constexpr double kLandingHeavyFlareBaseElevator = -0.08;
constexpr double kLandingHeavyFlareProgressElevator = -0.12;
constexpr double kLandingStandardFlareScaleM = 30.0;
constexpr double kLandingStandardFlareBaseElevator = -0.15;
constexpr double kLandingStandardFlareProgressElevator = -0.25;
constexpr double kLandingBounceRecoveryAglM = 10.0;
constexpr double kLandingBounceRecoverySinkMps = 0.0;
constexpr double kLandingBounceRecoveryElevator = 0.10;
constexpr double kLandingFloatRecoveryAglM = 10.0;
constexpr double kLandingFloatRecoverySinkMps = -0.5;
constexpr double kLandingFloatRecoveryDelaySec = 5.0;
constexpr double kLandingFloatRecoveryElevator = 0.05;
constexpr double kLandingGroundContactAglM = 0.10;
constexpr double kLandingTouchdownRolloutSpeedFps = 30.0;
constexpr double kLandingCompleteSpeedFps = 10.0;

double TakeoffIdleThrottle(propulsion::EngineType type) {
  switch (type) {
    case propulsion::EngineType::kPiston:
      return 0.15;
    case propulsion::EngineType::kTurboprop:
      return 0.65;
    case propulsion::EngineType::kTurbine:
      return 0.20;
    default:
      return 0.10;
  }
}

double EngineStartDurationSec(propulsion::EngineType type) {
  switch (type) {
    case propulsion::EngineType::kPiston:
      return 1.5;
    case propulsion::EngineType::kTurboprop:
      return 1.5;
    case propulsion::EngineType::kTurbine:
      return 1.0;
    default:
      return 1.0;
  }
}

bool UsesStaticRunup(propulsion::EngineType type) {
  return type == propulsion::EngineType::kTurbine || type == propulsion::EngineType::kRocket;
}

double StaticRunupDurationSec(propulsion::EngineType type) {
  return type == propulsion::EngineType::kTurbine ? 1.5 : 0.5;
}

double StaticRunupThrottle(propulsion::EngineType type, double elapsed_sec) {
  double target = type == propulsion::EngineType::kTurbine ? 0.90 : 0.70;
  double duration = StaticRunupDurationSec(type);
  double progress = duration > 0.0 ? std::clamp(elapsed_sec / duration, 0.0, 1.0) : 1.0;
  return TakeoffIdleThrottle(type) + (target - TakeoffIdleThrottle(type)) * progress;
}

double TakeoffRampDurationSec(propulsion::EngineType type) {
  switch (type) {
    case propulsion::EngineType::kTurboprop:
      return 2.0;
    case propulsion::EngineType::kPiston:
      return 3.0;
    case propulsion::EngineType::kTurbine:
      return 2.0;
    default:
      return 3.0;
  }
}

double TakeoffPowerThrottle(propulsion::EngineType type) {
  switch (type) {
    case propulsion::EngineType::kTurboprop:
      return 1.0;
    default:
      return 1.0;
  }
}

double ScheduledTakeoffThrottle(propulsion::EngineType type, double elapsed_sec) {
  double start = UsesStaticRunup(type) ? StaticRunupThrottle(type, StaticRunupDurationSec(type))
                                       : TakeoffIdleThrottle(type);
  double target = TakeoffPowerThrottle(type);
  double duration = TakeoffRampDurationSec(type);
  double progress = duration > 0.0 ? std::clamp(elapsed_sec / duration, 0.0, 1.0) : 1.0;
  return start + (target - start) * progress;
}

double NormalizeRad(double angle_rad) {
  while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
  while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
  return angle_rad;
}

double ComputeClockwiseOrbitHeadingRad(const JSBSim::FGLocation& location, const Waypoint& center,
                                       double radius_m, double speed_mps) {
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
  double lookahead_m = std::max(2000.0, speed_mps * 10.0);
  
  double d = distance_m;
  double r = radius_m;
  double l = lookahead_m;
  
  // By law of cosines: L^2 = d^2 + R^2 - 2 d R cos(theta)
  double cos_theta = (d * d + r * r - l * l) / (2.0 * d * r);
  
  double carrot_n, carrot_e;
  if (cos_theta > -1.0 && cos_theta < 1.0) {
    double theta = std::acos(cos_theta);
    double carrot_angle = radial_angle + theta; // clockwise -> increase angle
    carrot_n = r * std::cos(carrot_angle);
    carrot_e = r * std::sin(carrot_angle);
    return NormalizeRad(std::atan2(carrot_e - east_m, carrot_n - north_m));
  } else {
    // Carrot does not intersect. Intercept the tangent.
    double tangent_heading = radial_angle + M_PI / 2.0;
    double radial_error = d - r;
    double intercept = std::atan2(radial_error, l);
    intercept = std::clamp(intercept, -M_PI/4.0, M_PI/4.0);
    return NormalizeRad(tangent_heading + intercept);
  }
}

/// Compute heading for a figure-8 segment: CW or CCW orbit around a center.
/// Works by computing the orbit tangent heading with configurable direction.
/// CW:  tangent = radial_angle + π/2  (same as clockwise orbit)
/// CCW: tangent = radial_angle - π/2
double ComputeFigure8HeadingRad(const JSBSim::FGLocation& location, const Waypoint& center,
                                double radius_m, bool is_cw, double speed_mps) {
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
  double lookahead_m = std::max(2000.0, speed_mps * 10.0);
  
  double d = distance_m;
  double r = radius_m;
  double l = lookahead_m;
  
  double cos_theta = (d * d + r * r - l * l) / (2.0 * d * r);
  
  double carrot_n, carrot_e;
  if (cos_theta > -1.0 && cos_theta < 1.0) {
    double theta = std::acos(cos_theta);
    double carrot_angle = radial_angle + (is_cw ? theta : -theta);
    carrot_n = r * std::cos(carrot_angle);
    carrot_e = r * std::sin(carrot_angle);
    return NormalizeRad(std::atan2(carrot_e - east_m, carrot_n - north_m));
  } else {
    double tangent_heading = radial_angle + (is_cw ? M_PI / 2.0 : -M_PI / 2.0);
    double radial_error = d - r;
    double intercept = std::atan2(radial_error, l);
    intercept = std::clamp(intercept, -M_PI/4.0, M_PI/4.0);
    if (!is_cw) intercept = -intercept; // turn opposite direction to intercept
    return NormalizeRad(tangent_heading + intercept);
  }
}

/*
/// Compute local (north_m, east_m) offset from a reference center.
void LatLonToLocal(double lat_rad, double lon_rad, double ref_lat_rad, double ref_lon_rad,
                   double& north_m, double& east_m) {
  double cos_lat = std::cos(ref_lat_rad);
  north_m = (lat_rad - ref_lat_rad) * kEarthRadiusM;
  east_m = (lon_rad - ref_lon_rad) * kEarthRadiusM * cos_lat;
}
*/

/// Apply local (north_m, east_m) offset to produce a lat/lon Waypoint.
Waypoint LocalToWaypoint(double ref_lat_rad, double ref_lon_rad,
                          double north_m, double east_m, double altitude_m) {
  double cos_lat = std::cos(ref_lat_rad);
  Waypoint wp;
  wp.latitude_rad = ref_lat_rad + north_m / kEarthRadiusM;
  wp.longitude_rad = ref_lon_rad + east_m / (kEarthRadiusM * cos_lat);
  wp.altitude_m = altitude_m;
  return wp;
}

double BearingFromCenterRad(const JSBSim::FGLocation& location, const Waypoint& center) {
  double lat = location.GetGeodLatitudeRad();
  double lon = location.GetLongitude();
  double cos_lat = std::cos(center.latitude_rad);
  double dn = (lat - center.latitude_rad) * kEarthRadiusM;
  double de = (lon - center.longitude_rad) * kEarthRadiusM * cos_lat;
  return std::atan2(de, dn);
}

/*
/// Compute along-track distance from entry_pos in direction heading_rad.
/// Returns distance in meters projected onto the heading direction.
double AlongTrackDistanceM(const JSBSim::FGLocation& location,
                           const Waypoint& entry_pos, double heading_rad) {
  double lat = location.GetGeodLatitudeRad();
  double lon = location.GetLongitude();
  double cos_lat = std::cos(entry_pos.latitude_rad);
  double dn = (lat - entry_pos.latitude_rad) * kEarthRadiusM;
  double de = (lon - entry_pos.longitude_rad) * kEarthRadiusM * cos_lat;
  return dn * std::cos(heading_rad) + de * std::sin(heading_rad);
}
*/

/// Compute cross-track distance from the track defined by entry_pos and heading_rad.
/// Returns distance in meters. Positive means the aircraft is to the right of the track.
double CrossTrackDistanceM(const JSBSim::FGLocation& location,
                           const Waypoint& entry_pos, double heading_rad) {
  double lat = location.GetGeodLatitudeRad();
  double lon = location.GetLongitude();
  double cos_lat = std::cos(entry_pos.latitude_rad);
  double dn = (lat - entry_pos.latitude_rad) * kEarthRadiusM;
  double de = (lon - entry_pos.longitude_rad) * kEarthRadiusM * cos_lat;
  return -dn * std::sin(heading_rad) + de * std::cos(heading_rad);
}

}  // namespace

ManeuverExecutor::ManeuverExecutor(adapter::JsbsimAdapter& adapter, autopilot::Autopilot& ap,
                                   WaypointManager& wp_manager, propulsion::EngineManager& engines)
    : adapter_(adapter), ap_(ap), wp_manager_(wp_manager), engines_(engines) {}

void ManeuverExecutor::ExecuteFlyTo(const Waypoint& target) {
  // Clamp waypoint altitude to aircraft ceiling.
  Waypoint clamped_target = target;
  double ceiling = ap_.GetControlProfile().ceiling_m;
  if (ceiling > 0.0 && target.altitude_m > ceiling) {
    spdlog::info("[FLYTO] Clamping waypoint altitude {:.0f}m → ceiling {:.0f}m",
                 target.altitude_m, ceiling);
    clamped_target.altitude_m = ceiling;
  }

  current_maneuver_.type = ManeuverType::kFlyToWaypoint;
  current_maneuver_.target = clamped_target;
  active_ = true;
  elapsed_sec_ = 0.0;

  wp_manager_.ClearWaypoints();
  wp_manager_.AddWaypoint(clamped_target);
  wp_manager_.Start();

  ap_.SetLateralGuidanceMode(autopilot::LateralGuidanceMode::kHeading);
  ap_.SetHeadingSourceIsWaypoint(true);
  ap_.SetRollAttitudeMode(1);
  ap_.SetRollAutopilotOn(true);
  ap_.SetHeadingHold(true);
  ap_.SetAltitudeTargetM(clamped_target.altitude_m);
  ap_.SetAltitudeHold(true);

  // Speed target priority:
  //   1. Waypoint speed (if specified > 0)
  //   2. Profile cruise_speed_mps (if > 0)
  //   3. Current TAS (fallback)
  // Always clamp to [min_speed, max_speed] from profile.
  const auto& prof = ap_.GetControlProfile();
  double target_spd = 0.0;
  if (target.speed_mps > 0.0) {
    target_spd = target.speed_mps;
  } else if (prof.cruise_speed_mps > 0.0) {
    target_spd = prof.cruise_speed_mps;
  } else {
    target_spd = ap_.GetTrueSpeedMps();
  }
  if (prof.min_speed_mps > 0.0 && target_spd < prof.min_speed_mps) {
    target_spd = prof.min_speed_mps;
  }
  if (prof.max_speed_mps > 0.0 && target_spd > prof.max_speed_mps) {
    target_spd = prof.max_speed_mps;
  }
  ap_.SetSpeedTargetMps(target_spd);
  ap_.SetSpeedHold(true);
}

void ManeuverExecutor::ExecuteOrbit(const Waypoint& center, double radius_m, double duration_sec) {
  // Clamp orbit altitude to aircraft ceiling.
  Waypoint clamped_center = center;
  double ceiling = ap_.GetControlProfile().ceiling_m;
  if (ceiling > 0.0 && center.altitude_m > ceiling) {
    spdlog::info("[ORBIT] Clamping orbit altitude {:.0f}m → ceiling {:.0f}m",
                 center.altitude_m, ceiling);
    clamped_center.altitude_m = ceiling;
  }

  current_maneuver_.type = ManeuverType::kOrbit;
  current_maneuver_.target = clamped_center;
  current_maneuver_.value = std::abs(radius_m);
  current_maneuver_.duration_sec = duration_sec;
  if (current_maneuver_.value < 1.0) current_maneuver_.value = 1.0;
  active_ = true;
  elapsed_sec_ = 0.0;

  // Use a waypoint at center + offset for circular track
  Waypoint orbit_wp = clamped_center;
  orbit_wp.radius_m = radius_m;
  wp_manager_.ClearWaypoints();
  wp_manager_.AddWaypoint(orbit_wp);
  wp_manager_.Start();

  ap_.SetLateralGuidanceMode(autopilot::LateralGuidanceMode::kOrbit);
  ap_.SetHeadingSourceIsWaypoint(false);
  ap_.SetRollAttitudeMode(1);
  ap_.SetRollAutopilotOn(true);
  ap_.SetHeadingHold(true);
  ap_.SetAltitudeTargetM(clamped_center.altitude_m);
  ap_.SetAltitudeHold(true);
  // Speed target: use waypoint speed or profile cruise default.
  const auto& prof = ap_.GetControlProfile();
  double orbit_spd = prof.cruise_speed_mps > 0.0 ? prof.cruise_speed_mps
                                                 : ap_.GetTrueSpeedMps();
  if (center.speed_mps > 0.0) orbit_spd = center.speed_mps;

  double max_bank_rad = prof.max_roll_angle_deg * (M_PI / 180.0);
  if (max_bank_rad < 0.1) max_bank_rad = 0.1;
  double max_turn_spd = std::sqrt(radius_m * 9.80665 * std::tan(max_bank_rad));
  if (orbit_spd > max_turn_spd) orbit_spd = max_turn_spd;

  if (prof.max_speed_mps > 0.0 && orbit_spd > prof.max_speed_mps) {
    orbit_spd = prof.max_speed_mps;
  }
  ap_.SetSpeedTargetMps(orbit_spd);
  ap_.SetSpeedHold(true);
}

void ManeuverExecutor::ExecuteRacetrack(const Waypoint& start, double heading_rad,
                                         double leg_length_m, double turn_radius_m,
                                         int num_laps) {
  double r = std::abs(turn_radius_m);
  if (r < 1.0) r = 1.0;
  if (leg_length_m < 1.0) leg_length_m = 1.0;
  if (num_laps < 1) num_laps = 1;

  current_maneuver_.type = ManeuverType::kRacetrack;
  current_maneuver_.target = start;
  active_ = true;
  elapsed_sec_ = 0.0;

  racetrack_heading_ = heading_rad;
  racetrack_leg_len_ = leg_length_m;
  racetrack_turn_r_ = r;
  racetrack_target_laps_ = num_laps;
  racetrack_lap_ = 0;
  racetrack_phase_ = RacetrackPhase::kApproach;
  racetrack_entry_ = start;

  // Precompute turn centers.
  // LEG1: fly heading_rad for leg_len meters.
  // End of LEG1 = start + leg_len * (cos(ψ), sin(ψ)) in (north, east).
  // Center1 is r meters to the RIGHT of heading_rad:
  //   right(ψ) = (-sin(ψ), cos(ψ)) in (north, east)
  double cos_h = std::cos(heading_rad);
  double sin_h = std::sin(heading_rad);
  double leg_end_north = leg_length_m * cos_h;
  double leg_end_east  = leg_length_m * sin_h;

  // Absolute geographic entry point for Leg 1
  racetrack_leg1_entry_ = start;

  // Absolute geographic entry point for Leg 2
  // Leg 1 is from start to start + leg_length * (cos, sin)
  // Leg 2 starts 2*r to the right of Leg 1's end.
  // Right vector is (-sin, cos).
  double leg2_start_n = leg_end_north + 2.0 * r * (-sin_h);
  double leg2_start_e = leg_end_east  + 2.0 * r * cos_h;
  racetrack_leg2_entry_ = LocalToWaypoint(start.latitude_rad, start.longitude_rad,
                                          leg2_start_n, leg2_start_e, start.altitude_m);

  // Geographically fixed centers
  double c1_n = leg_end_north + r * (-sin_h);
  double c1_e = leg_end_east  + r * cos_h;
  racetrack_center1_ = LocalToWaypoint(start.latitude_rad, start.longitude_rad,
                                       c1_n, c1_e, start.altitude_m);

  double end_of_leg2_n = leg2_start_n - leg_length_m * cos_h;
  double end_of_leg2_e = leg2_start_e - leg_length_m * sin_h;
  double c2_n = end_of_leg2_n + r * sin_h;
  double c2_e = end_of_leg2_e + r * (-cos_h);
  racetrack_center2_ = LocalToWaypoint(start.latitude_rad, start.longitude_rad,
                                       c2_n, c2_e, start.altitude_m);

  // Configure autopilot
  ap_.SetLateralGuidanceMode(autopilot::LateralGuidanceMode::kHeading);
  ap_.SetHeadingSourceIsWaypoint(false);
  ap_.SetRollAttitudeMode(1);
  ap_.SetRollAutopilotOn(true);
  ap_.SetHeadingHold(true);
  ap_.SetAltitudeTargetM(start.altitude_m);
  ap_.SetAltitudeHold(true);

  const auto& prof = ap_.GetControlProfile();
  double cruise_spd = prof.cruise_speed_mps > 0.0 ? prof.cruise_speed_mps : ap_.GetTrueSpeedMps();
  if (start.speed_mps > 0.0) cruise_spd = start.speed_mps;
  if (prof.max_speed_mps > 0.0 && cruise_spd > prof.max_speed_mps) cruise_spd = prof.max_speed_mps;

  double max_bank_rad = prof.max_roll_angle_deg * (M_PI / 180.0);
  if (max_bank_rad < 0.1) max_bank_rad = 0.1;
  double max_turn_spd = std::sqrt(turn_radius_m * 9.80665 * std::tan(max_bank_rad));

  // Speed scheduling: fly fast on straight legs, decelerate for turns.
  // The turn speed is capped by the bank-angle physics; the cruise speed
  // is the profile's preferred cruise (may be much higher for FBW aircraft).
  racetrack_cruise_spd_ = cruise_spd;
  racetrack_turn_spd_   = std::min(cruise_spd, max_turn_spd);

  ap_.SetSpeedTargetMps(racetrack_cruise_spd_);  // start legs at cruise
  ap_.SetSpeedHold(true);
}

void ManeuverExecutor::ExecuteFigure8(const Waypoint& center, double radius_m,
                                       double axis_heading_rad, int num_cycles) {
  double r = std::abs(radius_m);
  if (r < 1.0) r = 1.0;
  if (num_cycles < 1) num_cycles = 1;

  current_maneuver_.type = ManeuverType::kFigure8;
  current_maneuver_.target = center;
  active_ = true;
  elapsed_sec_ = 0.0;

  figure8_radius_ = r;
  figure8_target_cycles_ = num_cycles;
  figure8_cycle_ = 0;
  figure8_phase_ = Figure8Phase::kCw;
  figure8_bearing_accum_ = 0.0;

  // ── Two-lobe geometry ──
  // The figure-8 is formed by two tangent circles (each radius r) along
  // axis_heading_rad.  The user-provided "center" is the midpoint where
  // the two lobes meet.
  //   center1 (CW lobe)  = midpoint + r × axis_dir
  //   center2 (CCW lobe) = midpoint - r × axis_dir
  // Both circles are tangent at the midpoint, enabling a smooth
  // transition when the aircraft crosses from one lobe to the other.
  double cos_h = std::cos(axis_heading_rad);
  double sin_h = std::sin(axis_heading_rad);
  double dn = r * cos_h;
  double de = r * sin_h;

  figure8_center_ = LocalToWaypoint(center.latitude_rad, center.longitude_rad,
                                     dn, de, center.altitude_m);
  figure8_center2_ = LocalToWaypoint(center.latitude_rad, center.longitude_rad,
                                      -dn, -de, center.altitude_m);

  const auto& loc = adapter_.GetPropagate().GetLocation();
  figure8_prev_bearing_ = BearingFromCenterRad(loc, figure8_center_);

  // AP config
  ap_.SetLateralGuidanceMode(autopilot::LateralGuidanceMode::kHeading);
  ap_.SetHeadingSourceIsWaypoint(false);
  ap_.SetRollAttitudeMode(1);
  ap_.SetRollAutopilotOn(true);
  ap_.SetHeadingHold(true);
  ap_.SetAltitudeTargetM(center.altitude_m);
  ap_.SetAltitudeHold(true);

  const auto& prof = ap_.GetControlProfile();
  double spd = prof.cruise_speed_mps > 0.0 ? prof.cruise_speed_mps : ap_.GetTrueSpeedMps();
  if (center.speed_mps > 0.0) spd = center.speed_mps;

  double max_bank_rad = prof.max_roll_angle_deg * (M_PI / 180.0);
  if (max_bank_rad < 0.1) max_bank_rad = 0.1;
  double max_turn_spd = std::sqrt(r * 9.80665 * std::tan(max_bank_rad));
  if (spd > max_turn_spd) spd = max_turn_spd;

  if (prof.max_speed_mps > 0.0 && spd > prof.max_speed_mps) spd = prof.max_speed_mps;
  ap_.SetSpeedTargetMps(spd);
  ap_.SetSpeedHold(true);
}

void ManeuverExecutor::ExecuteSTurn(double base_heading_rad, double amplitude_deg,
                                     double period_sec, double duration_sec) {
  current_maneuver_.type = ManeuverType::kSTurn;
  current_maneuver_.value = base_heading_rad;
  current_maneuver_.duration_sec = duration_sec;
  active_ = true;
  elapsed_sec_ = 0.0;

  sturn_base_heading_ = base_heading_rad;
  sturn_amplitude_rad_ = amplitude_deg * M_PI / 180.0;
  sturn_freq_ = period_sec > 0.01 ? (2.0 * M_PI / period_sec) : 0.0;

  // Lock current altitude and speed during sinusoidal heading modulation.
  // Without explicit targets, altitude hold has no reference and the aircraft
  // bleeds altitude during bank oscillations (f16: -500m in 10s, L410: -217m).
  ap_.SetAltitudeTargetM(ap_.GetAltitudeASLM());
  ap_.SetAltitudeHold(true);
  const auto& prof = ap_.GetControlProfile();
  double spd = prof.cruise_speed_mps > 0.0 ? prof.cruise_speed_mps : ap_.GetTrueSpeedMps();
  ap_.SetSpeedTargetMps(spd);
  ap_.SetSpeedHold(true);

  ap_.SetLateralGuidanceMode(autopilot::LateralGuidanceMode::kHeading);
  ap_.SetHeadingSourceIsWaypoint(false);
  ap_.SetRollAttitudeMode(1);
  ap_.SetRollAutopilotOn(true);
  ap_.SetHeadingHold(true);
}

void ManeuverExecutor::ExecuteSetHeading(double heading_rad, double tolerance_rad) {
  current_maneuver_.type = ManeuverType::kSetHeading;
  current_maneuver_.value = heading_rad;
  current_maneuver_.heading_tolerance_rad = tolerance_rad;
  active_ = true;
  elapsed_sec_ = 0.0;

  ap_.SetLateralGuidanceMode(autopilot::LateralGuidanceMode::kHeading);
  ap_.SetHeadingSourceIsWaypoint(false);
  ap_.SetHeadingTargetRad(heading_rad);
  ap_.SetRollAttitudeMode(1);
  ap_.SetRollAutopilotOn(true);
  ap_.SetHeadingHold(true);
}

void ManeuverExecutor::ExecuteSetAltitude(double altitude_m, double tolerance_m) {
  double ceiling = ap_.GetControlProfile().ceiling_m;
  double clamped_alt = altitude_m;
  if (ceiling > 0.0 && altitude_m > ceiling) {
    spdlog::info("[SETALT] Clamping target altitude {:.0f}m → ceiling {:.0f}m",
                 altitude_m, ceiling);
    clamped_alt = ceiling;
  }

  current_maneuver_.type = ManeuverType::kSetAltitude;
  current_maneuver_.value = clamped_alt;
  current_maneuver_.altitude_tolerance_m = tolerance_m;
  active_ = true;
  elapsed_sec_ = 0.0;

  ap_.SetAltitudeTargetM(clamped_alt);
  ap_.SetAltitudeHold(true);
}

void ManeuverExecutor::ExecuteSetPitch(double pitch_deg, double duration_sec) {
  current_maneuver_.type = ManeuverType::kSetPitch;
  current_maneuver_.value = pitch_deg;
  current_maneuver_.duration_sec = duration_sec;
  active_ = true;
  elapsed_sec_ = 0.0;

  ap_.SetAltitudeHold(false);   // release takeoff/cruise altitude hold
  ap_.SetPitchTargetDeg(pitch_deg);
  ap_.SetPitchHold(true);

  // Speed protection (L1): lock current airspeed so energy management
  // maintains it via throttle.  Using cruise_speed as the target would
  // create an unreachable goal for low-power aircraft, causing the energy
  // management to misbehave (throttle oscillates, speed fluctuates).
  // Instead, lock the current speed (clamped above a safe floor) so the
  // aircraft only fights to recover speed it lost from the pitch change,
  // not from a mismatch between current conditions and cruise target.
  const auto& prof = ap_.GetControlProfile();
  double target_spd = ap_.GetTrueSpeedMps();
  constexpr double kMinSpeedMargin = 1.15;
  double min_safe = prof.min_speed_mps * kMinSpeedMargin;
  if (min_safe > 0.0 && target_spd < min_safe) {
    target_spd = min_safe;
  }
  ap_.SetSpeedTargetMps(target_spd);
  ap_.SetSpeedHold(true);
}

void ManeuverExecutor::ExecuteSetRoll(int roll_mode) {
  current_maneuver_.type = ManeuverType::kSetRoll;
  current_maneuver_.value = static_cast<double>(roll_mode);
  active_ = true;
  elapsed_sec_ = 0.0;

  ap_.SetRollAttitudeMode(roll_mode);
  ap_.SetRollAutopilotOn(true);
}

void ManeuverExecutor::ExecuteTakeoff(double target_altitude_m, double target_heading_rad,
                                      double target_speed_mps) {
  // Clamp to aircraft service ceiling so the takeoff never targets an
  // unreachable altitude that would cause infinite climb → TIMEOUT.
  double ceiling = ap_.GetControlProfile().ceiling_m;
  double clamped_alt = target_altitude_m;
  if (ceiling > 0.0 && target_altitude_m > ceiling) {
    spdlog::info("[TKO] Clamping target altitude {:.0f}m → ceiling {:.0f}m",
                 target_altitude_m, ceiling);
    clamped_alt = ceiling;
  }

  current_maneuver_.type = ManeuverType::kTakeoff;
  current_maneuver_.value = clamped_alt;
  current_maneuver_.duration_sec = target_speed_mps;
  current_maneuver_.target.altitude_m = clamped_alt;
  active_ = true;
  elapsed_sec_ = 0.0;
  takeoff_phase_elapsed_sec_ = 0.0;
  takeoff_phase_ = TakeoffPhase::kEngineStart;
  takeoff_target_altitude_m_ = clamped_alt;
  takeoff_target_heading_rad_ = target_heading_rad;

  StartEngine();
}

void ManeuverExecutor::StartEngine() {
  engines_.SetBrakes(true);
  engines_.SetThrottle(TakeoffIdleThrottle(engines_.GetType()));
  engines_.Start();
  // Wings-level only during ground roll — heading hold causes integrator
  // windup at low speeds where ailerons have no aerodynamic authority.
  ap_.SetRollAttitudeMode(0);  // wings level
  ap_.SetRollAutopilotOn(true);
}

void ManeuverExecutor::ConfigureForTakeoffRoll() {
  // Release brakes, set takeoff flaps, then ramp throttle during the roll.
  engines_.SetBrakes(false);
  engines_.SetFlaps(0.33);
  engines_.SetThrottle(ScheduledTakeoffThrottle(engines_.GetType(), 0.0));
}

void ManeuverExecutor::ConfigureForClimb(double target_altitude_m, double target_heading_rad,
                                         double /*target_speed_mps*/) {
  // Rotation: elevator ramps gradually in kRotateAndClimb (no step input).
  rotation_elapsed_sec_ = 0.0;

  // Defer heading hold engagement until airborne.  Activating heading-based
  // roll while still on the ground causes aggressive aileron commands at high
  // speed, leading to roll departure before rotation completes (B747, XB-70).
  ap_.SetHeadingTargetRad(target_heading_rad);
  ap_.SetAltitudeTargetM(target_altitude_m);
  // Keep wings-level mode during ground rotation — the heading hold and
  // heading-based roll are engaged once WOW clears in kRotateAndClimb.
}

bool ManeuverExecutor::IsTouchingGround() const {
  if (current_maneuver_.type == ManeuverType::kTakeoff) {
    return engines_.IsWeightOnWheels() && (takeoff_phase_ == TakeoffPhase::kEngineStart ||
                                           takeoff_phase_ == TakeoffPhase::kStaticRunup ||
                                           takeoff_phase_ == TakeoffPhase::kTakeoffRoll);
  }
  if (current_maneuver_.type != ManeuverType::kLand) return false;
  return engines_.IsWeightOnWheels() &&
         (land_phase_ == LandPhase::kFlare || land_phase_ == LandPhase::kTouchdown ||
          land_phase_ == LandPhase::kRollout);
}

bool ManeuverExecutor::IsManeuverComplete() const {
  if (!active_) return true;

  switch (current_maneuver_.type) {
    case ManeuverType::kFlyToWaypoint: {
      // Use true airspeed (TAS) for turn radius — inertial velocity
      // includes Earth rotation (~465 m/s at equator), which inflates the
      // capture radius and causes premature maneuver completion.
      double speed_mps = adapter_.GetProperty("velocities/vtrue-fps") * 0.3048;
      if (speed_mps < 10.0) speed_mps = 10.0;
      double max_bank_rad = ap_.GetControlProfile().max_roll_angle_deg * 0.0174533;
      double min_turn_radius_m = (speed_mps * speed_mps) / (9.81 * std::tan(max_bank_rad));
      double effective_radius_m =
          std::max(current_maneuver_.target.radius_m, min_turn_radius_m * 1.5);
      if (wp_manager_.IsAtOrPastTarget(effective_radius_m)) return true;

      // Fly-past detection: if the aircraft started this maneuver already past
      // the waypoint (e.g., fast fighter with long takeoff roll), the standard
      // HasPassedActiveWaypoint check fails because the leg start coincides
      // with the aircraft's position.  Detect this by checking if the waypoint
      // is far behind the aircraft (> ~115° off the nose) and the distance
      // exceeds 3× the effective capture radius.
      if (elapsed_sec_ > 10.0) {
        double dist_m = wp_manager_.GetDistanceToActiveM();
        double heading_to_wp = wp_manager_.GetHeadingToActiveRad();
        double current_heading = adapter_.GetPropagate().GetEuler(3);
        double angle_off_nose = std::abs(NormalizeRad(heading_to_wp - current_heading));
        if (angle_off_nose > 2.0 && dist_m > effective_radius_m * 3.0) {
          return true;
        }
      }
      return false;
    }
    case ManeuverType::kOrbit:
      if (current_maneuver_.duration_sec > 0.0) {
        return elapsed_sec_ >= current_maneuver_.duration_sec;
      }
      return false;
    case ManeuverType::kRacetrack:
      return racetrack_phase_ == RacetrackPhase::kComplete;
    case ManeuverType::kFigure8:
      return figure8_phase_ == Figure8Phase::kComplete;
    case ManeuverType::kSTurn:
      return current_maneuver_.duration_sec > 0.0 &&
             elapsed_sec_ >= current_maneuver_.duration_sec;
    case ManeuverType::kSetHeading: {
      double angle_error = std::abs(ap_.GetAngleToHeadingRad());
      if (angle_error < current_maneuver_.heading_tolerance_rad) return true;
      // Best-effort convergence: the heading PD controller has no integral
      // term, so some aircraft reach steady-state with a small residual
      // heading error.  After a grace period, accept within relaxed tolerance.
      constexpr double kBestEffortHeadingSec = 30.0;
      constexpr double kBestEffortHeadingScale = 5.0;
      if (elapsed_sec_ > kBestEffortHeadingSec &&
          angle_error < current_maneuver_.heading_tolerance_rad * kBestEffortHeadingScale) {
        return true;
      }
      return false;
    }
    case ManeuverType::kSetAltitude: {
      double alt_error = std::abs(ap_.GetAltitudeASLM() - current_maneuver_.value);
      if (alt_error < current_maneuver_.altitude_tolerance_m) return true;
      // Best-effort convergence: the altitude PD controller has no integral
      // term, so at high altitude the available thrust may limit climb rate
      // and the aircraft reaches a steady-state below the target (observed
      // 12–76 m offset across 9 aircraft types).  After a grace period,
      // accept within relaxed tolerance to prevent fuel exhaustion and allow
      // the mission sequence to continue.
      constexpr double kBestEffortAltitudeSec = 120.0;
      constexpr double kBestEffortAltitudeScale = 10.0;
      if (elapsed_sec_ > kBestEffortAltitudeSec &&
          alt_error < current_maneuver_.altitude_tolerance_m * kBestEffortAltitudeScale) {
        return true;
      }
      return false;
    }
    case ManeuverType::kSetPitch:
      return current_maneuver_.duration_sec > 0.0 && elapsed_sec_ >= current_maneuver_.duration_sec;
    case ManeuverType::kSetRoll:
      return true;
    case ManeuverType::kTakeoff:
      return takeoff_phase_ == TakeoffPhase::kComplete;
    case ManeuverType::kLand:
      return land_phase_ == LandPhase::kComplete;
  }
  return true;
}

void ManeuverExecutor::Update(double dt_sec) {
  if (!active_) return;
  elapsed_sec_ += dt_sec;
  if (current_maneuver_.type == ManeuverType::kOrbit) {
    double radius_m = std::abs(current_maneuver_.value);
    if (radius_m < 1.0) radius_m = 1.0;
    double speed_mps = adapter_.GetProperty("velocities/vtrue-fps") * 0.3048;
    if (speed_mps < 10.0) speed_mps = 10.0;
    double heading_rad = ComputeClockwiseOrbitHeadingRad(
        adapter_.GetPropagate().GetLocation(), current_maneuver_.target, radius_m, speed_mps);
    ap_.SetHeadingTargetRad(heading_rad);
  }
  // ── Racetrack FSM: LEG1 → TURN1 → LEG2 → TURN2 → [repeat] ──
  //
  // Coordinate system: racetrack-aligned local frame with origin at
  // racetrack_leg1_entry_, x_prime = cross-track (positive = right
  // of heading), y_prime = along-track (positive = forward).
  //
  // Ideal shape (heading ψ = 0 / North):
  //   Leg1:  x=0,        y: 0 → leg_len      heading ψ
  //   Turn1: center (r, leg_len), CW semicircle
  //   Leg2:  x=2r,       y: leg_len → 0       heading ψ+π
  //   Turn2: center (r, 0),       CW semicircle
  //
  // Turn guidance uses the same carrot algorithm as the orbit
  // function but with a capped lookahead of (π/3)·r ≈ 60° of arc.
  // The default lookahead (≥ 2000 m) exceeds half the semicircle
  // for small turn radii, causing the aircraft to cut corners and
  // drift each lap.
  if (current_maneuver_.type == ManeuverType::kRacetrack) {
    const auto& loc = adapter_.GetPropagate().GetLocation();
    double speed = adapter_.GetProperty("velocities/vtrue-fps") * 0.3048;
    if (speed < 10.0) speed = 10.0;

    double lat = loc.GetGeodLatitudeRad();
    double lon = loc.GetLongitude();
    double cos_lat = std::cos(racetrack_leg1_entry_.latitude_rad);
    double dn = (lat - racetrack_leg1_entry_.latitude_rad) * kEarthRadiusM;
    double de = (lon - racetrack_leg1_entry_.longitude_rad) * kEarthRadiusM * cos_lat;

    double cos_h = std::cos(racetrack_heading_);
    double sin_h = std::sin(racetrack_heading_);
    double x_prime = de * cos_h - dn * sin_h;
    double y_prime = de * sin_h + dn * cos_h;

    // Helper: CW orbit heading with reduced lookahead for turns.
    // Caps lookahead to (π/3)·radius ≈ 60° of arc so the carrot
    // does not extend past the semicircle endpoint.
    auto turn_heading = [&](const Waypoint& center, double radius) -> double {
      double c_cos = std::cos(center.latitude_rad);
      double n_m = (lat - center.latitude_rad) * kEarthRadiusM;
      double e_m = (lon - center.longitude_rad) * kEarthRadiusM * c_cos;
      double dist = std::hypot(n_m, e_m);
      if (dist < 1.0) return loc.GetHeadingTo(center.longitude_rad, center.latitude_rad);
      double radial = std::atan2(e_m, n_m);
      double la = std::max(200.0, std::min(speed * 5.0, M_PI / 3.0 * radius));
      double cos_t = (dist * dist + radius * radius - la * la) / (2.0 * dist * radius);
      if (cos_t > -1.0 && cos_t < 1.0) {
        double t = std::acos(cos_t);
        double ca = radial + t;
        double cn = radius * std::cos(ca);
        double ce = radius * std::sin(ca);
        return NormalizeRad(std::atan2(ce - e_m, cn - n_m));
      } else {
        double th = radial + M_PI / 2.0;
        double re = dist - radius;
        double ic = std::clamp(std::atan2(re, la), -M_PI / 4.0, M_PI / 4.0);
        return NormalizeRad(th + ic);
      }
    };

    switch (racetrack_phase_) {
      case RacetrackPhase::kApproach: {
        // ── Approach phase: navigate to racetrack with heading alignment ──
        //
        // Finds the nearest point on the racetrack perimeter, then flies
        // toward a LOOK-AHEAD point ahead on the track. This naturally
        // aligns the aircraft's heading with the track direction as it
        // approaches, enabling a smooth transition to the FSM phase.
        //
        // Racetrack perimeter in local frame (x=cross-track, y=along-track):
        //   Leg1:  x=0,   y∈[0, L]
        //   Turn1: center (r, L), CW semicircle
        //   Leg2:  x=2r,  y∈[0, L]
        //   Turn2: center (r, 0), CW semicircle
        ap_.SetSpeedTargetMps(racetrack_cruise_spd_);

        double r = racetrack_turn_r_;
        double L = racetrack_leg_len_;
        double best_dist = 1e18;
        double best_nx = 0.0, best_ny = 0.0;
        int best_seg = 0;  // 0=Leg1, 1=Turn1, 2=Leg2, 3=Turn2

        // Nearest point on Leg1: (0, clamp(y, 0, L))
        {
          double ny = std::clamp(y_prime, 0.0, L);
          double d = std::hypot(x_prime, y_prime - ny);
          if (d < best_dist) {
            best_dist = d; best_nx = 0.0; best_ny = ny; best_seg = 0;
          }
        }
        // Nearest point on Leg2: (2r, clamp(y, 0, L))
        {
          double ny = std::clamp(y_prime, 0.0, L);
          double d = std::hypot(x_prime - 2.0 * r, y_prime - ny);
          if (d < best_dist) {
            best_dist = d; best_nx = 2.0 * r; best_ny = ny; best_seg = 2;
          }
        }
        // Nearest point on Turn1: center (r, L), project onto circle
        {
          double dx = x_prime - r, dy = y_prime - L;
          double d = std::hypot(dx, dy);
          if (d < 1.0) d = 1.0;
          double pd = std::abs(d - r);
          if (pd < best_dist) {
            best_dist = pd;
            best_nx = r + r * dx / d;
            best_ny = L + r * dy / d;
            best_seg = 1;
          }
        }
        // Nearest point on Turn2: center (r, 0), project onto circle
        {
          double dx = x_prime - r, dy = y_prime;
          double d = std::hypot(dx, dy);
          if (d < 1.0) d = 1.0;
          double pd = std::abs(d - r);
          if (pd < best_dist) {
            best_dist = pd;
            best_nx = r + r * dx / d;
            best_ny = r * dy / d;
            best_seg = 3;
          }
        }

        // Capture threshold: close enough to join the pattern.
        double capture_dist = std::max(r * 1.5, 1000.0);

        if (best_dist <= capture_dist) {
          spdlog::debug("[RACETRACK] Approach captured: dist={:.0f}m → segment {}",
                        best_dist, best_seg);
          switch (best_seg) {
            case 0: racetrack_phase_ = RacetrackPhase::kLeg1; break;
            case 1: racetrack_phase_ = RacetrackPhase::kTurn1; break;
            case 2: racetrack_phase_ = RacetrackPhase::kLeg2; break;
            case 3: racetrack_phase_ = RacetrackPhase::kTurn2; break;
          }
          break;
        }

        // Compute look-ahead point: advance along the track from the
        // nearest point. This aligns the approach with the track heading.
        double la = std::max(1000.0, speed * 5.0);
        double la_nx, la_ny;

        switch (best_seg) {
          case 0:  // Leg1: track direction is +y
            la_nx = best_nx;
            la_ny = std::min(best_ny + la, L);
            break;
          case 2:  // Leg2: track direction is -y
            la_nx = best_nx;
            la_ny = std::max(best_ny - la, 0.0);
            break;
          case 1:   // Turn1: CW along circle
          case 3: {  // Turn2: CW along circle
            double cx = r;
            double cy = (best_seg == 1) ? L : 0.0;
            double dx = best_nx - cx, dy = best_ny - cy;
            double d = std::hypot(dx, dy);
            if (d < 1.0) d = 1.0;
            // Advance angle CW by la/r radians.
            // Radial angle in atan2(east, north) convention.
            double angle = std::atan2(dx, dy);
            double advance = la / r;
            double new_angle = angle + advance;
            la_nx = cx + r * std::sin(new_angle);
            la_ny = cy + r * std::cos(new_angle);
            break;
          }
        }

        // Navigate toward look-ahead point on perimeter.
        double tgt_n = la_ny * cos_h - la_nx * sin_h;
        double tgt_e = la_ny * sin_h + la_nx * cos_h;
        Waypoint tgt_wp = LocalToWaypoint(
            racetrack_leg1_entry_.latitude_rad,
            racetrack_leg1_entry_.longitude_rad,
            tgt_n, tgt_e, racetrack_leg1_entry_.altitude_m);
        double hdg = loc.GetHeadingTo(tgt_wp.longitude_rad, tgt_wp.latitude_rad);
        ap_.SetHeadingTargetRad(hdg);
        break;
      }

      case RacetrackPhase::kLeg1: {
        // Straight-leg speed: cruise (faster).
        ap_.SetSpeedTargetMps(racetrack_cruise_spd_);
        // Steer along heading ψ with cross-track correction.
        double xtk_m = x_prime;
        double lookahead_m = std::max(2000.0, speed * 10.0);
        double intercept_angle = std::atan2(xtk_m, lookahead_m);
        intercept_angle = std::clamp(intercept_angle, -M_PI / 4.0, M_PI / 4.0);
        double intercept_hdg = racetrack_heading_ - intercept_angle;
        ap_.SetHeadingTargetRad(NormalizeRad(intercept_hdg));

        if (y_prime >= racetrack_leg_len_) {
          racetrack_phase_ = RacetrackPhase::kTurn1;
        }
        break;
      }

      case RacetrackPhase::kTurn1: {
        // Turn speed: decelerated to fit bank-angle physics.
        ap_.SetSpeedTargetMps(racetrack_turn_spd_);
        double hdg = turn_heading(racetrack_center1_, racetrack_turn_r_);
        ap_.SetHeadingTargetRad(hdg);

        if (x_prime > racetrack_turn_r_ && y_prime <= racetrack_leg_len_) {
          racetrack_phase_ = RacetrackPhase::kLeg2;
        }
        break;
      }

      case RacetrackPhase::kLeg2: {
        // Back to cruise speed on the straight leg.
        ap_.SetSpeedTargetMps(racetrack_cruise_spd_);
        // Steer along heading ψ+π with cross-track correction in
        // the racetrack frame.  Track line is at x_prime = 2r.
        double xtk_m = x_prime - 2.0 * racetrack_turn_r_;
        double lookahead_m = std::max(2000.0, speed * 10.0);
        double intercept_angle = std::atan2(xtk_m, lookahead_m);
        intercept_angle = std::clamp(intercept_angle, -M_PI / 4.0, M_PI / 4.0);
        // For reverse heading, add the intercept angle (sign flips
        // because the heading direction is opposite).
        double track2_hdg = NormalizeRad(racetrack_heading_ + M_PI);
        double intercept_hdg = track2_hdg + intercept_angle;
        ap_.SetHeadingTargetRad(NormalizeRad(intercept_hdg));

        if (y_prime <= 0.0) {
          racetrack_phase_ = RacetrackPhase::kTurn2;
        }
        break;
      }

      case RacetrackPhase::kTurn2: {
        // Turn speed: decelerated to fit bank-angle physics.
        ap_.SetSpeedTargetMps(racetrack_turn_spd_);
        double hdg = turn_heading(racetrack_center2_, racetrack_turn_r_);
        ap_.SetHeadingTargetRad(hdg);

        if (x_prime < racetrack_turn_r_ && y_prime >= 0.0) {
          racetrack_lap_++;
          if (racetrack_lap_ >= racetrack_target_laps_) {
            racetrack_phase_ = RacetrackPhase::kComplete;
          } else {
            racetrack_phase_ = RacetrackPhase::kLeg1;
          }
        }
        break;
      }

      case RacetrackPhase::kComplete:
        break;
    }
  }

  // ── Figure-8 FSM: CW (center1) → CCW (center2) → [repeat] ──
  // Two-lobe design: center1 and center2 are offset by r along the
  // axis heading.  The CW lobe orbits center1; the CCW lobe orbits
  // center2.  The tangent point (user-provided center) is where the
  // aircraft transitions between lobes.
  if (current_maneuver_.type == ManeuverType::kFigure8) {
    const auto& loc = adapter_.GetPropagate().GetLocation();
    double speed = adapter_.GetProperty("velocities/vtrue-fps") * 0.3048;
    if (speed < 10.0) speed = 10.0;

    // Use the correct center for each phase.
    const Waypoint& active_center =
        (figure8_phase_ == Figure8Phase::kCw) ? figure8_center_ : figure8_center2_;

    double bearing = BearingFromCenterRad(loc, active_center);
    figure8_bearing_accum_ += NormalizeRad(bearing - figure8_prev_bearing_);
    figure8_prev_bearing_ = bearing;

    bool is_cw = (figure8_phase_ == Figure8Phase::kCw);
    double hdg = ComputeFigure8HeadingRad(loc, active_center, figure8_radius_, is_cw, speed);
    ap_.SetHeadingTargetRad(hdg);

    // Switch lobe after accumulating 2π of bearing change around the
    // active center.
    if (std::abs(figure8_bearing_accum_) >= 2.0 * M_PI - 0.05) {
      figure8_bearing_accum_ = 0.0;
      if (figure8_phase_ == Figure8Phase::kCw) {
        figure8_phase_ = Figure8Phase::kCcw;
        // Reset bearing tracking for the new center.
        figure8_prev_bearing_ = BearingFromCenterRad(loc, figure8_center2_);
      } else {
        figure8_cycle_++;
        figure8_phase_ = Figure8Phase::kCw;
        figure8_prev_bearing_ = BearingFromCenterRad(loc, figure8_center_);
        if (figure8_cycle_ >= figure8_target_cycles_) {
          figure8_phase_ = Figure8Phase::kComplete;
        }
      }
    }
  }

  // ── S-Turn: sinusoidal heading modulation ──
  if (current_maneuver_.type == ManeuverType::kSTurn) {
    double offset = sturn_amplitude_rad_ * std::sin(sturn_freq_ * elapsed_sec_);
    ap_.SetHeadingTargetRad(NormalizeRad(sturn_base_heading_ + offset));
  }
  if (current_maneuver_.type == ManeuverType::kTakeoff) {
    takeoff_phase_elapsed_sec_ += dt_sec;
    double vc_kts = adapter_.GetProperty("velocities/vc-kts");
    double agl_ft = adapter_.GetProperty("position/h-agl-ft");
    double agl_m = agl_ft * 0.3048;
    sink_rate_mps_ = (agl_m - prev_alt_m_) / dt_sec;
    prev_alt_m_ = agl_m;

    switch (takeoff_phase_) {
      case TakeoffPhase::kEngineStart:
        engines_.SetBrakes(true);
        engines_.SetThrottle(TakeoffIdleThrottle(engines_.GetType()));
        if (takeoff_phase_elapsed_sec_ >= EngineStartDurationSec(engines_.GetType())) {
          takeoff_phase_elapsed_sec_ = 0.0;
          if (UsesStaticRunup(engines_.GetType())) {
            takeoff_phase_ = TakeoffPhase::kStaticRunup;
          } else {
            ConfigureForTakeoffRoll();
            takeoff_phase_ = TakeoffPhase::kTakeoffRoll;
          }
        }
        break;
      case TakeoffPhase::kStaticRunup:
        engines_.SetBrakes(true);
        engines_.SetThrottle(StaticRunupThrottle(engines_.GetType(), takeoff_phase_elapsed_sec_));
        if (takeoff_phase_elapsed_sec_ >= StaticRunupDurationSec(engines_.GetType())) {
          ConfigureForTakeoffRoll();
          takeoff_phase_elapsed_sec_ = 0.0;
          takeoff_phase_ = TakeoffPhase::kTakeoffRoll;
        }
        break;
      case TakeoffPhase::kTakeoffRoll:
        engines_.SetThrottle(
            ScheduledTakeoffThrottle(engines_.GetType(), takeoff_phase_elapsed_sec_));
        {
          double vr_kts = engines_.GetRotationSpeedKts();
          takeoff_vr_kts_ = vr_kts;  // cache for airspeed checks during climb
          if (vc_kts >= vr_kts * 0.79 && vc_kts < vr_kts) {
            spdlog::info("[TAKEOFF] vc={:.1f} kts  Vr={:.1f} kts  pre-rot el={:.3f}",
                         vc_kts, vr_kts,
                         -ap_.GetControlProfile().rotation_max_elevator * 0.8 *
                             std::clamp((vc_kts - vr_kts * 0.80) / (vr_kts * 0.20), 0.0, 1.0));
          }
          if (vc_kts >= vr_kts) {
            spdlog::info("[TAKEOFF] ROTATE at vc={:.1f} kts (Vr={:.1f}), entering kRotateAndClimb",
                         vc_kts, vr_kts);
            // Seed the rotation ramp from the pre-rotation elevator level
            // so there is no step-down at the critical rotation moment.
            double pre_rot_el = -ap_.GetControlProfile().rotation_max_elevator * 0.8;
            rotation_ramp_origin_ = pre_rot_el;
            ConfigureForClimb(takeoff_target_altitude_m_, takeoff_target_heading_rad_,
                              current_maneuver_.duration_sec);
            takeoff_phase_elapsed_sec_ = 0.0;
            takeoff_phase_ = TakeoffPhase::kRotateAndClimb;
          } else if (!engines_.IsWeightOnWheels() && agl_m > 10.0 && vc_kts > 25.0) {
            // Uncommanded liftoff: delta-wing / high-lift aircraft may lift
            // off before reaching computed Vr.  Require at least 25 kts to
            // filter out JSBSim initialization bounces (WOW flickers during
            // engine start can clear WOW at low speed — MD11 regression).
            // Transition to climb phase with the rotation ramp marked
            // complete — the gradual pitch recovery in kRotateAndClimb will
            // gently reduce pitch toward
            // the climb target without a full-elevator step (XB-70, DHC6).
            spdlog::info("[TAKEOFF] Uncommanded liftoff at vc={:.1f} kts (Vr={:.1f}), "
                         "entering climb",
                         vc_kts, vr_kts);
            ConfigureForClimb(takeoff_target_altitude_m_, takeoff_target_heading_rad_,
                              current_maneuver_.duration_sec);
            rotation_elapsed_sec_ = ap_.GetControlProfile().rotation_ramp_sec + 0.01;
            takeoff_phase_elapsed_sec_ = 0.0;
            takeoff_phase_ = TakeoffPhase::kRotateAndClimb;
          } else if (vc_kts >= vr_kts * 0.80) {
            // Pre-rotation: light back-pressure starting at 80% Vr.
            // Skip for delta-wing aircraft (XB-70, Concorde): vortex lift
            // generates excessive pitch-up at low speed; the aircraft will
            // rotate naturally without elevator input.
            bool is_delta_wing = false;
            {
              double b = adapter_.GetProperty("metrics/bw-ft");
              double s = adapter_.GetProperty("metrics/Sw-sqft");
              if (b > 1.0 && s > 1.0 && (b * b) / s < 2.5) is_delta_wing = true;
            }
            if (!is_delta_wing) {
              double pre_rot_frac = (vc_kts - vr_kts * 0.80) / (vr_kts * 0.20);
              pre_rot_frac = std::clamp(pre_rot_frac, 0.0, 1.0);
              double el = -ap_.GetControlProfile().rotation_max_elevator * 0.8 * pre_rot_frac;
              double pitch_deg = adapter_.GetProperty("attitude/pitch-rad") * 57.2958;
              if (pitch_deg > 15.0) el = 0.0;
              bool use_pitch_trim =
                  ap_.GetControlProfile().fbw_subtype != autopilot::FbwSubtype::kNone;
              if (use_pitch_trim)
                adapter_.SetProperty("fcs/pitch-trim-cmd-norm", el);
              else
                adapter_.SetProperty("fcs/elevator-cmd-norm", el);
            }
          }
        }
        break;
      case TakeoffPhase::kRotateAndClimb: {
        engines_.SetThrottle(TakeoffPowerThrottle(engines_.GetType()));
        rotation_elapsed_sec_ += dt_sec;

        bool is_rate_int =
            ap_.GetControlProfile().fbw_subtype == autopilot::FbwSubtype::kRateIntegratorActuator;
        bool use_pitch_trim = ap_.GetControlProfile().fbw_subtype != autopilot::FbwSubtype::kNone;
        auto set_tko_el = [&](double el) {
          if (use_pitch_trim)
            adapter_.SetProperty("fcs/pitch-trim-cmd-norm", el);
          else
            adapter_.SetProperty("fcs/elevator-cmd-norm", el);
        };

        // Engage heading hold only after lifting off AND achieving adequate
        // airspeed for roll control authority.  On the ground, heading-based
        // roll control generates aggressive aileron at high speed that causes
        // roll departure before rotation completes.
        //   Normal rotation: require 85% Vr (some roll authority available).
        //   Uncommanded liftoff (XB-70, DHC6): require 100% Vr — the
        //     aircraft lifted off with excessive pitch and has very low
        //     forward airspeed component; engaging heading hold too early
        //     causes roll divergence.
        bool is_airborne = !engines_.IsWeightOnWheels() && agl_m > 5.0;
        double min_hdg_kts = (rotation_elapsed_sec_ > ap_.GetControlProfile().rotation_ramp_sec)
                                 ? takeoff_vr_kts_
                                 : takeoff_vr_kts_ * 0.85;
        bool has_roll_authority = vc_kts >= min_hdg_kts;
        if (is_airborne && has_roll_authority && !ap_.GetControlProfile().has_own_autopilot) {
          ap_.SetRollAttitudeMode(1);  // heading-based roll
          ap_.SetRollAutopilotOn(true);
          ap_.SetHeadingHold(true);
        }

        // Rotation: gradual ramp from pre-rotation level to max, scaled by
        // pitch MOI (heavy jets need longer ramp).  The ramp continues until
        // fully complete regardless of altitude — cutting it short at agl=10m
        // caused an abrupt elevator step from -0.30 to +0.10 that destabilized
        // heavy aircraft (MD11).
        const auto& rot_profile = ap_.GetControlProfile();
        double ramp_progress = std::min(rotation_elapsed_sec_ / rot_profile.rotation_ramp_sec, 1.0);
        bool ramp_complete = ramp_progress >= 1.0;

        if (!ramp_complete && (!is_rate_int || rotation_elapsed_sec_ < dt_sec * 2)) {
          double el;
          if (is_rate_int) {
            el = -0.05;  // brief impulse for rate-integrator
          } else {
            double target_el = -rot_profile.rotation_max_elevator;
            el = rotation_ramp_origin_ + (target_el - rotation_ramp_origin_) * ramp_progress;
          }
          set_tko_el(el);
        } else if (!ramp_complete) {
          // FBW rate-integrator: idle while ramp nominally runs.
          set_tko_el(0.0);
        } else {
          // Ramp complete: transition to stabilized climb.
          // Gear and flaps retraction after positive climb confirmed.
          if (agl_m > 20.0) engines_.SetGearDown(false);
          if (agl_m > 30.0) engines_.SetFlaps(0.0);

          constexpr double kAltHoldHandoffAltM = 150.0;
          if (agl_m > kAltHoldHandoffAltM &&
              !ap_.GetControlProfile().has_own_autopilot) {
            // Above handoff altitude: switch from pitch hold to altitude hold.
            // The AP pitch channel PD controller smoothly transitions to
            // altitude-based targeting without the oscillation of the previous
            // direct climb-rate controller.
            ap_.SetPitchHold(false);
            ap_.SetAltitudeHold(true);
          } else if (!ap_.GetControlProfile().has_own_autopilot && !is_rate_int) {
            // Below handoff altitude: use AP pitch hold for stabilized climb.
            // The AP's PD controller with pitch-rate damping is far more stable
            // than the direct climb-rate P controller which oscillated badly on
            // heavy aircraft (MD11, B747).
            double target_pitch = engines_.GetClimbPitchDeg();
            double current_pitch_deg = adapter_.GetProperty("attitude/pitch-rad") * 57.2958;
            // Gradual pitch recovery for uncommanded liftoff: limit per-frame
            // pitch target change to avoid full-elevator corrections that
            // trigger roll departure.  Delta-wing aircraft (AR < 2.5) skip
            // this limit — they have strong pitch-up moments at low speed
            // that require aggressive elevator to counter.
            constexpr double kMaxPitchRecoveryDegPerSec = 5.0;
            bool is_delta_wing = false;
            {
              double b = adapter_.GetProperty("metrics/bw-ft");
              double s = adapter_.GetProperty("metrics/Sw-sqft");
              if (b > 1.0 && s > 1.0 && (b * b) / s < 2.5) is_delta_wing = true;
            }
            if (current_pitch_deg > target_pitch + 5.0 && !is_delta_wing) {
              target_pitch = std::max(target_pitch,
                                      current_pitch_deg - kMaxPitchRecoveryDegPerSec * dt_sec);
            }
            ap_.SetPitchTargetDeg(target_pitch);
            ap_.SetPitchHold(true);
          } else if (!is_rate_int) {
            // Aircraft with own autopilot: direct climb-rate fallback.
            double target_climb_mps = rot_profile.rotation_climb_rate_mps;
            double climb_err = target_climb_mps - sink_rate_mps_;
            double elevator = std::clamp(-0.05 * climb_err, -0.5, 0.3);
            set_tko_el(elevator);
          }
        }
        if (ap_.GetControlProfile().has_own_autopilot) {
          if (agl_m >= takeoff_target_altitude_m_ * 0.95) {
            ap_.SetAltitudeHold(true);
            takeoff_phase_ = TakeoffPhase::kComplete;
          }
        } else {
          if (agl_m >= takeoff_target_altitude_m_ * 0.95) {
            takeoff_phase_ = TakeoffPhase::kComplete;
          }
        }
        break;
      }
      case TakeoffPhase::kComplete:
        break;
    }
  }
  if (current_maneuver_.type == ManeuverType::kLand) {
    double agl_m = adapter_.GetProperty("position/h-agl-ft") * 0.3048;
    double vc_mps = adapter_.GetProperty("velocities/vc-fps") * 0.3048;
    double vc_fps = adapter_.GetProperty("velocities/vc-fps");
    const auto& profile = ap_.GetControlProfile();

    sink_rate_mps_ = (agl_m - prev_alt_m_) / dt_sec;
    prev_alt_m_ = agl_m;

    bool is_fbw = profile.fbw_subtype != autopilot::FbwSubtype::kNone;
    // FBW aircraft (f16) use pitch-trim-cmd-norm which feeds through
    // the FBW rate limiter/scheduler. Direct-surface aircraft use elevator-cmd-norm.
    auto set_el = [&](double el_norm) {
      if (is_fbw) {
        adapter_.SetProperty("fcs/pitch-trim-cmd-norm", el_norm);
      } else {
        adapter_.SetProperty("fcs/elevator-cmd-norm", el_norm);
      }
    };
    auto neutralize_lateral = [&]() {
      ap_.SetHeadingHold(false);
      ap_.SetRollAutopilotOn(false);
      adapter_.SetProperty("fcs/aileron-cmd-norm", 0.0);
      const std::string& yaw_prop = profile.yaw_input_property;
      if (!yaw_prop.empty()) {
        adapter_.SetProperty(yaw_prop, 0.0);
      }
    };

    switch (land_phase_) {
      case LandPhase::kDecelerate: {
        // High-altitude descent: use AP altitude hold to bring the aircraft
        // down from cruise altitude before aggressive deceleration.  Cutting
        // throttle to idle at 8000m causes zoom-climb → phugoid divergence.
        if (agl_m > profile.landing_high_descent_agl_m && !is_fbw) {
          double intermediate_alt = std::max(land_target_alt_m_ + profile.landing_pattern_agl_m,
                                             profile.landing_staging_agl_m);
          ap_.SetAltitudeTargetM(intermediate_alt);
          ap_.SetAltitudeHold(true);
          ap_.SetSpeedHold(false);
          // Turboprops produce significant residual thrust at moderate throttle;
          // use a lower setting so the aircraft actually descends (DHC6).
          double descent_thr = profile.landing_descent_throttle >= 0.0
                                    ? profile.landing_descent_throttle
                                    : (engines_.GetType() == propulsion::EngineType::kTurboprop)
                                          ? 0.50
                                          : 0.70;
          engines_.SetThrottle(descent_thr);
          if (profile.landing_high_descent_orbit &&
              vc_mps > std::max(land_approach_speed_mps_ * kLandingHighSpeedOrbitFactor,
                                 kLandingHighAltitudeOrbitMinSpeedMps)) {
            double orbit_radius_m = std::max(kLandingHighAltitudeOrbitMinRadiusM,
                                             vc_mps * kLandingHighAltitudeOrbitRadiusPerSpeed);
            double heading_rad = ComputeClockwiseOrbitHeadingRad(
                adapter_.GetPropagate().GetLocation(), current_maneuver_.target, orbit_radius_m,
                std::max(vc_mps, kLandingOrbitHeadingMinSpeedMps));
            ap_.SetHeadingTargetRad(heading_rad);
            ap_.SetHeadingSourceIsWaypoint(false);
          }
          if (agl_m < intermediate_alt + kLandingPatternCaptureMarginM) {
            ap_.SetAltitudeHold(false);
            land_phase_ = LandPhase::kApproach;
          }
          break;
        }
        engines_.SetThrottle(kLandingIdleThrottle);
        if (is_fbw) {
          // FBW aircraft: wings-level, no heading hold — avoid extreme roll
          // at supersonic speeds. Just fly straight and decelerate.
          ap_.SetHeadingHold(false);
          set_el(kLandingFbwDecelElevator);
        } else if (profile.landing_high_descent_orbit &&
                   vc_mps > land_approach_speed_mps_ * kLandingHighSpeedOrbitFactor) {
          // Too fast for straight-in: orbit near landing point while holding
          // pattern altitude. Heavy transports otherwise skim the runway at
          // 200+ kt before reaching approach speed.
          double pattern_alt = land_target_alt_m_ + profile.landing_pattern_agl_m;
          ap_.SetAltitudeTargetM(pattern_alt);
          ap_.SetAltitudeHold(true);
          ap_.SetSpeedHold(false);
          double orbit_radius_m = std::max(kLandingPatternOrbitMinRadiusM,
                                           vc_mps * kLandingPatternOrbitRadiusPerSpeed);
          double heading_rad = ComputeClockwiseOrbitHeadingRad(
              adapter_.GetPropagate().GetLocation(), current_maneuver_.target, orbit_radius_m,
              std::max(vc_mps, kLandingOrbitHeadingMinSpeedMps));
          ap_.SetHeadingTargetRad(heading_rad);
          ap_.SetHeadingSourceIsWaypoint(false);
          ap_.SetHeadingHold(true);
        } else if (vc_mps > kLandingOrbitDisabledHighSpeedMps) {
          // High-speed fallback for aircraft with orbit disabled.
          double sink_err = sink_rate_mps_ - kLandingLevelSinkTargetMps;
          double el = std::clamp(kLandingHighSpeedFallbackElevatorGain * sink_err,
                                 -kLandingHighSpeedFallbackElevatorLimit,
                                 kLandingHighSpeedFallbackElevatorLimit);
          set_el(el);
        } else {
          ap_.SetAltitudeHold(false);
          set_el(kLandingFbwDecelElevator);
        }
        if (vc_mps < land_approach_speed_mps_ * kLandingApproachEntrySpeedFactor) {
          ap_.SetAltitudeHold(false);
          if (is_fbw) ap_.SetHeadingHold(true);
          land_phase_ = LandPhase::kApproach;
        }
        break;
      }
      case LandPhase::kApproach: {
        double alt_target = land_target_alt_m_ + profile.landing_pattern_agl_m;
        if (agl_m < alt_target) {
          ConfigureForLanding();
          land_phase_ = LandPhase::kFinalDescent;
          break;
        }
        double speed_err = land_approach_speed_mps_ - vc_mps;
        double alt_above = agl_m - alt_target;
        // High above pattern: prioritize controlled descent.
        // Aggressive pitch-up at high speed causes zoom climbs → PIO.
        if (alt_above > kLandingApproachHighAltitudeBandM) {
          double thr = std::clamp(kLandingApproachThrottleBase -
                                      kLandingApproachThrottleAltGain * alt_above,
                                  kLandingApproachThrottleMin, kLandingApproachThrottleMax);
          engines_.SetThrottle(thr);
          double el = std::clamp(kLandingApproachSpeedElevatorGain * speed_err,
                                 kLandingApproachElevatorMin, kLandingApproachElevatorMax);
          set_el(el);
        } else {
          // Near pattern altitude: fixed glide-slope attitude, throttle for speed.
          double target_fpa_deg = kLandingTargetFpaDeg;
          double cur_fpa_deg =
              vc_mps > 0 ? std::atan2(sink_rate_mps_, vc_mps) * kLandingRadToDeg : 0.0;
          double fpa_err = cur_fpa_deg - target_fpa_deg;
          double el = std::clamp(kLandingFpaElevatorGain * fpa_err,
                                 -kLandingFpaElevatorLimit, kLandingFpaElevatorLimit);
          set_el(el);
          double spd_err = vc_mps - land_approach_speed_mps_;
          double thr = std::clamp(kLandingNearPatternThrottleBase -
                                      kLandingNearPatternThrottleSpeedGain * spd_err,
                                  kLandingNearPatternThrottleMin,
                                  kLandingNearPatternThrottleMax);
          engines_.SetThrottle(thr);
        }
        break;
      }
      case LandPhase::kFinalDescent: {
        // Too fast for final descent: reduce throttle, gentle nose-up to bleed speed.
        if (vc_mps > land_approach_speed_mps_ * kLandingFinalTooFastFactor) {
          engines_.SetThrottle(0.0);
          set_el(kLandingFinalTooFastElevator);
          break;
        }
        // Flare initiation altitude: proportional to approach speed.
        // Fast aircraft need more altitude to arrest the descent.
        // Heavy transports that use the standard progressive flare law
        // need a lower flare start, because their large wing area
        // generates excess lift — starting too high causes prolonged
        // float and eventual stall (MD11 at 60m AGL).  Aircraft with
        // the explicit heavy flare law (B747, landing_heavy_flare=true)
        // use the standard scale — their gentler flare profile works
        // best with more altitude to work with.
        constexpr double kLandingHeavyFlareAltitudePerSpeed = 0.25;
        bool is_heavy = profile.engine_count >= 4 ||
            (profile.pitch_moi_lbsft2 > 0.0 &&
             std::log10(profile.pitch_moi_lbsft2) > 7.0);
        double flare_scale = (is_heavy && !profile.landing_heavy_flare)
                                 ? kLandingHeavyFlareAltitudePerSpeed
                                 : kLandingFlareAltitudePerSpeed;
        double flare_alt_m = std::max(kLandingFlareMinAltitudeM,
                                      vc_mps * flare_scale);
        if (agl_m < land_target_alt_m_ + flare_alt_m) {
          engines_.SetThrottle(0.0);
          // Heavy transports use a gentler initial flare elevator to prevent
          // the large wing from generating excessive lift at high speed.
          double init_flare_el = profile.landing_flare_initial_elevator;
          if (init_flare_el == 0.0) {
            init_flare_el = profile.landing_heavy_flare
                                 ? kLandingHeavyFlareInitialElevator
                                 : kLandingStandardFlareInitialElevator;
          }
          set_el(init_flare_el);
          flare_elapsed_sec_ = 0.0;
          land_phase_ = LandPhase::kFlare;
          break;
        }
        // Fixed glide-slope attitude, throttle for sink rate.
        double target_fpa_deg = kLandingTargetFpaDeg;
        double cur_fpa_deg =
            vc_mps > 0 ? std::atan2(sink_rate_mps_, vc_mps) * kLandingRadToDeg : 0.0;
        double fpa_err = cur_fpa_deg - target_fpa_deg;
        double el = std::clamp(kLandingFpaElevatorGain * fpa_err,
                               -kLandingFpaElevatorLimit, kLandingFpaElevatorLimit);
        set_el(el);
        double sink_err = kLandingFinalSinkTargetMps - sink_rate_mps_;
        double thr = std::clamp(kLandingFinalThrottleBase +
                                    kLandingFinalThrottleSinkGain * sink_err,
                                kLandingFinalThrottleMin, kLandingFinalThrottleMax);
        // Speed management: cap throttle when above approach speed to
        // allow deceleration.  Without this, the sink-rate-based throttle
        // keeps the aircraft fast through final descent, causing excess
        // lift at flare (B747 at 160 kts generates lift ≈ weight).
        if (vc_mps > land_approach_speed_mps_ * kLandingFinalThrottleCapSpeedFactor) {
          thr = std::min(thr, profile.landing_final_throttle_cap);
        }
        engines_.SetThrottle(thr);
        break;
      }
      case LandPhase::kFlare:
        flare_elapsed_sec_ += dt_sec;
        neutralize_lateral();
        engines_.SetThrottle(0.0);
        {
          if (profile.landing_heavy_flare) {
            // Heavy transport: altitude-based flare with bounce/float recovery.
            // The B747 arrives at the flare with a steep descent (~-7 m/s)
            // at high speed (~150 kts).  A gentle nose-up elevator arrests
            // the descent, but gear contact at ~5m AGL can cause a bounce.
            // The bounce/float recovery detects upward motion or hovering
            // and pushes the nose down to commit to touchdown.
            double flare_progress = 1.0 - std::min(agl_m / kLandingHeavyFlareScaleM, 1.0);
            double el_flare = kLandingHeavyFlareBaseElevator +
                              kLandingHeavyFlareProgressElevator * flare_progress;
            // Bounce recovery: if ascending at low altitude (gear rebound),
            // apply immediate nose-down to arrest the bounce.
            if (sink_rate_mps_ > kLandingBounceRecoverySinkMps &&
                agl_m < kLandingBounceRecoveryAglM) {
              el_flare = std::max(el_flare, kLandingBounceRecoveryElevator);
            }
            // Float recovery: if barely descending near the ground for
            // several seconds, push nose down to commit to touchdown.
            if (sink_rate_mps_ > kLandingFloatRecoverySinkMps &&
                agl_m < kLandingFloatRecoveryAglM &&
                flare_elapsed_sec_ > kLandingFloatRecoveryDelaySec) {
              el_flare = std::max(el_flare, kLandingFloatRecoveryElevator);
            }
            set_el(el_flare);
          } else {
            // Standard progressive flare: more nose-up as altitude decreases.
            double flare_progress = 1.0 - std::min(agl_m / kLandingStandardFlareScaleM, 1.0);
            double el_flare = kLandingStandardFlareBaseElevator +
                              kLandingStandardFlareProgressElevator * flare_progress;
            set_el(el_flare);
          }
        }
        // Touchdown detection: require WOW and an aircraft-specific AGL gate.
        // Large transports can report WOW while the body AGL is still well
        // above the default small-aircraft threshold due to tall landing gear.
        if ((engines_.IsWeightOnWheels() && agl_m < profile.landing_touchdown_agl_m) ||
            agl_m < kLandingGroundContactAglM) {
          engines_.SetBrakes(true);
          set_el(0.0);
          land_phase_ = LandPhase::kTouchdown;
        }
        break;
      case LandPhase::kTouchdown:
        neutralize_lateral();
        engines_.SetThrottle(0.0);
        engines_.SetBrakes(true);
        if (vc_fps < kLandingTouchdownRolloutSpeedFps) land_phase_ = LandPhase::kRollout;
        break;
      case LandPhase::kRollout:
        neutralize_lateral();
        engines_.SetThrottle(0.0);
        engines_.SetBrakes(true);
        if (vc_fps < kLandingCompleteSpeedFps) land_phase_ = LandPhase::kComplete;
        break;
      case LandPhase::kComplete:
        break;
    }
  }
}

void ManeuverExecutor::ExecuteLand(const Waypoint& target, double approach_speed_mps) {
  current_maneuver_.type = ManeuverType::kLand;
  current_maneuver_.target = target;
  current_maneuver_.value = approach_speed_mps;
  active_ = true;
  elapsed_sec_ = 0.0;
  land_phase_ = LandPhase::kDecelerate;

  ConfigureForApproach(target, approach_speed_mps);
}

void ManeuverExecutor::ConfigureForApproach(const Waypoint& target, double approach_speed_mps) {
  // Clear waypoint tracking — landing needs a fixed heading, not continuous
  // waypoint guidance which can cause orbiting (observed 737 at 45° bank).
  wp_manager_.ClearWaypoints();
  ap_.ReleaseHolds();

  // Level wings first — fast jets may be banked from cruise (737: 45°).
  ap_.SetRollAttitudeMode(0);  // wings level
  ap_.SetRollAutopilotOn(true);
  // Brief delay for wings to level, then engage heading to landing point.
  // (Heading hold will override roll mode on next AP update cycle.)

  // Compute a single heading to the landing point.
  const auto& loc = adapter_.GetPropagate().GetLocation();
  double hdg_rad = loc.GetHeadingTo(target.longitude_rad, target.latitude_rad);
  ap_.SetHeadingSourceIsWaypoint(false);
  ap_.SetHeadingTargetRad(hdg_rad);
  ap_.SetHeadingHold(true);

  // Disable AP altitude/speed hold — landing uses direct pitch+throttle control.
  // Fast jets: decelerate before engaging approach controls.
  // Don't try to hold altitude at high speed — let it descend naturally.
  double cur_kts = adapter_.GetProperty("velocities/vc-kts");
  if (cur_kts > 300.0) {
    engines_.SetThrottle(0.1);
  }
  ap_.SetAltitudeHold(false);
  ap_.SetPitchHold(false);
  ap_.SetSpeedHold(false);

  engines_.SetGearDown(true);

  // Approach speed: from parameter, Vr*1.3, or type-based default.
  double cur_spd = adapter_.GetProperty("velocities/vc-fps") * 0.3048;
  const auto& profile = ap_.GetControlProfile();
  if (profile.landing_approach_speed_mps > 0.0) {
    land_approach_speed_mps_ = profile.landing_approach_speed_mps;
  } else if (approach_speed_mps > 0.0) {
    land_approach_speed_mps_ = approach_speed_mps;
  } else {
    double vr_mps = engines_.GetRotationSpeedKts() * 0.514;
    if (vr_mps > 20.0) {
      land_approach_speed_mps_ = vr_mps * 1.3;
    } else {
      land_approach_speed_mps_ = engines_.GetDefaultApproachSpeedMps();
    }
  }
  // Upper bound: don't try to hold speed above 75% of current.
  if (land_approach_speed_mps_ > cur_spd * 0.75) land_approach_speed_mps_ = cur_spd * 0.75;
  land_target_alt_m_ = target.altitude_m;
  prev_alt_m_ = adapter_.GetProperty("position/h-agl-ft") * 0.3048;

  engines_.SetFlaps(profile.landing_approach_flaps_norm);
}

void ManeuverExecutor::ConfigureForLanding() {
  ap_.SetAltitudeHold(false);
  ap_.SetPitchHold(false);
  ap_.SetSpeedHold(false);
  engines_.SetGearDown(true);
  engines_.SetFlaps(ap_.GetControlProfile().landing_final_flaps_norm);
  engines_.SetThrottle(0.3);
  land_target_alt_m_ = current_maneuver_.target.altitude_m;
}

void ManeuverExecutor::Abort() { active_ = false; }

}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq
