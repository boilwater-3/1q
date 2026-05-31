#include "1q/flight_dynamic/guidance/Maneuver.h"

#include <algorithm>
#include <cmath>

#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/guidance/WaypointManager.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/propulsion/EngineManager.h"
#include "flight_dynamic/adapter/PropertyNames.h"
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
  double tangent_heading = radial_angle + M_PI / 2.0;

  double radial_error = distance_m - radius_m;

  // Intercept angle: proportional correction toward orbit track.
  // At high speed / tight radius the required bank exceeds structural limits,
  // causing the orbit to widen — the intercept naturally shrinks as the
  // aircraft converges.  Speed is logged but not used to cap the intercept
  // because capping prevents high-performance aircraft from capturing the
  // orbit track quickly enough.
  double intercept = std::atan2(radial_error, radius_m);

  (void)speed_mps;

  return NormalizeRad(tangent_heading + intercept);
}

}  // namespace

ManeuverExecutor::ManeuverExecutor(adapter::JsbsimAdapter& adapter, autopilot::Autopilot& ap,
                                   WaypointManager& wp_manager,
                                   propulsion::EngineManager& engines)
    : adapter_(adapter), ap_(ap), wp_manager_(wp_manager), engines_(engines) {}

void ManeuverExecutor::ExecuteFlyTo(const Waypoint& target) {
  current_maneuver_.type = ManeuverType::kFlyToWaypoint;
  current_maneuver_.target = target;
  active_ = true;
  elapsed_sec_ = 0.0;

  wp_manager_.ClearWaypoints();
  wp_manager_.AddWaypoint(target);
  wp_manager_.Start();

  ap_.SetLateralGuidanceMode(autopilot::LateralGuidanceMode::kHeading);
  ap_.SetHeadingSourceIsWaypoint(true);
  ap_.SetRollAttitudeMode(1);
  ap_.SetRollAutopilotOn(true);
  ap_.SetHeadingHold(true);
  ap_.SetAltitudeTargetM(target.altitude_m);
  ap_.SetAltitudeHold(true);

  // Speed target: cap at profile ref_speed to prevent overspeed (f16: 844kts).
  double target_spd = ap_.GetTrueSpeedMps();
  double ref = ap_.GetControlProfile().ref_speed_mps;
  if (ref > 0.0 && target_spd > ref) target_spd = ref;
  ap_.SetSpeedTargetMps(target_spd);
  ap_.SetSpeedHold(true);
}

void ManeuverExecutor::ExecuteOrbit(const Waypoint& center, double radius_m, double duration_sec) {
  current_maneuver_.type = ManeuverType::kOrbit;
  current_maneuver_.target = center;
  current_maneuver_.value = std::abs(radius_m);
  current_maneuver_.duration_sec = duration_sec;
  if (current_maneuver_.value < 1.0) current_maneuver_.value = 1.0;
  active_ = true;
  elapsed_sec_ = 0.0;

  // Use a waypoint at center + offset for circular track
  Waypoint orbit_wp = center;
  orbit_wp.radius_m = radius_m;
  wp_manager_.ClearWaypoints();
  wp_manager_.AddWaypoint(orbit_wp);
  wp_manager_.Start();

  ap_.SetLateralGuidanceMode(autopilot::LateralGuidanceMode::kOrbit);
  ap_.SetHeadingSourceIsWaypoint(false);
  ap_.SetRollAttitudeMode(1);
  ap_.SetRollAutopilotOn(true);
  ap_.SetHeadingHold(true);
  ap_.SetAltitudeTargetM(center.altitude_m);
  ap_.SetAltitudeHold(true);
  ap_.SetSpeedTargetMps(ap_.GetTrueSpeedMps());
  ap_.SetSpeedHold(true);
}

void ManeuverExecutor::ExecuteSetHeading(double heading_rad) {
  current_maneuver_.type = ManeuverType::kSetHeading;
  current_maneuver_.value = heading_rad;
  active_ = true;
  elapsed_sec_ = 0.0;

  ap_.SetLateralGuidanceMode(autopilot::LateralGuidanceMode::kHeading);
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

void ManeuverExecutor::ExecuteTakeoff(double target_altitude_m, double target_heading_rad,
                                      double target_speed_mps) {
  current_maneuver_.type = ManeuverType::kTakeoff;
  current_maneuver_.value = target_altitude_m;
  current_maneuver_.duration_sec = target_speed_mps;
  current_maneuver_.target.altitude_m = target_altitude_m;
  active_ = true;
  elapsed_sec_ = 0.0;
  takeoff_phase_ = TakeoffPhase::kEngineStart;
  takeoff_target_altitude_m_ = target_altitude_m;
  takeoff_target_heading_rad_ = target_heading_rad;

  StartEngine();
}

void ManeuverExecutor::StartEngine() {
  engines_.SetBrakes(true);
  engines_.SetThrottle(1.0);
  engines_.Start();
}

void ManeuverExecutor::ConfigureForTakeoffRoll() {
  // Release brakes, takeoff flaps, full throttle for takeoff roll.
  engines_.SetBrakes(false);
  engines_.SetFlaps(0.33);
  engines_.SetThrottle(1.0);
}

void ManeuverExecutor::ConfigureForClimb(double target_altitude_m, double target_heading_rad,
                                         double /*target_speed_mps*/) {
  // Rotate: elevator back. Gear/flaps stay until positive climb confirmed.
  rotation_elapsed_sec_ = 0.0;
  double el_rot = ap_.GetControlProfile().fbw_subtype ==
                          autopilot::FbwSubtype::kRateIntegratorActuator
                  ? -0.05 : -0.3;
  adapter_.SetProperty("fcs/elevator-cmd-norm", el_rot);

  ap_.SetRollAttitudeMode(1);
  ap_.SetRollAutopilotOn(true);
  ap_.SetHeadingTargetRad(target_heading_rad);
  ap_.SetHeadingHold(true);
  ap_.SetAltitudeTargetM(target_altitude_m);
}

bool ManeuverExecutor::IsManeuverComplete() const {
  if (!active_) return true;

  switch (current_maneuver_.type) {
    case ManeuverType::kFlyToWaypoint:
      if (ap_.GetControlProfile().lateral_interface ==
          autopilot::LateralControlInterface::kFbwRateCommand) {
        return wp_manager_.IsAtTarget(current_maneuver_.target.radius_m * 1.6);
      }
      return wp_manager_.IsAtTarget();
    case ManeuverType::kOrbit:
      if (current_maneuver_.duration_sec > 0.0) {
        return elapsed_sec_ >= current_maneuver_.duration_sec;
      }
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
    double speed_mps = adapter_.GetPropagate().GetInertialVelocityMagnitude() * 0.3048;
    if (speed_mps < 10.0) speed_mps = 10.0;
    double heading_rad = ComputeClockwiseOrbitHeadingRad(adapter_.GetPropagate().GetLocation(),
                                                         current_maneuver_.target, radius_m,
                                                         speed_mps);
    ap_.SetHeadingTargetRad(heading_rad);
  }
  if (current_maneuver_.type == ManeuverType::kTakeoff) {
    double vc_kts = adapter_.GetProperty("velocities/vc-kts");
    double agl_ft = adapter_.GetProperty("position/h-agl-ft");
    double agl_m = agl_ft * 0.3048;
    sink_rate_mps_ = (agl_m - prev_alt_m_) / dt_sec;
    prev_alt_m_ = agl_m;

    switch (takeoff_phase_) {
      case TakeoffPhase::kEngineStart:
        if (elapsed_sec_ >= 2.0) {
          ConfigureForTakeoffRoll();
          takeoff_phase_ = TakeoffPhase::kTakeoffRoll;
        }
        break;
      case TakeoffPhase::kTakeoffRoll:
        engines_.SetThrottle(1.0);
        if (vc_kts >= engines_.GetRotationSpeedKts()) {
          ConfigureForClimb(takeoff_target_altitude_m_,
                            takeoff_target_heading_rad_,
                            current_maneuver_.duration_sec);
          takeoff_phase_ = TakeoffPhase::kRotateAndClimb;
        }
        break;
      case TakeoffPhase::kRotateAndClimb: {
        engines_.SetThrottle(1.0);
        rotation_elapsed_sec_ += dt_sec;
        bool is_fbw = ap_.GetControlProfile().fbw_subtype ==
                      autopilot::FbwSubtype::kRateIntegratorActuator;
        // Rotation: single impulse for FBW (integrator processes once),
        // continuous hold for direct-surface aircraft.
        if (agl_m < 10.0 && (!is_fbw || rotation_elapsed_sec_ < dt_sec * 2)) {
          double el = is_fbw ? -0.05 : -0.3;
          adapter_.SetProperty("fcs/elevator-cmd-norm", el);
        } else if (agl_m < 10.0) {
          adapter_.SetProperty("fcs/elevator-cmd-norm", 0.0);
        } else {
          // Gear and flaps retraction after positive climb confirmed.
          if (agl_m > 20.0) engines_.SetGearDown(false);
          if (agl_m > 30.0) engines_.SetFlaps(0.0);
          // Vertical speed control: pitch adjusts climb rate, not pitch angle.
          // Target 5 m/s climb, use pitch to regulate.
          double target_climb_mps = 5.0;
          double climb_err = target_climb_mps - sink_rate_mps_;
          double elevator = std::clamp(-0.05 * climb_err, -0.5, 0.3);
          adapter_.SetProperty("fcs/elevator-cmd-norm", elevator);
          if (!ap_.GetControlProfile().has_own_autopilot) {
            ap_.SetAltitudeHold(true);
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

    sink_rate_mps_ = (agl_m - prev_alt_m_) / dt_sec;
    prev_alt_m_ = agl_m;

    switch (land_phase_) {
      case LandPhase::kApproach: {
        double alt_target = land_target_alt_m_ + 200.0;
        if (agl_m < alt_target) {
          ConfigureForLanding();
          land_phase_ = LandPhase::kFinalDescent;
          break;
        }
        // Pitch for speed, throttle for altitude.
        double speed_err = land_approach_speed_mps_ - vc_mps;
        double el = std::clamp(0.02 * speed_err, -0.5, 0.3);
        adapter_.SetProperty("fcs/elevator-cmd-norm", el);
        double alt_err = agl_m - alt_target;
        // Above target → reduce throttle to descend.
        double thr = std::clamp(0.50 - 0.003 * alt_err, 0.15, 0.8);
        engines_.SetThrottle(thr);
        break;
      }
      case LandPhase::kFinalDescent: {
        if (agl_m < land_target_alt_m_ + 15.0) {
          engines_.SetThrottle(0.0);
          adapter_.SetProperty("fcs/elevator-cmd-norm", -0.15);
          land_phase_ = LandPhase::kFlare;
          break;
        }
        double speed_err = land_approach_speed_mps_ - vc_mps;
        double el = std::clamp(0.02 * speed_err, -0.5, 0.3);
        adapter_.SetProperty("fcs/elevator-cmd-norm", el);
        double sink_err = -3.0 - sink_rate_mps_;
        double thr = std::clamp(0.25 + 0.05 * sink_err, 0.0, 0.6);
        engines_.SetThrottle(thr);
        break;
      }
      case LandPhase::kFlare:
        engines_.SetThrottle(0.0);
        adapter_.SetProperty("fcs/elevator-cmd-norm", -0.15);
        if (engines_.IsWeightOnWheels() ||
            (agl_m < 2.0 && vc_fps < 5.0)) {
          engines_.SetBrakes(true);
          adapter_.SetProperty("fcs/elevator-cmd-norm", 0.0);
          land_phase_ = LandPhase::kTouchdown;
        }
        break;
      case LandPhase::kTouchdown:
        engines_.SetThrottle(0.0);
        engines_.SetBrakes(true);
        if (vc_fps < 30.0) land_phase_ = LandPhase::kRollout;
        break;
      case LandPhase::kRollout:
        engines_.SetThrottle(0.0);
        engines_.SetBrakes(true);
        if (vc_fps < 10.0) land_phase_ = LandPhase::kComplete;
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
  land_phase_ = LandPhase::kApproach;

  ConfigureForApproach(target, approach_speed_mps);
}

void ManeuverExecutor::ConfigureForApproach(const Waypoint& target,
                                            double approach_speed_mps) {
  wp_manager_.ClearWaypoints();
  wp_manager_.AddWaypoint(target);
  wp_manager_.Start();

  ap_.SetLateralGuidanceMode(autopilot::LateralGuidanceMode::kHeading);
  ap_.SetHeadingSourceIsWaypoint(true);
  ap_.SetRollAttitudeMode(1);
  ap_.SetRollAutopilotOn(true);
  ap_.SetHeadingHold(true);

  // Disable AP altitude/speed hold — landing uses direct pitch+throttle control.
  ap_.SetAltitudeHold(false);
  ap_.SetSpeedHold(false);

  // Approach speed: use parameter or compute from stall speed.
  if (approach_speed_mps > 0.0) {
    land_approach_speed_mps_ = approach_speed_mps;
  } else {
    land_approach_speed_mps_ = engines_.GetRotationSpeedKts() * 0.514 * 1.2;
  }
  land_target_alt_m_ = target.altitude_m;
  prev_alt_m_ = adapter_.GetProperty("position/h-agl-ft") * 0.3048;

  engines_.SetFlaps(0.5);
}

void ManeuverExecutor::ConfigureForLanding() {
  engines_.SetFlaps(1.0);
  engines_.SetThrottle(0.3);
  land_target_alt_m_ = current_maneuver_.target.altitude_m;
}

void ManeuverExecutor::Abort() { active_ = false; }

}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq
