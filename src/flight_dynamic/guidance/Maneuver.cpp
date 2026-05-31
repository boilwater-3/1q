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

bool ManeuverExecutor::IsTouchingGround() const {
  if (current_maneuver_.type != ManeuverType::kLand) return false;
  return land_phase_ == LandPhase::kFlare ||
         land_phase_ == LandPhase::kTouchdown ||
         land_phase_ == LandPhase::kRollout;
}

bool ManeuverExecutor::IsManeuverComplete() const {
  if (!active_) return true;

  switch (current_maneuver_.type) {
    case ManeuverType::kFlyToWaypoint: {
      double speed_mps = adapter_.GetPropagate().GetInertialVelocityMagnitude() * 0.3048;
      double max_bank_rad = ap_.GetControlProfile().max_roll_angle_deg * 0.0174533;
      double min_turn_radius_m =
          (speed_mps * speed_mps) / (9.81 * std::tan(max_bank_rad));
      double effective_radius_m = std::max(current_maneuver_.target.radius_m,
                                           min_turn_radius_m * 1.5);
      return wp_manager_.IsAtTarget(effective_radius_m);
    }
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

    bool is_fbw = ap_.GetControlProfile().fbw_subtype !=
                  autopilot::FbwSubtype::kNone;
    // FBW aircraft (f16, f22) use pitch-trim-cmd-norm which feeds through
    // the FBW rate limiter/scheduler. Direct-surface aircraft use elevator-cmd-norm.
    auto set_el = [&](double el_norm) {
      if (is_fbw) {
        adapter_.SetProperty("fcs/pitch-trim-cmd-norm", el_norm);
      } else {
        adapter_.SetProperty("fcs/elevator-cmd-norm", el_norm);
      }
    };

    switch (land_phase_) {
      case LandPhase::kDecelerate: {
        engines_.SetThrottle(0.1);
        if (is_fbw) {
          // FBW aircraft: wings-level, no heading hold — avoid extreme roll
          // at supersonic speeds. Just fly straight and decelerate.
          ap_.SetHeadingHold(false);
          set_el(-0.05);
        } else if (vc_mps > 200.0) {
          // Too fast for straight-in: orbit near landing point to bleed speed.
          double orbit_radius_m = std::max(2000.0, vc_mps * 15.0);
          double heading_rad = ComputeClockwiseOrbitHeadingRad(
              adapter_.GetPropagate().GetLocation(),
              current_maneuver_.target, orbit_radius_m,
              std::max(vc_mps, 10.0));
          ap_.SetHeadingTargetRad(heading_rad);
          ap_.SetHeadingSourceIsWaypoint(false);
          double sink_err = sink_rate_mps_ - 0.0;
          double el = std::clamp(0.05 * sink_err, -0.15, 0.15);
          set_el(el);
        } else {
          set_el(-0.05);
        }
        if (vc_mps < land_approach_speed_mps_ * 1.15) {
          if (is_fbw) ap_.SetHeadingHold(true);
          land_phase_ = LandPhase::kApproach;
        }
        break;
      }
      case LandPhase::kApproach: {
        double alt_target = land_target_alt_m_ + 200.0;
        if (agl_m < alt_target) {
          ConfigureForLanding();
          land_phase_ = LandPhase::kFinalDescent;
          break;
        }
        double speed_err = land_approach_speed_mps_ - vc_mps;
        double alt_above = agl_m - alt_target;
        // High above pattern: prioritize controlled descent.
        // Aggressive pitch-up at high speed causes zoom climbs → PIO.
        if (alt_above > 500.0) {
          double thr = std::clamp(0.3 - 0.0005 * alt_above, 0.1, 0.5);
          engines_.SetThrottle(thr);
          double el = std::clamp(0.005 * speed_err, -0.15, 0.1);
          set_el(el);
        } else {
          // Near pattern altitude: fixed glide-slope attitude, throttle for speed.
          double target_fpa_deg = -3.0;
          double cur_fpa_deg = vc_mps > 0 ? std::atan2(sink_rate_mps_, vc_mps) * 57.3 : 0.0;
          double fpa_err = cur_fpa_deg - target_fpa_deg;
          double el = std::clamp(0.1 * fpa_err, -0.2, 0.2);
          set_el(el);
          double spd_err = vc_mps - land_approach_speed_mps_;
          double thr = std::clamp(0.3 - 0.005 * spd_err, 0.1, 0.6);
          engines_.SetThrottle(thr);
        }
        break;
      }
      case LandPhase::kFinalDescent: {
        // Too fast for final descent: reduce throttle, gentle nose-up to bleed speed.
        if (vc_mps > land_approach_speed_mps_ * 1.3) {
          engines_.SetThrottle(0.0);
          set_el(-0.1);
          break;
        }
        double flare_alt_m = std::max(15.0, vc_mps * 0.6);
        if (agl_m < land_target_alt_m_ + flare_alt_m) {
          engines_.SetThrottle(0.0);
          set_el(-0.25);
          land_phase_ = LandPhase::kFlare;
          break;
        }
        // Fixed glide-slope attitude, throttle for sink rate.
        double target_fpa_deg = -3.0;
        double cur_fpa_deg = vc_mps > 0 ? std::atan2(sink_rate_mps_, vc_mps) * 57.3 : 0.0;
        double fpa_err = cur_fpa_deg - target_fpa_deg;
        double el = std::clamp(0.1 * fpa_err, -0.2, 0.2);
        set_el(el);
        double sink_err = -3.0 - sink_rate_mps_;
        double thr = std::clamp(0.25 + 0.05 * sink_err, 0.0, 0.6);
        engines_.SetThrottle(thr);
        break;
      }
      case LandPhase::kFlare:
        engines_.SetThrottle(0.0);
        // Progressive flare: more nose-up as altitude decreases.
        {
          double flare_progress = 1.0 - std::min(agl_m / 30.0, 1.0);
          double el_flare = -0.15 - 0.25 * flare_progress;
          set_el(el_flare);
        }
        if (engines_.IsWeightOnWheels() || agl_m < 0.1) {
          engines_.SetBrakes(true);
          set_el(0.0);
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
  land_phase_ = LandPhase::kDecelerate;

  ConfigureForApproach(target, approach_speed_mps);
}

void ManeuverExecutor::ConfigureForApproach(const Waypoint& target,
                                            double approach_speed_mps) {
  // Clear waypoint tracking — landing needs a fixed heading, not continuous
  // waypoint guidance which can cause orbiting (observed 737 at 45° bank).
  wp_manager_.ClearWaypoints();

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
  ap_.SetSpeedHold(false);

  engines_.SetGearDown(true);

  // Approach speed: from parameter, Vr*1.3, or type-based default.
  double cur_spd = adapter_.GetProperty("velocities/vc-fps") * 0.3048;
  if (approach_speed_mps > 0.0) {
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
  if (land_approach_speed_mps_ > cur_spd * 0.75)
    land_approach_speed_mps_ = cur_spd * 0.75;
  land_target_alt_m_ = target.altitude_m;
  prev_alt_m_ = adapter_.GetProperty("position/h-agl-ft") * 0.3048;

  engines_.SetFlaps(0.5);
}

void ManeuverExecutor::ConfigureForLanding() {
  engines_.SetGearDown(true);
  engines_.SetFlaps(1.0);
  engines_.SetThrottle(0.3);
  land_target_alt_m_ = current_maneuver_.target.altitude_m;
}

void ManeuverExecutor::Abort() { active_ = false; }

}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq
