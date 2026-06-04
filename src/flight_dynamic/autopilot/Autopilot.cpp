#include "1q/flight_dynamic/autopilot/Autopilot.h"

#include <cmath>

#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/adapter/PropertyNames.h"
#include "math/FGLocation.h"
#include "models/FGPropagate.h"
#include "models/FGPropulsion.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace oneq {
namespace flight_dynamic {
namespace autopilot {

namespace {

double Clamp(double value, double min_value, double max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

double NormalizeRad(double angle_rad) {
  while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
  while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
  return angle_rad;
}

double RadToDeg360(double angle_rad) {
  double deg = angle_rad * 180.0 / M_PI;
  while (deg < 0.0) deg += 360.0;
  while (deg >= 360.0) deg -= 360.0;
  return deg;
}

constexpr double kFtToM = 0.3048;
constexpr double kMToFt = 1.0 / kFtToM;
constexpr double kRefSpeedFps = 164.0;  // ~50 m/s reference (c172x cruise)

double ReadPropertyOrDefault(adapter::JsbsimAdapter& adapter, const char* name,
                             double default_value) {
  auto* pm = adapter.GetFdmExec().GetPropertyManager().get();
  auto* node = pm ? pm->GetNode(name) : nullptr;
  return node ? node->getDoubleValue() : default_value;
}

bool HasProperty(adapter::JsbsimAdapter& adapter, const char* name) {
  auto* pm = adapter.GetFdmExec().GetPropertyManager().get();
  return pm && pm->GetNode(name) != nullptr;
}

// Set energy-management defaults based on detected aircraft capability.
// No model-name hardcoding — classifies by engine count, FBW presence,
// and propulsion type from the property tree.
void ApplyEnergyDefaults(AircraftControlProfile* profile) {
  if (!profile) return;

  const bool has_fbw = profile->has_fbw_override || profile->has_roll_rate_command;
  // Heavy classification: 4+ engines OR large pitch MOI (>1e7 slugs·ft²).
  // Captures trijets like MD11 (Iyy=3.8e7, 3 engines) that are heavy
  // transports that don't meet the 4-engine threshold.
  const bool is_heavy = profile->engine_count >= 4 ||
      (profile->pitch_moi_lbsft2 > 0.0 && std::log10(profile->pitch_moi_lbsft2) > 7.0);
  const bool is_medium =
      !is_heavy && profile->pitch_moi_lbsft2 > 0.0 &&
      std::log10(profile->pitch_moi_lbsft2) > 6.0;
  const int n_eng = profile->engine_count;

  if (has_fbw) {
    // High-performance FBW fighter (f16 class)
    profile->ref_speed_mps = 200.0;
    profile->cruise_speed_mps = 200.0;
    profile->min_speed_mps = 140.0;
    profile->max_speed_mps = 350.0;
    profile->max_pitch_command_deg = 15.0;
    profile->max_roll_angle_deg = 45.0;
    profile->min_throttle = 0.35;
    profile->speed_energy_priority = true;
  } else if (is_heavy && !profile->has_mixture) {
    // Heavy jet transport (B747, MD11, Concorde, XB-70).
    profile->ref_speed_mps = 500.0;
    profile->cruise_speed_mps = 250.0;
    profile->min_speed_mps = 130.0;
    profile->max_speed_mps = 300.0;
    profile->max_pitch_command_deg = 8.0;
    profile->max_roll_angle_deg = 35.0;
    profile->min_throttle = 0.55;
    profile->speed_energy_priority = true;
  } else if (is_medium && !profile->has_mixture) {
    // Medium jet transport (737 class, log10(Iyy) > 6).
    profile->ref_speed_mps = 250.0;
    profile->cruise_speed_mps = 200.0;
    profile->min_speed_mps = 100.0;
    profile->max_speed_mps = 260.0;
    profile->max_pitch_command_deg = 10.0;
    profile->max_roll_angle_deg = 35.0;
    profile->min_throttle = 0.40;
    profile->speed_energy_priority = true;
  } else if (profile->has_mixture) {
    if (is_heavy) {
      // Multi-engine piston (B17)
      profile->ref_speed_mps = 85.0;
      profile->cruise_speed_mps = 75.0;
      profile->min_speed_mps = 65.0;
      profile->max_speed_mps = 100.0;
      profile->max_pitch_command_deg = 10.0;
      profile->max_roll_angle_deg = 25.0;
      profile->min_throttle = 0.40;
      profile->speed_energy_priority = true;
    } else {
      // Single/twin piston GA (c172x, c310)
      profile->ref_speed_mps = n_eng >= 2 ? 65.0 : 50.0;
      profile->cruise_speed_mps = n_eng >= 2 ? 60.0 : 50.0;
      profile->min_speed_mps = n_eng >= 2 ? 55.0 : 40.0;
      profile->max_speed_mps = n_eng >= 2 ? 90.0 : 80.0;
      profile->max_pitch_command_deg = n_eng >= 2 ? 10.0 : 12.0;
      profile->max_roll_angle_deg = 30.0;
      profile->min_throttle = n_eng >= 2 ? 0.30 : 0.20;
      profile->speed_energy_priority = (n_eng >= 2);
    }
  } else {
    // Non-piston non-FBW non-heavy: light turbine / turboprop modeled as
    // <turbine_engine> (OV10, f15). Use struct default roll (45°), but set
    // reasonable speed defaults for energy management.
    profile->ref_speed_mps = 80.0;
    profile->cruise_speed_mps = 80.0;
    profile->min_speed_mps = 50.0;
    profile->max_speed_mps = 200.0;
  }
}

// XML guidance/* properties override dynamic defaults.  This keeps property-tree
// detection as the fallback while allowing aircraft XML to carry tuning that is
// specific to its flight envelope or model limitations.
void ApplyXmlProfileOverrides(adapter::JsbsimAdapter& adapter, AircraftControlProfile* profile) {
  if (!profile) return;

  profile->ref_speed_mps =
      ReadPropertyOrDefault(adapter, "guidance/ref-speed-mps", profile->ref_speed_mps);
  profile->cruise_speed_mps =
      ReadPropertyOrDefault(adapter, "guidance/cruise-speed-mps", profile->cruise_speed_mps);
  profile->min_speed_mps =
      ReadPropertyOrDefault(adapter, "guidance/min-speed-mps", profile->min_speed_mps);
  profile->max_speed_mps =
      ReadPropertyOrDefault(adapter, "guidance/max-speed-mps", profile->max_speed_mps);
  profile->max_pitch_command_deg = ReadPropertyOrDefault(
      adapter, "guidance/max-pitch-command-deg", profile->max_pitch_command_deg);
  profile->max_roll_angle_deg =
      ReadPropertyOrDefault(adapter, "guidance/max-roll-angle-deg", profile->max_roll_angle_deg);
  profile->min_throttle =
      ReadPropertyOrDefault(adapter, "guidance/min-throttle", profile->min_throttle);
  profile->max_throttle =
      ReadPropertyOrDefault(adapter, "guidance/max-throttle", profile->max_throttle);
  if (HasProperty(adapter, "guidance/speed-energy-priority")) {
    profile->speed_energy_priority =
        adapter.GetProperty("guidance/speed-energy-priority") > 0.5;
  }

  profile->rotation_ramp_sec =
      ReadPropertyOrDefault(adapter, "guidance/rotation-ramp-sec", profile->rotation_ramp_sec);
  profile->rotation_max_elevator = ReadPropertyOrDefault(
      adapter, "guidance/rotation-max-elevator", profile->rotation_max_elevator);
  profile->rotation_climb_rate_mps = ReadPropertyOrDefault(
      adapter, "guidance/rotation-climb-rate-mps", profile->rotation_climb_rate_mps);

  profile->landing_approach_speed_mps = ReadPropertyOrDefault(
      adapter, "guidance/landing-approach-speed-mps", profile->landing_approach_speed_mps);
  profile->landing_high_descent_agl_m = ReadPropertyOrDefault(
      adapter, "guidance/landing-high-descent-agl-m", profile->landing_high_descent_agl_m);
  profile->landing_staging_agl_m = ReadPropertyOrDefault(
      adapter, "guidance/landing-staging-agl-m", profile->landing_staging_agl_m);
  profile->landing_pattern_agl_m = ReadPropertyOrDefault(
      adapter, "guidance/landing-pattern-agl-m", profile->landing_pattern_agl_m);
  if (HasProperty(adapter, "guidance/landing-high-descent-orbit")) {
    profile->landing_high_descent_orbit =
        adapter.GetProperty("guidance/landing-high-descent-orbit") > 0.5;
  }
  profile->landing_descent_throttle = ReadPropertyOrDefault(
      adapter, "guidance/landing-descent-throttle", profile->landing_descent_throttle);
  profile->landing_approach_flaps_norm = ReadPropertyOrDefault(
      adapter, "guidance/landing-approach-flaps-norm", profile->landing_approach_flaps_norm);
  profile->landing_final_flaps_norm = ReadPropertyOrDefault(
      adapter, "guidance/landing-final-flaps-norm", profile->landing_final_flaps_norm);
  profile->landing_final_throttle_cap = ReadPropertyOrDefault(
      adapter, "guidance/landing-final-throttle-cap", profile->landing_final_throttle_cap);
  profile->landing_flare_initial_elevator = ReadPropertyOrDefault(
      adapter, "guidance/landing-flare-initial-elevator", profile->landing_flare_initial_elevator);
  profile->landing_touchdown_agl_m = ReadPropertyOrDefault(
      adapter, "guidance/landing-touchdown-agl-m", profile->landing_touchdown_agl_m);
}

// Set rotation/takeoff parameters based on pitch moment of inertia.
// Heavier aircraft have larger Iyy → need longer ramp to avoid step input,
// but do NOT reduce max elevator (they need MORE authority to rotate).
// Thresholds on log10(Iyy):
//   >7  heavy transport  (B747 3.3e7, MD11 3.8e7, XB-70 1.6e7, Concorde 1.9e7)
//   >6  medium transport (737 1.5e6, C130 2.4e6)
//   ≤6  light aircraft   (c172x 1.3e3, fighters ~5e4)
void ApplyRotationDefaults(AircraftControlProfile* profile) {
  if (!profile || profile->pitch_moi_lbsft2 <= 0.0) return;

  double log_moi = std::log10(profile->pitch_moi_lbsft2);
  if (log_moi > 7.0) {
    profile->rotation_ramp_sec = 6.0;
    profile->rotation_climb_rate_mps = 3.0;
  } else if (log_moi > 6.0) {
    profile->rotation_ramp_sec = 4.0;
    profile->rotation_climb_rate_mps = 4.0;
  }
  // rotation_max_elevator stays at 0.30 for all — heavy aircraft need
  // full authority to rotate.  The longer ramp prevents step-input departure.
}

}  // namespace

Autopilot::Autopilot(adapter::JsbsimAdapter& adapter) : adapter_(adapter) {
  auto* pm = adapter.GetFdmExec().GetPropertyManager().get();

  // Read the roll angle limit from JSBSim property tree (set by ConfigureIntegrators
  // per aircraft in the adapter). Apply a sustained-turn factor: sustained ≈ structural × 0.7,
  // because sustained turns generate induced drag that limits endurance at structural max.
  if (pm->GetNode(adapter::property::kGuidanceRollAngleLimit) != nullptr) {
    double roll_lim = adapter.GetProperty(adapter::property::kGuidanceRollAngleLimit);
    double sustained_factor = 0.7;
    control_profile_.max_roll_angle_deg = roll_lim * 180.0 / M_PI * sustained_factor;
  }

  // Tier 1: XML property probing — detect aircraft capabilities from
  // the JSBSim property tree.  No model-name hardcoding.
  control_profile_.has_own_autopilot = pm->GetNode(adapter::property::kApHeadingHold) != nullptr;
  control_profile_.has_generic_autopilot = pm->GetNode(adapter::property::kApRollOn) != nullptr;
  control_profile_.has_fbw_override = pm->GetNode(adapter::property::kApFbwOverride) != nullptr;
  control_profile_.has_roll_rate_command = pm->GetNode(adapter::property::kRollRateCommand) != nullptr ||
                                           pm->GetNode(adapter::property::kRollRateCmd) != nullptr;
  control_profile_.has_aileron_command = pm->GetNode(adapter::property::kAileronCmd) != nullptr;

  // Engine count and indexed throttle detection
  const auto propulsion = adapter_.GetFdmExec().GetPropulsion();
  if (propulsion) {
    control_profile_.engine_count = static_cast<int>(propulsion->GetNumEngines());
    // Multi-engine aircraft need per-engine throttle commands.
    control_profile_.indexed_throttle = control_profile_.engine_count > 1;
  }

  // Mixture detection (piston aircraft only — require both mixture and magneto,
  // since JSBSim creates indexed fcs/mixture-cmd-norm[n] for all engine types,
  // and propulsion/magneto_cmd only exists for FGPiston engines).
  control_profile_.has_mixture = pm->GetNode("fcs/mixture-cmd-norm") != nullptr &&
                                 pm->GetNode(adapter::property::kMagnetoCmd) != nullptr;

  // Pitch moment of inertia — determines rotation response and ramp scaling.
  // JSBSim stores this as inertia/iyy-slugs_ft2 (XML unit="SLUG*FT2").
  auto* iyy_node = pm->GetNode("inertia/iyy-slugs_ft2");
  if (iyy_node) {
    control_profile_.pitch_moi_lbsft2 = iyy_node->getDoubleValue();
  }
  ApplyRotationDefaults(&control_profile_);

  // FBW subtype detection: f16 has roll-rate PID
  if (pm->GetNode("fcs/aileron-act") != nullptr &&
      pm->GetNode(adapter::property::kRollCmd) != nullptr) {
    control_profile_.fbw_subtype = FbwSubtype::kRateIntegratorActuator;
  } else if (control_profile_.has_roll_rate_command) {
    control_profile_.fbw_subtype = FbwSubtype::kRollRatePid;
  }

  // Yaw input property detection
  if (pm->GetNode(adapter::property::kRudderCmd) != nullptr) {
    control_profile_.yaw_input_property = adapter::property::kRudderCmd;
  } else if (pm->GetNode("fcs/rudder-pedal-norm") != nullptr) {
    control_profile_.yaw_input_property = "fcs/rudder-pedal-norm";
  }

  // Pitch interface detection
  if (control_profile_.has_own_autopilot || control_profile_.has_generic_autopilot) {
    control_profile_.pitch_interface = PitchControlInterface::kNativeAutopilot;
  } else if (pm->GetNode("fcs/pitch-rate-cmd") != nullptr) {
    control_profile_.pitch_interface = PitchControlInterface::kFbwScheduled;
  }

  // Lateral interface selection.  FBW takes priority over native autopilot
  // because some FBW aircraft (f16) have ap/heading_hold from their flight
  // control system but need kFbwRateCommand for correct roll-rate handling.
  if (control_profile_.has_fbw_override || control_profile_.has_roll_rate_command) {
    control_profile_.lateral_interface = LateralControlInterface::kFbwRateCommand;
  } else if (control_profile_.has_own_autopilot) {
    control_profile_.lateral_interface = LateralControlInterface::kOwnAutopilot;
  } else if (control_profile_.has_generic_autopilot) {
    control_profile_.lateral_interface = LateralControlInterface::kGenericAutopilotBridge;
  } else {
    control_profile_.lateral_interface = LateralControlInterface::kDirectSurface;
  }

  // A single leaked ap/autopilot-roll-on from a shared system file
  // (Autopilot.xml) is not enough evidence for kGenericAutopilotBridge
  // when the aircraft has no own autopilot, no FBW, and fewer than 4
  // engines (indicating no real native AP system — hits OV10).
  if (control_profile_.lateral_interface == LateralControlInterface::kGenericAutopilotBridge &&
      !control_profile_.has_own_autopilot &&
      !control_profile_.has_fbw_override &&
      control_profile_.engine_count < 4) {
    control_profile_.lateral_interface = LateralControlInterface::kDirectSurface;
  }

  use_cpp_ap_ =
      control_profile_.lateral_interface != LateralControlInterface::kOwnAutopilot &&
      control_profile_.lateral_interface != LateralControlInterface::kGenericAutopilotBridge;
  ApplyEnergyDefaults(&control_profile_);
  ApplyXmlProfileOverrides(adapter_, &control_profile_);
}

void Autopilot::SetHeadingTargetRad(double heading_rad) {
  target_heading_rad_ = heading_rad;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kGuidanceHeadingRad, heading_rad);
    adapter_.SetProperty(adapter::property::kApHeadingSetpoint, RadToDeg360(heading_rad));
  }
}

void Autopilot::SetHeadingHold(bool on) {
  heading_hold_ = on;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kApHeadingHold, on ? 1.0 : 0.0);
  }
}

void Autopilot::SetHeadingSourceIsWaypoint(bool from_waypoint) {
  heading_src_wp_ = from_waypoint;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kGuidanceHeadingSwitch, from_waypoint ? 0.0 : 1.0);
  }
}

void Autopilot::SetAltitudeTargetM(double altitude_m) {
  target_altitude_m_ = altitude_m;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kApAltitudeSetpoint, altitude_m * kMToFt);
  }
}

void Autopilot::SetAltitudeHold(bool on) { altitude_hold_ = on; }

void Autopilot::SetPitchTargetDeg(double pitch_deg) {
  target_pitch_deg_ = pitch_deg;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kApPitchTarget, pitch_deg);
  }
}

void Autopilot::SetPitchHold(bool on) {
  pitch_hold_ = on;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kApPitchHold, on ? 1.0 : 0.0);
  }
}

void Autopilot::SetLateralGuidanceMode(LateralGuidanceMode mode) { lateral_guidance_mode_ = mode; }

void Autopilot::SetRollAttitudeMode(int mode) {
  roll_mode_ = mode;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kApRollAttitudeMode, static_cast<double>(mode));
  }
}

void Autopilot::SetRollAutopilotOn(bool on) {
  roll_ap_on_ = on;
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kApRollOn, on ? 1.0 : 0.0);
  }
}

void Autopilot::SetThrottleCmdNorm(double value) {
  adapter_.SetProperty(adapter::property::kThrottleCmd, value);
  if (!control_profile_.indexed_throttle &&
      control_profile_.lateral_interface != LateralControlInterface::kOwnAutopilot) {
    return;
  }
  const auto propulsion = adapter_.GetFdmExec().GetPropulsion();
  if (!propulsion) {
    return;
  }
  for (size_t engine = 0; engine < propulsion->GetNumEngines(); ++engine) {
    SetThrottleCmd(static_cast<int>(engine), value);
  }
}

void Autopilot::SetThrottleCmd(int engine, double value) {
  std::string prop = std::string(adapter::property::kThrottleCmd) + "[" + std::to_string(engine) + "]";
  adapter_.SetProperty(prop, value);
}

void Autopilot::SetYawDamper(bool on) {
  if (!use_cpp_ap_) {
    adapter_.SetProperty(adapter::property::kApYawDamper, on ? 1.0 : 0.0);
  }
}

void Autopilot::SetSpeedTargetMps(double speed_mps) { target_speed_mps_ = speed_mps; }
void Autopilot::SetSpeedHold(bool on) { speed_hold_ = on; }

void Autopilot::ReleaseHolds() {
  heading_hold_ = false;
  altitude_hold_ = false;
  pitch_hold_ = false;
  speed_hold_ = false;
  roll_ap_on_ = false;
  roll_mode_ = 0;
  lateral_guidance_mode_ = LateralGuidanceMode::kHeading;

  adapter_.SetProperty(adapter::property::kApHeadingHold, 0.0);
  adapter_.SetProperty(adapter::property::kApAltitudeHold, 0.0);
  adapter_.SetProperty(adapter::property::kApAttitudeHold, 0.0);
  adapter_.SetProperty(adapter::property::kApPitchHold, 0.0);
  adapter_.SetProperty(adapter::property::kApRollOn, 0.0);
  adapter_.SetProperty(adapter::property::kApYawDamper, 0.0);
}

double Autopilot::GetTrueSpeedMps() const {
  return adapter_.GetProperty("velocities/vtrue-fps") * kFtToM;
}

double Autopilot::GetAngleToHeadingRad() const {
  if (use_cpp_ap_ ||
      control_profile_.lateral_interface == LateralControlInterface::kFbwRateCommand) {
    const auto& propagate = adapter_.GetPropagate();
    double current_heading = propagate.GetEuler(3);
    double target_heading = target_heading_rad_;
    if (heading_src_wp_) {
      double target_lat = adapter_.GetProperty(adapter::property::kGuidanceTargetWpLat);
      double target_lon = adapter_.GetProperty(adapter::property::kGuidanceTargetWpLon);
      target_heading = propagate.GetLocation().GetHeadingTo(target_lon, target_lat);
    }
    return NormalizeRad(target_heading - current_heading);
  } else {
    return adapter_.GetProperty(adapter::property::kGuidanceAngleToHeading);
  }
}

double Autopilot::GetAltitudeAGLM() const {
  return adapter_.GetProperty(adapter::property::kHaglFt) * kFtToM;
}

double Autopilot::GetAltitudeASLM() const {
  return adapter_.GetPropagate().GetLocation().GetGeodAltitude() * kFtToM;
}

void Autopilot::Update(double /*dt_sec*/) {
  const auto& propagate = adapter_.GetPropagate();

  // kOwnAutopilot: delegate lateral to XML autopilot, but use C++ pitch
  // control for altitude hold (native AP lacks speed protection and can
  // stall the aircraft at high altitude — observed c310 at 2000m: 25° pitch,
  // 70kts, sinking).
  if (control_profile_.lateral_interface == LateralControlInterface::kOwnAutopilot) {
    UpdateOwnAutopilot();
    UpdateDirectHeadingLateral();
    UpdatePitchChannel();
    UpdateEnergyManagement();
    double r = propagate.GetPQR(3);
    ApplyYawDamping(r);
    return;
  }

  // Lateral dispatch by profile — each profile handles all guidance modes uniformly.
  switch (control_profile_.lateral_interface) {
    case LateralControlInterface::kFbwRateCommand:
      if (control_profile_.fbw_subtype == FbwSubtype::kRateIntegratorActuator ||
          lateral_guidance_mode_ == LateralGuidanceMode::kOrbit) {
        UpdateFbwRateCommandLateral();
      } else {
        UpdateDirectHeadingLateral();
      }
      break;
    case LateralControlInterface::kGenericAutopilotBridge:
      UpdateGenericApBridge();
      UpdateRollAnglePD();
      break;
    case LateralControlInterface::kDirectSurface:
      UpdateDirectHeadingLateral();
      break;
    default:
      break;
  }

  UpdateEnergyManagement();
  UpdatePitchChannel();

  double r = propagate.GetPQR(3);
  ApplyYawDamping(r);
}

void Autopilot::UpdateOwnAutopilot() {
  if (heading_hold_) {
    ApplyNativeHeadingSetpoint();
  }
  adapter_.SetProperty(adapter::property::kApHeadingHold, heading_hold_ ? 1.0 : 0.0);
  adapter_.SetProperty(adapter::property::kApAttitudeHold, heading_hold_ ? 1.0 : 0.0);
  adapter_.SetProperty(adapter::property::kApAltitudeHold, altitude_hold_ ? 1.0 : 0.0);
}

void Autopilot::UpdateGenericApBridge() {
  if (heading_hold_) {
    ApplyNativeHeadingSetpoint();
  }
}

void Autopilot::UpdateFbwRateCommandLateral() {
  const auto& propagate = adapter_.GetPropagate();
  const double roll_rad = propagate.GetEuler(1);
  const double bank_limit_rad =
      control_profile_.max_roll_angle_deg * 0.01745329;

  double roll_rate_cmd = 0.0;
  if (heading_hold_) {
    const double heading_err = GetAngleToHeadingRad();
    roll_rate_cmd = Clamp(0.45 * heading_err, -0.60, 0.60);

    // Bank angle limiting: when current bank approaches structural limit,
    // apply opposing rate command to prevent overshoot regardless of FBW type.
    double bank_ratio = std::abs(roll_rad) / bank_limit_rad;
    if (bank_ratio > 0.80) {
      double roll_sign = (roll_rad >= 0.0) ? 1.0 : -1.0;
      double excess = std::min((bank_ratio - 0.80) / 0.20, 1.0);
      // Blend from heading-driven command to roll-recovery command.
      roll_rate_cmd = roll_rate_cmd * (1.0 - excess) - roll_sign * excess * 0.60;
    }
  }
  if (roll_ap_on_ || heading_hold_ || roll_mode_ == 0) {
    adapter_.SetProperty("fcs/aileron-cmd-norm", roll_rate_cmd);
  }
}

void Autopilot::UpdateDirectHeadingLateral() {
  const auto& propagate = adapter_.GetPropagate();
  const double roll = propagate.GetEuler(1);
  const double p = propagate.GetPQR(1);
  double v_fps = propagate.GetInertialVelocityMagnitude();
  if (v_fps < 10.0) v_fps = 10.0;

  const double speed_ratio = Clamp(kRefSpeedFps / v_fps, 1.4, 1.8);
  double target_roll = 0.0;
  if (heading_hold_) {
    const double heading_err = GetAngleToHeadingRad();
    const double heading_gain = 0.6 * speed_ratio;
    const double roll_limit =
        control_profile_.lateral_interface == LateralControlInterface::kFbwRateCommand ? 1.60
                                                                                       : 0.52;
    target_roll = Clamp(heading_gain * heading_err, -roll_limit, roll_limit);
  }

  const double roll_err = target_roll - roll;
  const double kp_roll = 2.0 * speed_ratio;
  constexpr double kRollDamping = 0.8;
  double aileron = kp_roll * roll_err - kRollDamping * p;
  aileron = Clamp(aileron, -1.0, 1.0);

  if (roll_ap_on_ || heading_hold_ || roll_mode_ == 0) {
    adapter_.SetProperty(adapter::property::kAileronCmd, aileron);
  }
}

void Autopilot::UpdateRollAnglePD() {
  const auto& propagate = adapter_.GetPropagate();
  double roll = propagate.GetEuler(1);
  double p = propagate.GetPQR(1);

  double target_roll = 0.0;
  if (heading_hold_) {
    double heading_err = GetAngleToHeadingRad();
    target_roll = 1.2 * heading_err;
    target_roll = Clamp(target_roll, -0.785, 0.785);
  }

  double roll_err = target_roll - roll;
  double aileron = 3.0 * roll_err + 0.5 * roll_int_ - 0.3 * p;
  if (std::abs(aileron) < 1.0) {
    roll_int_ += 0.005 * roll_err;
    roll_int_ = Clamp(roll_int_, -0.4, 0.4);
  }
  aileron = Clamp(aileron, -1.0, 1.0);
  adapter_.SetProperty(adapter::property::kAileronCmd, aileron);
}

void Autopilot::UpdatePitchChannel() {
  const auto& propagate = adapter_.GetPropagate();
  double pitch = propagate.GetEuler(2);
  double q = propagate.GetPQR(2);

  double target_pitch = 0.0;
  bool pitch_control_active = false;

  if (altitude_hold_) {
    pitch_control_active = true;
    double target_alt_ft = target_altitude_m_ * kMToFt;
    double current_alt_ft = propagate.GetLocation().GetGeodAltitude();
    double alt_err_ft = target_alt_ft - current_alt_ft;

    // Base pitch: altitude PD
    target_pitch = 0.0005 * alt_err_ft;

    // Speed protection: reduce climb pitch if speed is too low.
    double current_speed_mps = GetTrueSpeedMps();
    double min_speed = control_profile_.min_speed_mps;
    if (min_speed > 0.0 && current_speed_mps < min_speed * 1.15 && target_pitch > 0.0) {
      double speed_deficit = Clamp((min_speed * 1.15 - current_speed_mps) / (min_speed * 0.2), 0.0, 1.0);
      target_pitch *= (1.0 - speed_deficit);
    }

    double max_pitch_rad = control_profile_.max_pitch_command_deg * M_PI / 180.0;
    target_pitch = Clamp(target_pitch, -max_pitch_rad, max_pitch_rad);
  } else if (pitch_hold_) {
    pitch_control_active = true;
    target_pitch = target_pitch_deg_ * M_PI / 180.0;
  }

  if (pitch_control_active) {
    double pitch_err = target_pitch - pitch;
    double elevator = -(2.0 * pitch_err - 0.2 * q);
    elevator = Clamp(elevator, -1.0, 1.0);
    if (control_profile_.fbw_subtype != FbwSubtype::kNone) {
      adapter_.SetProperty("fcs/pitch-trim-cmd-norm", elevator);
    } else {
      adapter_.SetProperty(adapter::property::kElevatorCmd, elevator);
    }
  }
}

void Autopilot::UpdateEnergyManagement() {
  if (!altitude_hold_ && !speed_hold_) return;

  const double current_speed_mps = GetTrueSpeedMps();
  // ref_speed normalizes the speed error term.  Use profile ref_speed if
  // available, otherwise cruise_speed, otherwise current speed (last resort).
  const double ref_speed = control_profile_.ref_speed_mps > 0.0
                               ? control_profile_.ref_speed_mps
                           : control_profile_.cruise_speed_mps > 0.0
                               ? control_profile_.cruise_speed_mps
                               : current_speed_mps;

  // Altitude error (potential energy proxy)
  const auto& propagate = adapter_.GetPropagate();
  const double current_alt_m = propagate.GetLocation().GetGeodAltitude() * kFtToM;
  double alt_err_m = altitude_hold_ ? (target_altitude_m_ - current_alt_m) : 0.0;

  // Speed error (kinetic energy proxy), clamped to envelope.
  double target_spd = target_speed_mps_;
  if (control_profile_.min_speed_mps > 0.0 && target_spd < control_profile_.min_speed_mps) {
    target_spd = control_profile_.min_speed_mps;
  }
  double speed_err_mps = speed_hold_ ? (target_spd - current_speed_mps) : 0.0;

  // Combined energy error: throttle manages total energy.
  double energy_err = alt_err_m / 500.0;
  if (ref_speed > 1.0) {
    energy_err += speed_err_mps / ref_speed * 0.3;
  }

  // Speed protection: if below min_speed, override energy demand to recover speed.
  const double min_speed = control_profile_.min_speed_mps;
  if (min_speed > 0.0 && current_speed_mps < min_speed * 1.1) {
    const double urgency = Clamp((min_speed * 1.1 - current_speed_mps) / (min_speed * 0.3), 0.0, 1.0);
    if (control_profile_.speed_energy_priority) {
      energy_err += urgency * 0.5;
    } else {
      energy_err += urgency * 0.25;
    }
  }

  // Speed protection: if above max_speed, reduce throttle aggressively.
  const double max_speed = control_profile_.max_speed_mps;
  if (max_speed > 0.0 && current_speed_mps > max_speed * 0.95) {
    const double excess = Clamp((current_speed_mps - max_speed * 0.95) / (max_speed * 0.10), 0.0, 1.0);
    energy_err -= excess * 0.5;
  }

  double throttle = Clamp(0.70 + Clamp(energy_err, -0.40, 0.40),
                          control_profile_.min_throttle,
                          control_profile_.max_throttle);
  SetThrottleCmdNorm(throttle);
}

void Autopilot::ApplyNativeHeadingSetpoint() {
  double heading_rad = target_heading_rad_;
  if (heading_src_wp_) {
    const auto& propagate = adapter_.GetPropagate();
    double target_lat = adapter_.GetProperty(adapter::property::kGuidanceTargetWpLat);
    double target_lon = adapter_.GetProperty(adapter::property::kGuidanceTargetWpLon);
    heading_rad = propagate.GetLocation().GetHeadingTo(target_lon, target_lat);
  }
  adapter_.SetProperty(adapter::property::kApHeadingSetpoint, RadToDeg360(heading_rad));
}

void Autopilot::ApplyYawDamping(double yaw_rate_rad_sec) {
  const std::string yaw_property = control_profile_.yaw_input_property.empty()
                                       ? std::string(adapter::property::kRudderCmd)
                                       : control_profile_.yaw_input_property;

  const auto& propagate = adapter_.GetPropagate();
  const double roll_rad = propagate.GetEuler(1);

  // Turn coordination: for a banked turn, the aircraft needs rudder to
  // produce the yaw rate for a coordinated turn.  Without this, the yaw
  // damper fights the sustained yaw rate and the turn radius balloons.
  double rudder = 0.0;
  if (std::abs(roll_rad) > 0.05) {
    // Coordinated turn requires r ∝ sin(φ).  Apply rudder proportional
    // to bank angle to assist the turn, then damp residual oscillations.
    double vt_fps = propagate.GetInertialVelocityMagnitude();
    if (vt_fps < 10.0) vt_fps = 10.0;
    double vt_mps = vt_fps * 0.3048;
    double coord_yaw_rate = 9.80665 * std::sin(roll_rad) / vt_mps;
    double yaw_err = yaw_rate_rad_sec - coord_yaw_rate;
    rudder = Clamp(-0.3 * yaw_err, -1.0, 1.0);
  } else {
    // Wings level: pure yaw damping to suppress Dutch roll.
    rudder = Clamp(-0.15 * yaw_rate_rad_sec, -1.0, 1.0);
  }
  adapter_.SetProperty(yaw_property, rudder);
}

}  // namespace autopilot
}  // namespace flight_dynamic
}  // namespace oneq
