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

// Tier 1: explicit profile override by aircraft model name.
// Takes priority over XML property probing (Tier 2) and conservative fallback
// (Tier 3). Based on AircraftProfiles/ProfileSnapshotTest snapshots.
const AircraftControlProfile* LookupExplicitProfile(const std::string& model_name) {
  using LI = LateralControlInterface;
  using PI = PitchControlInterface;
  using FBW = FbwSubtype;

  // Static table: one entry per supported aircraft.
  // Keep sorted by model_name for readability.
  static const struct Entry {
    const char* name;
    AircraftControlProfile profile;
  } kKnownProfiles[] = {
    {"B17",       {LI::kGenericAutopilotBridge, PI::kNativeAutopilot, FBW::kNone, false, true,  false, false, true,  false, 4, true, "fcs/rudder-cmd-norm"}},
    {"C130",      {LI::kGenericAutopilotBridge, PI::kNativeAutopilot, FBW::kNone, false, true,  false, false, true,  false, 4, true, "fcs/rudder-cmd-norm"}},
    {"Concorde",  {LI::kGenericAutopilotBridge, PI::kNativeAutopilot, FBW::kNone, false, true,  false, false, true,  false, 4, true, "fcs/rudder-cmd-norm"}},
    {"c172x",     {LI::kOwnAutopilot,           PI::kNativeAutopilot, FBW::kNone, true,  true,  false, false, true,  false, 1, true, "fcs/rudder-cmd-norm"}},
    {"c310",      {LI::kOwnAutopilot,           PI::kNativeAutopilot, FBW::kNone, true,  false, false, false, true,  false, 2, true, "fcs/rudder-cmd-norm"}},
    {"f15",       {LI::kGenericAutopilotBridge, PI::kNativeAutopilot, FBW::kNone, false, true,  false, false, true,  false, 2, true, "fcs/rudder-cmd-norm"}},
    {"f16",       {LI::kFbwRateCommand,         PI::kNativeAutopilot, FBW::kRollRatePid,           false, true,  true,  true,  true,  false, 1, true, "fcs/rudder-cmd-norm"}},
    {"f22",       {LI::kFbwRateCommand,         PI::kNativeAutopilot, FBW::kRateIntegratorActuator, false, true,  false, true,  true,  true,  2, true, "fcs/rudder-cmd-norm"}},
  };

  for (const auto& entry : kKnownProfiles) {
    if (model_name == entry.name) {
      return &entry.profile;
    }
  }
  return nullptr;
}

void ApplyEnergyManagementProfile(const std::string& model_name, AircraftControlProfile* profile) {
  if (!profile) return;

  if (model_name == "Concorde") {
    profile->ref_speed_mps = 500.0;
    profile->min_speed_mps = 250.0;
    profile->max_pitch_command_deg = 8.0;
    profile->max_roll_angle_deg = 35.0;
    profile->min_throttle = 0.55;
    profile->max_throttle = 1.0;
    profile->speed_energy_priority = true;
  } else if (model_name == "f16" || model_name == "f15" || model_name == "f22") {
    profile->ref_speed_mps = 200.0;
    profile->min_speed_mps = 140.0;
    profile->max_pitch_command_deg = 15.0;
    profile->max_roll_angle_deg = 45.0;
    profile->min_throttle = 0.35;
    profile->max_throttle = 1.0;
    profile->speed_energy_priority = true;
  } else if (model_name == "B17") {
    profile->ref_speed_mps = 80.0;
    profile->min_speed_mps = 65.0;
    profile->max_pitch_command_deg = 8.0;
    profile->max_roll_angle_deg = 25.0;
    profile->min_throttle = 0.45;
    profile->max_throttle = 1.0;
    profile->speed_energy_priority = true;
  } else if (model_name == "C130") {
    profile->ref_speed_mps = 90.0;
    profile->min_speed_mps = 70.0;
    profile->max_pitch_command_deg = 10.0;
    profile->max_roll_angle_deg = 30.0;
    profile->min_throttle = 0.35;
    profile->max_throttle = 1.0;
    profile->speed_energy_priority = true;
  } else if (model_name == "c172x") {
    profile->ref_speed_mps = 50.0;
    profile->min_speed_mps = 40.0;
    profile->max_pitch_command_deg = 12.0;
    profile->max_roll_angle_deg = 30.0;
    profile->min_throttle = 0.20;
    profile->max_throttle = 1.0;
  } else if (model_name == "c310") {
    profile->ref_speed_mps = 65.0;
    profile->min_speed_mps = 55.0;
    profile->max_pitch_command_deg = 10.0;
    profile->max_roll_angle_deg = 30.0;
    profile->min_throttle = 0.30;
    profile->max_throttle = 1.0;
    profile->speed_energy_priority = true;
  }
}

}  // namespace

Autopilot::Autopilot(adapter::JsbsimAdapter& adapter) : adapter_(adapter) {
  auto* pm = adapter.GetFdmExec().GetPropertyManager().get();
  const std::string& model_name = adapter.GetFdmExec().GetModelName();

  // Read the roll angle limit from JSBSim property tree (set by ConfigureIntegrators
  // per aircraft in the adapter). Apply a sustained-turn factor: sustained ≈ structural × 0.7,
  // because sustained turns generate induced drag that limits endurance at structural max.
  if (pm->GetNode(adapter::property::kGuidanceRollAngleLimit) != nullptr) {
    double roll_lim = adapter.GetProperty(adapter::property::kGuidanceRollAngleLimit);
    double sustained_factor = 0.7;
    control_profile_.max_roll_angle_deg = roll_lim * 180.0 / M_PI * sustained_factor;
  }

  // Tier 1: explicit profile override (replaces XML probing entirely).
  if (const auto* explicit_profile = LookupExplicitProfile(model_name)) {
    control_profile_ = *explicit_profile;
    ApplyEnergyManagementProfile(model_name, &control_profile_);
    use_cpp_ap_ =
        control_profile_.lateral_interface != LateralControlInterface::kOwnAutopilot &&
        control_profile_.lateral_interface != LateralControlInterface::kGenericAutopilotBridge;
    return;
  }

  // Tier 2: XML property probing.
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
    // indexed_throttle is determined from the explicit profile table for known
    // aircraft. For unknown aircraft, fall back to false (conservative).
    if (const auto* ep = LookupExplicitProfile(model_name)) {
      control_profile_.indexed_throttle = ep->indexed_throttle;
    }
  }

  // Mixture detection (piston aircraft)
  control_profile_.has_mixture = pm->GetNode("fcs/mixture-cmd-norm") != nullptr;

  // FBW subtype detection: f16 has roll-rate PID, f22 has rate integrator+actuator
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

  // Lateral interface selection
  if (control_profile_.has_own_autopilot) {
    control_profile_.lateral_interface = LateralControlInterface::kOwnAutopilot;
  } else if (control_profile_.has_fbw_override || control_profile_.has_roll_rate_command) {
    control_profile_.lateral_interface = LateralControlInterface::kFbwRateCommand;
  } else if (control_profile_.has_generic_autopilot) {
    control_profile_.lateral_interface = LateralControlInterface::kGenericAutopilotBridge;
  } else {
    control_profile_.lateral_interface = LateralControlInterface::kDirectSurface;
  }

  use_cpp_ap_ =
      control_profile_.lateral_interface != LateralControlInterface::kOwnAutopilot &&
      control_profile_.lateral_interface != LateralControlInterface::kGenericAutopilotBridge;
  ApplyEnergyManagementProfile(model_name, &control_profile_);
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
}

double Autopilot::GetTrueSpeedMps() const {
  return adapter_.GetPropagate().GetInertialVelocityMagnitude() * kFtToM;
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
    adapter_.SetProperty(adapter::property::kElevatorCmd, elevator);
  }
}

void Autopilot::UpdateEnergyManagement() {
  if (!altitude_hold_ && !speed_hold_) return;

  const auto& propagate = adapter_.GetPropagate();
  const double current_alt_m = propagate.GetLocation().GetGeodAltitude() * kFtToM;
  const double current_speed_mps = GetTrueSpeedMps();
  const double ref_speed = control_profile_.ref_speed_mps > 0.0
                               ? control_profile_.ref_speed_mps
                               : current_speed_mps;

  // Altitude error (potential energy proxy)
  double alt_err_m = altitude_hold_ ? (target_altitude_m_ - current_alt_m) : 0.0;

  // Speed error (kinetic energy proxy)
  double speed_err_mps = speed_hold_ ? (target_speed_mps_ - current_speed_mps) : 0.0;

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
