#include "1q/flight_dynamic/autopilot/Autopilot.h"

#include <cmath>

#include "flight_dynamic/adapter/JsbsimAdapter.h"
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

}  // namespace

Autopilot::Autopilot(adapter::JsbsimAdapter& adapter) : adapter_(adapter) {
  auto* pm = adapter.GetFdmExec().GetPropertyManager().get();
  control_profile_.has_own_autopilot = pm->GetNode("ap/heading_hold") != nullptr;
  control_profile_.has_generic_autopilot = pm->GetNode("ap/autopilot-roll-on") != nullptr;
  control_profile_.has_fbw_override = pm->GetNode("fcs/fbw-override") != nullptr;
  control_profile_.has_roll_rate_command = pm->GetNode("fcs/roll-rate-command") != nullptr ||
                                           pm->GetNode("fcs/roll-rate-cmd") != nullptr;
  control_profile_.has_aileron_command = pm->GetNode("fcs/aileron-cmd-norm") != nullptr;
  const auto propulsion = adapter_.GetFdmExec().GetPropulsion();
  if (propulsion) {
    control_profile_.engine_count = static_cast<int>(propulsion->GetNumEngines());
  }

  if (control_profile_.has_own_autopilot) {
    control_profile_.lateral_interface = LateralControlInterface::kOwnAutopilot;
  } else if (control_profile_.has_fbw_override || control_profile_.has_roll_rate_command) {
    control_profile_.lateral_interface = LateralControlInterface::kFbwRateCommand;
  } else if (control_profile_.has_generic_autopilot) {
    control_profile_.lateral_interface = LateralControlInterface::kGenericAutopilotBridge;
  } else {
    control_profile_.lateral_interface = LateralControlInterface::kDirectSurface;
  }

  use_own_ap_ = control_profile_.has_own_autopilot;
  use_cpp_ap_ =
      control_profile_.lateral_interface != LateralControlInterface::kOwnAutopilot &&
      control_profile_.lateral_interface != LateralControlInterface::kGenericAutopilotBridge;
}

void Autopilot::SetHeadingTargetRad(double heading_rad) {
  target_heading_rad_ = heading_rad;
  if (!use_cpp_ap_) {
    adapter_.SetProperty("guidance/specified-heading-rad", heading_rad);
    adapter_.SetProperty("ap/heading_setpoint", RadToDeg360(heading_rad));
  }
}

void Autopilot::SetHeadingHold(bool on) {
  heading_hold_ = on;
  if (!use_cpp_ap_) {
    adapter_.SetProperty("ap/heading_hold", on ? 1.0 : 0.0);
  }
}

void Autopilot::SetHeadingSourceIsWaypoint(bool from_waypoint) {
  heading_src_wp_ = from_waypoint;
  if (!use_cpp_ap_) {
    adapter_.SetProperty("guidance/heading-selector-switch", from_waypoint ? 0.0 : 1.0);
  }
}

void Autopilot::SetAltitudeTargetM(double altitude_m) {
  target_altitude_m_ = altitude_m;
  if (!use_cpp_ap_) {
    adapter_.SetProperty("ap/altitude_setpoint", altitude_m * kMToFt);
  }
}

void Autopilot::SetAltitudeHold(bool on) { altitude_hold_ = on; }

void Autopilot::SetPitchTargetDeg(double pitch_deg) {
  target_pitch_deg_ = pitch_deg;
  if (!use_cpp_ap_) {
    adapter_.SetProperty("ap/pitch-target-deg", pitch_deg);
  }
}

void Autopilot::SetPitchHold(bool on) {
  pitch_hold_ = on;
  if (!use_cpp_ap_) {
    adapter_.SetProperty("ap/pitch-hold", on ? 1.0 : 0.0);
  }
}

void Autopilot::SetLateralGuidanceMode(LateralGuidanceMode mode) { lateral_guidance_mode_ = mode; }

void Autopilot::SetRollAttitudeMode(int mode) {
  roll_mode_ = mode;
  if (!use_cpp_ap_) {
    adapter_.SetProperty("ap/roll-attitude-mode", static_cast<double>(mode));
  }
}

void Autopilot::SetRollAutopilotOn(bool on) {
  roll_ap_on_ = on;
  if (!use_cpp_ap_) {
    adapter_.SetProperty("ap/autopilot-roll-on", on ? 1.0 : 0.0);
  }
}

void Autopilot::SetThrottleCmdNorm(double value) {
  adapter_.SetProperty("fcs/throttle-cmd-norm", value);
  if (control_profile_.lateral_interface != LateralControlInterface::kOwnAutopilot) {
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
  std::string prop = "fcs/throttle-cmd-norm[" + std::to_string(engine) + "]";
  adapter_.SetProperty(prop, value);
}

void Autopilot::SetYawDamper(bool on) {
  if (!use_cpp_ap_) {
    adapter_.SetProperty("ap/yaw_damper", on ? 1.0 : 0.0);
  }
}

double Autopilot::GetAngleToHeadingRad() const {
  if (use_cpp_ap_ ||
      control_profile_.lateral_interface == LateralControlInterface::kFbwRateCommand) {
    const auto& propagate = adapter_.GetPropagate();
    double current_heading = propagate.GetEuler(3);
    double target_heading = target_heading_rad_;
    if (heading_src_wp_) {
      double target_lat = adapter_.GetProperty("guidance/target_wp_latitude_rad");
      double target_lon = adapter_.GetProperty("guidance/target_wp_longitude_rad");
      target_heading = propagate.GetLocation().GetHeadingTo(target_lon, target_lat);
    }
    return NormalizeRad(target_heading - current_heading);
  } else {
    return adapter_.GetProperty("guidance/angle-to-heading-rad");
  }
}

double Autopilot::GetAltitudeAGLM() const {
  return adapter_.GetProperty("position/h-agl-ft") * kFtToM;
}

double Autopilot::GetAltitudeASLM() const {
  return adapter_.GetPropagate().GetLocation().GetGeodAltitude() * kFtToM;
}

void Autopilot::Update(double /*dt_sec*/) {
  const auto& propagate = adapter_.GetPropagate();

  if (use_own_ap_) {
    UpdateOwnAutopilot();
    UpdateAltitudeThrottle();

    double r = propagate.GetPQR(3);
    double rudder = -0.15 * r;
    rudder = Clamp(rudder, -1.0, 1.0);
    adapter_.SetProperty("fcs/rudder-cmd-norm", rudder);
    return;
  }

  if (use_cpp_ap_ &&
      control_profile_.lateral_interface == LateralControlInterface::kFbwRateCommand &&
      lateral_guidance_mode_ == LateralGuidanceMode::kOrbit) {
    UpdateFbwRateCommandLateral();
    UpdatePitchChannel();
    UpdateAltitudeThrottle();

    double r = propagate.GetPQR(3);
    double rudder = -0.15 * r;
    rudder = Clamp(rudder, -1.0, 1.0);
    adapter_.SetProperty("fcs/rudder-cmd-norm", rudder);
    return;
  }

  if (!use_cpp_ap_) {
    if (control_profile_.lateral_interface == LateralControlInterface::kFbwRateCommand) {
      if (lateral_guidance_mode_ == LateralGuidanceMode::kOrbit) {
        UpdateFbwRateCommandLateral();
      } else {
        UpdateDirectHeadingLateral();
      }
      UpdatePitchChannel();
      UpdateAltitudeThrottle();

      double r = propagate.GetPQR(3);
      double rudder = -0.15 * r;
      rudder = Clamp(rudder, -1.0, 1.0);
      adapter_.SetProperty("fcs/rudder-cmd-norm", rudder);
      return;
    }

    UpdateGenericApBridge();
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
    adapter_.SetProperty("fcs/aileron-cmd-norm", aileron);

    UpdatePitchChannel();
    UpdateAltitudeThrottle();

    double r = propagate.GetPQR(3);
    double rudder = -0.15 * r;
    rudder = Clamp(rudder, -1.0, 1.0);
    adapter_.SetProperty("fcs/rudder-cmd-norm", rudder);
    return;
  }

  double r = propagate.GetPQR(3);
  UpdateDirectHeadingLateral();

  UpdatePitchChannel();
  UpdateAltitudeThrottle();

  // --- Yaw Damper ---
  double rudder = -0.15 * r;
  rudder = Clamp(rudder, -1.0, 1.0);
  adapter_.SetProperty("fcs/rudder-cmd-norm", rudder);
}

void Autopilot::UpdateOwnAutopilot() {
  if (heading_hold_) {
    ApplyNativeHeadingSetpoint();
  }
  adapter_.SetProperty("ap/heading_hold", heading_hold_ ? 1.0 : 0.0);
  adapter_.SetProperty("ap/attitude_hold", heading_hold_ ? 1.0 : 0.0);
  adapter_.SetProperty("ap/altitude_hold", altitude_hold_ ? 1.0 : 0.0);
}

void Autopilot::UpdateGenericApBridge() {
  if (heading_hold_) {
    ApplyNativeHeadingSetpoint();
  }
}

void Autopilot::UpdateFbwRateCommandLateral() {
  double roll_rate_cmd = 0.0;
  if (heading_hold_) {
    const double heading_err = GetAngleToHeadingRad();
    roll_rate_cmd = Clamp(0.45 * heading_err, -0.60, 0.60);
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
    adapter_.SetProperty("fcs/aileron-cmd-norm", aileron);
  }
}

void Autopilot::UpdateWingLeveler() {
  const auto& propagate = adapter_.GetPropagate();
  double roll = propagate.GetEuler(1);
  double p = propagate.GetPQR(1);
  double aileron = -1.0 * roll - 0.1 * p;
  aileron = Clamp(aileron, -1.0, 1.0);
  adapter_.SetProperty("fcs/aileron-cmd-norm", aileron);
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
    target_pitch = 0.0005 * alt_err_ft;
    target_pitch = Clamp(target_pitch, -0.35, 0.35);
  } else if (pitch_hold_) {
    pitch_control_active = true;
    target_pitch = target_pitch_deg_ * M_PI / 180.0;
  }

  if (pitch_control_active) {
    double pitch_err = target_pitch - pitch;
    double elevator = -(2.0 * pitch_err - 0.2 * q);
    elevator = Clamp(elevator, -1.0, 1.0);
    adapter_.SetProperty("fcs/elevator-cmd-norm", elevator);
  }
}

void Autopilot::UpdateAltitudeThrottle() {
  if (!altitude_hold_) return;

  const auto& propagate = adapter_.GetPropagate();
  double alt_err_m = target_altitude_m_ - propagate.GetLocation().GetGeodAltitude() * kFtToM;
  double climb_bias = Clamp(alt_err_m / 500.0, -0.30, 0.30);
  double throttle = Clamp(0.70 + climb_bias, 0.35, 1.0);
  SetThrottleCmdNorm(throttle);
}

void Autopilot::ApplyNativeHeadingSetpoint() {
  double heading_rad = target_heading_rad_;
  if (heading_src_wp_) {
    const auto& propagate = adapter_.GetPropagate();
    double target_lat = adapter_.GetProperty("guidance/target_wp_latitude_rad");
    double target_lon = adapter_.GetProperty("guidance/target_wp_longitude_rad");
    heading_rad = propagate.GetLocation().GetHeadingTo(target_lon, target_lat);
  }
  adapter_.SetProperty("ap/heading_setpoint", RadToDeg360(heading_rad));
}

}  // namespace autopilot
}  // namespace flight_dynamic
}  // namespace oneq
