#include "1q/flight_dynamic/autopilot/Autopilot.h"

#include <cmath>

#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "math/FGLocation.h"
#include "models/FGPropagate.h"

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

}  // namespace

Autopilot::Autopilot(adapter::JsbsimAdapter& adapter) : adapter_(adapter) {
  use_cpp_ap_ = (adapter.GetFdmExec().GetPropertyManager()->GetNode("ap/heading_hold") == nullptr);
}

void Autopilot::SetHeadingTargetRad(double heading_rad) {
  target_heading_rad_ = heading_rad;
  if (use_cpp_ap_) {
    return;
  } else {
    adapter_.SetProperty("guidance/specified-heading-rad", heading_rad);
    adapter_.SetProperty("ap/heading_setpoint", RadToDeg360(heading_rad));
  }
}

void Autopilot::SetHeadingHold(bool on) {
  heading_hold_ = on;
  if (use_cpp_ap_) {
  } else {
    adapter_.SetProperty("ap/heading_hold", on ? 1.0 : 0.0);
  }
}

void Autopilot::SetHeadingSourceIsWaypoint(bool from_waypoint) {
  heading_src_wp_ = from_waypoint;
  if (use_cpp_ap_) {
  } else {
    // 0=waypoint heading (from GNCUtilities), 1=user specified
    adapter_.SetProperty("guidance/heading-selector-switch", from_waypoint ? 0.0 : 1.0);
  }
}

void Autopilot::SetAltitudeTargetM(double altitude_m) {
  target_altitude_m_ = altitude_m;
  if (use_cpp_ap_) {
  } else {
    double altitude_ft = altitude_m / 0.3048;
    adapter_.SetProperty("ap/altitude_setpoint", altitude_ft);
  }
}

void Autopilot::SetAltitudeHold(bool on) {
  altitude_hold_ = on;
  if (use_cpp_ap_) {
  } else {
    adapter_.SetProperty("ap/altitude_hold", 0.0);
  }
}

void Autopilot::SetPitchTargetDeg(double pitch_deg) {
  target_pitch_deg_ = pitch_deg;
  if (use_cpp_ap_) {
  } else {
    adapter_.SetProperty("ap/pitch-target-deg", pitch_deg);
  }
}

void Autopilot::SetPitchHold(bool on) {
  pitch_hold_ = on;
  if (use_cpp_ap_) {
  } else {
    adapter_.SetProperty("ap/pitch-hold", on ? 1.0 : 0.0);
  }
}

void Autopilot::SetRollAttitudeMode(int mode) {
  roll_mode_ = mode;
  if (use_cpp_ap_) {
  } else {
    // 0=wings level, 1=roll angle hold (used with heading hold)
    adapter_.SetProperty("ap/roll-attitude-mode", static_cast<double>(mode));
  }
}

void Autopilot::SetRollAutopilotOn(bool on) {
  roll_ap_on_ = on;
  if (use_cpp_ap_) {
  } else {
    adapter_.SetProperty("ap/autopilot-roll-on", on ? 1.0 : 0.0);
  }
}

void Autopilot::SetThrottleCmdNorm(double value) {
  adapter_.SetProperty("fcs/throttle-cmd-norm", value);
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
  if (use_cpp_ap_) {
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
  return adapter_.GetProperty("position/h-agl-ft") * 0.3048;
}

double Autopilot::GetAltitudeASLM() const {
  return adapter_.GetPropagate().GetLocation().GetGeodAltitude() * 0.3048;
}

void Autopilot::Update(double /*dt_sec*/) {
  if (!use_cpp_ap_) {
    UpdateNativeAutopilotBridge();
    if (!heading_hold_) {
      UpdateWingLeveler();
    }
    UpdatePitchChannel();
    UpdateAltitudeThrottle();
    return;
  }

  const auto& propagate = adapter_.GetPropagate();
  double roll = propagate.GetEuler(1);
  double p = propagate.GetPQR(1);
  double r = propagate.GetPQR(3);

  // --- Roll Control Channel ---
  double target_roll = 0.0;

  if (heading_hold_) {
    double heading_err = GetAngleToHeadingRad();
    target_roll = 0.5 * heading_err;
    target_roll = Clamp(target_roll, -0.35, 0.35);
  }

  double roll_err = target_roll - roll;
  double aileron = 0.15 * roll_err - 1.0 * p;
  aileron = Clamp(aileron, -0.25, 0.25);

  if (roll_ap_on_ || heading_hold_ || roll_mode_ == 0) {
    adapter_.SetProperty("fcs/aileron-cmd-norm", aileron);
  }

  UpdatePitchChannel();
  UpdateAltitudeThrottle();

  // --- Yaw Control Channel (Yaw Damper) ---
  double rudder = -0.1 * r;
  rudder = Clamp(rudder, -1.0, 1.0);
  adapter_.SetProperty("fcs/rudder-cmd-norm", rudder);
}

void Autopilot::UpdateNativeAutopilotBridge() {
  if (heading_hold_) {
    ApplyNativeHeadingSetpoint();
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
    double target_alt_ft = target_altitude_m_ / 0.3048;
    double current_alt_ft = propagate.GetLocation().GetGeodAltitude();
    double alt_err_ft = target_alt_ft - current_alt_ft;
    target_pitch = 0.0005 * alt_err_ft;
    target_pitch = Clamp(target_pitch, -0.26, 0.26);
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
  double alt_err_m = target_altitude_m_ - propagate.GetLocation().GetGeodAltitude() * 0.3048;
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
