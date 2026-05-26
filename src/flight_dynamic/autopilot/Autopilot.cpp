#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"

namespace oneq {
namespace flight_dynamic {
namespace autopilot {

Autopilot::Autopilot(adapter::JsbsimAdapter& adapter) : adapter_(adapter) {}

void Autopilot::SetHeadingTargetRad(double heading_rad) {
  adapter_.SetProperty("guidance/specified-heading-rad", heading_rad);
}

void Autopilot::SetHeadingHold(bool on) {
  adapter_.SetProperty("ap/heading_hold", on ? 1.0 : 0.0);
}

void Autopilot::SetHeadingSourceIsWaypoint(bool from_waypoint) {
  // 0=waypoint heading (from GNCUtilities), 1=user specified
  adapter_.SetProperty("guidance/heading-selector-switch",
                       from_waypoint ? 0.0 : 1.0);
}

void Autopilot::SetAltitudeTargetM(double altitude_m) {
  double altitude_ft = altitude_m / 0.3048;
  adapter_.SetProperty("ap/altitude_setpoint", altitude_ft);
}

void Autopilot::SetAltitudeHold(bool on) {
  adapter_.SetProperty("ap/altitude_hold", on ? 1.0 : 0.0);
}

void Autopilot::SetPitchTargetDeg(double pitch_deg) {
  adapter_.SetProperty("ap/pitch-target-deg", pitch_deg);
}

void Autopilot::SetPitchHold(bool on) {
  adapter_.SetProperty("ap/pitch-hold", on ? 1.0 : 0.0);
}

void Autopilot::SetRollAttitudeMode(int mode) {
  // 0=wings level, 1=roll angle hold (used with heading hold)
  adapter_.SetProperty("ap/roll-attitude-mode", static_cast<double>(mode));
}

void Autopilot::SetRollAutopilotOn(bool on) {
  adapter_.SetProperty("ap/autopilot-roll-on", on ? 1.0 : 0.0);
}

void Autopilot::SetThrottleCmdNorm(double value) {
  adapter_.SetProperty("fcs/throttle-cmd-norm", value);
}

void Autopilot::SetThrottleCmd(int engine, double value) {
  std::string prop = "fcs/throttle-cmd-norm[" + std::to_string(engine) + "]";
  adapter_.SetProperty(prop, value);
}

void Autopilot::SetYawDamper(bool on) {
  adapter_.SetProperty("ap/yaw_damper", on ? 1.0 : 0.0);
}

double Autopilot::GetAngleToHeadingRad() const {
  return adapter_.GetProperty("guidance/angle-to-heading-rad");
}

double Autopilot::GetAltitudeAGLM() const {
  return adapter_.GetProperty("position/h-agl-ft") * 0.3048;
}

}  // namespace autopilot
}  // namespace flight_dynamic
}  // namespace oneq
