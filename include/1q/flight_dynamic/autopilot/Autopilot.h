#ifndef ONEQ_FLIGHT_DYNAMIC_AUTOPILOT_AUTOPILOT_H_
#define ONEQ_FLIGHT_DYNAMIC_AUTOPILOT_AUTOPILOT_H_

namespace oneq {
namespace flight_dynamic {
namespace adapter {
class JsbsimAdapter;
}}}  // namespace oneq::flight_dynamic::adapter

namespace oneq {
namespace flight_dynamic {
namespace autopilot {

class Autopilot {
 public:
  explicit Autopilot(adapter::JsbsimAdapter& adapter);

  // --- Heading ---
  void SetHeadingTargetRad(double heading_rad);
  void SetHeadingHold(bool on);
  void SetHeadingSourceIsWaypoint(bool from_waypoint);

  // --- Altitude ---
  void SetAltitudeTargetM(double altitude_m);
  void SetAltitudeHold(bool on);

  // --- Pitch ---
  void SetPitchTargetDeg(double pitch_deg);
  void SetPitchHold(bool on);

  // --- Roll ---
  void SetRollAttitudeMode(int mode);  // 0=wings level, 1=angle hold
  void SetRollAutopilotOn(bool on);

  // --- Throttle ---
  void SetThrottleCmdNorm(double value);
  void SetThrottleCmd(int engine, double value);

  // --- Yaw damper ---
  void SetYawDamper(bool on);

  // --- Status queries ---
  double GetAngleToHeadingRad() const;
  double GetAltitudeAGLM() const;

 private:
  adapter::JsbsimAdapter& adapter_;
};

}  // namespace autopilot
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_AUTOPILOT_AUTOPILOT_H_
