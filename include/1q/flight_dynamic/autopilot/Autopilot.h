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

  void Update(double dt_sec);

 private:
  adapter::JsbsimAdapter& adapter_;
  bool use_cpp_ap_ = false;

  bool heading_hold_ = false;
  double target_heading_rad_ = 0.0;
  bool heading_src_wp_ = false;

  bool altitude_hold_ = false;
  double target_altitude_m_ = 0.0;

  bool pitch_hold_ = false;
  double target_pitch_deg_ = 0.0;

  int roll_mode_ = 0;
  bool roll_ap_on_ = false;
};

}  // namespace autopilot
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_AUTOPILOT_AUTOPILOT_H_
