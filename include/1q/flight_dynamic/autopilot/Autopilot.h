#ifndef ONEQ_FLIGHT_DYNAMIC_AUTOPILOT_AUTOPILOT_H_
#define ONEQ_FLIGHT_DYNAMIC_AUTOPILOT_AUTOPILOT_H_

#include <string>

namespace oneq {
namespace flight_dynamic {
namespace adapter {
class JsbsimAdapter;
}
}  // namespace flight_dynamic
}  // namespace oneq

namespace oneq {
namespace flight_dynamic {
namespace autopilot {

enum class LateralControlInterface {
  kDirectSurface,
  kGenericAutopilotBridge,
  kOwnAutopilot,
  kFbwRateCommand,
};

enum class FbwSubtype {
  kNone,
  kRollRatePid,
  kRateIntegratorActuator,
};

enum class PitchControlInterface {
  kDirectSurface,
  kFbwScheduled,
  kNativeAutopilot,
};

enum class LateralGuidanceMode {
  kHeading,
  kOrbit,
};

struct AircraftControlProfile {
  LateralControlInterface lateral_interface = LateralControlInterface::kDirectSurface;
  PitchControlInterface pitch_interface = PitchControlInterface::kDirectSurface;
  FbwSubtype fbw_subtype = FbwSubtype::kNone;

  bool has_own_autopilot = false;
  bool has_generic_autopilot = false;
  bool has_fbw_override = false;
  bool has_roll_rate_command = false;
  bool has_aileron_command = false;

  bool indexed_throttle = false;
  int engine_count = 0;
  bool has_mixture = false;

  std::string yaw_input_property;
};

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
  void SetLateralGuidanceMode(LateralGuidanceMode mode);
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
  double GetAltitudeASLM() const;
  const AircraftControlProfile& GetControlProfile() const { return control_profile_; }

  void Update(double dt_sec);

 private:
  void UpdateOwnAutopilot();
  void UpdateGenericApBridge();
  void UpdateFbwRateCommandLateral();
  void UpdateDirectHeadingLateral();
  void UpdateWingLeveler();
  void UpdatePitchChannel();
  void UpdateAltitudeThrottle();
  void ApplyNativeHeadingSetpoint();

  adapter::JsbsimAdapter& adapter_;
  AircraftControlProfile control_profile_;
  bool use_cpp_ap_ = false;
  bool use_own_ap_ = false;
  double roll_int_ = 0.0;

  bool heading_hold_ = false;
  double target_heading_rad_ = 0.0;
  bool heading_src_wp_ = false;
  LateralGuidanceMode lateral_guidance_mode_ = LateralGuidanceMode::kHeading;

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
