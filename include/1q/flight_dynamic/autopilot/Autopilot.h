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

  // --- Energy management profile ---
  // Derived from aircraft physics (V_stall, wing loading), not hardcoded
  // categories.  V_stall = sqrt(2W / ρ·S·CLmax) encodes actual weight,
  // wing area, and lift capability into a single base speed.
  double v_stall_mps = 0.0;           // clean stall speed (TAS, m/s) at sea level
  double wing_loading_lbs_ft2 = 0.0;  // weight / wing_area (lbs/ft²)
  double thrust_to_weight = 0.0;      // total static thrust / weight (dimensionless)
  double ref_speed_mps = 0.0;         // cruise / reference speed for energy distribution
  double min_speed_mps = 0.0;         // minimum task speed (stall margin)
  double max_speed_mps = 0.0;         // maximum task speed (structural / thrust limit)
  double cruise_speed_mps = 0.0;      // default cruise TAS by aircraft category
  double ceiling_m = 0.0;             // practical service ceiling (0 = derive from physics)
  double max_pitch_command_deg = 20.0;  // pitch command limit in altitude hold
  double max_roll_angle_deg = 45.0;     // structural/aero roll limit (used for orbit radius)
  double max_throttle = 1.0;
  double min_throttle = 0.15;
  bool speed_energy_priority = false;  // true = prioritize speed over altitude

  // --- Rotation / takeoff parameters (scaled by pitch MOI) ---
  double pitch_moi_lbsft2 = 0.0;        // pitch moment of inertia from property tree
  double rotation_ramp_sec = 3.0;        // elevator ramp-up duration during rotation
  double rotation_max_elevator = 0.30;   // peak elevator deflection during rotation
  double rotation_climb_rate_mps = 5.0;  // target climb rate after rotation

  // --- Landing / approach parameters ---
  double landing_approach_speed_mps = 0.0;        // 0 = derive from command/Vr
  double landing_high_descent_agl_m = 3000.0;     // above this, descend to staging altitude
  double landing_staging_agl_m = 3000.0;          // altitude used for high-speed decel
  double landing_pattern_agl_m = 200.0;           // final-descent handoff altitude
  bool landing_high_descent_orbit = true;
  double landing_descent_throttle = -1.0;         // <0 = engine-type default
  double landing_approach_flaps_norm = 0.5;
  double landing_final_flaps_norm = 1.0;
  double landing_final_throttle_cap = 0.60;  // default = no-op (matches sink-rate max); B747 XML overrides to 0.05
  double landing_flare_initial_elevator = 0.0;    // 0 = derive from aircraft class
  bool landing_heavy_flare = false;               // true = use transport bounce/float flare law
  double landing_touchdown_agl_m = 3.0;
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

  // --- Speed / energy management ---
  void SetSpeedTargetMps(double speed_mps);
  void SetSpeedHold(bool on);
  double GetTrueSpeedMps() const;

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

  // --- State management ---
  void ReleaseHolds();

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
  void UpdateRollAnglePD();
  void UpdateDirectHeadingLateral();
  void UpdatePitchChannel();
  void ApplyNativeHeadingSetpoint();
  void ApplyYawDamping(double yaw_rate_rad_sec);
  void UpdateEnergyManagement();

  adapter::JsbsimAdapter& adapter_;
  AircraftControlProfile control_profile_;
  bool use_cpp_ap_ = false;
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

  bool speed_hold_ = false;
  double target_speed_mps_ = 0.0;
};

}  // namespace autopilot
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_AUTOPILOT_AUTOPILOT_H_
