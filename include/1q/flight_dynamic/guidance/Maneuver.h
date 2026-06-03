#ifndef ONEQ_FLIGHT_DYNAMIC_GUIDANCE_MANEUVER_H_
#define ONEQ_FLIGHT_DYNAMIC_GUIDANCE_MANEUVER_H_

#include <vector>

#include "1q/flight_dynamic/guidance/Waypoint.h"

namespace oneq {
namespace flight_dynamic {

namespace adapter {
class JsbsimAdapter;
}
namespace autopilot {
class Autopilot;
}
namespace guidance {
class WaypointManager;
}
namespace propulsion {
class EngineManager;
}

namespace guidance {

enum class ManeuverType {
  kFlyToWaypoint,
  kOrbit,
  kSetHeading,
  kSetAltitude,
  kSetPitch,
  kSetRoll,
  kTakeoff,
  kLand,
};

struct Maneuver {
  ManeuverType type;
  Waypoint target;
  double value = 0.0;
  double duration_sec = 0.0;
};

class ManeuverExecutor {
 public:
  ManeuverExecutor(adapter::JsbsimAdapter& adapter, autopilot::Autopilot& ap,
                   WaypointManager& wp_manager, propulsion::EngineManager& engines);

  void ExecuteFlyTo(const Waypoint& target);
  void ExecuteOrbit(const Waypoint& center, double radius_m, double duration_sec = 0.0);
  void ExecuteSetHeading(double heading_rad);
  void ExecuteSetAltitude(double altitude_m);
  void ExecuteSetPitch(double pitch_deg, double duration_sec);
  void ExecuteSetRoll(int roll_mode);
  void ExecuteTakeoff(double target_altitude_m, double target_heading_rad,
                      double target_speed_mps = 0.0);
  void ExecuteLand(const Waypoint& target, double approach_speed_mps = 0.0);

  bool IsManeuverComplete() const;
  bool IsTouchingGround() const;
  void Update(double dt_sec);
  void Abort();

 private:
  enum class TakeoffPhase {
    kEngineStart,
    kStaticRunup,
    kTakeoffRoll,
    kRotateAndClimb,
    kComplete,
  };

  enum class LandPhase {
    kDecelerate,
    kApproach,
    kFinalDescent,
    kFlare,
    kTouchdown,
    kRollout,
    kComplete,
  };

  void StartEngine();
  void ConfigureForTakeoffRoll();
  void ConfigureForClimb(double target_altitude_m, double target_heading_rad,
                         double target_speed_mps);
  void ConfigureForApproach(const Waypoint& target, double approach_speed_mps);
  void ConfigureForLanding();

  adapter::JsbsimAdapter& adapter_;
  autopilot::Autopilot& ap_;
  WaypointManager& wp_manager_;
  propulsion::EngineManager& engines_;
  Maneuver current_maneuver_;
  bool active_ = false;
  double elapsed_sec_ = 0.0;
  TakeoffPhase takeoff_phase_ = TakeoffPhase::kEngineStart;
  double takeoff_target_altitude_m_ = 0.0;
  double takeoff_target_heading_rad_ = 0.0;
  double takeoff_phase_elapsed_sec_ = 0.0;
  double rotation_elapsed_sec_ = 0.0;
  double rotation_ramp_origin_ = 0.0;  // elevator level from pre-rotation phase
  double takeoff_vr_kts_ = 0.0;       // cached Vr for airspeed checks during climb
  LandPhase land_phase_ = LandPhase::kApproach;
  double land_approach_speed_mps_ = 0.0;
  double land_target_alt_m_ = 0.0;
  double prev_alt_m_ = 0.0;
  double sink_rate_mps_ = 0.0;
};

}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_GUIDANCE_MANEUVER_H_
