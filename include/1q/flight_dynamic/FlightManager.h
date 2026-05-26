#ifndef ONEQ_FLIGHT_DYNAMIC_FLIGHTMANAGER_H_
#define ONEQ_FLIGHT_DYNAMIC_FLIGHTMANAGER_H_

#include <memory>
#include <vector>

#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "1q/flight_dynamic/guidance/Maneuver.h"
#include "1q/flight_dynamic/guidance/Waypoint.h"
#include "1q/flight_dynamic/model/VehicleState.h"

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

enum class FlightManagerState {
  kIdle,
  kReady,
  kExecuting,
  kCompleted,
  kAborted,
};

struct ManeuverCommand {
  guidance::ManeuverType type;
  guidance::Waypoint target;
  double value = 0.0;
  double duration_sec = 0.0;
};

class FlightManager {
 public:
  explicit FlightManager(const config::FlightDynamicConfig& config);
  ~FlightManager();

  FlightManager(const FlightManager&) = delete;
  FlightManager& operator=(const FlightManager&) = delete;

  bool Step(double dt_sec);
  void Reset(const config::FlightDynamicConfig& config);
  void Abort();

  // Maneuver sequencing
  void PushManeuver(const ManeuverCommand& cmd);
  void ClearManeuvers();

  // State queries
  FlightManagerState GetState() const { return state_; }
  const model::VehicleState& GetVehicleState() const { return vehicle_state_; }

  // Direct access for lower-level control
  adapter::JsbsimAdapter& GetAdapter() { return *adapter_; }
  autopilot::Autopilot& GetAutopilot() { return *ap_; }
  guidance::WaypointManager& GetWaypointManager() { return *wp_manager_; }

 private:
  void ExecuteNextManeuver();

  std::unique_ptr<adapter::JsbsimAdapter> adapter_;
  std::unique_ptr<autopilot::Autopilot> ap_;
  std::unique_ptr<guidance::WaypointManager> wp_manager_;
  std::unique_ptr<guidance::ManeuverExecutor> maneuver_exec_;

  model::VehicleState vehicle_state_;
  std::vector<ManeuverCommand> maneuver_queue_;
  size_t current_maneuver_index_ = 0;
  FlightManagerState state_ = FlightManagerState::kIdle;
  double sim_time_sec_ = 0.0;
};

}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_FLIGHTMANAGER_H_
