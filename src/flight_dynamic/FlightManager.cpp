#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/guidance/WaypointManager.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/model/VehicleStateMapper.h"

namespace oneq {
namespace flight_dynamic {

FlightManager::FlightManager(const config::FlightDynamicConfig& config) {
  adapter_.reset(new adapter::JsbsimAdapter(config));
  ap_.reset(new autopilot::Autopilot(*adapter_));
  wp_manager_.reset(new guidance::WaypointManager(*adapter_));
  maneuver_exec_.reset(new guidance::ManeuverExecutor(
      *adapter_, *ap_, *wp_manager_));
  state_ = FlightManagerState::kReady;
}

FlightManager::~FlightManager() = default;

bool FlightManager::Step(double dt_sec) {
  if (state_ == FlightManagerState::kAborted ||
      state_ == FlightManagerState::kCompleted) {
    return false;
  }

  ap_->Update(dt_sec);
  adapter_->SetDeltaT(dt_sec);
  bool running = adapter_->Run();
  sim_time_sec_ += dt_sec;

  // Update vehicle state
  vehicle_state_ = model::VehicleStateMapper::Map(
      adapter_->GetPropagate(),
      adapter_->GetAccelerations(),
      adapter_->GetFdmExec(),
      sim_time_sec_);

  // Update active maneuver
  if (state_ == FlightManagerState::kExecuting) {
    maneuver_exec_->Update(dt_sec);
    if (maneuver_exec_->IsManeuverComplete()) {
      ExecuteNextManeuver();
    }
  }

  return running;
}

void FlightManager::Reset(const config::FlightDynamicConfig& config) {
  adapter_.reset(new adapter::JsbsimAdapter(config));
  ap_.reset(new autopilot::Autopilot(*adapter_));
  wp_manager_.reset(new guidance::WaypointManager(*adapter_));
  maneuver_exec_.reset(new guidance::ManeuverExecutor(
      *adapter_, *ap_, *wp_manager_));
  maneuver_queue_.clear();
  current_maneuver_index_ = 0;
  sim_time_sec_ = 0.0;
  state_ = FlightManagerState::kReady;
}

void FlightManager::Abort() {
  state_ = FlightManagerState::kAborted;
  maneuver_exec_->Abort();
}

void FlightManager::PushManeuver(const ManeuverCommand& cmd) {
  maneuver_queue_.push_back(cmd);
  if (state_ == FlightManagerState::kReady) {
    state_ = FlightManagerState::kExecuting;
    ExecuteNextManeuver();
  }
}

void FlightManager::ClearManeuvers() {
  maneuver_queue_.clear();
  current_maneuver_index_ = 0;
}

void FlightManager::ExecuteNextManeuver() {
  if (current_maneuver_index_ >= maneuver_queue_.size()) {
    state_ = FlightManagerState::kCompleted;
    return;
  }

  const auto& cmd = maneuver_queue_[current_maneuver_index_];
  state_ = FlightManagerState::kExecuting;
  ++current_maneuver_index_;

  using guidance::ManeuverType;
  switch (cmd.type) {
    case ManeuverType::kFlyToWaypoint:
      maneuver_exec_->ExecuteFlyTo(cmd.target);
      break;
    case ManeuverType::kOrbit:
      maneuver_exec_->ExecuteOrbit(cmd.target, cmd.value);
      break;
    case ManeuverType::kSetHeading:
      maneuver_exec_->ExecuteSetHeading(cmd.value);
      break;
    case ManeuverType::kSetAltitude:
      maneuver_exec_->ExecuteSetAltitude(cmd.value);
      break;
    case ManeuverType::kSetPitch:
      maneuver_exec_->ExecuteSetPitch(cmd.value, cmd.duration_sec);
      break;
    case ManeuverType::kSetRoll:
      maneuver_exec_->ExecuteSetRoll(static_cast<int>(cmd.value));
      break;
  }
}

}  // namespace flight_dynamic
}  // namespace oneq
