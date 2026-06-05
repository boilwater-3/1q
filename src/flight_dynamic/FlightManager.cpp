#include <cmath>
#include <iostream>

#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/guidance/WaypointManager.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/model/VehicleStateMapper.h"
#include "flight_dynamic/propulsion/EngineManager.h"

namespace oneq {
namespace flight_dynamic {

FlightManager::FlightManager(const config::FlightDynamicConfig& config) {
  adapter_.reset(new adapter::JsbsimAdapter(config));
  engines_.reset(new propulsion::EngineManager(*adapter_));
  ap_.reset(new autopilot::Autopilot(*adapter_));
  wp_manager_.reset(new guidance::WaypointManager(*adapter_));
  maneuver_exec_.reset(new guidance::ManeuverExecutor(
      *adapter_, *ap_, *wp_manager_, *engines_));
  state_ = FlightManagerState::kReady;
}

FlightManager::~FlightManager() = default;

bool FlightManager::Step(double dt_sec) {
  if (state_ == FlightManagerState::kAborted ||
      state_ == FlightManagerState::kCompleted) {
    return false;
  }

  adapter_->SetDeltaT(dt_sec);

  ap_->Update(dt_sec);
  if (state_ == FlightManagerState::kExecuting) {
    maneuver_exec_->Update(dt_sec);
  }

  bool running = adapter_->Run();
  sim_time_sec_ += dt_sec;

  // Update vehicle state
  vehicle_state_ = model::VehicleStateMapper::Map(
      adapter_->GetPropagate(),
      adapter_->GetAccelerations(),
      adapter_->GetFdmExec(),
      sim_time_sec_);

  // Track diagnostics
  if (state_ == FlightManagerState::kExecuting) {
    diagnostics_.Update(vehicle_state_);
  }

  // Update active maneuver
  if (state_ == FlightManagerState::kExecuting) {
    if (diagnostics_.crashed && !maneuver_exec_->IsTouchingGround()) {
      diagnostics_.last_failure_reason = "crashed";
      diagnostics_.outcome = ManeuverOutcome::kCrashed;
      diagnostics_.Print();
      state_ = FlightManagerState::kAborted;
      maneuver_exec_->Abort();
      return false;
    }
    // Hard crash: well below ground even during landing (gear collapse).
    if (vehicle_state_.altitude_agl_m < -5.0) {
      diagnostics_.last_failure_reason = "crashed";
      diagnostics_.outcome = ManeuverOutcome::kCrashed;
      diagnostics_.Print();
      state_ = FlightManagerState::kAborted;
      maneuver_exec_->Abort();
      return false;
    }
    if (maneuver_exec_->IsManeuverComplete()) {
      diagnostics_.outcome = ManeuverOutcome::kCompleted;
      diagnostics_.Print();
      ExecuteNextManeuver();
    }
  }

  return running;
}

void FlightManager::Reset(const config::FlightDynamicConfig& config) {
  adapter_.reset(new adapter::JsbsimAdapter(config));
  engines_.reset(new propulsion::EngineManager(*adapter_));
  ap_.reset(new autopilot::Autopilot(*adapter_));
  wp_manager_.reset(new guidance::WaypointManager(*adapter_));
  maneuver_exec_.reset(new guidance::ManeuverExecutor(
      *adapter_, *ap_, *wp_manager_, *engines_));
  maneuver_queue_.clear();
  current_maneuver_index_ = 0;
  sim_time_sec_ = 0.0;
  state_ = FlightManagerState::kReady;
}

void FlightManager::Abort() {
  state_ = FlightManagerState::kAborted;
  diagnostics_.outcome = ManeuverOutcome::kAborted;
  maneuver_exec_->Abort();
  ap_->ReleaseHolds();
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

  // Reset diagnostics for new maneuver.
  diagnostics_ = ManeuverDiagnostics();
  diagnostics_.current_type = cmd.type;

  using guidance::ManeuverType;
  switch (cmd.type) {
    case ManeuverType::kFlyToWaypoint:
      maneuver_exec_->ExecuteFlyTo(cmd.target);
      break;
    case ManeuverType::kOrbit:
      maneuver_exec_->ExecuteOrbit(cmd.target, cmd.value, cmd.duration_sec);
      break;
    case ManeuverType::kSetHeading:
      maneuver_exec_->ExecuteSetHeading(cmd.value, cmd.heading_tolerance_rad);
      break;
    case ManeuverType::kSetAltitude:
      maneuver_exec_->ExecuteSetAltitude(cmd.value, cmd.altitude_tolerance_m);
      break;
    case ManeuverType::kSetPitch:
      maneuver_exec_->ExecuteSetPitch(cmd.value, cmd.duration_sec);
      break;
    case ManeuverType::kSetRoll:
      maneuver_exec_->ExecuteSetRoll(static_cast<int>(cmd.value));
      break;
    case ManeuverType::kTakeoff:
      maneuver_exec_->ExecuteTakeoff(cmd.target.altitude_m,
                                     cmd.target.latitude_rad,
                                     cmd.value);
      break;
    case ManeuverType::kLand:
      maneuver_exec_->ExecuteLand(cmd.target, cmd.value);
      break;
    case ManeuverType::kRacetrack:
      maneuver_exec_->ExecuteRacetrack(
          cmd.target, cmd.value, cmd.duration_sec,
          cmd.heading_tolerance_rad,
          static_cast<int>(cmd.altitude_tolerance_m));
      break;
    case ManeuverType::kFigure8:
      maneuver_exec_->ExecuteFigure8(
          cmd.target, cmd.value, cmd.duration_sec,
          static_cast<int>(cmd.heading_tolerance_rad));
      break;
    case ManeuverType::kSTurn:
      // Field convention: value=base_heading_rad, heading_tolerance_rad=amplitude_deg,
      // altitude_tolerance_m=period_sec, duration_sec=duration_sec
      maneuver_exec_->ExecuteSTurn(
          cmd.value, cmd.heading_tolerance_rad,
          cmd.altitude_tolerance_m, cmd.duration_sec);
      break;
  }
}

void ManeuverDiagnostics::Update(const model::VehicleState& s) {
  if (s.altitude_geod_m < min_altitude_m) min_altitude_m = s.altitude_geod_m;
  if (s.vtrue_mps < min_speed_mps) min_speed_mps = s.vtrue_mps;
  double r = std::abs(s.phi_rad) * 57.2958;
  if (r > max_roll_deg) max_roll_deg = r;
  double p = std::abs(s.theta_rad) * 57.2958;
  if (p > max_pitch_deg) max_pitch_deg = p;
  steps++;
  total_time_sec = s.sim_time_sec;
  // Landing gear compression tolerance: allow up to 0.5m below ground
  // before declaring crash (handles one-frame touchdown delay).
  if (s.altitude_agl_m <= -0.5) crashed = true;
}

void ManeuverDiagnostics::Print() const {
  const char* outcome_str = "none";
  switch (outcome) {
    case ManeuverOutcome::kCompleted: outcome_str = "completed"; break;
    case ManeuverOutcome::kCrashed: outcome_str = "crashed"; break;
    case ManeuverOutcome::kAborted: outcome_str = "aborted"; break;
    case ManeuverOutcome::kTimeout: outcome_str = "timeout"; break;
    case ManeuverOutcome::kNearPass: outcome_str = "near-pass"; break;
    default: break;
  }
  std::cout << "[DIAG] type=" << static_cast<int>(current_type)
            << " | outcome=" << outcome_str
            << " | steps=" << steps
            << " | time=" << total_time_sec << "s"
            << " | alt_min=" << min_altitude_m << "m"
            << " | spd_min=" << min_speed_mps << "m/s"
            << " | roll_max=" << max_roll_deg << "deg"
            << " | pitch_max=" << max_pitch_deg << "deg"
            << " | crashed=" << (crashed ? "YES" : "no")
            << " | reason=" << last_failure_reason
            << std::endl;
}

}  // namespace flight_dynamic
}  // namespace oneq
