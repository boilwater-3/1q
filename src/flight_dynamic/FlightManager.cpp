#include <cmath>

#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/guidance/WaypointManager.h"
#include "common/logging/ProjectLog.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/model/VehicleStateMapper.h"
#include "flight_dynamic/propulsion/EngineManager.h"

namespace oneq {
namespace flight_dynamic {

namespace {

// 航点完成事件环形记录容量：超出时丢弃最旧，防止长时间会话无界增长。
constexpr std::size_t kMaxWaypointEvents = 512;

bool BuildFlightDynamicComponents(
    const config::FlightDynamicConfig& config,
    std::unique_ptr<adapter::JsbsimAdapter>* adapter,
    std::unique_ptr<propulsion::EngineManager>* engines,
    std::unique_ptr<autopilot::Autopilot>* ap,
    std::unique_ptr<guidance::WaypointManager>* waypoint_manager,
    std::unique_ptr<guidance::ManeuverExecutor>* maneuver_executor) {
  adapter->reset(new adapter::JsbsimAdapter(config));
  if (!(*adapter)->IsValid()) {
    engines->reset();
    ap->reset();
    waypoint_manager->reset();
    maneuver_executor->reset();
    return false;
  }
  engines->reset(new propulsion::EngineManager(**adapter));
  ap->reset(new autopilot::Autopilot(**adapter));
  waypoint_manager->reset(new guidance::WaypointManager(**adapter));
  maneuver_executor->reset(new guidance::ManeuverExecutor(
      **adapter, **ap, **waypoint_manager, **engines));
  return true;
}

}  // namespace

FlightManager::FlightManager(const config::FlightDynamicConfig& config) {
  if (BuildFlightDynamicComponents(config, &adapter_, &engines_, &ap_, &wp_manager_,
                                   &maneuver_exec_)) {
    state_ = FlightManagerState::kReady;
    return;
  }
  diagnostics_.outcome = ManeuverOutcome::kAborted;
  diagnostics_.last_failure_reason = adapter_ ? adapter_->GetInitDiagnostics().failure_reason
                                              : "adapter allocation failed";
  state_ = FlightManagerState::kAborted;
}

FlightManager::~FlightManager() = default;

bool FlightManager::Step(double dt_sec) {
  if (state_ == FlightManagerState::kAborted ||
      state_ == FlightManagerState::kCompleted) {
    return false;
  }
  if (!adapter_ || !ap_ || !maneuver_exec_) {
    state_ = FlightManagerState::kAborted;
    diagnostics_.outcome = ManeuverOutcome::kAborted;
    diagnostics_.last_failure_reason = "flight dynamic components are not initialized";
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
    // 中间/最终航点语义按"当前队列"每步重估：kFlyToWaypoint 队列是增量 Push 的
    // （首航点派发时后继尚未入队），派发时一次性定死会把首航点误判为最终航点，
    // 导致整条紧间距航路在第 1 步坍缩。后继仍是 kFlyToWaypoint 的当前航点为中间航点。
    bool intermediate_waypoint = false;
    if (maneuver_exec_ &&
        diagnostics_.current_type == guidance::ManeuverType::kFlyToWaypoint) {
      intermediate_waypoint =
          current_maneuver_index_ < maneuver_queue_.size() &&
          maneuver_queue_[current_maneuver_index_].type ==
              guidance::ManeuverType::kFlyToWaypoint;
      maneuver_exec_->SetIntermediateWaypoint(intermediate_waypoint);
    }
    if (maneuver_exec_->IsManeuverComplete()) {
      diagnostics_.outcome = ManeuverOutcome::kCompleted;
      diagnostics_.Print();
      // 记录航点完成事件（决策快照由执行器填充，队列索引/仿真时间在此补齐；
      // 记录必须先于 ExecuteNextManeuver，其会重置诊断并派发下一机动）。
      if (diagnostics_.current_type == guidance::ManeuverType::kFlyToWaypoint) {
        if (const auto* event = maneuver_exec_->GetLastSequencingEvent()) {
          waypoint_events_.push_back(*event);
          waypoint_events_.back().sim_time_sec = sim_time_sec_;
          waypoint_events_.back().waypoint_index = current_maneuver_index_ - 1;
          waypoint_events_.back().intermediate = intermediate_waypoint;
          if (waypoint_events_.size() > kMaxWaypointEvents) {
            waypoint_events_.erase(waypoint_events_.begin());
          }
        }
      }
      ExecuteNextManeuver();
    }
  }

  return running;
}

void FlightManager::Reset(const config::FlightDynamicConfig& config) {
  maneuver_queue_.clear();
  current_maneuver_index_ = 0;
  sim_time_sec_ = 0.0;
  diagnostics_ = ManeuverDiagnostics();
  waypoint_events_.clear();
  if (BuildFlightDynamicComponents(config, &adapter_, &engines_, &ap_, &wp_manager_,
                                   &maneuver_exec_)) {
    state_ = FlightManagerState::kReady;
    return;
  }
  diagnostics_.outcome = ManeuverOutcome::kAborted;
  diagnostics_.last_failure_reason = adapter_ ? adapter_->GetInitDiagnostics().failure_reason
                                              : "adapter allocation failed";
  state_ = FlightManagerState::kAborted;
}

void FlightManager::Abort() {
  state_ = FlightManagerState::kAborted;
  diagnostics_.outcome = ManeuverOutcome::kAborted;
  if (maneuver_exec_) {
    maneuver_exec_->Abort();
  }
  if (ap_) {
    ap_->ReleaseHolds();
  }
}

void FlightManager::PushManeuver(const ManeuverCommand& cmd) {
  maneuver_queue_.push_back(cmd);
  if (state_ == FlightManagerState::kReady) {
    if (!maneuver_exec_) {
      state_ = FlightManagerState::kAborted;
      diagnostics_.outcome = ManeuverOutcome::kAborted;
      diagnostics_.last_failure_reason = "flight dynamic components are not initialized";
      return;
    }
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
  // 中译：机动动作诊断摘要：动作类型、结局、步数、耗时、最低高度、最低速度、
  //       最大横滚/俯仰角、是否坠毁与最后失败原因。
  // 标识：机动诊断一次性转储——仅在机动动作结束时输出，供人工核对动作执行质量；
  //       非周期性日志，不用于状态判断。
  PROJECT_LOG_INFO("[DIAG] type={} | outcome={} | steps={} | time={}s | alt_min={}m | "
                   "spd_min={}m/s | roll_max={}deg | pitch_max={}deg | crashed={} | reason={}",
                   static_cast<int>(current_type), outcome_str, steps, total_time_sec,
                   min_altitude_m, min_speed_mps, max_roll_deg, max_pitch_deg,
                   crashed ? "YES" : "no", last_failure_reason);
}

}  // namespace flight_dynamic
}  // namespace oneq
