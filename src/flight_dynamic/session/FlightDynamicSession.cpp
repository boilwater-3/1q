/**
 * @file FlightDynamicSession.cpp
 * @brief FlightDynamicSession 公共接口实现（PIMPL 转发）。
 */

#include "1q/flight_dynamic/session/FlightDynamicSession.h"
#include "1q/coordinate/position_transform.h"
#include "flight_dynamic/session/FlightDynamicSessionImpl.h"

namespace flight_dynamic {
namespace session {

// ---- 构造/析构 ----

FlightDynamicSession::FlightDynamicSession() = default;

FlightDynamicSession::~FlightDynamicSession() = default;

FlightDynamicSession::FlightDynamicSession(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}

FlightDynamicSession::FlightDynamicSession(FlightDynamicSession&&) noexcept = default;
FlightDynamicSession& FlightDynamicSession::operator=(FlightDynamicSession&&) noexcept = default;

// ---- 公共接口（转发到 Impl）----

model::FlightDynamicOutput FlightDynamicSession::Step(
    const model::FlightDynamicInput& input) {
  if (!impl_) {
    return model::FlightDynamicOutput{};
  }
  const bool ok = impl_->adapter.Run(input);
  if (ok) {
    impl_->last_output = impl_->mapper.Map(
        impl_->adapter.GetPropagate(),
        impl_->adapter.GetAccelerations(),
        impl_->adapter.GetFdmExec());
    impl_->last_output.ok = true;
  } else {
    impl_->last_output.ok = false;
  }
  return impl_->last_output;
}

void FlightDynamicSession::Reset(
    const oneq::coordinate::ExternalKinematics& kinematics) {
  if (!impl_) {
    return;
  }
  impl_->adapter.Reset(kinematics);
  impl_->last_output = impl_->mapper.Map(
      impl_->adapter.GetPropagate(),
      impl_->adapter.GetAccelerations(),
      impl_->adapter.GetFdmExec());
  impl_->last_output.ok = true;
}

model::FlightDynamicOutput FlightDynamicSession::GetCurrentState() const {
  if (!impl_) {
    return model::FlightDynamicOutput{};
  }
  return impl_->last_output;
}

// ---- 机动控制 ----

maneuver::ManeuverStepResult FlightDynamicSession::StepManeuver(
    const maneuver::ManeuverRequest& request) {
  maneuver::ManeuverStepResult result;

  if (!impl_) {
    return result;
  }

  // 机动切换时重置状态
  if (request.mode != impl_->active_maneuver_mode) {
    impl_->active_maneuver_mode = maneuver::ManeuverMode::kManual;
    impl_->maneuver_sim_time_s = 0.0;
    impl_->waypoint_index = 0;
    impl_->active_waypoints.clear();
    impl_->barrel_roll_state = maneuver::BarrelRollState{};
    impl_->orbit_state = maneuver::OrbitState{};
    impl_->evasion_state = maneuver::EvasionState{};
    impl_->active_maneuver_mode = request.mode;
  }

  maneuver::ManeuverStatus& status = result.status;
  status.active_mode = request.mode;
  status.active = true;

  model::FlightDynamicInput input{};
  input.dt_sec = request.dt_sec;

  switch (request.mode) {
    case maneuver::ManeuverMode::kManual: {
      status.active = false;
      break;
    }

    case maneuver::ManeuverMode::kPointToPoint: {
      bool reached = false;
      input = impl_->maneuver_ctrl.ComputePointToPoint(
          impl_->last_output, request.point_to_point, &reached);
      input.dt_sec = request.dt_sec;
      status.completed = reached;
      if (reached) status.active = false;
      break;
    }

    case maneuver::ManeuverMode::kWaypoint: {
      // 首次进入 kWaypoint 或列表变化时缓存航路点
      if (impl_->active_waypoints.empty() ||
          impl_->active_waypoints.size() != request.waypoints.size()) {
        impl_->active_waypoints = request.waypoints;
        impl_->waypoint_index = 0;
      }

      bool all_reached = false;
      input = impl_->maneuver_ctrl.ComputeWaypoint(
          impl_->last_output,
          impl_->active_waypoints,
          request.waypoint_params,
          &impl_->waypoint_index,
          &all_reached);
      input.dt_sec = request.dt_sec;
      status.waypoint_index = impl_->waypoint_index;
      status.waypoint_count = request.waypoints.size();
      status.completed = all_reached;
      if (all_reached) status.active = false;
      break;
    }

    case maneuver::ManeuverMode::kWeave: {
      input = impl_->maneuver_ctrl.ComputeWeave(
          impl_->last_output, request.weave, impl_->maneuver_sim_time_s);
      input.dt_sec = request.dt_sec;
      break;
    }

    case maneuver::ManeuverMode::kBarrelRoll: {
      input = impl_->maneuver_ctrl.ComputeBarrelRoll(
          impl_->last_output,
          request.barrel_roll,
          impl_->maneuver_sim_time_s,
          &impl_->barrel_roll_state);
      input.dt_sec = request.dt_sec;
      status.barrel_roll_phase = impl_->barrel_roll_state.phase;
      status.completed =
          impl_->barrel_roll_state.phase == maneuver::BarrelRollPhase::kCompleted;
      status.aborted =
          impl_->barrel_roll_state.phase == maneuver::BarrelRollPhase::kAborted;
      if (status.completed || status.aborted) status.active = false;
      break;
    }

    case maneuver::ManeuverMode::kOrbit: {
      input = impl_->maneuver_ctrl.ComputeOrbit(
          impl_->last_output,
          request.orbit,
          impl_->maneuver_sim_time_s,
          &impl_->orbit_state);
      input.dt_sec = request.dt_sec;

      // 报告到中心的距离
      oneq::coordinate::LlaPositionDegM current_lla{};
      if (impl_->last_output.kinematics.position_frame ==
          oneq::coordinate::PositionFrame::kLla) {
        current_lla = impl_->last_output.kinematics.position_lla_deg_m;
      } else {
        oneq::coordinate::TryEcefToLla(
            impl_->last_output.kinematics.position_ecef_m, &current_lla);
      }
      status.orbit_distance_m =
          maneuver::ComputeGreatCircleDistanceM(current_lla, request.orbit.center_lla);
      break;
    }

    case maneuver::ManeuverMode::kEvasion: {
      input = impl_->maneuver_ctrl.ComputeEvasion(
          impl_->last_output,
          request.evasion,
          impl_->maneuver_sim_time_s,
          &impl_->evasion_state);
      input.dt_sec = request.dt_sec;
      status.evasion_phase = impl_->evasion_state.phase;
      status.completed =
          impl_->evasion_state.phase == maneuver::EvasionPhase::kCompleted;
      if (status.completed) status.active = false;
      break;
    }
  }

  // 推进物理引擎
  result.output = Step(input);

  // 推进机动仿真时钟
  impl_->maneuver_sim_time_s += static_cast<double>(request.dt_sec);

  return result;
}

void FlightDynamicSession::ResetManeuver() {
  if (!impl_) {
    return;
  }
  impl_->active_maneuver_mode = maneuver::ManeuverMode::kManual;
  impl_->maneuver_sim_time_s = 0.0;
  impl_->waypoint_index = 0;
  impl_->active_waypoints.clear();
  impl_->barrel_roll_state = maneuver::BarrelRollState{};
  impl_->orbit_state = maneuver::OrbitState{};
  impl_->evasion_state = maneuver::EvasionState{};
}

}  // namespace session
}  // namespace flight_dynamic
