/**
 * @file FlightDynamicSessionImpl.h
 * @brief FlightDynamicSession::Impl 完整定义（内部使用，不对外暴露）。
 *
 * 此头文件仅由 FlightDynamicSession.cpp 和 FlightDynamicSessionFactory.cpp
 * 包含，不包含在任何公共头文件中。
 */

#ifndef FLIGHT_DYNAMIC_SESSION_FLIGHT_DYNAMIC_SESSION_IMPL_H_
#define FLIGHT_DYNAMIC_SESSION_FLIGHT_DYNAMIC_SESSION_IMPL_H_

#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/model/VehicleStateMapper.h"
#include "1q/flight_dynamic/maneuver/ManeuverController.h"
#include "1q/flight_dynamic/model/FlightDynamicOutput.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "1q/flight_dynamic/session/FlightDynamicSession.h"

namespace flight_dynamic {
namespace session {

struct FlightDynamicSession::Impl {
  // -- 物理引擎 --
  adapter::JsbsimAdapter adapter;
  model::VehicleStateMapper mapper;
  model::FlightDynamicOutput last_output;

  // -- 机动控制 --
  maneuver::ManeuverController maneuver_ctrl;
  maneuver::ManeuverMode active_maneuver_mode{maneuver::ManeuverMode::kManual};
  double maneuver_sim_time_s{0.0};

  // 航路点机动状态
  std::size_t waypoint_index{0};
  maneuver::WaypointList active_waypoints;

  // 滚筒机动状态
  maneuver::BarrelRollState barrel_roll_state{};

  // 绕圈盘旋状态
  maneuver::OrbitState orbit_state{};

  // 规避机动状态
  maneuver::EvasionState evasion_state{};

  explicit Impl(const config::FlightDynamicConfig& config)
      : adapter(config), mapper(), last_output() {
    last_output = mapper.Map(adapter.GetPropagate(),
                             adapter.GetAccelerations(),
                             adapter.GetFdmExec());
    last_output.ok = true;
  }
};

}  // namespace session
}  // namespace flight_dynamic

#endif  // FLIGHT_DYNAMIC_SESSION_FLIGHT_DYNAMIC_SESSION_IMPL_H_
