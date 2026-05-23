/**
 * @file ManeuverController.h
 * @brief 机动制导/控制律公共接口。
 *
 * ManeuverController 是无状态的纯计算工具：每种 Compute* 方法接收
 * 当前飞行器状态 + 机动参数，返回下一步的 ControlInput。
 *
 * 调用方可直接使用此接口实现自定义控制循环，也可通过
 * FlightDynamicSession::StepManeuver() 获取集成的机动执行能力。
 */

#ifndef ONEQ_FLIGHT_DYNAMIC_MANEUVER_MANEUVER_CONTROLLER_H_
#define ONEQ_FLIGHT_DYNAMIC_MANEUVER_MANEUVER_CONTROLLER_H_

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/flight_dynamic/maneuver/ManeuverTypes.h"
#include "1q/flight_dynamic/model/FlightDynamicInput.h"
#include "1q/flight_dynamic/model/FlightDynamicOutput.h"

namespace flight_dynamic {
namespace maneuver {

// ---- 几何工具 ----

/**
 * @brief 计算两个 LLA 点之间的大圆距离（Haversine 公式）。
 * @return 距离 (m)。
 */
ONEQ_API double ComputeGreatCircleDistanceM(
    const oneq::coordinate::LlaPositionDegM& from,
    const oneq::coordinate::LlaPositionDegM& to);

/**
 * @brief 计算从 from 到 to 的前向方位角（大圆航线初始方位）。
 * @return 方位角 (deg, 0=北, 90=东, 0~360)。
 */
ONEQ_API double ComputeForwardAzimuthDeg(
    const oneq::coordinate::LlaPositionDegM& from,
    const oneq::coordinate::LlaPositionDegM& to);

/**
 * @brief 规范化航向误差到 [-180, 180] 范围。
 */
inline double NormalizeHeadingErrorDeg(double error_deg) {
  while (error_deg > 180.0) error_deg -= 360.0;
  while (error_deg < -180.0) error_deg += 360.0;
  return error_deg;
}

// ---- 机动控制器 ----

/**
 * @brief 机动控制器——纯函数，无状态。
 *
 * 每种 Compute* 方法接收当前状态 + 机动参数，返回下一步的 ControlInput。
 * 机动状态机（如航路点索引、滚筒 PID）由调用方或 FlightDynamicSession 管理。
 */
class ONEQ_API ManeuverController {
 public:
  ManeuverController() = default;

  model::FlightDynamicInput ComputePointToPoint(
      const model::FlightDynamicOutput& current,
      const PointToPointParams& params,
      bool* reached) const;

  model::FlightDynamicInput ComputeWeave(
      const model::FlightDynamicOutput& current,
      const WeaveParams& params,
      double sim_time_s) const;

  model::FlightDynamicInput ComputeWaypoint(
      const model::FlightDynamicOutput& current,
      const WaypointList& waypoints,
      const WaypointParams& params,
      std::size_t* wp_index,
      bool* all_reached) const;

  model::FlightDynamicInput ComputeBarrelRoll(
      const model::FlightDynamicOutput& current,
      const BarrelRollParams& params,
      double sim_time_s,
      BarrelRollState* state) const;

  model::FlightDynamicInput ComputeOrbit(
      const model::FlightDynamicOutput& current,
      const OrbitParams& params,
      double sim_time_s,
      OrbitState* state) const;

  model::FlightDynamicInput ComputeEvasion(
      const model::FlightDynamicOutput& current,
      const EvasionParams& params,
      double sim_time_s,
      EvasionState* state) const;
};

}  // namespace maneuver
}  // namespace flight_dynamic

#endif  // ONEQ_FLIGHT_DYNAMIC_MANEUVER_MANEUVER_CONTROLLER_H_
