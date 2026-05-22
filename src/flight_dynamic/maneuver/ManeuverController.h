/**
 * @file ManeuverController.h
 * @brief 机动制导/控制律——根据机动模式计算 ControlInput（内部实现）。
 *
 * 与 FlightDynamicSession 解耦：ManeuverController 不持有 Session，
 * 仅根据当前状态 + 机动参数计算下一步的控制输入。
 * 调用方负责将 ControlInput 传入 FlightDynamicSession::Step()。
 */

#ifndef FLIGHT_DYNAMIC_MANEUVER_MANEUVER_CONTROLLER_H_
#define FLIGHT_DYNAMIC_MANEUVER_MANEUVER_CONTROLLER_H_

#include <cmath>
#include <cstdint>

#include "1q/flight_dynamic/model/FlightDynamicInput.h"
#include "1q/flight_dynamic/model/FlightDynamicOutput.h"
#include "1q/coordinate/types.h"

namespace flight_dynamic {
namespace maneuver {

// ---- 几何工具 ----

/**
 * @brief 计算两个 LLA 点之间的大圆距离（Haversine 公式）。
 * @return 距离 (m)。
 */
double ComputeGreatCircleDistanceM(const oneq::coordinate::LlaPositionDegM& from,
                                   const oneq::coordinate::LlaPositionDegM& to);

/**
 * @brief 计算从 from 到 to 的前向方位角（大圆航线初始方位）。
 * @return 方位角 (deg, 0=北, 90=东, 0~360)。
 */
double ComputeForwardAzimuthDeg(const oneq::coordinate::LlaPositionDegM& from,
                                const oneq::coordinate::LlaPositionDegM& to);

/**
 * @brief 规范化航向误差到 [-180, 180] 范围。
 */
inline double NormalizeHeadingErrorDeg(double error_deg) {
  while (error_deg > 180.0) error_deg -= 360.0;
  while (error_deg < -180.0) error_deg += 360.0;
  return error_deg;
}

// ---- 机动参数 ----

/** @brief 固定点 / 航路点机动参数。 */
struct PointToPointParams {
  oneq::coordinate::LlaPositionDegM target_lla{};  ///< 目标点
  double arrival_distance_m{50.0};                   ///< 到达判定距离阈值 (m)
  double heading_tolerance_deg{5.0};                 ///< 到达判定航向误差阈值 (deg)
  double cruise_speed_mps{50.0};                     ///< 巡航速度 (m/s)
  double base_throttle{0.75};                        ///< 基础油门
};

/** @brief 蛇形机动参数。 */
struct WeaveParams {
  double base_heading_deg{0.0};     ///< 基础航向 (deg)
  double amplitude_deg{30.0};       ///< 摆动幅度 (deg)
  double period_s{20.0};            ///< 摆动周期 (s)
};

// ---- 机动控制器 ----

/**
 * @brief 机动控制器——纯函数，无状态。
 *
 * 每种 Compute* 方法接收当前状态 + 机动参数，返回下一步的 ControlInput。
 * 机动状态机（如航路点索引）由调用方管理。
 */
class ManeuverController {
 public:
  ManeuverController() = default;

  /**
   * @brief 计算固定点机动的控制输入。
   *
   * 根据当前位置计算到目标点的方位角（→ heading_setpoint）和距离。
   * 距离 < arrival_distance_m 时到达，heading_hold 设为 false。
   *
   * @param current 当前飞行器状态。
   * @param params 固定点参数。
   * @param[out] reached 是否已到达目标点。
   * @return 控制输入。
   */
  model::FlightDynamicInput ComputePointToPoint(
      const model::FlightDynamicOutput& current,
      const PointToPointParams& params,
      bool* reached) const;

  /**
   * @brief 计算蛇形机动的控制输入。
   *
   * @param current 当前状态。
   * @param params 蛇形参数。
   * @param sim_time_s 当前仿真时间 (s)。
   * @return 控制输入。
   */
  model::FlightDynamicInput ComputeWeave(
      const model::FlightDynamicOutput& current,
      const WeaveParams& params,
      double sim_time_s) const;
};

}  // namespace maneuver
}  // namespace flight_dynamic

#endif  // FLIGHT_DYNAMIC_MANEUVER_MANEUVER_CONTROLLER_H_
