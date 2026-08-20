/**
 * @file Waypoint.h
 * @brief 定义 flight_dynamic 引导模块使用的地理航点类型。
 */

#ifndef ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINT_H_
#define ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINT_H_

#include <string>

namespace oneq {
namespace flight_dynamic {
namespace guidance {

/**
 * @brief 地理航点（WGS84 LLA + 速度/半径约束）。
 *
 * 角度统一采用弧度，高度采用米。由 WaypointManager 管理并通过 ManeuverExecutor
 * 用于各类机动的目标点定义。
 */
struct Waypoint {
  double latitude_rad = 0.0;  /**< 纬度（单位：rad） */
  double longitude_rad = 0.0; /**< 经度（单位：rad） */
  double altitude_m = 0.0;    /**< 椭球高（单位：m） */
  double radius_m = 100.0;    /**< 到达/盘旋判定半径（单位：m），默认 100 */
  double speed_mps = 0.0;     /**< 目标真空速 TAS（单位：m/s），0 表示沿用机型默认巡航速度 */
  std::string name;           /**< 可选航点名称，仅供标识/调试 */
};

}  // namespace guidance
}  // namespace flight_dynamic
}  // namespace oneq

#endif  // ONEQ_FLIGHT_DYNAMIC_GUIDANCE_WAYPOINT_H_
