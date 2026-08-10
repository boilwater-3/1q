/**
 * @file RoutePoint.h
 * @brief 定义中性航点类型与航路计划。
 */

#ifndef ONEQ_NAVIGATION_ROUTE_POINT_H_
#define ONEQ_NAVIGATION_ROUTE_POINT_H_

#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace navigation {

/**
 * @brief 中性航点（度制 LLA + 速度 + 到达半径）。
 * @note 语义对齐 coordinate 域度制惯例；与 flight_dynamic::guidance::Waypoint
 *       （弧度制）之间的单位转换属业务层适配职责。
 */
struct ONEQ_API RoutePoint {
  oneq::coordinate::LlaPositionDegM position{}; /**< 航点位置（度制 LLA，椭球高） */
  double speed_mps{0.0};                        /**< 期望速度（单位：m/s） */
  double radius_m{0.0};                         /**< 到达半径（单位：m） */
  RoutePoint() = default;
  RoutePoint(oneq::coordinate::LlaPositionDegM pos, double speed_mps_, double radius_m_)
      : position(pos), speed_mps(speed_mps_), radius_m(radius_m_) {}
};

/**
 * @brief 航路计划：按访问顺序排列的航点序列，相邻航点间以直线航段连接。
 */
using RoutePlan = std::vector<RoutePoint>;

}  // namespace navigation

#endif  // ONEQ_NAVIGATION_ROUTE_POINT_H_
