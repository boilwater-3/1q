/**
 * @file AreaCoveragePlanner.h
 * @brief 定义区域覆盖路径规划器。
 */

#ifndef ONEQ_NAVIGATION_AREA_COVERAGE_PLANNER_H_
#define ONEQ_NAVIGATION_AREA_COVERAGE_PLANNER_H_

#include "1q/api.hpp"
#include "1q/navigation/CoverageArea.h"
#include "1q/navigation/CoveragePlanConfig.h"
#include "1q/navigation/RoutePoint.h"

namespace navigation {

/**
 * @brief 区域覆盖路径规划器（首期：多边形牛耕式扫描、圆形单环/同心圆盘旋）。
 * @note 独立中立算法面，不绑定 flight_dynamic；输入非法时返回空计划并记录告警日志。
 */
class ONEQ_API AreaCoveragePlanner {
 public:
  /**
   * @brief 生成区域覆盖航路计划。
   * @param[in] area 覆盖区域（多边形 / 圆形）。
   * @param[in] config 覆盖参数。
   * @return 按访问顺序排列的航点序列；输入非法（顶点不足、间距/半径非正、
   *         模式与区域形态不匹配等）时为空。
   */
  RoutePlan Plan(const CoverageArea& area, const CoveragePlanConfig& config) const;
};

}  // namespace navigation

#endif  // ONEQ_NAVIGATION_AREA_COVERAGE_PLANNER_H_
