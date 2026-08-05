/**
 * @file CoveragePlanConfig.h
 * @brief 定义区域覆盖规划参数（首期最小面）。
 */

#ifndef ONEQ_NAVIGATION_COVERAGE_PLAN_CONFIG_H_
#define ONEQ_NAVIGATION_COVERAGE_PLAN_CONFIG_H_

#include <cstddef>

#include "1q/api.hpp"

namespace navigation {

/**
 * @brief 覆盖模式。
 */
enum class ONEQ_API CoverageMode {
  kScan = 0, /**< 扫描模式：多边形区域牛耕式扫描线覆盖 */
  kOrbit,    /**< 盘旋模式：圆形区域单环/同心圆盘旋 */
};

/**
 * @brief 区域覆盖规划参数。
 * @note 模式与区域形态必须匹配（kScan ↔ 多边形、kOrbit ↔ 圆形），
 *       不匹配时规划返回空计划并记录告警。
 */
struct ONEQ_API CoveragePlanConfig {
  CoverageMode mode{CoverageMode::kScan}; /**< 覆盖模式 */
  double scan_heading_deg{0.0};           /**< 扫描航向（单位：deg，0 = 扫描线沿正东） */
  double scan_spacing_m{0.0};             /**< 扫描间距（单位：m，相邻扫描线间隔，须 > 0） */
  double altitude_m{0.0};                 /**< 计划高度（单位：m，写入每个航点） */
  double speed_mps{0.0};                  /**< 计划速度（单位：m/s，0 = 执行侧默认，写入每个航点） */
  double arrival_radius_m{0.0};           /**< 到达半径（单位：m，写入每个航点） */
  std::size_t orbit_segments{8U};         /**< 每环取点数（盘旋模式，须 ≥ 3） */
  std::size_t orbit_rings{1U};            /**< 同心圆环数（盘旋模式，1 = 单环，须 ≥ 1） */
};

}  // namespace navigation

#endif  // ONEQ_NAVIGATION_COVERAGE_PLAN_CONFIG_H_
