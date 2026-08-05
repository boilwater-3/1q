/**
 * @file CoverageArea.h
 * @brief 定义覆盖区域类型（多边形 / 圆形）。
 */

#ifndef ONEQ_NAVIGATION_COVERAGE_AREA_H_
#define ONEQ_NAVIGATION_COVERAGE_AREA_H_

#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace navigation {

/**
 * @brief 覆盖区域形态。
 */
enum class ONEQ_API CoverageAreaKind {
  kPolygon = 0, /**< 多边形区域 */
  kCircle,      /**< 圆形区域 */
};

/**
 * @brief 多边形覆盖区域（LLA 顶点序列，度制，首尾不闭合）。
 */
struct ONEQ_API PolygonalArea {
  std::vector<oneq::coordinate::LlaPositionDegM> vertices{}; /**< 顶点序列，按绕行方向排列 */
};

/**
 * @brief 圆形覆盖区域（圆心 LLA + 半径）。
 */
struct ONEQ_API CircularArea {
  oneq::coordinate::LlaPositionDegM center{}; /**< 圆心（度制 LLA） */
  double radius_m{0.0};                       /**< 半径（单位：m） */
};

/**
 * @brief 覆盖区域（多边形 | 圆形，按 kind 判别载荷）。
 * @note 公共头守 C++11 子集，不使用 variant 容器；kind 与载荷一一对应，
 *       未使用载荷保持默认构造。
 */
struct ONEQ_API CoverageArea {
  CoverageAreaKind kind{CoverageAreaKind::kPolygon}; /**< 区域形态 */
  PolygonalArea polygon{};                           /**< 多边形载荷（kind == kPolygon 时有效） */
  CircularArea circle{};                             /**< 圆形载荷（kind == kCircle 时有效） */
};

}  // namespace navigation

#endif  // ONEQ_NAVIGATION_COVERAGE_AREA_H_
