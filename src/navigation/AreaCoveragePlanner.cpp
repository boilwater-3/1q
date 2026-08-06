/**
 * @file AreaCoveragePlanner.cpp
 * @brief AreaCoveragePlanner 实现：多边形牛耕式扫描线与圆形单环/同心圆盘旋规划。
 */

#include "1q/navigation/AreaCoveragePlanner.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "1q/coordinate/position_transform.h"
#include "common/logging/ProjectLog.h"
#include "common/numerics/Constants.h"

namespace navigation {

namespace {

using oneq::coordinate::EcefPositionM;
using oneq::coordinate::EnuPositionM;
using oneq::coordinate::LlaPositionDegM;

constexpr double kScanLineToleranceM = 1e-9;

// 把多边形顶点投影到以顶点平均为原点的 ENU 平面（东/北二维），返回平面坐标。
// 任一点转换失败则整体失败。
bool ProjectPolygonToPlane(const std::vector<LlaPositionDegM>& vertices,
                           LlaPositionDegM* origin,
                           std::vector<Eigen::Vector2d>* points) {
  // 原点取经纬度简单平均（首期不按面积加权）。
  LlaPositionDegM centroid{};
  for (const auto& vertex : vertices) {
    centroid.latitude_deg += vertex.latitude_deg / static_cast<double>(vertices.size());
    centroid.longitude_deg += vertex.longitude_deg / static_cast<double>(vertices.size());
    centroid.altitude_m += vertex.altitude_m / static_cast<double>(vertices.size());
  }
  *origin = centroid;
  points->clear();
  points->reserve(vertices.size());
  for (const auto& vertex : vertices) {
    EnuPositionM enu{};
    if (!oneq::coordinate::TryLlaToEnu(vertex, centroid, &enu)) {
      return false;
    }
    points->push_back(Eigen::Vector2d(enu.east_m, enu.north_m));
  }
  return true;
}

// 求扫描线（v = v_line）与多边形边的全部交点 x 坐标。
// 半开规则：边两端点一个严格在上、一个不在上时计一次交点，天然跳过水平边，
// 且扫描线恰好穿过顶点时按 even-odd 语义只计一次。
std::vector<double> ScanLineCrossings(const std::vector<Eigen::Vector2d>& polygon,
                                      double v_line) {
  std::vector<double> xs;
  const std::size_t vertex_count = polygon.size();
  for (std::size_t i = 0; i < vertex_count; ++i) {
    const Eigen::Vector2d& a = polygon[i];
    const Eigen::Vector2d& b = polygon[(i + 1U) % vertex_count];
    const bool a_above = a.y() > v_line;
    const bool b_above = b.y() > v_line;
    if (a_above == b_above) {
      continue;
    }
    const double t = (v_line - a.y()) / (b.y() - a.y());
    xs.push_back(a.x() + t * (b.x() - a.x()));
  }
  return xs;
}

// 把扫描系平面点反变换回 LLA 并写入计划（高度/速度/到达半径取自配置）。
void AppendRoutePoint(RoutePlan* plan, const Eigen::Vector2d& frame_point,
                      const LlaPositionDegM& origin, double heading_rad,
                      const CoveragePlanConfig& config) {
  const double cos_h = std::cos(heading_rad);
  const double sin_h = std::sin(heading_rad);
  const double east = frame_point.x() * cos_h - frame_point.y() * sin_h;
  const double north = frame_point.x() * sin_h + frame_point.y() * cos_h;
  EnuPositionM enu{east, north, 0.0};
  EcefPositionM ecef{};
  LlaPositionDegM lla{};
  if (!oneq::coordinate::TryEnuToEcef(enu, origin, &ecef) ||
      !oneq::coordinate::TryEcefToLla(ecef, &lla)) {
    // 标识：航点坐标回变换失败时跳过该航点继续规划；多为坐标非法或
    //       投影参考原点异常。
    PROJECT_LOG_WARN("navigation: 航点回变换失败，跳过 (east={}, north={})", east, north);
    return;
  }
  lla.altitude_m = config.altitude_m;
  plan->push_back(RoutePoint{lla, config.speed_mps, config.arrival_radius_m});
}

RoutePlan PlanPolygonScan(const PolygonalArea& polygon, const CoveragePlanConfig& config) {
  if (polygon.vertices.size() < 3U) {
    // 标识：覆盖规划前置校验——多边形至少 3 个顶点才能生成扫描计划。
    PROJECT_LOG_WARN("navigation: 多边形顶点数不足（{}），需要 >= 3", polygon.vertices.size());
    return {};
  }
  if (!(config.scan_spacing_m > 0.0)) {
    // 标识：覆盖规划前置校验——扫描间距必须为正，否则无法布设扫描线。
    PROJECT_LOG_WARN("navigation: 扫描间距非正（{}），无法规划", config.scan_spacing_m);
    return {};
  }
  if (!std::isfinite(config.scan_heading_deg)) {
    // 标识：覆盖规划前置校验——扫描航向必须是有限值，否则无法规划。
    PROJECT_LOG_WARN("navigation: 扫描航向非有限值（{}），无法规划", config.scan_heading_deg);
    return {};
  }
  for (const auto& vertex : polygon.vertices) {
    if (!oneq::coordinate::IsValid(vertex)) {
      // 标识：覆盖规划前置校验——任一顶点坐标非法则无法规划。
      PROJECT_LOG_WARN("navigation: 多边形顶点 LLA 非法，无法规划");
      return {};
    }
  }

  LlaPositionDegM origin{};
  std::vector<Eigen::Vector2d> plane_points;
  if (!ProjectPolygonToPlane(polygon.vertices, &origin, &plane_points)) {
    // 标识：几何投影失败——多边形无法投影到 ENU 平面时无法规划。
    PROJECT_LOG_WARN("navigation: 多边形投影到 ENU 平面失败，无法规划");
    return {};
  }

  // 旋转到扫描系：u 沿扫描航向，v 为扫描线法向（扫描线为 v = const）。
  const double heading_rad =
      oneq::common::numerics::DegToRad(config.scan_heading_deg);
  const double cos_h = std::cos(heading_rad);
  const double sin_h = std::sin(heading_rad);
  std::vector<Eigen::Vector2d> frame_points;
  frame_points.reserve(plane_points.size());
  for (const auto& point : plane_points) {
    frame_points.push_back(Eigen::Vector2d(
        point.x() * cos_h + point.y() * sin_h, -point.x() * sin_h + point.y() * cos_h));
  }

  double v_min = std::numeric_limits<double>::infinity();
  double v_max = -std::numeric_limits<double>::infinity();
  for (const auto& point : frame_points) {
    v_min = std::min(v_min, point.y());
    v_max = std::max(v_max, point.y());
  }

  RoutePlan plan;
  bool forward = true;  // 首条扫描线沿 +u 方向。
  // 生成一条扫描线航点（段内方向由扫描线方向决定；反向线自右向左取段，
  // 保证与上一条扫描线的过渡最短）。
  const auto emit_scan_line = [&](double v_line) {
    std::vector<double> xs = ScanLineCrossings(frame_points, v_line);
    if (xs.size() < 2U) {
      return;
    }
    std::sort(xs.begin(), xs.end());
    for (std::size_t i = 0; i + 1U < xs.size(); i += 2U) {
      const std::size_t lo = forward ? i : xs.size() - 2U - i;
      const std::size_t hi = lo + 1U;
      const double u_first = forward ? xs[lo] : xs[hi];
      const double u_second = forward ? xs[hi] : xs[lo];
      AppendRoutePoint(&plan, Eigen::Vector2d(u_first, v_line), origin, heading_rad, config);
      AppendRoutePoint(&plan, Eigen::Vector2d(u_second, v_line), origin, heading_rad, config);
    }
    forward = !forward;
  };
  // 首条扫描线取半间距偏移，规避扫描线恰好穿过多边形极值顶点；
  // 末条线若恰好落在 v_max（跨度恰为间距整数倍时），下压一个容差，
  // 避免与极值边重合产生零交点而被静默跳过。
  for (double v_line = v_min + 0.5 * config.scan_spacing_m;
       v_line <= v_max + kScanLineToleranceM; v_line += config.scan_spacing_m) {
    v_line = std::min(v_line, v_max - kScanLineToleranceM);
    emit_scan_line(v_line);
  }
  // 间距大于区域跨度时扫描线循环为空：退回区域中线单条扫描线。
  // 中线也产生不了任何扫描段（退化多边形，如近共线顶点）→ 空计划 + 告警。
  if (plan.empty()) {
    emit_scan_line(0.5 * (v_min + v_max));
    if (plan.empty()) {
      // 标识：退化形状检测——近共线顶点导致无法生成扫描段，返回空计划。
      PROJECT_LOG_WARN("navigation: 多边形退化为不可覆盖形状（无法生成扫描段），无法规划");
      return {};
    }
  }
  return plan;
}

RoutePlan PlanCircleOrbit(const CircularArea& circle, const CoveragePlanConfig& config) {
  if (!(circle.radius_m > 0.0)) {
    // 标识：覆盖规划前置校验——圆形半径必须为正。
    PROJECT_LOG_WARN("navigation: 圆形半径非正（{}），无法规划", circle.radius_m);
    return {};
  }
  if (!oneq::coordinate::IsValid(circle.center)) {
    // 标识：覆盖规划前置校验——圆心坐标非法时无法规划。
    PROJECT_LOG_WARN("navigation: 圆心 LLA 非法，无法规划");
    return {};
  }
  if (config.orbit_segments < 3U) {
    // 标识：覆盖规划前置校验——每环至少 3 个取点才能构成环。
    PROJECT_LOG_WARN("navigation: 每环取点数不足（{}），需要 >= 3", config.orbit_segments);
    return {};
  }
  if (config.orbit_rings < 1U) {
    // 标识：覆盖规划前置校验——同心圆环数至少为 1。
    PROJECT_LOG_WARN("navigation: 同心圆环数非法（{}），需要 >= 1", config.orbit_rings);
    return {};
  }

  const double kTwoPi = 2.0 * static_cast<double>(oneq::common::numerics::kPi);
  RoutePlan plan;
  // 外 → 内逐环：第 j 环半径 = r * (num_rings - j + 1) / num_rings。
  for (std::size_t ring = 0; ring < config.orbit_rings; ++ring) {
    const double radius =
        circle.radius_m *
        static_cast<double>(config.orbit_rings - ring) /
        static_cast<double>(config.orbit_rings);
    for (std::size_t k = 0; k < config.orbit_segments; ++k) {
      // 逆时针（CCW），自正北方向起均匀取点。
      const double theta =
          kTwoPi * static_cast<double>(k) / static_cast<double>(config.orbit_segments);
      const EnuPositionM enu{radius * std::sin(theta), radius * std::cos(theta), 0.0};
      EcefPositionM ecef{};
      LlaPositionDegM lla{};
      if (!oneq::coordinate::TryEnuToEcef(enu, circle.center, &ecef) ||
          !oneq::coordinate::TryEcefToLla(ecef, &lla)) {
        // 标识：盘旋航点坐标回变换失败时跳过该航点继续规划。
        PROJECT_LOG_WARN("navigation: 盘旋航点回变换失败，跳过");
        continue;
      }
      lla.altitude_m = config.altitude_m;
      plan.push_back(RoutePoint{lla, config.speed_mps, config.arrival_radius_m});
    }
  }
  return plan;
}

}  // namespace

RoutePlan AreaCoveragePlanner::Plan(const CoverageArea& area,
                                    const CoveragePlanConfig& config) const {
  if (config.mode == CoverageMode::kScan) {
    if (area.kind != CoverageAreaKind::kPolygon) {
      // 标识：模式-区域匹配校验——扫描模式忽略圆形区域。
      PROJECT_LOG_WARN("navigation: 扫描模式要求多边形区域，忽略圆形");
      return {};
    }
    return PlanPolygonScan(area.polygon, config);
  }
  if (area.kind != CoverageAreaKind::kCircle) {
    // 标识：模式-区域匹配校验——盘旋模式忽略多边形区域。
    PROJECT_LOG_WARN("navigation: 盘旋模式要求圆形区域，忽略多边形");
    return {};
  }
  return PlanCircleOrbit(area.circle, config);
}

}  // namespace navigation
