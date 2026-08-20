// Copyright 2026. All Rights Reserved.
//
// @file navigation_area_coverage_planner_test.cpp
// @brief 验证区域覆盖规划器：多边形牛耕式扫描与圆形盘旋的几何不变量。
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/navigation/navigation.hpp"

namespace navigation {
namespace {

using oneq::coordinate::EcefPositionM;
using oneq::coordinate::EnuPositionM;
using oneq::coordinate::LlaPositionDegM;

constexpr double kPi = 3.14159265358979323846;

LlaPositionDegM MakeLla(double latitude_deg, double longitude_deg, double altitude_m) {
  LlaPositionDegM lla;
  lla.latitude_deg = latitude_deg;
  lla.longitude_deg = longitude_deg;
  lla.altitude_m = altitude_m;
  return lla;
}

// 以 origin 为参考点，把 ENU 偏移换算为 LLA（与规划器同路径的几何生成助手）。
LlaPositionDegM OffsetLla(const LlaPositionDegM& origin, double east_m, double north_m) {
  EnuPositionM enu{east_m, north_m, 0.0};
  EcefPositionM ecef{};
  LlaPositionDegM lla{};
  EXPECT_TRUE(oneq::coordinate::TryEnuToEcef(enu, origin, &ecef));
  EXPECT_TRUE(oneq::coordinate::TryEcefToLla(ecef, &lla));
  return lla;
}

// ECEF 欧氏距离（独立于规划器实现的几何校验路径）。
double EcefDistanceM(const LlaPositionDegM& a, const LlaPositionDegM& b) {
  EcefPositionM ea{};
  EcefPositionM eb{};
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(a, &ea));
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(b, &eb));
  const double dx = ea.x_m - eb.x_m;
  const double dy = ea.y_m - eb.y_m;
  const double dz = ea.z_m - eb.z_m;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 以 origin 为参考点的 ENU 平面投影。
std::pair<double, double> ToEnu(const LlaPositionDegM& lla, const LlaPositionDegM& origin) {
  EnuPositionM enu{};
  EXPECT_TRUE(oneq::coordinate::TryLlaToEnu(lla, origin, &enu));
  return {enu.east_m, enu.north_m};
}

// 射线法点在多边形内判断（ENU 平面，首期测试用简单矩形/正方形）。
bool PointInPolygon(const std::vector<std::pair<double, double>>& polygon, double x,
                    double y) {
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1U; i < polygon.size(); j = i++) {
    const auto& pi = polygon[i];
    const auto& pj = polygon[j];
    if ((pi.second > y) != (pj.second > y) &&
        x < (pj.first - pi.first) * (y - pi.second) / (pj.second - pi.second) + pi.first) {
      inside = !inside;
    }
  }
  return inside;
}

// 1 km × 1 km 正方形区域（中心 30N/120E，CCW 顶点序）。
PolygonalArea MakeSquareArea() {
  const LlaPositionDegM center = MakeLla(30.0, 120.0, 0.0);
  PolygonalArea area;
  area.vertices = {OffsetLla(center, 500.0, 500.0), OffsetLla(center, -500.0, 500.0),
                   OffsetLla(center, -500.0, -500.0), OffsetLla(center, 500.0, -500.0)};
  return area;
}

CoverageArea MakePolygonArea(const PolygonalArea& polygon) {
  CoverageArea area;
  area.kind = CoverageAreaKind::kPolygon;
  area.polygon = polygon;
  return area;
}

CoverageArea MakeCircleArea(const CircularArea& circle) {
  CoverageArea area;
  area.kind = CoverageAreaKind::kCircle;
  area.circle = circle;
  return area;
}

CoveragePlanConfig MakeScanConfig() {
  CoveragePlanConfig config;
  config.mode = CoverageMode::kScan;
  config.scan_heading_deg = 0.0;
  config.scan_spacing_m = 200.0;
  config.altitude_m = 3000.0;
  config.speed_mps = 150.0;
  config.arrival_radius_m = 100.0;
  return config;
}

TEST(AreaCoveragePlannerTest, SquareScanProducesBoustrophedonLines) {
  const LlaPositionDegM center = MakeLla(30.0, 120.0, 0.0);
  const CoverageArea area = MakePolygonArea(MakeSquareArea());
  const RoutePlan plan = AreaCoveragePlanner().Plan(area, MakeScanConfig());

  // 1 km 跨度 / 200 m 间距 → 5 条扫描线 × 2 航点。
  ASSERT_EQ(plan.size(), 10U);

  // 配置值写入每个航点。
  for (const RoutePoint& point : plan) {
    EXPECT_DOUBLE_EQ(point.position.altitude_m, 3000.0);
    EXPECT_DOUBLE_EQ(point.speed_mps, 150.0);
    EXPECT_DOUBLE_EQ(point.radius_m, 100.0);
  }

  // 相邻两航点属于同一条扫描线：段长 ≈ 区域跨度 1000 m。
  for (std::size_t k = 0; k + 1U < plan.size(); k += 2U) {
    EXPECT_NEAR(EcefDistanceM(plan[k].position, plan[k + 1U].position), 1000.0, 1.0);
  }

  // 相邻扫描线中点间距 ≈ 配置间距 200 m。
  for (std::size_t k = 2; k + 1U < plan.size(); k += 2U) {
    const LlaPositionDegM midpoint_a =
        MakeLla(0.5 * (plan[k - 2U].position.latitude_deg + plan[k - 1U].position.latitude_deg),
                0.5 * (plan[k - 2U].position.longitude_deg + plan[k - 1U].position.longitude_deg),
                0.0);
    const LlaPositionDegM midpoint_b =
        MakeLla(0.5 * (plan[k].position.latitude_deg + plan[k + 1U].position.latitude_deg),
                0.5 * (plan[k].position.longitude_deg + plan[k + 1U].position.longitude_deg),
                0.0);
    EXPECT_NEAR(EcefDistanceM(midpoint_a, midpoint_b), 200.0, 1.0);
  }

  // 牛耕式方向交替：沿正东扫描时，段内 east 增量符号逐线翻转。
  for (std::size_t k = 0; k + 1U < plan.size(); k += 2U) {
    const auto p0 = ToEnu(plan[k].position, center);
    const auto p1 = ToEnu(plan[k + 1U].position, center);
    const double delta_east = p1.first - p0.first;
    const bool forward = (k / 2U) % 2U == 0U;
    if (forward) {
      EXPECT_GT(delta_east, 0.0) << "line " << k / 2U << " should run eastward";
    } else {
      EXPECT_LT(delta_east, 0.0) << "line " << k / 2U << " should run westward";
    }
  }

  // 扫描段中点位于多边形内（ENU 平面射线法）。段端点恰在多边形边上
  // （端点即扫描线与边的交点），故以段中点作为区域覆盖的判定点。
  LlaPositionDegM centroid;
  const std::vector<LlaPositionDegM>& vertices = area.polygon.vertices;
  for (const auto& vertex : vertices) {
    centroid.latitude_deg += vertex.latitude_deg / static_cast<double>(vertices.size());
    centroid.longitude_deg += vertex.longitude_deg / static_cast<double>(vertices.size());
    centroid.altitude_m += vertex.altitude_m / static_cast<double>(vertices.size());
  }
  std::vector<std::pair<double, double>> polygon;
  for (const auto& vertex : vertices) {
    polygon.push_back(ToEnu(vertex, centroid));
  }
  for (std::size_t k = 0; k + 1U < plan.size(); k += 2U) {
    const auto p0 = ToEnu(plan[k].position, centroid);
    const auto p1 = ToEnu(plan[k + 1U].position, centroid);
    EXPECT_TRUE(PointInPolygon(polygon, 0.5 * (p0.first + p1.first),
                               0.5 * (p0.second + p1.second)))
        << "scan line " << k / 2U << " midpoint outside polygon";
  }
}

TEST(AreaCoveragePlannerTest, HeadingRotatesScanLines) {
  const LlaPositionDegM center = MakeLla(30.0, 120.0, 0.0);
  const CoverageArea area = MakePolygonArea(MakeSquareArea());
  CoveragePlanConfig config = MakeScanConfig();
  config.scan_heading_deg = 90.0;

  const RoutePlan plan = AreaCoveragePlanner().Plan(area, config);
  ASSERT_EQ(plan.size(), 10U);

  // 90° 航向 → 扫描线沿南北：段内 east 基本不变、north 跨度 ≈ 1000 m。
  for (std::size_t k = 0; k + 1U < plan.size(); k += 2U) {
    const auto p0 = ToEnu(plan[k].position, center);
    const auto p1 = ToEnu(plan[k + 1U].position, center);
    EXPECT_NEAR(std::abs(p0.first - p1.first), 0.0, 1.0);
    EXPECT_NEAR(std::abs(p0.second - p1.second), 1000.0, 1.0);
  }
}

TEST(AreaCoveragePlannerTest, SpacingLargerThanExtentFallsBackToCenterLine) {
  const CoverageArea area = MakePolygonArea(MakeSquareArea());
  CoveragePlanConfig config = MakeScanConfig();
  config.scan_spacing_m = 5000.0;

  const RoutePlan plan = AreaCoveragePlanner().Plan(area, config);
  // 单条中线扫描线：2 个航点，段长 ≈ 1000 m。
  ASSERT_EQ(plan.size(), 2U);
  EXPECT_NEAR(EcefDistanceM(plan[0].position, plan[1].position), 1000.0, 1.0);
}

TEST(AreaCoveragePlannerTest, ConcavePolygonProducesMultipleSegmentsPerLine) {
  // 顶部带缺口的凹多边形（1200 m × 800 m 外框，缺口 400 m × 400 m 自顶切入）。
  // 缺口两侧的扫描线产生 4 个交点 → 每线 2 段，校验多段配对与反向段序。
  const LlaPositionDegM center = MakeLla(30.0, 120.0, 0.0);
  PolygonalArea polygon;
  const double kExtent = 600.0;
  const double kNotchHalf = 200.0;
  polygon.vertices = {OffsetLla(center, -kExtent, -400.0),
                      OffsetLla(center, kExtent, -400.0),
                      OffsetLla(center, kExtent, 400.0),
                      OffsetLla(center, kNotchHalf, 400.0),
                      OffsetLla(center, kNotchHalf, 0.0),
                      OffsetLla(center, -kNotchHalf, 0.0),
                      OffsetLla(center, -kNotchHalf, 400.0),
                      OffsetLla(center, -kExtent, 400.0)};
  const CoverageArea area = MakePolygonArea(polygon);

  CoveragePlanConfig config = MakeScanConfig();
  config.scan_spacing_m = 200.0;
  const RoutePlan plan = AreaCoveragePlanner().Plan(area, config);

  // 4 条扫描线：低区 2 条 × 1 段（1200 m），缺口区 2 条 × 2 段（400 m）。
  ASSERT_EQ(plan.size(), 12U);
  const double expected_lengths[6] = {1200.0, 1200.0, 400.0, 400.0, 400.0, 400.0};
  for (std::size_t k = 0; k + 1U < plan.size(); k += 2U) {
    EXPECT_NEAR(EcefDistanceM(plan[k].position, plan[k + 1U].position),
                expected_lengths[k / 2U], 1.0);
  }

  // 段内方向按扫描线翻转（+u / -u 交替），同一条线内多段方向一致；
  // 反向线自右向左取段。4 条线的段数依次为 1/1/2/2。
  const bool expected_forward[6] = {true, false, true, true, false, false};
  for (std::size_t k = 0; k + 1U < plan.size(); k += 2U) {
    const auto p0 = ToEnu(plan[k].position, center);
    const auto p1 = ToEnu(plan[k + 1U].position, center);
    const double delta_east = p1.first - p0.first;
    if (expected_forward[k / 2U]) {
      EXPECT_GT(delta_east, 0.0) << "segment " << k / 2U << " should run eastward";
    } else {
      EXPECT_LT(delta_east, 0.0) << "segment " << k / 2U << " should run westward";
    }
  }

  // 全部段中点位于多边形内（缺口内不得出现扫描段）。
  LlaPositionDegM centroid;
  const std::vector<LlaPositionDegM>& vertices = area.polygon.vertices;
  for (const auto& vertex : vertices) {
    centroid.latitude_deg += vertex.latitude_deg / static_cast<double>(vertices.size());
    centroid.longitude_deg += vertex.longitude_deg / static_cast<double>(vertices.size());
    centroid.altitude_m += vertex.altitude_m / static_cast<double>(vertices.size());
  }
  std::vector<std::pair<double, double>> polygon_enu;
  for (const auto& vertex : vertices) {
    polygon_enu.push_back(ToEnu(vertex, centroid));
  }
  for (std::size_t k = 0; k + 1U < plan.size(); k += 2U) {
    const auto p0 = ToEnu(plan[k].position, centroid);
    const auto p1 = ToEnu(plan[k + 1U].position, centroid);
    EXPECT_TRUE(PointInPolygon(polygon_enu, 0.5 * (p0.first + p1.first),
                               0.5 * (p0.second + p1.second)))
        << "segment " << k / 2U << " midpoint outside polygon";
  }
}

TEST(AreaCoveragePlannerTest, CircleSingleRingOrbit) {
  // 圆心高度与计划高度一致，保证 ECEF 距离校验纯水平（半径）语义。
  const LlaPositionDegM center = MakeLla(30.0, 120.0, 3000.0);
  CircularArea circle;
  circle.center = center;
  circle.radius_m = 500.0;
  const CoverageArea area = MakeCircleArea(circle);

  CoveragePlanConfig config;
  config.mode = CoverageMode::kOrbit;
  config.altitude_m = 3000.0;
  config.speed_mps = 120.0;
  config.arrival_radius_m = 80.0;
  config.orbit_segments = 8U;
  config.orbit_rings = 1U;

  const RoutePlan plan = AreaCoveragePlanner().Plan(area, config);
  ASSERT_EQ(plan.size(), 8U);

  // 每点距圆心 ≈ 半径；相邻点间距 ≈ 圆内接八边形边长。
  const double chord = 2.0 * 500.0 * std::sin(kPi / 8.0);
  for (std::size_t k = 0; k < plan.size(); ++k) {
    EXPECT_NEAR(EcefDistanceM(plan[k].position, center), 500.0, 1.0);
    EXPECT_DOUBLE_EQ(plan[k].position.altitude_m, 3000.0);
    EXPECT_DOUBLE_EQ(plan[k].speed_mps, 120.0);
    EXPECT_DOUBLE_EQ(plan[k].radius_m, 80.0);
    const LlaPositionDegM next = plan[(k + 1U) % plan.size()].position;
    EXPECT_NEAR(EcefDistanceM(plan[k].position, next), chord, 1.0);
  }

  // 起始点位于圆心正北。
  const auto first = ToEnu(plan[0].position, center);
  EXPECT_NEAR(first.first, 0.0, 1.0);
  EXPECT_NEAR(first.second, 500.0, 1.0);
}

TEST(AreaCoveragePlannerTest, CircleConcentricRingsDescendOutwardToInward) {
  // 圆心高度与计划高度一致（同上）。
  const LlaPositionDegM center = MakeLla(30.0, 120.0, 2000.0);
  CircularArea circle;
  circle.center = center;
  circle.radius_m = 400.0;
  const CoverageArea area = MakeCircleArea(circle);

  CoveragePlanConfig config;
  config.mode = CoverageMode::kOrbit;
  config.altitude_m = 2000.0;
  config.orbit_segments = 4U;
  config.orbit_rings = 2U;

  const RoutePlan plan = AreaCoveragePlanner().Plan(area, config);
  ASSERT_EQ(plan.size(), 8U);

  // 外 → 内：前 4 点半径 ≈ 400 m，后 4 点半径 ≈ 200 m。
  for (std::size_t k = 0; k < plan.size(); ++k) {
    const double expected_radius = k < 4U ? 400.0 : 200.0;
    EXPECT_NEAR(EcefDistanceM(plan[k].position, center), expected_radius, 1.0);
  }
}

TEST(AreaCoveragePlannerTest, InvalidInputsReturnEmptyPlan) {
  const AreaCoveragePlanner planner;

  // 顶点不足。
  PolygonalArea polygon;
  polygon.vertices = {MakeLla(30.0, 120.0, 0.0), MakeLla(30.0, 120.01, 0.0)};
  EXPECT_TRUE(planner.Plan(MakePolygonArea(polygon), MakeScanConfig()).empty());

  // 非法 LLA。
  polygon.vertices = {MakeLla(95.0, 120.0, 0.0), MakeLla(30.0, 120.01, 0.0),
                      MakeLla(30.0, 120.02, 0.0)};
  EXPECT_TRUE(planner.Plan(MakePolygonArea(polygon), MakeScanConfig()).empty());

  // 间距非正 / 非有限扫描航向。
  polygon.vertices = {MakeLla(30.0, 120.0, 0.0), MakeLla(30.0, 120.01, 0.0),
                      MakeLla(30.1, 120.01, 0.0)};
  CoveragePlanConfig config = MakeScanConfig();
  config.scan_spacing_m = 0.0;
  EXPECT_TRUE(planner.Plan(MakePolygonArea(polygon), config).empty());
  config = MakeScanConfig();
  config.scan_spacing_m = -100.0;
  EXPECT_TRUE(planner.Plan(MakePolygonArea(polygon), config).empty());
  config = MakeScanConfig();
  config.scan_heading_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(planner.Plan(MakePolygonArea(polygon), config).empty());

  // 退化多边形：全部顶点重合 → 任何扫描线都无交点 → 空计划 + 告警。
  polygon.vertices = {MakeLla(30.0, 120.0, 0.0), MakeLla(30.0, 120.0, 0.0),
                      MakeLla(30.0, 120.0, 0.0)};
  EXPECT_TRUE(planner.Plan(MakePolygonArea(polygon), MakeScanConfig()).empty());

  // 近共线条带（已知限制）：产生退化短段航路而非空计划（见 algorithms.md）。
  polygon.vertices = {MakeLla(30.0, 120.0, 0.0), MakeLla(30.0, 120.01, 0.0),
                      MakeLla(30.0, 120.02, 0.0)};
  EXPECT_FALSE(planner.Plan(MakePolygonArea(polygon), MakeScanConfig()).empty());

  // 模式与区域形态不匹配。
  CircularArea circle;
  circle.center = MakeLla(30.0, 120.0, 0.0);
  circle.radius_m = 500.0;
  EXPECT_TRUE(planner.Plan(MakeCircleArea(circle), MakeScanConfig()).empty());
  CoveragePlanConfig orbit_config;
  orbit_config.mode = CoverageMode::kOrbit;
  EXPECT_TRUE(planner.Plan(MakePolygonArea(polygon), orbit_config).empty());

  // 圆形半径非正 / 圆心非法 / 取点数不足 / 环数为零。
  circle.radius_m = 0.0;
  EXPECT_TRUE(planner.Plan(MakeCircleArea(circle), orbit_config).empty());
  circle.radius_m = -100.0;
  EXPECT_TRUE(planner.Plan(MakeCircleArea(circle), orbit_config).empty());
  circle.radius_m = 500.0;
  circle.center = MakeLla(95.0, 120.0, 0.0);
  EXPECT_TRUE(planner.Plan(MakeCircleArea(circle), orbit_config).empty());
  circle.center = MakeLla(30.0, 120.0, 0.0);
  orbit_config.orbit_segments = 2U;
  EXPECT_TRUE(planner.Plan(MakeCircleArea(circle), orbit_config).empty());
  orbit_config.orbit_segments = 8U;
  orbit_config.orbit_rings = 0U;
  EXPECT_TRUE(planner.Plan(MakeCircleArea(circle), orbit_config).empty());
}

}  // namespace
}  // namespace navigation
