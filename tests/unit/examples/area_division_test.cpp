/**
 * @file area_division_test.cpp
 * @brief 编队区域切分（examples/scenes/area_division.*）单元测试。
 *
 * 覆盖：多边形等宽条带切分（跨度/扫描线并集/航向旋转）、圆形同心环切分
 * （环半径序列 + 每机单环）、单机整区原样返回、非法输入报错。断言全部为
 * 手算几何不变量（ENU 独立换算 + ECEF 距离），与 navigation 规划器单测
 * 同风格。
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/navigation/AreaCoveragePlanner.h"
#include "scenes/area_division.h"

namespace component_attachment {
namespace app {
namespace {

using oneq::coordinate::EcefPositionM;
using oneq::coordinate::EnuPositionM;
using oneq::coordinate::LlaPositionDegM;

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

// ECEF 欧氏距离（独立于实现几何的校验路径）。
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

// 以 origin 为参考点的 ENU 平面投影（east, north）。
std::pair<double, double> ToEnu(const LlaPositionDegM& lla, const LlaPositionDegM& origin) {
  EnuPositionM enu{};
  EXPECT_TRUE(oneq::coordinate::TryLlaToEnu(lla, origin, &enu));
  return {enu.east_m, enu.north_m};
}

// 子区域顶点 ENU 极值（east/north 各 [min, max]，以 origin 为参考点——
// 与切分前的原区域中心同参考系，便于断言条带边界）。
struct EnuExtents {
  double east_min, east_max, north_min, north_max;
};
EnuExtents ExtentsOf(const navigation::CoverageArea& area, const LlaPositionDegM& origin) {
  EnuExtents extents{1e9, -1e9, 1e9, -1e9};
  for (const auto& vertex : area.polygon.vertices) {
    const auto [east, north] = ToEnu(vertex, origin);
    extents.east_min = std::min(extents.east_min, east);
    extents.east_max = std::max(extents.east_max, east);
    extents.north_min = std::min(extents.north_min, north);
    extents.north_max = std::max(extents.north_max, north);
  }
  return extents;
}

// 2 km（东西）× 1 km（南北）矩形区域（中心 30N/120E，ENU CCW 顶点序）。
navigation::CoverageArea MakeRectangleArea() {
  const LlaPositionDegM center = MakeLla(30.0, 120.0, 0.0);
  navigation::CoverageArea area;
  area.kind = navigation::CoverageAreaKind::kPolygon;
  area.polygon.vertices = {OffsetLla(center, -1000.0, -500.0),
                           OffsetLla(center, 1000.0, -500.0),
                           OffsetLla(center, 1000.0, 500.0),
                           OffsetLla(center, -1000.0, 500.0)};
  return area;
}

// 1 km × 1 km 正方形区域（中心 30N/120E）。
navigation::CoverageArea MakeSquareArea() {
  const LlaPositionDegM center = MakeLla(30.0, 120.0, 0.0);
  navigation::CoverageArea area;
  area.kind = navigation::CoverageAreaKind::kPolygon;
  area.polygon.vertices = {OffsetLla(center, 500.0, 500.0),
                           OffsetLla(center, -500.0, 500.0),
                           OffsetLla(center, -500.0, -500.0),
                           OffsetLla(center, 500.0, -500.0)};
  return area;
}

navigation::CoverageArea MakeCircleArea(double radius_m) {
  navigation::CoverageArea area;
  area.kind = navigation::CoverageAreaKind::kCircle;
  area.circle.center = MakeLla(30.0, 120.0, 0.0);
  area.circle.radius_m = radius_m;
  return area;
}

navigation::CoveragePlanConfig MakeScanConfig() {
  navigation::CoveragePlanConfig config;
  config.mode = navigation::CoverageMode::kScan;
  config.scan_heading_deg = 0.0;
  config.scan_spacing_m = 250.0;
  config.altitude_m = 3000.0;
  config.speed_mps = 150.0;
  config.arrival_radius_m = 100.0;
  return config;
}

navigation::CoveragePlanConfig MakeOrbitConfig() {
  navigation::CoveragePlanConfig config;
  config.mode = navigation::CoverageMode::kOrbit;
  config.altitude_m = 400.0;
  config.speed_mps = 50.0;
  config.arrival_radius_m = 200.0;
  config.orbit_segments = 8U;
  config.orbit_rings = 2U;  // 传入多环：切分应强制每机单环
  return config;
}

TEST(AreaDivisionTest, RectangleSplitsIntoEqualWidthStrips) {
  // 2 km × 1 km 矩形、航向 0（扫描线沿东西）→ v 法向 = 南北，2 机 →
  // 南/北各半：每机子区域 1 km × 500 m。间距 250 m → 每机 500 m 跨度
  // 内 2 条扫描线 × 2 端点 = 4 航点；两机扫描线并集 = 整区规划的 4 条线
  // （偏移 −375/−125/+125/+375 m）。
  const navigation::CoverageArea area = MakeRectangleArea();
  const navigation::CoveragePlanConfig config = MakeScanConfig();

  const FormationDivisionResult division = DivideArea(area, config, 2U);
  ASSERT_TRUE(division.ok) << division.error;
  ASSERT_EQ(division.sub_areas.size(), 2U);
  ASSERT_EQ(division.sub_configs.size(), 2U);

  // 每机子区域为矩形（4 顶点），条带边界与整区中心对齐（南北各半）。
  ASSERT_EQ(division.sub_areas[0].polygon.vertices.size(), 4U);
  ASSERT_EQ(division.sub_areas[1].polygon.vertices.size(), 4U);
  const LlaPositionDegM center = MakeLla(30.0, 120.0, 0.0);
  const EnuExtents south = ExtentsOf(division.sub_areas[0], center);
  const EnuExtents north = ExtentsOf(division.sub_areas[1], center);
  EXPECT_NEAR(south.north_min, -500.0, 1.0);
  EXPECT_NEAR(south.north_max, 0.0, 1.0);
  EXPECT_NEAR(north.north_min, 0.0, 1.0);
  EXPECT_NEAR(north.north_max, 500.0, 1.0);
  EXPECT_NEAR(south.east_min, -1000.0, 1.0);
  EXPECT_NEAR(south.east_max, 1000.0, 1.0);
  EXPECT_NEAR(north.east_min, -1000.0, 1.0);
  EXPECT_NEAR(north.east_max, 1000.0, 1.0);

  // 逐条带交给 AreaCoveragePlanner：各 2 条扫描线 × 2 端点 = 4 航点；
  // 扫描线偏移并集 == 整区规划（−375/−125/+125/+375 m，容差 1 m）。
  std::vector<double> strip_line_offsets;
  std::vector<double> whole_line_offsets;
  const navigation::AreaCoveragePlanner planner;
  const navigation::RoutePlan whole_plan = planner.Plan(area, config);
  for (std::size_t i = 0; i + 1U < whole_plan.size(); i += 2U) {
    const auto [east, north] = ToEnu(whole_plan[i].position, center);
    (void)east;
    whole_line_offsets.push_back(north);
  }
  for (const auto& sub_area : division.sub_areas) {
    const navigation::RoutePlan sub_plan = planner.Plan(sub_area, division.sub_configs[0]);
    ASSERT_EQ(sub_plan.size(), 4U);
    for (std::size_t i = 0; i + 1U < sub_plan.size(); i += 2U) {
      const auto [east, north] = ToEnu(sub_plan[i].position, center);
      (void)east;
      strip_line_offsets.push_back(north);
    }
    // 配置值写入每个航点（高度/速度/到达半径）。
    for (const auto& wp : sub_plan) {
      EXPECT_DOUBLE_EQ(wp.position.altitude_m, config.altitude_m);
      EXPECT_DOUBLE_EQ(wp.speed_mps, config.speed_mps);
      EXPECT_DOUBLE_EQ(wp.radius_m, config.arrival_radius_m);
    }
  }
  ASSERT_EQ(whole_line_offsets.size(), 4U);
  ASSERT_EQ(strip_line_offsets.size(), 4U);
  for (const double expected : whole_line_offsets) {
    bool found = false;
    for (const double actual : strip_line_offsets) {
      if (std::abs(actual - expected) < 1.0) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "strip plans miss whole-plan scan line at offset " << expected;
  }
}

TEST(AreaDivisionTest, HeadingRotatesStripBoundaries) {
  // 1 km × 1 km 正方形、航向 90（扫描线沿南北）→ v 法向 = 东西，2 机 →
  // 西/东各半：每机子区域 500 m × 1 km。
  const navigation::CoverageArea area = MakeSquareArea();
  navigation::CoveragePlanConfig config = MakeScanConfig();
  config.scan_heading_deg = 90.0;

  const FormationDivisionResult division = DivideArea(area, config, 2U);
  ASSERT_TRUE(division.ok) << division.error;
  const EnuExtents west = ExtentsOf(division.sub_areas[0], MakeLla(30.0, 120.0, 0.0));
  const EnuExtents east = ExtentsOf(division.sub_areas[1], MakeLla(30.0, 120.0, 0.0));
  EXPECT_NEAR(west.east_min, 0.0, 1.0);
  EXPECT_NEAR(west.east_max, 500.0, 1.0);
  EXPECT_NEAR(east.east_min, -500.0, 1.0);
  EXPECT_NEAR(east.east_max, 0.0, 1.0);
  EXPECT_NEAR(west.north_min, -500.0, 1.0);
  EXPECT_NEAR(west.north_max, 500.0, 1.0);
  EXPECT_NEAR(east.north_min, -500.0, 1.0);
  EXPECT_NEAR(east.north_max, 500.0, 1.0);
}

TEST(AreaDivisionTest, CircleSplitsIntoConcentricRings) {
  // 半径 2000 m 圆、3 机 → 环半径 2000/1333.33/666.67（外 → 内算术均匀，
  // 主机最外环）；每机强制单环（orbit_rings=1），其余参数透传。
  const navigation::CoverageArea area = MakeCircleArea(2000.0);
  const navigation::CoveragePlanConfig config = MakeOrbitConfig();

  const FormationDivisionResult division = DivideArea(area, config, 3U);
  ASSERT_TRUE(division.ok) << division.error;
  ASSERT_EQ(division.sub_areas.size(), 3U);
  ASSERT_EQ(division.sub_configs.size(), 3U);

  const double expected_radii[3] = {2000.0, 2000.0 * 2.0 / 3.0, 2000.0 / 3.0};
  const navigation::AreaCoveragePlanner planner;
  for (std::size_t j = 0; j < 3U; ++j) {
    ASSERT_EQ(division.sub_areas[j].kind, navigation::CoverageAreaKind::kCircle);
    EXPECT_NEAR(division.sub_areas[j].circle.radius_m, expected_radii[j], 1e-6);
    EXPECT_EQ(division.sub_configs[j].orbit_rings, 1U);
    EXPECT_EQ(division.sub_configs[j].orbit_segments, config.orbit_segments);
    EXPECT_DOUBLE_EQ(division.sub_configs[j].altitude_m, config.altitude_m);
    // 每机单环 8 段 = 8 航点，均落在环半径附近（ECEF 距离校验）。
    const navigation::RoutePlan plan =
        planner.Plan(division.sub_areas[j], division.sub_configs[j]);
    ASSERT_EQ(plan.size(), config.orbit_segments);
    for (const auto& wp : plan) {
      // 高度元数据不参与水平半径校验（航点 alt = 配置 400 m，直接 ECEF
      // 距离会虚增 ~40 m）：先归零再量弦长。
      LlaPositionDegM ground = wp.position;
      ground.altitude_m = 0.0;
      EXPECT_NEAR(EcefDistanceM(ground, area.circle.center), expected_radii[j], 1.0);
    }
  }
}

TEST(AreaDivisionTest, SingleAircraftReturnsWholeArea) {
  // 单机编队：整区原样返回（多边形顶点逐点一致；圆形半径一致），配置透传
  // （含 orbit_rings 不强制——单机不切分，环数由调用方配置决定）。
  const navigation::CoverageArea polygon = MakeSquareArea();
  const FormationDivisionResult scan_division = DivideArea(polygon, MakeScanConfig(), 1U);
  ASSERT_TRUE(scan_division.ok) << scan_division.error;
  ASSERT_EQ(scan_division.sub_areas.size(), 1U);
  ASSERT_EQ(scan_division.sub_areas[0].polygon.vertices.size(),
            polygon.polygon.vertices.size());
  for (std::size_t i = 0U; i < polygon.polygon.vertices.size(); ++i) {
    EXPECT_NEAR(EcefDistanceM(scan_division.sub_areas[0].polygon.vertices[i],
                              polygon.polygon.vertices[i]),
                0.0, 1e-6);
  }

  const navigation::CoverageArea circle = MakeCircleArea(2000.0);
  const FormationDivisionResult orbit_division = DivideArea(circle, MakeOrbitConfig(), 1U);
  ASSERT_TRUE(orbit_division.ok) << orbit_division.error;
  ASSERT_EQ(orbit_division.sub_areas.size(), 1U);
  EXPECT_NEAR(orbit_division.sub_areas[0].circle.radius_m, 2000.0, 1e-6);
  EXPECT_EQ(orbit_division.sub_configs[0].orbit_rings, 2U);  // 原样透传
}

TEST(AreaDivisionTest, IrregularConvexPolygonSplitsIntoSlantedStrips) {
  // 凸不规则多边形（梯形，两腰斜率不同 → 扫描线宽度随 v 线性变化）：
  // 顶点 ENU (−600,1000)/(1200,1000)/(2000,−1000)/(0,−1000)（中心 30N/120E）。
  // 左腰 x_L(y) = −600 + 0.3·(1000−y)、右腰 x_R(y) = 1200 + 0.4·(1000−y)，
  // 线宽 width(y) = 1800 + 0.1·(1000−y)。2 机 → 南北两半（各 4 顶点）：
  // 下条带 east ∈ [0, 2000]、上条带 east ∈ [−600, 1600]（y=0 处腰交点
  // x_L = −300、x_R = 1600）。间距 300 m → 每机 3 线 × 2 = 6 航点；
  // 上条带首线 y=150：x_L = −345、x_R = 1540，线宽 1885 m。
  const LlaPositionDegM center = MakeLla(30.0, 120.0, 0.0);
  navigation::CoverageArea area;
  area.kind = navigation::CoverageAreaKind::kPolygon;
  area.polygon.vertices = {OffsetLla(center, -600.0, 1000.0),
                           OffsetLla(center, 1200.0, 1000.0),
                           OffsetLla(center, 2000.0, -1000.0),
                           OffsetLla(center, 0.0, -1000.0)};
  navigation::CoveragePlanConfig config = MakeScanConfig();
  config.scan_spacing_m = 300.0;

  const FormationDivisionResult division = DivideArea(area, config, 2U);
  ASSERT_TRUE(division.ok) << division.error;
  ASSERT_EQ(division.sub_areas.size(), 2U);
  ASSERT_EQ(division.sub_areas[0].polygon.vertices.size(), 4U);
  ASSERT_EQ(division.sub_areas[1].polygon.vertices.size(), 4U);

  const EnuExtents lower = ExtentsOf(division.sub_areas[0], center);
  const EnuExtents upper = ExtentsOf(division.sub_areas[1], center);
  // 下条带为梯形（非矩形）：左腰从 (0,−1000) 斜到 (−300,0)，故 east 左界 = −300。
  EXPECT_NEAR(lower.east_min, -300.0, 1.0);
  EXPECT_NEAR(lower.east_max, 2000.0, 1.0);
  EXPECT_NEAR(lower.north_min, -1000.0, 1.0);
  EXPECT_NEAR(lower.north_max, 0.0, 1.0);
  EXPECT_NEAR(upper.east_min, -600.0, 1.0);
  EXPECT_NEAR(upper.east_max, 1600.0, 1.0);
  EXPECT_NEAR(upper.north_min, 0.0, 1.0);
  EXPECT_NEAR(upper.north_max, 1000.0, 1.0);

  // 逐条带规划：各 3 线 × 2 = 6 航点；上条带首线（y=150）两航点 east
  // 跨度 = x_R − x_L = 1885 m（斜腰几何下的线宽随条带内 v 位置变化）。
  const navigation::AreaCoveragePlanner planner;
  const navigation::RoutePlan upper_plan =
      planner.Plan(division.sub_areas[1], division.sub_configs[1]);
  ASSERT_EQ(upper_plan.size(), 6U);
  const auto [first_east, first_north] = ToEnu(upper_plan[0].position, center);
  const auto [second_east, second_north] = ToEnu(upper_plan[1].position, center);
  (void)first_north;
  (void)second_north;
  EXPECT_NEAR(second_east - first_east, 1885.0, 1.0);
  const navigation::RoutePlan lower_plan =
      planner.Plan(division.sub_areas[0], division.sub_configs[0]);
  ASSERT_EQ(lower_plan.size(), 6U);
}

TEST(AreaDivisionTest, ConcaveLSplitProducesHexagonStrip) {
  // 凹多边形（L 形，参考 (30.0, 120.01)，用与场景同值的 LLA 顶点——
  // 顶边两角因 ENU 曲率差 ~0.06 m，能复现末条带 v_hi 浮点边界的伪重复
  // 顶点输入）：顶点 ENU (−1000,−1000)/(1000,−1000)/(1000,0)/(500,0)/
  // (500,1000)/(−1000,1000)（凹口 = east (500,1000] × north (0,1000]）。
  // 航向 0 → 3 条等宽条带（各 ≈ 666.7 m）：条带 0（南，全宽）与条带 2
  // （北，半宽）= 矩形 4 顶点；条带 1（跨凹口底缘 y=0）= 六边形 6 顶点
  // （east [−1000,1000] 底 + east [−1000,500] 顶）。间距 250 m → 每机
  // 3 线 × 2 = 6 航点。
  const LlaPositionDegM ref = MakeLla(30.0, 120.01, 0.0);
  navigation::CoverageArea area;
  area.kind = navigation::CoverageAreaKind::kPolygon;
  area.polygon.vertices = {MakeLla(29.991006, 119.999628, 0.0),
                           MakeLla(29.991006, 120.020372, 0.0),
                           MakeLla(30.000000, 120.020372, 0.0),
                           MakeLla(30.000000, 120.015186, 0.0),
                           MakeLla(30.008994, 120.015186, 0.0),
                           MakeLla(30.008994, 119.999628, 0.0)};
  navigation::CoveragePlanConfig config = MakeScanConfig();
  config.scan_spacing_m = 250.0;

  const FormationDivisionResult division = DivideArea(area, config, 3U);
  ASSERT_TRUE(division.ok) << division.error;
  ASSERT_EQ(division.sub_areas.size(), 3U);

  // 子区域形状：矩形 / 六边形（凹口跨条带边界）/ 矩形。
  ASSERT_EQ(division.sub_areas[0].polygon.vertices.size(), 4U);
  ASSERT_EQ(division.sub_areas[1].polygon.vertices.size(), 6U);
  ASSERT_EQ(division.sub_areas[2].polygon.vertices.size(), 4U);
  // 顶点互异（守护末条带 v_hi 浮点边界修复：v_hi 舍入 < v_max 时
  // Sutherland–Hodgman 在相邻两边各生成一个几乎重合的交点 → 重复顶点）。
  for (const auto& sub_area : division.sub_areas) {
    const auto& verts = sub_area.polygon.vertices;
    for (std::size_t i = 0U; i < verts.size(); ++i) {
      for (std::size_t k = i + 1U; k < verts.size(); ++k) {
        EXPECT_GT(EcefDistanceM(verts[i], verts[k]), 1e-3);
      }
    }
  }

  const EnuExtents strip0 = ExtentsOf(division.sub_areas[0], ref);
  const EnuExtents strip1 = ExtentsOf(division.sub_areas[1], ref);
  const EnuExtents strip2 = ExtentsOf(division.sub_areas[2], ref);
  // 条带边界 = 2000/3 ≈ 666.67 m（无缝共享）。容差 ±5 m：JSON 顶点按
  // "1° ≈ 111.19 km"（全局平均）撰写，30°N 当地子午线弧长 ≈ 110.87 km/°，
  // 差 0.3%（1 km 处 ≈ 2.6 m）——文档手算同此惯例（"≈"量级）。
  EXPECT_NEAR(strip0.north_min, -1000.0, 5.0);
  EXPECT_NEAR(strip0.north_max, -333.33, 5.0);
  EXPECT_NEAR(strip0.east_min, -1000.0, 5.0);
  EXPECT_NEAR(strip0.east_max, 1000.0, 5.0);
  EXPECT_NEAR(strip1.north_min, -333.33, 5.0);
  EXPECT_NEAR(strip1.north_max, 333.33, 5.0);
  EXPECT_NEAR(strip1.east_min, -1000.0, 5.0);
  EXPECT_NEAR(strip1.east_max, 1000.0, 5.0);
  EXPECT_NEAR(strip2.north_min, 333.33, 5.0);
  EXPECT_NEAR(strip2.north_max, 1000.0, 5.0);
  EXPECT_NEAR(strip2.east_min, -1000.0, 5.0);
  EXPECT_NEAR(strip2.east_max, 500.0, 5.0);

  // 每机 3 线 × 2 = 6 航点（条带 1 首/次线全宽、末线半宽，均 2 交点）。
  const navigation::AreaCoveragePlanner planner;
  for (std::size_t j = 0; j < 3U; ++j) {
    const navigation::RoutePlan plan =
        planner.Plan(division.sub_areas[j], division.sub_configs[j]);
    ASSERT_EQ(plan.size(), 6U);
  }
}

TEST(AreaDivisionTest, InvalidInputsFail) {
  const navigation::CoverageArea polygon = MakeSquareArea();
  const navigation::CoverageArea circle = MakeCircleArea(2000.0);
  const navigation::CoveragePlanConfig scan = MakeScanConfig();
  const navigation::CoveragePlanConfig orbit = MakeOrbitConfig();

  // 编队数 0。
  const FormationDivisionResult zero = DivideArea(polygon, scan, 0U);
  EXPECT_FALSE(zero.ok);
  EXPECT_FALSE(zero.error.empty());

  // 模式-区域不匹配。
  EXPECT_FALSE(DivideArea(circle, scan, 2U).ok);   // 扫描模式 + 圆形
  EXPECT_FALSE(DivideArea(polygon, orbit, 2U).ok);  // 盘旋模式 + 多边形

  // 多边形顶点不足。
  navigation::CoverageArea two_vertices = polygon;
  two_vertices.polygon.vertices.resize(2U);
  EXPECT_FALSE(DivideArea(two_vertices, scan, 2U).ok);

  // 间距非正 / 半径非正。
  navigation::CoveragePlanConfig bad_spacing = scan;
  bad_spacing.scan_spacing_m = 0.0;
  EXPECT_FALSE(DivideArea(polygon, bad_spacing, 2U).ok);
  navigation::CoverageArea bad_radius = circle;
  bad_radius.circle.radius_m = -1.0;
  EXPECT_FALSE(DivideArea(bad_radius, orbit, 2U).ok);

  // 坐标非法（NaN 纬度）。
  navigation::CoverageArea bad_vertex = polygon;
  bad_vertex.polygon.vertices[0].latitude_deg =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(DivideArea(bad_vertex, scan, 2U).ok);

  // 退化多边形（近共线，扫描法向跨度 0）。
  const LlaPositionDegM center = MakeLla(30.0, 120.0, 0.0);
  navigation::CoverageArea collinear;
  collinear.kind = navigation::CoverageAreaKind::kPolygon;
  collinear.polygon.vertices = {OffsetLla(center, -1000.0, 0.0),
                                OffsetLla(center, 0.0, 0.0),
                                OffsetLla(center, 1000.0, 0.0)};
  const FormationDivisionResult degenerate = DivideArea(collinear, scan, 2U);
  EXPECT_FALSE(degenerate.ok);
  EXPECT_FALSE(degenerate.error.empty());
}

}  // namespace
}  // namespace app
}  // namespace component_attachment