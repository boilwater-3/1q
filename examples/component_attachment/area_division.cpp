/**
 * @file area_division.cpp
 * @brief 编队区域切分实现（见 area_division.h）。
 */

#include "area_division.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "1q/coordinate/position_transform.h"

namespace component_attachment {
namespace demo {
namespace {

using oneq::coordinate::EcefPositionM;
using oneq::coordinate::EnuPositionM;
using oneq::coordinate::LlaPositionDegM;

// 示例层局部常量（公共头不暴露角度工具；与 navigation 单测同模式）。
constexpr double kPi = 3.14159265358979323846;

// 度 → 弧度。
double DegToRad(double deg) { return deg * kPi / 180.0; }

// 平面点（ENU 东/北或扫描帧 u/v；示例层局部类型，不依赖 Eigen include 路径）。
// 值构造：VS2015 的聚合 + NSDMI 花括号初始化支持不完整（C2440），与库内
// 公共结构体的处理同款。
struct PlanePoint {
  double x{0.0};
  double y{0.0};

  PlanePoint() = default;
  PlanePoint(double x_in, double y_in) : x(x_in), y(y_in) {}
};

// 多边形顶点投影到以顶点平均为原点的 ENU 平面（东/北二维）。
// 与 AreaCoveragePlanner 同投影惯例（原点取经纬度简单平均），任一点转换
// 失败则整体失败。
bool ProjectPolygonToPlane(const std::vector<LlaPositionDegM>& vertices,
                           LlaPositionDegM* origin, std::vector<PlanePoint>* points) {
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
    points->push_back(PlanePoint{enu.east_m, enu.north_m});
  }
  return true;
}

// 用半平面 v >= limit（keep_above）或 v <= limit（!keep_above）裁剪多边形
// （Sutherland–Hodgman 单步）。边界点含等号保留：相邻条带共享边界边，
// 保证条带并集恰为原区域（无缝隙、无重叠）。
std::vector<PlanePoint> ClipHalfPlane(const std::vector<PlanePoint>& polygon, double v_limit,
                                      bool keep_above) {
  std::vector<PlanePoint> out;
  const std::size_t vertex_count = polygon.size();
  for (std::size_t i = 0; i < vertex_count; ++i) {
    const PlanePoint& a = polygon[i];
    const PlanePoint& b = polygon[(i + 1U) % vertex_count];
    const bool a_inside = keep_above ? a.y >= v_limit : a.y <= v_limit;
    const bool b_inside = keep_above ? b.y >= v_limit : b.y <= v_limit;
    if (a_inside) {
      out.push_back(a);
    }
    if (a_inside != b_inside) {
      const double t = (v_limit - a.y) / (b.y - a.y);
      out.push_back(PlanePoint{a.x + t * (b.x - a.x), v_limit});
    }
  }
  return out;
}

// 扫描帧平面点反变换回 LLA（原点 = 投影参考点；高度取规划高度）。
// 失败返回 false（坐标非法/投影参考点异常）。
bool AppendLlaVertex(std::vector<LlaPositionDegM>* vertices, const PlanePoint& frame_point,
                     const LlaPositionDegM& origin, double heading_rad, double altitude_m) {
  const double cos_h = std::cos(heading_rad);
  const double sin_h = std::sin(heading_rad);
  const double east = frame_point.x * cos_h - frame_point.y * sin_h;
  const double north = frame_point.x * sin_h + frame_point.y * cos_h;
  EnuPositionM enu{east, north, 0.0};
  EcefPositionM ecef{};
  LlaPositionDegM lla{};
  if (!oneq::coordinate::TryEnuToEcef(enu, origin, &ecef) ||
      !oneq::coordinate::TryEcefToLla(ecef, &lla)) {
    return false;
  }
  lla.altitude_m = altitude_m;
  vertices->push_back(lla);
  return true;
}

// 多边形 → 沿扫描航向的等宽条带切分（条带法向 = 扫描线法向 v，与
// AreaCoveragePlanner 扫描帧一致）。任一条带裁剪退化（< 3 顶点）报错。
bool DividePolygonStrips(const navigation::PolygonalArea& polygon,
                         const navigation::CoveragePlanConfig& config,
                         std::size_t aircraft_count,
                         std::vector<navigation::CoverageArea>* sub_areas,
                         std::string* error) {
  LlaPositionDegM origin{};
  std::vector<PlanePoint> plane_points;
  if (!ProjectPolygonToPlane(polygon.vertices, &origin, &plane_points)) {
    *error = "polygon projection to ENU plane failed";
    return false;
  }

  // 旋转到扫描系：u 沿扫描航向，v 为扫描线法向（条带 = v 区间）。
  const double heading_rad = DegToRad(config.scan_heading_deg);
  const double cos_h = std::cos(heading_rad);
  const double sin_h = std::sin(heading_rad);
  std::vector<PlanePoint> frame_points;
  frame_points.reserve(plane_points.size());
  for (const auto& point : plane_points) {
    frame_points.push_back(
        PlanePoint{point.x * cos_h + point.y * sin_h, -point.x * sin_h + point.y * cos_h});
  }

  double v_min = std::numeric_limits<double>::infinity();
  double v_max = -std::numeric_limits<double>::infinity();
  for (const auto& point : frame_points) {
    v_min = std::min(v_min, point.y);
    v_max = std::max(v_max, point.y);
  }
  const double strip_width = (v_max - v_min) / static_cast<double>(aircraft_count);
  if (!(strip_width > 0.0)) {
    *error = "polygon degenerate: zero extent along scan normal";
    return false;
  }

  sub_areas->clear();
  sub_areas->reserve(aircraft_count);
  for (std::size_t j = 0; j < aircraft_count; ++j) {
    const double v_lo = v_min + strip_width * static_cast<double>(j);
    // 末条带 v_hi 显式取 v_max：v_min + width*(j+1) 因浮点舍入可能比 v_max
    // 小约 1 ulp，使位于 v_max 的顶点被判为"略在外侧"，Sutherland–Hodgman
    // 在相邻两边各生成一个几乎重合的交点（子区域出现重复顶点）。取 v_max
    // 后该顶点恰在边界上（含等号判内），无双交点，且并集仍无缝覆盖。
    const double v_hi = (j + 1U == aircraft_count)
                            ? v_max
                            : v_min + strip_width * static_cast<double>(j + 1U);
    std::vector<PlanePoint> clipped = ClipHalfPlane(frame_points, v_hi, false);
    clipped = ClipHalfPlane(clipped, v_lo, true);
    // 退化检查：顶点 < 3，或 u/v 跨度任一不显著（共线多边形经 ENU 回变换
    // 有 ~1e-9 m 浮点噪声，用 1e-6 m 判据排除）→ 该条带无有效覆盖区域。
    if (clipped.size() < 3U) {
      *error = "strip " + std::to_string(j) + " has no valid sub-area (degenerate polygon)";
      return false;
    }
    double u_min = std::numeric_limits<double>::infinity();
    double u_max = -std::numeric_limits<double>::infinity();
    double v_span_min = std::numeric_limits<double>::infinity();
    double v_span_max = -std::numeric_limits<double>::infinity();
    for (const auto& point : clipped) {
      u_min = std::min(u_min, point.x);
      u_max = std::max(u_max, point.x);
      v_span_min = std::min(v_span_min, point.y);
      v_span_max = std::max(v_span_max, point.y);
    }
    if (!(u_max - u_min > 1e-6) || !(v_span_max - v_span_min > 1e-6)) {
      *error = "strip " + std::to_string(j) + " has no valid sub-area (degenerate polygon)";
      return false;
    }
    navigation::CoverageArea sub_area;
    sub_area.kind = navigation::CoverageAreaKind::kPolygon;
    for (const auto& point : clipped) {
      if (!AppendLlaVertex(&sub_area.polygon.vertices, point, origin, heading_rad,
                           config.altitude_m)) {
        *error = "strip " + std::to_string(j) + " vertex back-transform failed";
        return false;
      }
    }
    sub_areas->push_back(std::move(sub_area));
  }
  return true;
}

}  // namespace

FormationDivisionResult DivideArea(const navigation::CoverageArea& area,
                                   const navigation::CoveragePlanConfig& config,
                                   std::size_t aircraft_count) {
  FormationDivisionResult result;
  if (aircraft_count == 0U) {
    result.error = "aircraft_count must be >= 1";
    return result;
  }
  if (aircraft_count == 1U) {
    // 单机编队：整区原样返回（等价于不切分，配置原样透传）。
    result.sub_areas = {area};
    result.sub_configs = {config};
    result.ok = true;
    return result;
  }

  if (config.mode == navigation::CoverageMode::kScan) {
    if (area.kind != navigation::CoverageAreaKind::kPolygon) {
      result.error = "scan mode requires polygon area";
      return result;
    }
    if (area.polygon.vertices.size() < 3U) {
      result.error = "polygon needs >= 3 vertices";
      return result;
    }
    if (!(config.scan_spacing_m > 0.0)) {
      result.error = "scan_spacing_m must be > 0";
      return result;
    }
    if (!std::isfinite(config.scan_heading_deg)) {
      result.error = "scan_heading_deg must be finite";
      return result;
    }
    for (const auto& vertex : area.polygon.vertices) {
      if (!oneq::coordinate::IsValid(vertex)) {
        result.error = "polygon vertex LLA invalid";
        return result;
      }
    }
    if (!DividePolygonStrips(area.polygon, config, aircraft_count, &result.sub_areas,
                             &result.error)) {
      return result;
    }
    // 扫描模式：规划参数原样透传（航向/间距/高度/速度不变，逐条带各自规划）。
    result.sub_configs.assign(aircraft_count, config);
  } else if (config.mode == navigation::CoverageMode::kOrbit) {
    if (area.kind != navigation::CoverageAreaKind::kCircle) {
      result.error = "orbit mode requires circle area";
      return result;
    }
    if (!(area.circle.radius_m > 0.0)) {
      result.error = "circle radius_m must be > 0";
      return result;
    }
    if (!oneq::coordinate::IsValid(area.circle.center)) {
      result.error = "circle center LLA invalid";
      return result;
    }
    if (config.orbit_segments < 3U) {
      result.error = "orbit_segments must be >= 3";
      return result;
    }
    // 外 → 内环：第 j 机环半径 = r * (count - j) / count（主机最外环，与
    // AreaCoveragePlanner 多环外 → 内取点惯例一致）。
    result.sub_areas.reserve(aircraft_count);
    result.sub_configs.reserve(aircraft_count);
    for (std::size_t j = 0; j < aircraft_count; ++j) {
      navigation::CoverageArea sub_area;
      sub_area.kind = navigation::CoverageAreaKind::kCircle;
      sub_area.circle.center = area.circle.center;
      sub_area.circle.radius_m =
          area.circle.radius_m * static_cast<double>(aircraft_count - j) /
          static_cast<double>(aircraft_count);
      result.sub_areas.push_back(std::move(sub_area));
      // 每机单环：环数由切分决定（编队数 = 环数），强制 1，其余参数透传。
      navigation::CoveragePlanConfig sub_config = config;
      sub_config.orbit_rings = 1U;
      result.sub_configs.push_back(std::move(sub_config));
    }
  } else {
    result.error = "unknown coverage mode";
    return result;
  }
  result.ok = true;
  return result;
}

}  // namespace demo
}  // namespace component_attachment
