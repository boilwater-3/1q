/**
 * @file outputs.cpp
 * @brief 统一契约 v2 可视化 CSV 落盘实现（见 outputs.h）。
 *
 * 集成端日志（integration_events.log / integration_views.log + 库日志
 * 1q_library.log）由 logger/logger.h 承担（组件源文件内日志宏 + 每周期
 * 视图直写）——本文件只负责可视化 CSV 落盘。
 */
#include "outputs.h"

#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

#include "1q/coordinate/position_transform.h"
#include "logger/logger.h"

namespace component_attachment {
namespace app {

namespace {

/// 定点格式化（CSV 数值统一走 %.Nf，避免 %g 的科学计数法干扰查看器解析）。
/// 直写 iostream 定点而非 CA_FMT_FORMAT：动态精度 {:.{}f} 不在其支持子集内。
std::string Fmt(double value, int precision) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

}  // namespace

AppOutputs::AppOutputs(const std::string& output_dir)
    : platform_csv_(output_dir + "/platform_track.csv",
                    "cycle,t_sec,aircraft_id,lat_deg,lon_deg,alt_m,heading_deg,"
                    "speed_mps,wp_index,wp_count,model"),
      truth_csv_(output_dir + "/target_truth.csv",
                 "cycle,t_sec,target_id,entity_type,lat_deg,lon_deg,alt_m,rcs"),
      zones_csv_(output_dir + "/zones.csv",
                 "name,kind,lat_deg,lon_deg,alt_m,radius_m"),
      route_csv_(output_dir + "/route_plan.csv",
                 "aircraft_id,index,lat_deg,lon_deg,alt_m,speed_mps,radius_m") {
  // CsvWriter 构造失败即 abort（与既有 CSV 语义一致）。
}

void AppOutputs::RecordPlatformRow(std::uint32_t cycle, double t_sec,
                                    std::uint32_t aircraft_id,
                                    const FlightComponent& flight) {
  platform_csv_.WriteRow(
      std::to_string(cycle) + "," + Fmt(t_sec, 2) + "," + std::to_string(aircraft_id) +
      "," + Fmt(flight.position().latitude_deg, 7) + "," +
      Fmt(flight.position().longitude_deg, 7) + "," +
      Fmt(flight.position().altitude_m, 1) + "," + Fmt(flight.heading_deg(), 1) + "," +
      Fmt(flight.speed_mps(), 1) + "," + std::to_string(flight.next_waypoint_index()) +
      "," + std::to_string(flight.route().size()) +
      "," + (flight.fd_active() ? "jsbsim" : "kinematic"));
  ++platform_rows_;
}

void AppOutputs::RecordTruthRow(std::uint32_t cycle, double t_sec,
                                 const TargetEcefState& target) {
  oneq::coordinate::LlaPositionDegM lla;
  if (!oneq::coordinate::TryEcefToLla(target.position, &lla)) {
    return;  // ECEF 非法：跳过该目标本周期（真值脚本不应触发）
  }
  truth_csv_.WriteRow(
      std::to_string(cycle) + "," + Fmt(t_sec, 2) + "," + std::to_string(target.id) + "," +
      target.type + "," + Fmt(lla.latitude_deg, 7) + "," + Fmt(lla.longitude_deg, 7) + "," +
      Fmt(lla.altitude_m, 1) + "," + Fmt(target.rcs, 2));
}

void AppOutputs::RecordZones(const std::string& name,
                              const navigation::CoverageArea& area) {
  // 多边形：每顶点一行（viewer 按 name 聚合成闭合线）；圆形：一行 + 半径。
  if (area.kind == navigation::CoverageAreaKind::kPolygon) {
    for (const auto& vertex : area.polygon.vertices) {
      zones_csv_.WriteRow(name + ",polygon," + Fmt(vertex.latitude_deg, 7) + "," +
                          Fmt(vertex.longitude_deg, 7) + "," + Fmt(vertex.altitude_m, 2) +
                          ",");
    }
  } else {
    const auto& center = area.circle.center;
    zones_csv_.WriteRow(name + ",circle," + Fmt(center.latitude_deg, 7) + "," +
                        Fmt(center.longitude_deg, 7) + "," + Fmt(center.altitude_m, 2) +
                        "," + Fmt(area.circle.radius_m, 2));
  }
}

void AppOutputs::RecordRoute(std::uint32_t aircraft_id,
                              const std::vector<navigation::RoutePoint>& route) {
  for (std::size_t i = 0U; i < route.size(); ++i) {
    const auto& wp = route[i];
    route_csv_.WriteRow(
        std::to_string(aircraft_id) + "," + std::to_string(i) + "," +
        Fmt(wp.position.latitude_deg, 7) + "," + Fmt(wp.position.longitude_deg, 7) + "," +
        Fmt(wp.position.altitude_m, 1) + "," + Fmt(wp.speed_mps, 1) + "," +
        Fmt(wp.radius_m, 1));
  }
}

void AppOutputs::Flush() {
  platform_csv_.Flush();
  truth_csv_.Flush();
  zones_csv_.Flush();
  route_csv_.Flush();
}

}  // namespace app
}  // namespace component_attachment
