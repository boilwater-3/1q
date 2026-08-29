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
#include "components/sensor_utils.h"
#include "logger/logger.h"

namespace component_attachment {
namespace app {

namespace {

namespace rir = remote_identification_radar::session;

/// 定点格式化（CSV 数值统一走 %.Nf，避免 %g 的科学计数法干扰查看器解析）。
/// 直写 iostream 定点而非 CA_FMT_FORMAT：动态精度 {:.{}f} 不在其支持子集内。
std::string Fmt(double value, int precision) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

/// 航迹调试状态 → 稳定 ASCII token（查看器四阶段判定，避免中文解析歧义）。
const char* RirStatusToken(rir::RirDebugTargetStatus status) {
  switch (status) {
    case rir::RirDebugTargetStatus::kConfirmed:
      return "confirmed";
    case rir::RirDebugTargetStatus::kTentative:
      return "tentative";
    case rir::RirDebugTargetStatus::kLost:
      return "lost";
    case rir::RirDebugTargetStatus::kNotInOutput:
      return "no_track";
    case rir::RirDebugTargetStatus::kCycleNotCompleted:
      return "not_completed";
  }
  return "no_track";
}

/// 识别状态机 → 稳定 ASCII token。
const char* RirRecognitionStateToken(rir::RirRecognitionState state) {
  switch (state) {
    case rir::RirRecognitionState::kDisabled:
      return "disabled";
    case rir::RirRecognitionState::kAccumulating:
      return "accumulating";
    case rir::RirRecognitionState::kCategoryConfirmed:
      return "category_confirmed";
    case rir::RirRecognitionState::kModelConfirmed:
      return "model_confirmed";
    case rir::RirRecognitionState::kUnknown:
      return "unknown";
    case rir::RirRecognitionState::kStale:
      return "stale";
  }
  return "disabled";
}

/// 识别大类 → 稳定 ASCII token。
const char* RirCategoryToken(rir::RirRecognitionCategory category) {
  switch (category) {
    case rir::RirRecognitionCategory::kBallistic:
      return "ballistic";
    case rir::RirRecognitionCategory::kNearSpace:
      return "near_space";
    case rir::RirRecognitionCategory::kOther:
      return "other";
    case rir::RirRecognitionCategory::kUnknown:
      return "unknown";
    case rir::RirRecognitionCategory::kFighter:
      return "fighter";
    case rir::RirRecognitionCategory::kBomber:
      return "bomber";
    case rir::RirRecognitionCategory::kMissile:
      return "missile";
  }
  return "unknown";
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
                 "aircraft_id,index,lat_deg,lon_deg,alt_m,speed_mps,radius_m"),
      output_dir_(output_dir) {
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

void AppOutputs::RecordRirSite(
    const oneq::coordinate::LlaPositionDegM& site_origin,
    const remote_identification_radar::config::RirSessionConfig& rir_config) {
  // 惰性创建：仅 RIR 场景产出本文件。扫描扇区绝对方位 = scan_center_az +
  // [az_min, az_max]（相对可扫描限位）；俯仰域为绝对 ENU；max_range 定扇区半径。
  if (rir_site_csv_ == nullptr) {
    rir_site_csv_ = std::unique_ptr<examples::CsvWriter>(new examples::CsvWriter(
        output_dir_ + "/rir_site.csv",
        "site_lat_deg,site_lon_deg,site_alt_m,scan_center_az_deg,scan_center_el_deg,"
        "az_min_deg,az_max_deg,el_min_deg,el_max_deg,max_range_m,"
        "scan_win_az_min_deg,scan_win_az_max_deg,scan_win_el_min_deg,scan_win_el_max_deg"));
  }
  const auto& volume = rir_config.orientation;
  const auto& mission = rir_config.mission;
  // 硬件最大可扫描体积（az/el 限位）+ 任务扫描子窗（用户指定作战搜索扇区，相对
  // scan_center 的 az、绝对 el）：查看器据此画「最大扇面」与「任务扇面」两个区域。
  const auto& window = mission.scan_window_deg;
  rir_site_csv_->WriteRow(
      Fmt(site_origin.latitude_deg, 7) + "," + Fmt(site_origin.longitude_deg, 7) + "," +
      Fmt(site_origin.altitude_m, 1) + "," + Fmt(mission.scan_center_deg.az_deg, 2) + "," +
      Fmt(mission.scan_center_deg.el_deg, 2) + "," + Fmt(volume.az_min_deg, 2) + "," +
      Fmt(volume.az_max_deg, 2) + "," + Fmt(volume.el_min_deg, 2) + "," +
      Fmt(volume.el_max_deg, 2) + "," + Fmt(mission.max_range_m, 1) + "," +
      Fmt(window.az_min_deg, 2) + "," + Fmt(window.az_max_deg, 2) + "," +
      Fmt(window.el_min_deg, 2) + "," + Fmt(window.el_max_deg, 2));
}

void AppOutputs::RecordRirCycle(std::uint32_t cycle, double t_sec,
                                const oneq::coordinate::LlaPositionDegM& site_origin,
                                const rir::RirOutputDebugView& view,
                                float max_detected_slant_range_m) {
  if (rir_targets_csv_ == nullptr) {
    rir_targets_csv_ = std::unique_ptr<examples::CsvWriter>(new examples::CsvWriter(
        output_dir_ + "/rir_targets.csv",
        "cycle,t_sec,target_id,target_name,present_in_input,has_track,status,"
        "look_az_deg,look_el_deg,slant_range_m,pos_lat_deg,pos_lon_deg,pos_alt_m,"
        "speed_mps,recognition_state,target_category,target_model,confidence,"
        "designation_active,dwell_center_az_deg,dwell_center_el_deg,"
        "cycle_max_detected_slant_m"));
  }
  const std::string dwell_az = Fmt(view.dwell_center_deg.az_deg, 2);
  const std::string dwell_el = Fmt(view.dwell_center_deg.el_deg, 2);
  const std::string designation_active = view.designation_active ? "1" : "0";
  // 库上报「实际有效目标最大斜距」（RirCycleResult.max_detected_slant_range_m）：
  // 逐目标行重复本周期标量，查看器取任一行显示（区别于 max_range_m 粗筛门）。
  const std::string cycle_max_slant = Fmt(static_cast<double>(max_detected_slant_range_m), 1);
  for (const auto& state : view.targets) {
    if (state.external_target_id == 0U) {
      continue;  // 零 ID 目标无法按 ID 关联（诊断行），不进可视化。
    }
    // 有航迹时用滤波 ENU 位置换 LLA；否则位置列留空（查看器回退 target_truth）。
    std::string pos_lat;
    std::string pos_lon;
    std::string pos_alt;
    oneq::coordinate::LlaPositionDegM lla;
    if (state.has_track &&
        TryEnuMetersToLla(state.position_enu_x_m, state.position_enu_y_m,
                          state.position_enu_z_m, site_origin, &lla)) {
      pos_lat = Fmt(lla.latitude_deg, 7);
      pos_lon = Fmt(lla.longitude_deg, 7);
      pos_alt = Fmt(lla.altitude_m, 1);
    }
    rir_targets_csv_->WriteRow(
        std::to_string(cycle) + "," + Fmt(t_sec, 2) + "," +
        std::to_string(static_cast<unsigned long long>(state.external_target_id)) + "," +
        examples::EscapeCsvField(state.target_name) + "," +
        (state.present_in_input ? "1" : "0") + "," + (state.has_track ? "1" : "0") + "," +
        RirStatusToken(state.status) + "," + Fmt(state.look_az_deg, 3) + "," +
        Fmt(state.look_el_deg, 3) + "," + Fmt(state.slant_range_m, 1) + "," + pos_lat + "," +
        pos_lon + "," + pos_alt + "," + Fmt(state.speed_m_per_s, 1) + "," +
        RirRecognitionStateToken(state.recognition_state) + "," +
        RirCategoryToken(state.target_category) + "," +
        examples::EscapeCsvField(state.target_model) + "," +
        Fmt(static_cast<double>(state.confidence), 3) + "," + designation_active + "," +
        dwell_az + "," + dwell_el + "," + cycle_max_slant);
  }
}

void AppOutputs::Flush() {
  platform_csv_.Flush();
  truth_csv_.Flush();
  zones_csv_.Flush();
  route_csv_.Flush();
  if (rir_site_csv_ != nullptr) {
    rir_site_csv_->Flush();
  }
  if (rir_targets_csv_ != nullptr) {
    rir_targets_csv_->Flush();
  }
}

}  // namespace app
}  // namespace component_attachment
