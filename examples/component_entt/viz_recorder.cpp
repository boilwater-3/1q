/**
 * @file viz_recorder.cpp
 * @brief 行为层演示可视化数据记录器实现。
 *
 * CSV 列设计约定：
 * - 时间为仿真秒（t_sec = cycle × kBehaviorDtSec），周期号跨表对齐主键；
 * - 位置为度制 WGS84 LLA（真值/融合通道量测），AR 航迹保留雷达局部 ENU；
 * - 枚举以人类可读字符串落盘（status/mode/threat/gate），查看器直接展示；
 * - 空量测（如融合目标某通道无位置）写空字段，解析侧按缺失处理。
 */

#include "viz_recorder.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
#include <unordered_map>

#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "1q/coordinate/position_transform.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"
#include "1q/electronic_surveillance_radar/session/EsrOutputTypes.h"
#include "1q/fusion/FusedTarget.h"

namespace component_entt {

namespace {

/// 定点格式化（CSV 数值统一走 %.Nf，避免 %g 的科学计数法干扰查看器解析）。
std::string Fmt(double value, int precision) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", precision, value);
  return std::string(buf);
}

/// AR 航迹状态 → 可读字符串。
const char* TrackStatusName(airborne_radar::session::TrackStatus status) {
  using airborne_radar::session::TrackStatus;
  switch (status) {
    case TrackStatus::kTentative:
      return "tentative";
    case TrackStatus::kConfirmed:
      return "confirmed";
    case TrackStatus::kLost:
      return "lost";
  }
  return "unknown";
}

/// ESR 辐射源工作模式假设 → 可读字符串。
const char* EmitterModeName(electronic_surveillance_radar::session::EsrEmitterMode mode) {
  using electronic_surveillance_radar::session::EsrEmitterMode;
  switch (mode) {
    case EsrEmitterMode::kUnknown:
      return "unknown";
    case EsrEmitterMode::kSearch:
      return "search";
    case EsrEmitterMode::kTracking:
      return "tracking";
    case EsrEmitterMode::kGuidance:
      return "guidance";
    case EsrEmitterMode::kContinuousIllumination:
      return "cw";
  }
  return "unknown";
}

/// ESR 威胁等级 → 可读字符串。
const char* ThreatLevelName(electronic_surveillance_radar::session::EsrThreatLevel level) {
  using electronic_surveillance_radar::session::EsrThreatLevel;
  switch (level) {
    case EsrThreatLevel::kLow:
      return "low";
    case EsrThreatLevel::kMedium:
      return "medium";
    case EsrThreatLevel::kHigh:
      return "high";
  }
  return "unknown";
}

/// 打开输出目录下的 CSV（表头见各列定义处）。
std::unique_ptr<examples::CsvWriter> OpenCsv(const std::string& dir, const std::string& name,
                                             const std::string& header) {
  return std::make_unique<examples::CsvWriter>(dir + "/" + name, header);
}

}  // namespace

VizRecorder::VizRecorder(const std::string& output_dir, bool flight_model_jsbsim)
    : output_dir_(output_dir), flight_model_jsbsim_(flight_model_jsbsim) {
  std::filesystem::create_directories(output_dir_);
  platform_track_ = OpenCsv(output_dir_, "platform_track.csv",
                            "cycle,t_sec,aircraft_id,lat_deg,lon_deg,alt_m,heading_deg,"
                            "speed_mps,wp_index,wp_count,model");
  target_truth_ = OpenCsv(output_dir_, "target_truth.csv",
                          "cycle,t_sec,target_id,entity_type,lat_deg,lon_deg,alt_m,rcs");
  ar_tracks_ = OpenCsv(output_dir_, "ar_tracks.csv",
                       "cycle,t_sec,key,target_id,status,pos_x_m,pos_y_m,pos_z_m,"
                       "speed_mps,rcs,hit_count,miss_count");
  eos_detections_ =
      OpenCsv(output_dir_, "eos_detections.csv",
              "cycle,t_sec,det_id,target_id,range_m,az_deg,el_deg,snr_db,detected");
  esr_hypotheses_ = OpenCsv(output_dir_, "esr_hypotheses.csv",
                            "cycle,t_sec,hyp_id,bearing_az_deg,bearing_el_deg,"
                            "confidence,mode,threat_level,last_seen_cycle");
  fused_tracks_ = OpenCsv(output_dir_, "fused_tracks.csv",
                          "cycle,t_sec,key,confidence,last_update_cycle,"
                          "ar_samples,esr_samples,eos_samples,lat_deg,lon_deg,alt_m,"
                          "bearing_az_deg");
  waypoint_events_ = OpenCsv(output_dir_, "waypoint_events.csv",
                             "t_sec,waypoint_index,intermediate,gate,distance_m,"
                             "cross_track_m,along_track_m,threshold_m");
}

void VizRecorder::RecordCycle(std::uint32_t cycle, double t_sec,
                              const FleetStatusComponent& fleet,
                              const RoutePlanComponent& route,
                              const FusedSituationComponent& situation,
                              const airborne_radar::session::ArCycleResult& ar,
                              const electronic_surveillance_radar::session::EsrCycleResult& esr,
                              const electro_optical_sensor::session::EosCycleResult& eos,
                              const std::vector<TruthTargetRow>& truths) {
  // 平台轨迹（多机契约 v2：aircraft_id 列，单机示例恒为 1；模型列：
  // jsbsim 真实飞行 / kinematic 运动学回退）。
  platform_track_->WriteRow(
      std::to_string(cycle) + "," + Fmt(t_sec, 3) + ",1," +
      Fmt(fleet.position.latitude_deg, 7) + "," + Fmt(fleet.position.longitude_deg, 7) + "," +
      Fmt(fleet.position.altitude_m, 2) + "," + Fmt(fleet.heading_deg, 2) + "," +
      Fmt(fleet.speed_mps, 2) + "," + std::to_string(route.next_index) + "," +
      std::to_string(route.route.size()) + "," + (flight_model_jsbsim_ ? "jsbsim" : "kinematic"));

  // 世界真值目标（ECEF → 度制 LLA；entity_type 透出空中/地面）。
  for (const TruthTargetRow& truth : truths) {
    oneq::coordinate::LlaPositionDegM lla;
    if (!oneq::coordinate::TryEcefToLla(truth.position, &lla)) {
      continue;  // ECEF 非法：跳过该目标本周期（真值脚本不应触发）
    }
    target_truth_->WriteRow(
        std::to_string(cycle) + "," + Fmt(t_sec, 3) + "," + std::to_string(truth.target_id) +
        "," + truth.entity_type + "," + Fmt(lla.latitude_deg, 7) + "," +
        Fmt(lla.longitude_deg, 7) + "," + Fmt(lla.altitude_m, 2) + "," +
        Fmt(truth.rcs, 3));
  }

  // AR 航迹快照（雷达局部 ENU：东/北/上，含平台姿态旋转）。
  for (const auto& track : ar.track_output_frame.tracks) {
    ar_tracks_->WriteRow(
        std::to_string(cycle) + "," + Fmt(t_sec, 3) + "," +
        std::to_string(track.association_key) + "," +
        std::to_string(track.external_target_id) + "," +
        TrackStatusName(track.status) + "," + Fmt(track.position_x, 2) + "," +
        Fmt(track.position_y, 2) + "," + Fmt(track.position_z, 2) + "," +
        Fmt(track.speed, 2) + "," + Fmt(track.rcs, 3) + "," +
        std::to_string(track.hit_count) + "," + std::to_string(track.miss_count));
  }

  // EOS 光电探测（平台局部方位/俯仰 + 斜距 + 融合 SNR + 真值目标归属）。
  // 归属映射（仿真面，非真实传感器输出）把 detection_id 关联回真值目标 ID。
  std::unordered_map<std::uint64_t, std::uint64_t> eos_attribution;
  for (const auto& attr : eos.detection_attributions) {
    eos_attribution[attr.detection_id] = attr.target_id;
  }
  for (const auto& det : eos.output_frame.detections) {
    const auto it = eos_attribution.find(det.detection_id);
    const std::string target_id =
        it != eos_attribution.end() ? std::to_string(it->second) : "";
    eos_detections_->WriteRow(
        std::to_string(cycle) + "," + Fmt(t_sec, 3) + "," +
        std::to_string(det.detection_id) + "," + target_id + "," + Fmt(det.range_m, 2) + "," +
        Fmt(det.azimuth_deg, 3) + "," + Fmt(det.elevation_deg, 3) + "," +
        Fmt(det.fused_snr_db, 2) + "," + (det.detected ? "1" : "0"));
  }

  // ESR 辐射源假设（方位线 + 信号参数 + 假设置信度）。
  for (const auto& hyp : esr.output_frame.emitter_output.hypotheses) {
    esr_hypotheses_->WriteRow(
        std::to_string(cycle) + "," + Fmt(t_sec, 3) + "," +
        std::to_string(hyp.hypothesis_id) + "," + Fmt(hyp.bearing_az_deg, 3) + "," +
        Fmt(hyp.bearing_el_deg, 3) + "," + Fmt(hyp.confidence, 4) + "," +
        EmitterModeName(hyp.mode) + "," + ThreatLevelName(hyp.threat_level) + "," +
        std::to_string(hyp.last_seen_cycle));
  }

  // 融合态势（各通道 sample_count + AR 通道位置 + ESR 通道方位）。
  for (const auto& target : situation.targets) {
    std::string ar_samples = "0", esr_samples = "0", eos_samples = "0";
    std::string lat, lon, alt, bearing;
    for (const auto& channel : target.channels) {
      switch (channel.source_id) {
        case kArSourceId:
          ar_samples = std::to_string(channel.sample_count);
          if (channel.has_position) {
            lat = Fmt(channel.position.latitude_deg, 7);
            lon = Fmt(channel.position.longitude_deg, 7);
            alt = Fmt(channel.position.altitude_m, 2);
          }
          break;
        case kEsrSourceId:
          esr_samples = std::to_string(channel.sample_count);
          if (channel.has_bearing) {
            bearing = Fmt(channel.bearing_az_deg, 3);
          }
          break;
        case kEosSourceId:
          eos_samples = std::to_string(channel.sample_count);
          break;
        default:
          break;  // 未知通道：忽略（融合配置索引 0 未用）
      }
    }
    fused_tracks_->WriteRow(
        std::to_string(cycle) + "," + Fmt(t_sec, 3) + "," + std::to_string(target.key) + "," +
        Fmt(target.confidence, 4) + "," + std::to_string(target.last_update_cycle) + "," +
        ar_samples + "," + esr_samples + "," + eos_samples + "," + lat + "," + lon + "," +
        alt + "," + bearing);
  }
}

void VizRecorder::RecordRoute(const RoutePlanComponent& route) {
  if (route_recorded_ || route.route.empty()) {
    return;  // 幂等：仅首次写入
  }
  route_recorded_ = true;
  route_plan_ = OpenCsv(output_dir_, "route_plan.csv",
                        "aircraft_id,index,lat_deg,lon_deg,alt_m,speed_mps,radius_m");
  for (std::size_t i = 0U; i < route.route.size(); ++i) {
    const auto& wp = route.route[i];
    route_plan_->WriteRow(
        "1," + std::to_string(i) + "," + Fmt(wp.position.latitude_deg, 7) + "," +
        Fmt(wp.position.longitude_deg, 7) + "," + Fmt(wp.position.altitude_m, 2) + "," +
        Fmt(wp.speed_mps, 2) + "," + Fmt(wp.radius_m, 2));
  }
}

void VizRecorder::RecordZones(const std::string& name,
                              const navigation::CoverageArea& area) {
  if (std::find(zones_recorded_.begin(), zones_recorded_.end(), name) !=
      zones_recorded_.end()) {
    return;  // 幂等：同名区域只写一次
  }
  zones_recorded_.push_back(name);
  if (zones_ == nullptr) {
    zones_ = OpenCsv(output_dir_, "zones.csv",
                     "name,kind,lat_deg,lon_deg,alt_m,radius_m");
  }
  // 多边形：每顶点一行（viewer 按 name 聚合成闭合线）；圆形：一行 + 半径。
  if (area.kind == navigation::CoverageAreaKind::kPolygon) {
    for (const auto& vertex : area.polygon.vertices) {
      zones_->WriteRow(name + ",polygon," + Fmt(vertex.latitude_deg, 7) + "," +
                       Fmt(vertex.longitude_deg, 7) + "," + Fmt(vertex.altitude_m, 2) + ",");
    }
  } else {
    const auto& center = area.circle.center;
    zones_->WriteRow(name + ",circle," + Fmt(center.latitude_deg, 7) + "," +
                     Fmt(center.longitude_deg, 7) + "," + Fmt(center.altitude_m, 2) + "," +
                     Fmt(area.circle.radius_m, 2));
  }
}

void VizRecorder::RecordWaypointEvents(const std::vector<WaypointEventRow>& events) {
  // 事件环形记录（容量 512，写满覆盖最旧）不能用"已写条数"去重（满后新事件
  // 会被静默丢弃）。完成时刻在本会话内单调递增，按 t_sec 去重即可：
  // 只追加晚于已写最后一条的事件（Reset 清空事件后旧事件 t_sec 不再出现）。
  for (const auto& e : events) {
    if (e.t_sec <= last_waypoint_event_t_sec_) {
      continue;
    }
    last_waypoint_event_t_sec_ = e.t_sec;
    waypoint_events_->WriteRow(
        Fmt(e.t_sec, 3) + "," + std::to_string(e.waypoint_index) + "," +
        (e.intermediate ? "1" : "0") + "," + e.gate + "," + Fmt(e.distance_m, 2) + "," +
        Fmt(e.cross_track_m, 2) + "," + Fmt(e.along_track_m, 2) + "," +
        Fmt(e.threshold_m, 2));
  }
}

void VizRecorder::Flush() {
  platform_track_->Flush();
  target_truth_->Flush();
  ar_tracks_->Flush();
  eos_detections_->Flush();
  esr_hypotheses_->Flush();
  fused_tracks_->Flush();
  if (route_plan_ != nullptr) {
    route_plan_->Flush();
  }
  waypoint_events_->Flush();
  if (zones_ != nullptr) {
    zones_->Flush();
  }
}

}  // namespace component_entt
