/**
 * @file demo_output.cpp
 * @brief 输出落盘与事件消费实现（见 demo_output.h）。
 *
 * 集成端日志（integration_events.log / integration_views.log + 库日志
 * 1q_library.log）由 logger/logger.h 承担（组件源文件内日志宏 + 每周期
 * 视图直写）——本文件只负责统一契约 v2 可视化 CSV 落盘与决策事件链。
 */
#include "demo_output.h"

#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

#include "1q/coordinate/position_transform.h"
#include "logger/logger.h"
#include "core/events.h"

namespace component_attachment {
namespace demo {

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

DemoOutputs::DemoOutputs(const std::string& output_dir)
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

void DemoOutputs::RecordPlatformRow(std::uint32_t cycle, double t_sec,
                                    std::uint32_t aircraft_id,
                                    const FlightComponent& flight) {
  platform_csv_.WriteRow(
      CA_FMT_FORMAT(
          "{},{:.2f},{},{:.7f},{:.7f},{:.1f},{:.1f},{:.1f},{},{}", cycle, t_sec, aircraft_id,
          flight.position().latitude_deg, flight.position().longitude_deg,
          flight.position().altitude_m, flight.heading_deg(), flight.speed_mps(),
          flight.next_waypoint_index(), flight.route().size()) +
      "," + (flight.fd_active() ? "jsbsim" : "kinematic"));
  ++platform_rows_;
}

void DemoOutputs::RecordTruthRow(std::uint32_t cycle, double t_sec,
                                 const TargetEcefState& target) {
  oneq::coordinate::LlaPositionDegM lla;
  if (!oneq::coordinate::TryEcefToLla(target.position, &lla)) {
    return;  // ECEF 非法：跳过该目标本周期（真值脚本不应触发）
  }
  truth_csv_.WriteRow(CA_FMT_FORMAT(
      "{},{:.2f},{},{},{:.7f},{:.7f},{:.1f},{:.2f}", cycle, t_sec, target.id, target.type,
      lla.latitude_deg, lla.longitude_deg, lla.altitude_m, target.rcs));
}

void DemoOutputs::RecordZones(const std::string& name,
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

void DemoOutputs::RecordRoute(std::uint32_t aircraft_id,
                              const std::vector<navigation::RoutePoint>& route) {
  for (std::size_t i = 0U; i < route.size(); ++i) {
    const auto& wp = route[i];
    route_csv_.WriteRow(CA_FMT_FORMAT(
        "{},{},{:.7f},{:.7f},{:.1f},{:.1f},{:.1f}", aircraft_id, i, wp.position.latitude_deg,
        wp.position.longitude_deg, wp.position.altitude_m, wp.speed_mps, wp.radius_m));
  }
}

void DemoOutputs::Flush() {
  platform_csv_.Flush();
  truth_csv_.Flush();
  zones_csv_.Flush();
  route_csv_.Flush();
}

DecisionListener::DecisionListener(World& world, double high_threat_confidence)
    : world_(world), high_threat_confidence_(high_threat_confidence) {
  world_.signals().on_fusion_updated.connect([this](const FusionUpdatedEvent& e) {
    if (e.confidence >= high_threat_confidence_ && !issued_) {
      issued_ = true;
      CommandIssuedEvent command;
      command.cycle = e.cycle;
      command.kind = CommandKind::kEnableAntiFalseTarget;
      command.command = "ENABLE_ANTI_FALSE_TARGET_DISCRIMINATION";
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string command_issued_event_log =
          std::string("指令=") +
          (command.command.c_str());
      CA_LOG_EVENT(world_, "command_issued", "指令={}", command.command.c_str());
      world_.signals().on_command_issued(command);
    }
  });
  // 威胁链路：威胁等级 HIGH 首次出现即触发指令（门限 = 评估器 HIGH 阈值；
  // 升级关键事件由 ThreatComponent 直写，此处消费信号补决策指令）。指令带
  // 融合键（target_key），CommandRouter 翻译为外部目标 ID 后下发 AR STT
  // 锁定 + RIR 指定识别（决策 → 行动的闭环执行段）。
  world_.signals().on_threat_updated.connect([this](const ThreatUpdatedEvent& e) {
    if (e.result.level == threat_assessment::ThreatLevel::kHigh && !issued_) {
      issued_ = true;
      CommandIssuedEvent command;
      command.cycle = e.cycle;
      command.kind = CommandKind::kEngageHighThreat;
      command.target_key = e.result.key;
      command.command = "ENGAGE_HIGH_THREAT";
      // 中译：高威胁目标键 {} 触发交战指令（威胁分 {:.2f}）。
      // 标识：威胁→决策指令链——威胁等级 HIGH 首次出现即下发一次指令，
      //       与融合置信度门限指令互斥（issued_ 共享）。
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string command_issued_event_log_2 =
          std::string("指令=") +
          (command.command.c_str()) +
          " 键=" +
          std::to_string(static_cast<unsigned long long>(e.result.key)) +
          " 威胁分=" +
          std::to_string(e.result.threat_score);
      CA_LOG_EVENT(world_, "command_issued", "指令={} 键={} 威胁分={:.2f}",
                   command.command.c_str(),
                   static_cast<unsigned long long>(e.result.key),
                   e.result.threat_score);
      world_.signals().on_command_issued(command);
    }
  });
}

CommandRouter::CommandRouter(World& world, ArSensorComponent* ar, RirSensorComponent* rir,
                             const std::vector<ScriptedTarget>& scene_targets)
    : world_(world), ar_(ar), rir_(rir) {
  scene_target_ids_.reserve(scene_targets.size());
  for (const auto& target : scene_targets) {
    scene_target_ids_.push_back(static_cast<std::uint64_t>(target.id));
  }
  connection_ = world_.signals().on_command_issued.connect(
      [this](const CommandIssuedEvent& event) { OnCommand(event); });
}

std::uint64_t CommandRouter::ResolveExternalTargetId(std::uint64_t target_key) const {
  // 解析顺序：① 键即场景外部目标 ID（RIR 通道直挂 / 场景指令脚本指定）；
  // ② AR 通道融合键 = 内部 association_key → 最近成功周期归属表翻译。
  for (const auto& id : scene_target_ids_) {
    if (id == target_key) {
      return id;
    }
  }
  if (ar_ != nullptr) {
    for (const auto& attribution : ar_->last_track_attributions()) {
      if (attribution.association_key == target_key) {
        return attribution.external_target_id;
      }
    }
  }
  return 0U;
}

void CommandRouter::OnCommand(const CommandIssuedEvent& event) {
  switch (event.kind) {
    case CommandKind::kEnableAntiFalseTarget:
      return;  // 纯日志演示指令：无库运行期接口，路由器不执行
    case CommandKind::kEngageHighThreat:
    case CommandKind::kDesignateTarget: {
      const std::uint64_t external_id = ResolveExternalTargetId(event.target_key);
      if (external_id == 0U || (ar_ == nullptr && rir_ == nullptr)) {
        ++dropped_;
        // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
        const std::string command_dropped_event_log =
            std::string("指令=") +
            (event.command.c_str()) +
            " 键=" +
            std::to_string(static_cast<unsigned long long>(event.target_key)) +
            " 成因=" +
            (external_id == 0U ? "键无法解析" : "无传感器可下发");
        CA_LOG_EVENT(world_, "command_dropped", "指令={} 键={} 成因={}",
                     event.command.c_str(),
                     static_cast<unsigned long long>(event.target_key),
                     external_id == 0U ? "键无法解析" : "无传感器可下发");
        return;
      }
      bool patch_rejected = false;
      if (ar_ != nullptr) {
        // STT 指定只在 work_mode == kStt 下被消费：同补丁切模式（指定航迹
        // 丢失/未确认时库自动回退 TWS，见 ArRuntimeConfigPatch 指定语义）。
        airborne_radar::config::ArRuntimeConfigPatch patch;
        patch.has_work_mode = true;
        patch.work_mode = airborne_radar::config::ArWorkMode::kStt;
        patch.has_designated_target_id = true;
        patch.designated_external_target_id = external_id;
        if (event.duration_cycles > 0U) {
          patch.has_designation_duration_cycles = true;
          patch.designation_duration_cycles = event.duration_cycles;
        }
        patch_rejected = patch_rejected || !ar_->TryApplyRuntimeConfig(patch);
      }
      if (rir_ != nullptr) {
        // 会话已 kIdentify（示例配置基线），指定直接可消费；识别达成/窗口
        // 耗尽由会话状态机自动回扫。
        remote_identification_radar::config::RirRuntimeConfigPatch patch;
        patch.has_designated_target_id = true;
        patch.designated_external_target_id = external_id;
        if (event.duration_cycles > 0U) {
          patch.has_designation_duration_cycles = true;
          patch.designation_duration_cycles = event.duration_cycles;
        }
        patch_rejected = patch_rejected || !rir_->TryApplyRuntimeConfig(patch);
      }
      if (patch_rejected) {
        ++dropped_;
        // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
        const std::string command_rejected_event_log =
            std::string("指令=") +
            (event.command.c_str()) +
            " 键=" +
            std::to_string(static_cast<unsigned long long>(event.target_key)) +
            " 成因=补丁被原子拒绝";
        CA_LOG_EVENT(world_, "command_dropped", "指令={} 键={} 成因=补丁被原子拒绝",
                     event.command.c_str(),
                     static_cast<unsigned long long>(event.target_key));
        return;
      }
      ++executed_;
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string command_executed_event_log =
          std::string("指令=") +
          (event.command.c_str()) +
          " 键=" +
          std::to_string(static_cast<unsigned long long>(event.target_key)) +
          " 目标=" +
          std::to_string(static_cast<unsigned long long>(external_id)) +
          " 落点=AR锁定" +
          (rir_ != nullptr ? "+RIR指定" : "");
      CA_LOG_EVENT(world_, "command_executed", "指令={} 键={} 目标={} 落点=AR锁定{}",
                   event.command.c_str(),
                   static_cast<unsigned long long>(event.target_key),
                   static_cast<unsigned long long>(external_id),
                   rir_ != nullptr ? "+RIR指定" : "");
      break;
    }
    case CommandKind::kClearDesignation: {
      bool patch_rejected = false;
      if (ar_ != nullptr) {
        // 清除指定（designated_external_target_id == 0）并回扫描模式
        //（示例配置基线 kTas；STT 无指定时波束按 scan_center 扫描）。
        airborne_radar::config::ArRuntimeConfigPatch patch;
        patch.has_designated_target_id = true;
        patch.has_work_mode = true;
        patch.work_mode = airborne_radar::config::ArWorkMode::kTas;
        patch_rejected = patch_rejected || !ar_->TryApplyRuntimeConfig(patch);
      }
      if (rir_ != nullptr) {
        remote_identification_radar::config::RirRuntimeConfigPatch patch;
        patch.has_designated_target_id = true;
        patch_rejected = patch_rejected || !rir_->TryApplyRuntimeConfig(patch);
      }
      if (patch_rejected) {
        ++dropped_;
        // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
        const std::string command_rejected_event_log_2 =
            std::string("指令=") +
            (event.command.c_str()) +
            " 成因=补丁被原子拒绝";
        CA_LOG_EVENT(world_, "command_dropped", "指令={} 成因=补丁被原子拒绝",
                     event.command.c_str());
        return;
      }
      ++executed_;
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string command_executed_event_log_2 =
          std::string("指令=") +
          (event.command.c_str()) +
          " 动作=清除指定回扫描";
      CA_LOG_EVENT(world_, "command_executed", "指令={} 动作=清除指定回扫描",
                   event.command.c_str());
      break;
    }
  }
}

}  // namespace demo
}  // namespace component_attachment
