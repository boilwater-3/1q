/**
 * @file demo_output.cpp
 * @brief 输出落盘与事件消费实现（见 demo_output.h）。
 *
 * 集成端日志（integration_events.log / integration_views.log + 库日志
 * 1q_library.log）由 components/demo_log.h 承担（组件源文件内日志宏 + 每周期
 * 视图直写）——本文件只负责平台轨迹 CSV 落盘与决策事件链。
 */
#include "demo_output.h"

#include "components/demo_log.h"
#include "core/events.h"

namespace component_attachment {
namespace demo {

DemoOutputs::DemoOutputs(const std::string& output_dir)
    : platform_csv_(output_dir + "/platform_track.csv",
                    "cycle,t_sec,lat_deg,lon_deg,alt_m,heading_deg,speed_mps,wp_index") {
  // CsvWriter 构造失败即 abort（与既有 CSV 语义一致）。
}

void DemoOutputs::RecordPlatformRow(std::uint32_t cycle, double t_sec,
                                    const FlightComponent& flight) {
  platform_csv_.WriteRow(
      spdlog::fmt_lib::format("{},{:.2f},{:.7f},{:.7f},{:.1f},{:.1f},{:.1f},{}", cycle, t_sec,
                              flight.position().latitude_deg, flight.position().longitude_deg,
                              flight.position().altitude_m, flight.heading_deg(),
                              flight.speed_mps(), flight.next_waypoint_index()));
  ++platform_rows_;
}

void DemoOutputs::Flush() {
  platform_csv_.Flush();
}

DecisionListener::DecisionListener(World& world, double high_threat_confidence)
    : world_(world), high_threat_confidence_(high_threat_confidence) {
  world_.signals().on_fusion_updated.connect([this](const FusionUpdatedEvent& e) {
    if (e.confidence >= high_threat_confidence_ && !issued_) {
      issued_ = true;
      CommandIssuedEvent command;
      command.cycle = e.cycle;
      command.command = "ENABLE_ANTI_FALSE_TARGET_DISCRIMINATION";
      CA_LOG_EVENT(world_, "command_issued", "指令={}", command.command.c_str());
      world_.signals().on_command_issued(command);
    }
  });
  // 威胁链路：威胁等级 HIGH 首次出现即触发指令（门限 = 评估器 HIGH 阈值；
  // 升级关键事件由 ThreatComponent 直写，此处消费信号补决策指令）。
  world_.signals().on_threat_updated.connect([this](const ThreatUpdatedEvent& e) {
    if (e.result.level == threat_assessment::ThreatLevel::kHigh && !issued_) {
      issued_ = true;
      CommandIssuedEvent command;
      command.cycle = e.cycle;
      command.command = "ENGAGE_HIGH_THREAT";
      // 中译：高威胁目标键 {} 触发交战指令（威胁分 {:.2f}）。
      // 标识：威胁→决策指令链——威胁等级 HIGH 首次出现即下发一次指令，
      //       与融合置信度门限指令互斥（issued_ 共享）。
      CA_LOG_EVENT(world_, "command_issued", "指令={} 键={} 威胁分={:.2f}",
                   command.command.c_str(),
                   static_cast<unsigned long long>(e.result.key),
                   e.result.threat_score);
      world_.signals().on_command_issued(command);
    }
  });
}

}  // namespace demo
}  // namespace component_attachment
