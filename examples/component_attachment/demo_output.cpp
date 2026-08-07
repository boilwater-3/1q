/**
 * @file demo_output.cpp
 * @brief 输出落盘与事件消费实现（见 demo_output.h）。
 *
 * 集成端日志（integration.log + 库日志 1q_library.log）由 components/demo_log.h
 * 承担（组件源文件内日志宏 + 每周期视图直写）——本文件只负责平台轨迹 CSV
 * 落盘与决策事件链。
 */

#include "demo_output.h"

#include "components/demo_log.h"
#include "core/events.h"
#include "demo_config.h"

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

DecisionListener::DecisionListener(World& world) : world_(world) {
  world_.signals().on_fusion_updated.connect([this](const FusionUpdatedEvent& e) {
    if (e.confidence >= kHighThreatConfidence && !issued_) {
      issued_ = true;
      CommandIssuedEvent command;
      command.cycle = e.cycle;
      command.command = "ENABLE_ANTI_FALSE_TARGET_DISCRIMINATION";
      // 事件日志：字符串就地填充（发布处记录，与组件宏同模式）。
      CA_LOG_EVENT(world_, "command_issued", "cmd={}", command.command.c_str());
      world_.signals().on_command_issued(command);
    }
  });
}

}  // namespace demo
}  // namespace component_attachment
