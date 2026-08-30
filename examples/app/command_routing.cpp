/**
 * @file command_routing.cpp
 * @brief 决策监听与指令路由实现（见 command_routing.h）。
 */
#include "command_routing.h"

#include <string>

#include "logger/logger.h"
#include "core/events.h"

namespace component_attachment {
namespace app {

DecisionListener::DecisionListener(World& world, double high_threat_confidence)
    : world_(world), high_threat_confidence_(high_threat_confidence) {
  EnsureSignalConnections();
}

void DecisionListener::EnsureSignalConnections() {
  if (!fusion_connection_.connected()) {
    fusion_connection_ = world_.signals().on_fusion_updated.connect(
        [this](const FusionUpdatedEvent& event) { OnFusionUpdated(event); });
  }
  if (!threat_connection_.connected()) {
    threat_connection_ = world_.signals().on_threat_updated.connect(
        [this](const ThreatUpdatedEvent& event) { OnThreatUpdated(event); });
  }
}

void DecisionListener::OnFusionUpdated(const FusionUpdatedEvent& event) {
  if (event.confidence >= high_threat_confidence_ && !issued_) {
    issued_ = true;
    CommandIssuedEvent command;
    command.cycle = event.cycle;
    command.kind = CommandKind::kEnableAntiFalseTarget;
    command.command = "ENABLE_ANTI_FALSE_TARGET_DISCRIMINATION";
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string command_issued_event_log =
        std::string("指令=") +
        (command.command.c_str());
    CA_LOG_EVENT(world_, "command_issued", "指令={}", command.command.c_str());
    world_.signals().on_command_issued(command);
  }
}

void DecisionListener::OnThreatUpdated(const ThreatUpdatedEvent& event) {
  // 威胁链路：威胁等级 HIGH 首次出现即触发指令（门限 = 评估器 HIGH 阈值；
  // 升级关键事件由 ThreatComponent 直写，此处消费信号补决策指令）。指令带
  // 融合键（target_key），CommandRouter 翻译为外部目标 ID 后下发 AR STT
  // 锁定 + RIR 指定识别（决策 → 行动的闭环执行段）。
  if (event.level == EventThreatLevel::kHigh && !issued_) {
    issued_ = true;
    CommandIssuedEvent command;
    command.cycle = event.cycle;
    command.kind = CommandKind::kEngageHighThreat;
    command.target_key = event.key;
    command.command = "ENGAGE_HIGH_THREAT";
    // 中译：高威胁目标键 {} 触发交战指令（威胁分 {:.2f}）。
    // 标识：威胁→决策指令链——威胁等级 HIGH 首次出现即下发一次指令，
    //       与融合置信度门限指令互斥（issued_ 共享）。
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string command_issued_event_log_2 =
        std::string("指令=") +
        (command.command.c_str()) +
        " 键=" +
        std::to_string(static_cast<unsigned long long>(event.key)) +
        " 威胁分=" +
        std::to_string(event.threat_score);
    CA_LOG_EVENT(world_, "command_issued", "指令={} 键={} 威胁分={:.2f}",
                 command.command.c_str(),
                 static_cast<unsigned long long>(event.key),
                 event.threat_score);
    world_.signals().on_command_issued(command);
  }
}

CommandRouter::CommandRouter(World& world, ArSensorComponent* ar, RirSensorComponent* rir,
                             const std::vector<ScriptedTarget>& scene_targets)
    : world_(world), ar_(ar), rir_(rir) {
  scene_target_ids_.reserve(scene_targets.size());
  for (const auto& target : scene_targets) {
    scene_target_ids_.push_back(static_cast<std::uint64_t>(target.id));
  }
  EnsureSignalConnections();
}

void CommandRouter::EnsureSignalConnections() {
  if (!connection_.connected()) {
    connection_ = world_.signals().on_command_issued.connect(
        [this](const CommandIssuedEvent& event) { OnCommand(event); });
  }
}

ImpactForecastReceiptListener::ImpactForecastReceiptListener(World& world,
                                                             std::uint64_t receiver_entity_id)
    : world_(world), receiver_entity_id_(receiver_entity_id) {
  EnsureSignalConnections();
}

void ImpactForecastReceiptListener::EnsureSignalConnections() {
  if (!impact_forecast_connection_.connected()) {
    impact_forecast_connection_ = world_.signals().on_impact_forecast_published.connect(
        [this](const ImpactForecastEvent& event) { OnImpactForecastPublished(event); });
  }
}

void ImpactForecastReceiptListener::OnImpactForecastPublished(
    const ImpactForecastEvent& event) {
  LogEvent(
      event.cycle, static_cast<double>(event.cycle), "impact_forecast",
      "目标键=" + std::to_string(event.key) + " 落点LLA=(" +
          std::to_string(event.impact_latitude_deg) + "," +
          std::to_string(event.impact_longitude_deg) + "," +
          std::to_string(event.impact_altitude_m) + ") 落点1σ=" +
          std::to_string(event.impact_position_sigma_m) + "m 关机点1σ=" +
          (event.has_burnout_sigma ? std::to_string(event.burnout_position_sigma_m) + "m"
                                   : std::string("无")) +
          " 置信度=" + std::to_string(event.confidence) +
          " 分发对象ID=" + std::to_string(receiver_entity_id_));
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
    case CommandKind::kEnableAntiFalseTarget: {
      // 纯日志演示指令（无库运行期接口）：路由器以"受理"终态收口并计入执行数，
      // 保证 指令下发数 = 执行数 + 丢弃数 恒闭合（不留无终态沿的指令）。
      ++executed_;
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string command_executed_event_log =
          std::string("指令=") +
          (event.command.c_str()) +
          " 处置=受理(纯日志演示,无库接口)";
      CA_LOG_EVENT(world_, "command_executed", "指令={} 处置=受理(纯日志演示,无库接口)",
                   event.command.c_str());
      return;
    }
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

}  // namespace app
}  // namespace component_attachment
