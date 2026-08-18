/**
 * @file esr_sensor_component.cpp
 * @brief ESR 传感器组件实现（会话驱动 + 假设事件发布）。
 *
 * 1. EsrCycleInput 直接构造（平台 ECEF 运动学 + 消费方注入的辐射源真值帧），
 *    驱动 EsrSession，输出假设适配为泛型探测记录（fusion::AdaptEsrHypothesesToDetectionRecords）；
 * 2. 每条假设（键 ≠ 0）经 World 信号发布 EmitterHypothesisEvent；
 * 3. 事件直写集成端日志（CA_LOG_EVENT，中文人读行）。
 */

#include "esr_sensor_component.h"

#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/fusion/SensorAdapters.h"
#include "core/events.h"
#include "logger/logger.h"
#include "flight_component.h"
#include "core/world.h"
#include "scene_types.h"
#include "sensor_utils.h"

namespace component_attachment {

namespace {

/// 排除原因门内归因 → 中文名（人读日志，规则 13b 门内归因条款）。
const char* EsrExclusionCauseName(electronic_surveillance_radar::session::EsrIssueCause cause) {
  switch (cause) {
    case electronic_surveillance_radar::session::EsrIssueCause::kNone:
      return "无归因";
    case electronic_surveillance_radar::session::EsrIssueCause::kTransmitSilent:
      return "发射静默";
    case electronic_surveillance_radar::session::EsrIssueCause::kOverlapWindow:
      return "时频重叠为零";
    case electronic_surveillance_radar::session::EsrIssueCause::kPropagationLoss:
      return "传播损耗";
    case electronic_surveillance_radar::session::EsrIssueCause::kHardGateFailed:
      return "硬门失败";
    case electronic_surveillance_radar::session::EsrIssueCause::kStatisticalGateFailed:
      return "统计门失败";
    case electronic_surveillance_radar::session::EsrIssueCause::kUnknown:
      return "未知主因";
  }
  return "未知主因";
}

}  // namespace

EsrSensorComponent::EsrSensorComponent(electronic_surveillance_radar::session::EsrSession session)
    : session_(std::move(session)) {
  // 排除原因跨周期差分事件由库内 recorder 承担（StepWithResult 内部自动喂）。
  // ESR 无既有 lifecycle recorder，exclusion_ 为首个 recorder 成员。
  session_.AttachExclusionCauseRecorder(&exclusion_);
}

bool EsrSensorComponent::TryApplyRuntimeConfig(
    const electronic_surveillance_radar::config::EsrRuntimeConfigPatch& patch) {
  const bool applied = session_.TryApplyRuntimeConfig(patch);
  if (applied && patch.has_sensor_enabled) {
    powered_on_ = patch.sensor_enabled;  // 电源状态由补丁唯一维护（组件层电源门控）
  }
  return applied;
}

electronic_surveillance_radar::session::EsrRuntimeConfigApplyResult
EsrSensorComponent::ApplyRuntimeConfigWithResult(
    const electronic_surveillance_radar::config::EsrRuntimeConfigPatch& patch) {
  const auto result = session_.ApplyRuntimeConfigWithResult(patch);
  if (result.applied && patch.has_sensor_enabled) {
    powered_on_ = patch.sensor_enabled;  // 电源状态由补丁唯一维护（组件层电源门控）
  }
  return result;
}

void EsrSensorComponent::Step(World& world, double dt_sec) {
  detections_.clear();

  if (!powered_on_) {
    scan_azimuth_deg_ = 0.0f;  // 关机：不驱动会话，角度无有效值（清零）
    return;
  }

  const FlightComponent* flight = host_ != nullptr ? host_->Find<FlightComponent>() : nullptr;
  if (flight == nullptr) {
    return;  // 无平台动力学（空场景）：不产生探测
  }

  const auto& scene = static_cast<const DemoSceneState&>(world.scene_state());
  electronic_surveillance_radar::session::EsrCycleInput input;
  input.cycle_index = static_cast<std::uint32_t>(scene.cycle);
  input.cycle_start_time_s = scene.t_sec;
  input.dt_sec = static_cast<float>(dt_sec);
  input.platform_entity_id = 1U;  // 平台实体标识（本示例单平台）
  input.has_platform_ecef_kinematics = true;
  ResolvePlatformEcef(flight->position(), flight->heading_deg(), flight->speed_mps(),
                      &input.platform_position_ecef_m, &input.platform_velocity_ecef_mps);
  // RF 场景包络必须与本周期权威时间一致（含空帧亦须填齐）。
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  input.rf_emissions.emissions = scene.emitters;  // 辐射源真值由消费方脚本注入

  const electronic_surveillance_radar::session::EsrCycleResult result = session_.StepWithResult(input);
  scan_azimuth_deg_ = result.output_frame.scan_azimuth_deg;  // 扫描方位随周期结果刷新（拒绝周期为空帧 → 0）
  if (result.status !=
      electronic_surveillance_radar::session::EsrCycleExecutionStatus::kCompleted) {
    return;  // 周期被拒绝：本周期无假设
  }

  const auto& hypotheses = result.output_frame.emitter_output.hypotheses;
  for (const auto& hypothesis : hypotheses) {
    if (hypothesis.hypothesis_id == 0U) {
      continue;  // 库内键 0 = 无身份，不发布事件
    }
    EmitterHypothesisEvent event;
    event.cycle = scene.cycle;
    event.hypothesis_id = hypothesis.hypothesis_id;
    event.bearing_az_deg = hypothesis.bearing_az_deg;
    event.confidence = hypothesis.confidence;
    event.mode = hypothesis.mode;
    // 假设事件每周期重复（目标恒在时）：事件模式一下不落盘（信号照常发布）。
    // 威胁等级已随 TARGET-OQ-2 处置退出传感器公开输出（威胁评估归 threat_assessment）。
    CA_LOG_EVENT_DUP(world, "emitter_hypothesis", "假设={} 方位={:.1f}° 置信={:.2f} 模式={}",
                     static_cast<unsigned long long>(event.hypothesis_id),
                     event.bearing_az_deg, event.confidence, static_cast<int>(event.mode));
    world.signals().on_emitter_hypothesis(event);
  }

  // 排除原因跨周期差分事件（规则 13e）：纯诊断观测，仅落事件日志（不发 World
  // 信号——不驱动融合/威胁等下游组件）。ESR 无 target_id，以发射源标识
  // （platform/equipment/emission 三元组）为实体关联键。差分键为 (code,cause)
  // 组合对——co-site↔zero-power 切换（kNone→细分 cause）亦产事件。
  for (const auto& event : exclusion_.GetLastEvents()) {
    const auto& id = event.identity;
    if (event.kind == electronic_surveillance_radar::session::EsrExclusionCauseEventKind::kEntered) {
      CA_LOG_EVENT(world, "exclusion_cause",
                   "辐射源=(platform={},equipment={},emission={}) 类型=进入排除 排除码={} 主因={}",
                   static_cast<unsigned long long>(id.platform_id),
                   static_cast<unsigned long long>(id.equipment_id),
                   static_cast<unsigned long long>(id.emission_id),
                   event.current_code, EsrExclusionCauseName(event.current_cause));
    } else if (event.kind == electronic_surveillance_radar::session::EsrExclusionCauseEventKind::kChanged) {
      CA_LOG_EVENT(world, "exclusion_cause",
                   "辐射源=(platform={},equipment={},emission={}) 类型=原因变化 旧码={} 旧主因={} 新码={} 新主因={}",
                   static_cast<unsigned long long>(id.platform_id),
                   static_cast<unsigned long long>(id.equipment_id),
                   static_cast<unsigned long long>(id.emission_id),
                   event.previous_code, EsrExclusionCauseName(event.previous_cause),
                   event.current_code, EsrExclusionCauseName(event.current_cause));
    } else if (event.kind == electronic_surveillance_radar::session::EsrExclusionCauseEventKind::kExited) {
      CA_LOG_EVENT(world, "exclusion_cause",
                   "辐射源=(platform={},equipment={},emission={}) 类型=退出排除 旧码={} 旧主因={}",
                   static_cast<unsigned long long>(id.platform_id),
                   static_cast<unsigned long long>(id.equipment_id),
                   static_cast<unsigned long long>(id.emission_id),
                   event.previous_code, EsrExclusionCauseName(event.previous_cause));
    }
  }

  detections_ = fusion::AdaptEsrHypothesesToDetectionRecords(
      fusion::kEsrSourceId, hypotheses);
}

}  // namespace component_attachment
