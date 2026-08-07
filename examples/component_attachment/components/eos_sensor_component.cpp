/**
 * @file eos_sensor_component.cpp
 * @brief EOS 传感器组件实现（会话驱动 + 探测生命周期事件转发）。
 *
 * 1. EosCycleInputAdapter 一步构建周期输入（零姿态共享平台局部系），驱动
 *    EosSession，输出探测适配为泛型探测记录（sensor_utils.h）；
 * 2. 探测事件（首发现/更新/丢失）由库内 EosDetectionLifecycleRecorder 差分
 *    产出，组件经归属映射关联目标 ID 后发布 EosDetectionEvent；
 * 3. 事件与调试视图直写集成端日志（CA_LOG_EVENT / CA_LOG_VIEW，中文人读行）。
 */

#include "eos_sensor_component.h"

#include "1q/electro_optical_sensor/session/EosCycleInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "core/events.h"
#include "demo_log.h"
#include "flight_component.h"
#include "core/world.h"
#include "scene_types.h"
#include "sensor_adapt.h"
#include "sensor_utils.h"

namespace component_attachment {

namespace {

/// 探测记录 ID → 仿真目标 ID（无归属返回 0）。
std::uint64_t AttributionTargetId(
    std::uint64_t detection_id,
    const electro_optical_sensor::attribution::EosDetectionAttributionRecordList& attributions) {
  for (const auto& attribution : attributions) {
    if (attribution.detection_id == detection_id) {
      return attribution.target_id;
    }
  }
  return 0U;  // 无归属（调试视图缺失）：未知目标
}

/// 生命周期事件类型 → 示例事件类型（kNotDetected 诊断事件不转发）。
EosDetectionEventKind ToDemoKind(
    electro_optical_sensor::session::EosDetectionLifecycleEventKind kind) {
  switch (kind) {
    case electro_optical_sensor::session::EosDetectionLifecycleEventKind::kFirstDetected:
      return EosDetectionEventKind::kFirstDetected;
    case electro_optical_sensor::session::EosDetectionLifecycleEventKind::kUpdated:
      return EosDetectionEventKind::kUpdated;
    case electro_optical_sensor::session::EosDetectionLifecycleEventKind::kLost:
      return EosDetectionEventKind::kLost;
    default:
      break;  // kNotDetected：组件未开启诊断事件，调用方显式跳过，不会到达
  }
  return EosDetectionEventKind::kUpdated;
}

/// 示例事件类型 → 中文名（人读日志）。
const char* EosEventKindName(EosDetectionEventKind kind) {
  switch (kind) {
    case EosDetectionEventKind::kFirstDetected:
      return "首发现";
    case EosDetectionEventKind::kUpdated:
      return "更新";
    case EosDetectionEventKind::kLost:
      return "丢失";
  }
  return "未知";
}

/// 调试目标状态 → 中文名（人读日志）。
const char* EosTargetStatusName(electro_optical_sensor::session::EosDebugTargetStatus status) {
  switch (status) {
    case electro_optical_sensor::session::EosDebugTargetStatus::kDetected:
      return "已检测";
    case electro_optical_sensor::session::EosDebugTargetStatus::kObservedBelowThreshold:
      return "低于门限";
    case electro_optical_sensor::session::EosDebugTargetStatus::kNotInOutput:
      return "不在输出";
    case electro_optical_sensor::session::EosDebugTargetStatus::kCycleNotExecuted:
      return "周期未执行";
  }
  return "未知";
}

}  // namespace

EosSensorComponent::EosSensorComponent(electro_optical_sensor::session::EosSession session)
    : session_(std::move(session)) {
  // 探测生命周期事件由库内 recorder 承担（StepWithResult 内部自动喂）。
  session_.AttachDetectionLifecycleRecorder(&lifecycle_);
}

bool EosSensorComponent::TryApplyRuntimeConfig(
    const electro_optical_sensor::config::EosRuntimeConfigPatch& patch) {
  const bool applied = session_.TryApplyRuntimeConfig(patch);
  if (applied && patch.has_sensor_enabled) {
    powered_on_ = patch.sensor_enabled;  // 电源状态由补丁唯一维护（组件层电源门控）
  }
  return applied;
}

// 视图行写入（三模式，宏门控——未选中的模式不参与编译，见 demo_log.h 模式选择
// 区）。DebugView 每周期都构建，落多少、怎么落由集成方按需求选择。
void EosSensorComponent::LogDebugView(
    const electro_optical_sensor::session::EosOutputDebugView& view) {
#if defined(CA_VIEW_LOG_MODE_NONNOMINAL)
  // 模式一（只落非标称行）：跳过已检测（标称）目标，日志量 ∝ 异常数。
  std::size_t non_nominal = 0U;
  for (const auto& target : view.targets) {
    if (target.status == electro_optical_sensor::session::EosDebugTargetStatus::kDetected) {
      continue;
    }
    ++non_nominal;
    CA_LOG_VIEW("eos", "周期={} 目标={} 状态={} 距离={:.1f}m 方位={:.1f}°",
                view.input_cycle_index, target.target_id,
                EosTargetStatusName(target.status), target.range_m, target.azimuth_deg);
  }
  if (non_nominal == 0U) {
    CA_LOG_VIEW("eos", "周期={} 全部正常（{} 个目标均已检测）", view.input_cycle_index,
                view.targets.size());
  }
#elif defined(CA_VIEW_LOG_MODE_DELTA)
  // 模式二（跨周期状态增量）：上一周期状态表由组件持有（target_id → status），
  // 首次出现视为变化；表只增不减，目标集长期收缩时调用方可按需清理。
  std::size_t changed = 0U;
  for (const auto& target : view.targets) {
    const auto it = prev_target_status_.find(target.target_id);
    if (it == prev_target_status_.end() || it->second != target.status) {
      ++changed;
      CA_LOG_VIEW("eos", "周期={} 目标={} 状态={}",
                  view.input_cycle_index, target.target_id,
                  EosTargetStatusName(target.status));
    }
    prev_target_status_[target.target_id] = target.status;
  }
  if (changed == 0U) {
    CA_LOG_VIEW("eos", "周期={} 无状态变化", view.input_cycle_index);
  }
#else  // CA_VIEW_LOG_MODE_SUMMARY（默认）
  // 模式三（每周期摘要行）：目标状态明细 + 问题 code 列表，一眼可读。
  std::string targets_text;
  for (const auto& target : view.targets) {
    if (!targets_text.empty()) {
      targets_text += ", ";
    }
    targets_text += spdlog::fmt_lib::format("{} {}", target.target_id,
                                            EosTargetStatusName(target.status));
  }
  std::string issues_text;
  for (const auto& issue : view.issues) {
    if (!issues_text.empty()) {
      issues_text += ", ";
    }
    issues_text += issue.code;
  }
  CA_LOG_VIEW("eos", "周期={} 执行={} 目标=[{}] 问题=[{}]",
              view.input_cycle_index, view.executed_this_cycle ? "是" : "否",
              targets_text.empty() ? "无" : targets_text.c_str(),
              issues_text.empty() ? "无" : issues_text.c_str());
#endif  // CA_VIEW_LOG_MODE_*
}

void EosSensorComponent::Step(World& world, double dt_sec) {
  detections_.clear();

  if (!powered_on_) {
    scan_azimuth_deg_ = 0.0f;  // 关机：不驱动会话，角度无有效值（清零）
    last_debug_view_ = electro_optical_sensor::session::EosOutputDebugView{};  // 关机：调试视图清零（无有效周期）
    LogDebugView(last_debug_view_);
    return;
  }

  const FlightComponent* flight = host_ != nullptr ? host_->Find<FlightComponent>() : nullptr;
  if (flight == nullptr) {
    return;  // 无平台动力学（空场景）：不产生探测
  }

  const auto& scene = static_cast<const DemoSceneState&>(world.scene_state());

  // 外部平台运动学（零姿态：三会话共享同一平台局部坐标系）。
  electro_optical_sensor::session::EosExternalPoseInput pose;
  ResolvePlatformEcef(flight->position(), flight->heading_deg(), flight->speed_mps(),
                      &pose.platform_position_ecef_m, &pose.platform_velocity_mps);

  electro_optical_sensor::session::EosCycleInput input;
  electro_optical_sensor::session::EosCoordinateStatus status;
  if (!electro_optical_sensor::session::EosCycleInputAdapter::Build(
          pose, scene.optical_targets, static_cast<float>(dt_sec), &input, &status)) {
    return;  // 坐标适配失败：本周期不产生探测
  }
  input.cycle_index = static_cast<std::uint32_t>(scene.cycle);

  const electro_optical_sensor::session::EosCycleResult result = session_.StepWithResult(input);
  scan_azimuth_deg_ = result.output_frame.scan_azimuth_deg;  // 扫描方位随周期结果刷新（拒绝周期为空帧 → 0）
  // 规则 12 落盘示范：每周期构建调试视图快照（拒绝周期为 kCycleNotExecuted，
  // 含规则 13b kInfo 排除诊断），供调用方结构化持久化；本示例经 LogDebugView
  // 直写中文人读行（三模式由集成方按需选择）。
  last_debug_view_ = electro_optical_sensor::session::EosOutputDebugViewBuilder::Build(input, result);
  LogDebugView(last_debug_view_);
  if (result.status != electro_optical_sensor::session::EosCycleStatus::kCompleted) {
    return;  // 周期被拒绝：本周期无探测
  }

  // 库内 recorder 已按跨周期状态差分产出本周期事件（首发现/更新/丢失）。recorder
  // 事件无方位/探测 ID 字段：非丢失事件从本周期输出帧按归属目标回查；丢失事件
  // 目标不在帧内，字段留默认。kNotDetected 诊断事件未开启，显式跳过（防御）。
  const auto& records = result.output_frame.detections;
  for (const auto& event : lifecycle_.GetLastEvents()) {
    if (event.kind == electro_optical_sensor::session::EosDetectionLifecycleEventKind::kNotDetected) {
      continue;  // 诊断事件（未开启 emit_not_detected_events）：不发布
    }
    EosDetectionEvent eos_event;
    eos_event.cycle = scene.cycle;
    eos_event.kind = ToDemoKind(event.kind);
    eos_event.target_id = event.target_id;
    eos_event.snr_db = event.fused_snr_db;
    if (event.kind != electro_optical_sensor::session::EosDetectionLifecycleEventKind::kLost) {
      for (const auto& record : records) {
        if (!record.detected) {
          continue;  // 未过探测门限不参与回查
        }
        if (AttributionTargetId(record.detection_id, result.detection_attributions) ==
            event.target_id) {
          eos_event.detection_id = record.detection_id;
          eos_event.az_deg = record.azimuth_deg;
          break;
        }
      }
    }
    if (eos_event.kind == EosDetectionEventKind::kUpdated) {
      // 更新类事件每周期重复：事件模式一下不落盘（信号照常发布）。
      CA_LOG_EVENT_DUP(world, "eos_detection", "类型={} 探测ID={} 目标={} 信噪比={:.1f}dB 方位={:.1f}°",
                       EosEventKindName(eos_event.kind),
                       static_cast<unsigned long long>(eos_event.detection_id),
                       static_cast<unsigned long long>(eos_event.target_id), eos_event.snr_db,
                       eos_event.az_deg);
    } else {
      CA_LOG_EVENT(world, "eos_detection", "类型={} 探测ID={} 目标={} 信噪比={:.1f}dB 方位={:.1f}°",
                   EosEventKindName(eos_event.kind),
                   static_cast<unsigned long long>(eos_event.detection_id),
                   static_cast<unsigned long long>(eos_event.target_id), eos_event.snr_db,
                   eos_event.az_deg);
    }
    world.signals().on_eos_detection(eos_event);
  }

  detections_ = examples::sensor_adapt::AdaptEosDetectionsToDetections(
      examples::sensor_adapt::kEosSourceId, records);
}

}  // namespace component_attachment
