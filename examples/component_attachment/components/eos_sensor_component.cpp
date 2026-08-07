/**
 * @file eos_sensor_component.cpp
 * @brief EOS 传感器组件实现（会话驱动 + 探测生命周期事件转发）。
 *
 * 驱动模式与行为层 recon_system::DriveEosSession 同构：EosCycleInputAdapter
 * 一步构建周期输入（零姿态：共享平台局部系，方位可直接相干比较），
 * 输出探测适配为泛型探测记录（sensor_utils.h）。探测生命周期事件（首发
 * 现/更新/丢失）由库内 EosDetectionLifecycleRecorder 承担（Attach 后
 * StepWithResult 内部自动驱动），组件把差分事件经归属映射关联目标 ID
 * 后发布 EosDetectionEvent（kind 标注生命周期类型）。
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

void EosSensorComponent::Step(World& world, double dt_sec) {
  detections_.clear();

  if (!powered_on_) {
    scan_azimuth_deg_ = 0.0f;  // 关机：不驱动会话，角度无有效值（清零）
    last_debug_view_ = electro_optical_sensor::session::EosOutputDebugView{};  // 关机：调试视图清零（无有效周期）
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
  // 扫描方位随周期结果刷新：被拒绝周期输出帧为默认空帧 → 0。
  scan_azimuth_deg_ = result.output_frame.scan_azimuth_deg;
  // 规则 12 落盘示范：每周期构建调试视图快照（拒绝周期为 kCycleNotExecuted），
  // 供调用方序列化为 JSON 写进自己的日志（含规则 13b kInfo 排除诊断）。
  last_debug_view_ = electro_optical_sensor::session::EosOutputDebugViewBuilder::Build(input, result);
  if (result.status != electro_optical_sensor::session::EosCycleStatus::kCompleted) {
    return;  // 周期被拒绝：本周期无探测
  }

  // 库内 recorder 已按跨周期状态差分产出本周期事件（首发现/更新/丢失）。
  // recorder 事件无方位/探测 ID 字段：非丢失事件从本周期输出帧按归属目标
  // 回查；丢失事件目标不在帧内，字段留默认。kNotDetected 诊断事件未开启，
  // 显式跳过（防御）。
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
    // 事件日志：字符串就地填充（日志宏 + 组件源文件内格式化串）。
    CA_LOG_EVENT(world, "eos_detection", "kind=%d det=%llu target=%llu snr=%.1fdB az=%.1f",
                 static_cast<int>(eos_event.kind),
                 static_cast<unsigned long long>(eos_event.detection_id),
                 static_cast<unsigned long long>(eos_event.target_id), eos_event.snr_db,
                 eos_event.az_deg);
    world.signals().on_eos_detection(eos_event);
  }

  detections_ = examples::sensor_adapt::AdaptEosDetectionsToDetections(
      examples::sensor_adapt::kEosSourceId, records);
}

}  // namespace component_attachment
