/**
 * @file sbirs_sensor_component.cpp
 * @brief SBIRS 传感器组件实现（会话驱动 + 探测生命周期事件转发）。
 *
 * 驱动模式与 EOS 组件同构：从共享场景状态读取卫星 ECEF 位置与红外目标
 * 真值，构建 SbirsCycleInput 一步驱动 SbirsSession。探测生命周期事件（首
 * 发现/更新/coasting/丢失）由库内 SbirsDetectionLifecycleRecorder 承担
 * （Attach 后 StepWithResult 内部自动驱动），组件把差分事件经归属映射
 * 关联目标 ID 后发布 SbirsDetectionEvent（kind 标注生命周期类型）。
 * 注意：天基平台方位参考系与机载通道不同（见 README"示例简化"声明），
 * 融合方位相干关联在示例编排几何下成立。
 */

#include "sbirs_sensor_component.h"

#include "core/events.h"
#include "core/world.h"
#include "flight_component.h"
#include "scene_types.h"
#include "sensor_adapt.h"

namespace component_attachment {

namespace {

/// 生命周期事件类型 → 示例事件类型（kNotDetected 诊断事件不转发）。
SbirsDetectionEventKind ToDemoKind(
    sbirs_sensor::session::SbirsDetectionLifecycleEventKind kind) {
  switch (kind) {
    case sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kFirstDetected:
      return SbirsDetectionEventKind::kFirstDetected;
    case sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kUpdated:
      return SbirsDetectionEventKind::kUpdated;
    case sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kCoasting:
      return SbirsDetectionEventKind::kCoasting;
    case sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kLost:
      return SbirsDetectionEventKind::kLost;
    default:
      break;  // kNotDetected：组件未开启诊断事件，调用方显式跳过，不会到达
  }
  return SbirsDetectionEventKind::kUpdated;
}

/// 检测记录 ID → 仿真目标 ID（无归属返回 0）。
std::uint64_t AttributionTargetId(
    std::uint64_t detection_id,
    const sbirs_sensor::attribution::SbirsDetectionAttributionRecordList& attributions) {
  for (const auto& attribution : attributions) {
    if (attribution.detection_id == detection_id) {
      return attribution.target_id;
    }
  }
  return 0U;  // 无归属（调试视图缺失）：未知目标
}

}  // namespace

SbirsSensorComponent::SbirsSensorComponent(sbirs_sensor::session::SbirsSession session)
    : session_(std::move(session)) {
  // 探测生命周期事件由库内 recorder 承担（StepWithResult 内部自动喂）。
  session_.AttachDetectionLifecycleRecorder(&lifecycle_);
}

bool SbirsSensorComponent::TryApplyRuntimeConfig(
    const sbirs_sensor::config::SbirsRuntimeConfigPatch& patch) {
  return session_.TryApplyRuntimeConfig(patch);
}

void SbirsSensorComponent::Step(World& world, double dt_sec) {
  detections_.clear();

  const FlightComponent* flight = host_ != nullptr ? host_->Find<FlightComponent>() : nullptr;
  if (flight == nullptr) {
    return;  // 无平台动力学（空场景）：不产生探测
  }

  const auto& scene = static_cast<const DemoSceneState&>(world.scene_state());

  // 天基平台（卫星）位置由消费方每周期注入共享场景状态（世界模型驱动）。
  sbirs_sensor::session::SbirsCycleInput input;
  input.cycle_index = static_cast<std::uint32_t>(scene.cycle);
  input.dt_sec = static_cast<float>(dt_sec);
  input.has_satellite_position = true;
  input.satellite_position_ecef_m = scene.sbirs_satellite_position_ecef_m;
  input.scene = scene.sbirs_targets;

  const sbirs_sensor::session::SbirsCycleResult result = session_.StepWithResult(input);
  if (result.status != sbirs_sensor::session::SbirsCycleStatus::kCompleted) {
    return;  // 周期被拒绝/电源关闭：本周期无探测
  }

  // 库内 recorder 已按跨周期状态差分产出本周期事件（首发现/更新/coasting/
  // 丢失）。recorder 事件无方位/探测 ID 字段：非丢失事件从本周期输出帧按
  // 归属目标回查；丢失事件目标不在帧内，字段留默认。kNotDetected 诊断事件
  // 未开启，显式跳过（防御）。
  const auto& records = result.output_frame.detections;
  for (const auto& event : lifecycle_.GetLastEvents()) {
    if (event.kind == sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kNotDetected) {
      continue;  // 诊断事件（未开启 emit_not_detected_events）：不发布
    }
    SbirsDetectionEvent sbirs_event;
    sbirs_event.cycle = scene.cycle;
    sbirs_event.kind = ToDemoKind(event.kind);
    sbirs_event.target_id = event.target_id;
    sbirs_event.infrared_snr_linear = event.infrared_snr_linear;
    if (event.kind != sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kLost) {
      for (const auto& record : records) {
        if (!record.detected) {
          continue;  // 未过探测门限不参与回查
        }
        if (AttributionTargetId(record.detection_id, result.detection_attributions) ==
            event.target_id) {
          sbirs_event.detection_id = record.detection_id;
          sbirs_event.az_deg = record.azimuth_deg;
          break;
        }
      }
    }
    world.signals().on_sbirs_detection(sbirs_event);
  }

  detections_ = examples::sensor_adapt::AdaptSbirsDetectionsToDetections(
      examples::sensor_adapt::kSbirsSourceId, records);
}

}  // namespace component_attachment
