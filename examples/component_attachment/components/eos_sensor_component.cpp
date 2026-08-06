/**
 * @file eos_sensor_component.cpp
 * @brief EOS 传感器组件实现（会话驱动 + 探测事件发布）。
 *
 * 驱动模式与行为层 recon_system::DriveEosSession 同构：EosCycleInputAdapter
 * 一步构建周期输入（零姿态：共享平台局部系，方位可直接相干比较），
 * 输出探测适配为泛型探测记录（sensor_utils.h）。通过门限的探测经归属
 * 映射（detection_attributions）关联目标 ID 后发布 EosDetectionEvent。
 */

#include "eos_sensor_component.h"

#include "1q/electro_optical_sensor/session/EosCycleInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "core/events.h"
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

}  // namespace

EosSensorComponent::EosSensorComponent(electro_optical_sensor::session::EosSession session)
    : session_(std::move(session)) {}

void EosSensorComponent::Step(World& world, double dt_sec) {
  detections_.clear();

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
  if (result.status != electro_optical_sensor::session::EosCycleStatus::kCompleted) {
    return;  // 周期被拒绝/电源关闭：本周期无探测
  }

  const auto& records = result.output_frame.detections;
  for (const auto& record : records) {
    if (!record.detected) {
      continue;  // 未过探测门限不产生探测
    }
    EosDetectionEvent event;
    event.cycle = scene.cycle;
    event.detection_id = record.detection_id;
    event.target_id = AttributionTargetId(record.detection_id, result.detection_attributions);
    event.snr_db = record.fused_snr_db;
    event.az_deg = record.azimuth_deg;
    world.signals().on_eos_detection(event);
  }
  detections_ = examples::sensor_adapt::AdaptEosDetectionsToDetections(
      examples::sensor_adapt::kEosSourceId, records);
}

}  // namespace component_attachment
