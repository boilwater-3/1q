/**
 * @file esr_sensor_component.cpp
 * @brief ESR 传感器组件实现（会话驱动 + 假设事件发布）。
 *
 * 驱动模式与行为层 recon_system::DriveEsrSession 同构：EsrCycleInput
 * 直接构造（平台 ECEF 运动学 + 消费方注入的辐射源真值帧），输出假设
 * 适配为泛型探测记录（sensor_utils.h）。每条假设（键 ≠ 0）经 World
 * 信号发布 EmitterHypothesisEvent，供日志/决策订阅。
 */

#include "esr_sensor_component.h"

#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "core/events.h"
#include "flight_component.h"
#include "core/world.h"
#include "scene_types.h"
#include "sensor_adapt.h"
#include "sensor_utils.h"

namespace component_attachment {

EsrSensorComponent::EsrSensorComponent(electronic_surveillance_radar::session::EsrSession session)
    : session_(std::move(session)) {}

bool EsrSensorComponent::TryApplyRuntimeConfig(
    const electronic_surveillance_radar::config::EsrRuntimeConfigPatch& patch) {
  return session_.TryApplyRuntimeConfig(patch);
}

electronic_surveillance_radar::session::EsrRuntimeConfigApplyResult
EsrSensorComponent::ApplyRuntimeConfigWithResult(
    const electronic_surveillance_radar::config::EsrRuntimeConfigPatch& patch) {
  return session_.ApplyRuntimeConfigWithResult(patch);
}

void EsrSensorComponent::Step(World& world, double dt_sec) {
  detections_.clear();

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
  if (result.status !=
      electronic_surveillance_radar::session::EsrCycleExecutionStatus::kCompleted) {
    return;  // 周期被拒绝/电源关闭：本周期无假设
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
    event.threat_level = hypothesis.threat_level;
    world.signals().on_emitter_hypothesis(event);
  }
  detections_ = examples::sensor_adapt::AdaptHypothesesToDetections(
      examples::sensor_adapt::kEsrSourceId, hypotheses);
}

}  // namespace component_attachment
